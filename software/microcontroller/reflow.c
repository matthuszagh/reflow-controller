/* reflow.c
 *
 * The main source file for the reflow controller. This will both configure the device to be used
 * as a CDC ACM device and control the PID operation of the toaster oven with a state machine.
 */

#include "reflow.h"

/** Port B pins. */
#define SS 0
#define SCK 1
#define MOSI 2
#define MISO 3

/** PWM parameters. */
#define TOP 6250

/** Temperature parameters. */
#define ROOM_TEMP 100

/** Temperature profile to load into EEPROM. Units are 1/4 of a degree C and 1 second. We use 1/4 of
 *  a degree since that is the temperature resolution of the MAX31855 thermocouple ADC. The PID
 *  coefficients should be adjusted to optimal values. `EEMEM` is an attribute that causes
 *  eeprom_profile to be allocated in the .eeprom section.
 */
static profile_t EEMEM eeprom_profile = {
  .preheat_rate = 1*4,
  .soak_temp1 = 100*4,
  .soak_temp2 = 150*4,
  .soak_time = 90,
  .liquides_temp = 183*4,
  .time_to_liquides = 30,
  .peak_temp = 235*4,
  .time_to_peak = 60,
  .cool_rate = 2*4,
  .pid_p = 360,
  .pid_i = 80,
  .pid_d = 0
};

static volatile bool usb_connected;
/** Update the PID every time the counter reaches its top value and resets. */
static volatile bool update_pid;
static bool tx_flag;  /** Set to 1 when we should transmit data to the host computer. */
/** Counts to 5 to update the target and flag that a transmission should be made to the host
 *  computer. Since the period of a PWM waveform is 0.2s, this will update the target and message
 *  the host every second. The OCR1A value will still be updated every 0.2s but the target can only
 *  change at most once per second and there is no need to inundate the host computer.
 */
static uint8_t target_update;
static profile_t profile;  /** Temperature profile in SRAM. */
static state reflow_state; /** Reflow state. */
static uint16_t room_temp; /** Room temperature. */
/** Target temperature. Initialize it to the starting temperature which is 25C. */
static uint16_t target = ROOM_TEMP;
static int16_t integral;  /** Keeps track of the integral term of the PID. */
/** Keeps track of the previous error value for the derivative term of the PID. */
static int32_t prev_error;

/** LUFA CDC Class driver interface configuration and state information. This structure is passed to
 *  all CDC Class driver functions, so that multiple instances of the same class within a device can
 *  be differentiated from one another.
 */
static USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
{
  .Config =
  {
    .ControlInterfaceNumber   = INTERFACE_ID_CDC_CCI,
    .DataINEndpoint           =
    {
      .Address          = CDC_TX_EPADDR,
      .Size             = CDC_TXRX_EPSIZE,
      .Banks            = 1,
    },
    .DataOUTEndpoint =
    {
      .Address          = CDC_RX_EPADDR,
      .Size             = CDC_TXRX_EPSIZE,
      .Banks            = 1,
    },
    .NotificationEndpoint =
    {
      .Address          = CDC_NOTIFICATION_EPADDR,
      .Size             = CDC_NOTIFICATION_EPSIZE,
      .Banks            = 1,
    },
  }
};

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs.
 */
static FILE USBSerialStream;

int main(void)
{
  uint16_t temp = 0; /** Thermocouple temperature. */
  uint16_t timer = 0; /** Timer keeps track of location in the different reflow stages. */

  setup_hardware();
  /** Read temperature profile from EEPROM into SRAM. */
  eeprom_read_block(&profile, &eeprom_profile, sizeof(profile));
  GlobalInterruptEnable();
  reflow_state = PREHEAT;

  while (1) {
    if (usb_connected) {
      usb_rx();
      /** Flush data to USB. */
      CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
      USB_USBTask();
    }
    if (usb_connected && tx_flag) {
      tx_flag = 0;
      fprintf(&USBSerialStream,
              "temp: %" PRIu16 ";\t"
              "room: %" PRIu16 ";\t"
              "target: %" PRIu16 ";\t"
              "PWM: %" PRIu16 ";\t"
              "state: %d\n",
              temp, room_temp, target, OCR1A, reflow_state);
    }
    /** Update OCR1A. This doesn't require that a host computer is connected to function properly. */
    if (update_pid) {
      if (target_update++ == 5) {
        tx_flag = 1;
        target_update = 0;
        ++timer;
        target = get_target(temp, &timer);
      }
      temp = read_temp();
      update_pid = 0;  /** Wait for the next interrupt before updating the PID again. */
      OCR1A = pid(temp, target);
    }
  }
}

/** Configures the board hardware and chip peripherals. */
void setup_hardware(void)
{
  /** Disables the watchdog timer and clears the Watchdog System Reset Flag (`WDRF`) since we will
   *  not use the watchdog timer. The first line sets the `WDRF` bit of the MCU Status Register
   *  (`MCUSR`) to 0. The reason we do this is to prevent an eternal loop of resets if the watchdog
   *  timer is set by a brownout condition or runaway pointer as described on page 54 of the
   *  ATMega8U2 datasheet. `WDRF` is a macro defined as 3 in `<avr/iom8.h>` and `MCUSR` is a
   *  reference to the address of the `MCUSR` register also defined as a macro in
   *  `<avr/iom8.h>`. `wdt_disable()` disables the operation of the watchdog timer and is defined in
   *  `<avr/wdt.h>`.
   */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();

  /** Disables clock division (i.e. we use the external clock frequency as
   *  is). `clock_prescale_set()` is defined in `<avr/power.h>` and sets the Clock Prescale Register
   *  (`CLKPR`) as specified on page 39 of the ATMega8U2 datasheet. `clock_div_1` sets all bits to 0
   *  in accordance with no clock division.
   */
  clock_prescale_set(clock_div_1);

  /** LUFA function to initialize USB and associates it with a character device so that `<stdio.h>`
   * functions can be used to communicate with it.
   */
  USB_Init();
  CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

  /** Configure pin directions. `DDRB` is a Data Direction Register for port B (follows the syntax
   *  `DDRx` where `x` specifies the port). `1` configures the pin as an output pin and `0`
   *  configures it as an input pin. The method for configuring IO ports starts on page 67 of the
   *  ATMega8U2 datasheet and the register description for DDRB is given on page 82. Bit 0
   *  configures PB0, bit 1 configures PB1, etc. As indicated on page 82, the DDRB register is 0
   *  initialized, so only output pins need to be explicitly configured. We only set SS and SCK as
   *  outputs since we do not send data via MOSI, and MISO should be an input anyway.
   */
  DDRB = (1 << SS) | (1 << SCK);

  /** Initialize `SS` to high and `SCK` to low. The port B data register is `PORTB` */
  PORTB |= (1 << SS);
  PORTB &= ~(1 << SCK);

  /** The documentation for register TCCR1A starts on page 129 of the datasheet. It is used to
   *  control the operation of the PWM output.
   *
   *  Bit 7:6 - COM1A1:0: Compare Output Mode for Channel A. `10` configures the OC1A (PC6) output
   *  to be driven low when a compare match is made (when the compare register equals TOP). This is
   *  used since we connect PC6 as an open collector pin.
   *
   *  Bit 5:4 - COM1B1:0: Compare Output Mode for Channel B. Since channel B is unused, we can set
   *  this as 00, which leaves OC1B disconnected.
   *
   *  Bit 3:2 - COM1C1:0: Compare Output Mode for Channel C. Same as channel B.
   *
   *  Bit 1:0 - WGM1[1:0]: Waveform Generation Mode. Together with WGM1[3:2] in TCCR1B, these bits
   *  set the source for the TOP counter value and the type of waveform generation to be used. Our
   *  configuration uses Fast PWM (described on page 121 of the datasheet) and sets ICR1 as the top
   *  value.
   */
  TCCR1A = 0b10000010;

  /** The documentation for this register is on page 133 of the datasheet. It is used for the PWM
   *  output.
   *
   *  Bit 7 - Input Capture Noise Canceler. Filters input from ICP1/PC7. We haven't hooked up PC7 so
   *  this shouldn't be necessary (i.e. should be set to 0).
   *
   *  Bit 6 - Input Capture Edge Select. Also related to PC7 and can be set to 0.
   *
   *  Bit 5 - Reserved bit that should always be set to 0.
   *
   *  Bit 4:3 - WGM1[3:2]: Waveform Generation Mode. Together with the values specified for
   *  WGM1[1:0] in TCCR1A specify we should use the Fast PWM timer/counter mode of operation.
   *
   *  Bit 2:0 - Clock select. 100 uses the prescaler to generate a frequency of 8MHz/256 = 31.25kHz.
   *
   *  Note: I have deviated here from Forsten's code.
   */
  TCCR1B = 0b00011100;

  /** The timer/counter 16-bit register is used to hold the current value of the counter for PWM. We
   *  initialize it to 0.
   */
  TCNT1 = 0x0000;

  /** ICR1 holds the top value that TCNT1 counts up to. */
  ICR1 = TOP;

  /** OCR1A holds the count at which we set OCN1 to 0 and can be updated to adjust the duty cycle of
   *  the PWM output. We initialize it to TOP so that the default setting is for OCN1 to be high
   *  (i.e. the PWM is off).
   */
  OCR1A = TOP;

  OCR1B = 1;

  /** DDRC is the data direction register for channel C. Since we use PC6 as the relay signal we
   *  must set it to write=1.
   */
  DDRC = 0b01000000;

  /** Enable an interrupt vector to be signaled when TCNT1 reaches the top value. This will be used
   *  to update OCR1A value.
   */
  TIMSK1 = (1<<OCIE1B);

  /** SPCR is the SPI Control Register. When SPE is set to 1, SPI is enabled and when MSTR is set to
   *  1, it is in master mode. This is described on page 145.
   */
  SPCR = (1<<SPE) | (1<<MSTR);

  /** SPSR is the SPI Status Register. When we set SPI2X to 1, the frequency of SPI in master mode
   *  is cut in half (i.e. 4MHz). This must be done since the max operating frequency of the
   *  thermocouple ADC is 5MHz.
   */
  SPSR = (1<<SPI2X);
}

uint16_t read_temp(void) {
  /** Thermocouple ADC bits.
   *  31      : sign bit for thermocouple temp.
   *  30 - 18 : thermocouple temp (MSB - LSB). Each bit is 1/4 of a degree.
   *  17      : reserved.
   *  16      : 1 if fault.
   *  15      : sign bit for cold junction temperature.
   *  14 - 4  : cold junction temperature data. Each bit is 1/16 of a degree so we must divide by 4 to
   *            convert to our scale.
   *  3       : reserved.
   *  2       : 1 if thermocouple shorted to VCC.
   *  1       : 1 if thermocouple shorted to GND.
   *  0       : 1 if thermocouple is open circuit.
   */

  /** Sensor retrieves data from the SPI shift register when it is full. Since the thermocouple ADC
   *  transmits 32 bits, we need 4 bytes to hold  all the data.
   */
  uint8_t sensor[4];
  /** Variable to hold thermocouple temperature. */
  uint16_t temp;

  /** Pull SS low to receive data from the thermocouple ADC. */
  PORTB = (0<<SS);

  /** Read sensor data. */
  for (int8_t i=0; i<4; ++i) {
    SPDR = 0x00; /** Clear the data register. */
    /** Wait for the shift register to fully populate. This is indicated by bit 7 (SPIF) of SPSR
     *  being set to 1.
     */
    while (!(SPSR & 1<<SPIF));
    /** Read data from the shift register when it is fully populated. */
    sensor[i] = SPDR;
  }

  /** Retrieve the thermocouple temperature. */
  if (sensor[0] & (1<<7)) {  /** temperature negative. */
    temp = 0;  /** Set it to 0. */
  } else {
    temp = (((uint16_t)sensor[0])<<6) + (sensor[1]>>2);
  }

  /** Retrieve room temperature reading. */
  if (sensor[2]&(1<<7)) {  /** temperature negative. */
    room_temp = 0;  /** Set it to 0. */
  } else {
    room_temp = (((uint16_t)sensor[2])<<4) + (sensor[3]>>4);
    room_temp = room_temp/4;  /** Each bit is 1/16 of a degree C and we are using quarters. */
  }

  if (sensor[1] & 0x01) {  /** Fault bit. */
    /** If a fault occurs, write it to the USB stream and pass the least significant 3 bits of the
     *  data stream to indicate why the fault occurred.
     */
    fprintf(&USBSerialStream, "Fault:%u\n", sensor[3] & 0b00000111);
  }

  PORTB = (1<<SS);  /** Deassert SS. */
  return temp;
}

uint16_t get_target(uint16_t temp, uint16_t* timer)
{
  switch (reflow_state) {
    case STOP:
      target = ROOM_TEMP;
      *timer = 0;
      integral = 0;
      break;
    case PREHEAT:
      if (temp >= profile.soak_temp1) {
        reflow_state = SOAK;
        *timer = 0;
      } else {
        target = target + *timer;
      }
      break;
    case SOAK:
      if (*timer >= profile.soak_time) {
        reflow_state = REFLOW_RAMP;
        *timer = 0;
      } else {
        target = (*timer/profile.soak_time) * (profile.soak_temp2-profile.soak_temp1);
      }
      break;
    case REFLOW_RAMP:
      if (*timer >= profile.time_to_liquides) {
        reflow_state = REFLOW_TAL;
        *timer = 0;
      } else {
        target = profile.soak_temp2 + ((*timer/profile.time_to_liquides)*(profile.liquides_temp-profile.soak_temp2));
      }
      break;
    case REFLOW_TAL:
      if (*timer >= profile.time_to_peak) {
        reflow_state = COOL;
        *timer = 0;
      } else {
        target = profile.liquides_temp + ((*timer/profile.time_to_peak)*(profile.peak_temp-profile.liquides_temp));
      }
      break;
    case COOL:
      if (temp <= ROOM_TEMP) {
        reflow_state = STOP;
        *timer = 0;
      } else {
        target = target - profile.cool_rate;
      }
      break;
  }
  return target;
}

/** Set the value of OCR1A using a PID controller. The value of OCR1A in relation to the top value,
 *  6250, will determine the duty cycle of the PWM waveform.
 */
uint16_t pid(uint16_t temp, uint16_t target)
{
  int32_t error = (int32_t)target - (int32_t)temp;
  integral = integral + error;
  int32_t p_term = profile.pid_p * error;
  int32_t i_term = profile.pid_i * integral;
  int32_t d_term = profile.pid_d * (error - prev_error);
  prev_error = error;
  int32_t result = p_term + i_term + d_term;
  /** Ensure that result is between 0 and 6250. */
  return (uint16_t)(result<0 ? 0 : (result>TOP ? TOP : result));
}

/** Receive messages from host. */
void usb_rx(void)
{
  char rx_char;
  int rx_byte;
  /** Start all messages from host computer with `!`. */
  if ((rx_char = (char)fgetc(&USBSerialStream)) != '!')
    return;
  /** Get the command. */
  if ((rx_byte = fgetc(&USBSerialStream)) == EOF)
    return;
  rx_char = (char)rx_byte;
  switch (rx_char) {
    case 'W':
      write_profile();
      break;
    case 'O':
      print_profile();
      break;
    case 'S':
      reflow_state = PREHEAT;
      break;
    case 'H':
      reflow_state = STOP;
      break;
    default:
      break;
  }
}

/** Print the current profile to the host computer. */
void print_profile(void)
{
  fprintf(&USBSerialStream, "preheat_rate: %" PRIu16 "\n"
                            "soak_temp1: %" PRIu16 "\n"
                            "soak_temp2: %" PRIu16 "\n"
                            "soak_time: %" PRIu16 "\n"
                            "liquides_temp: %" PRIu16 "\n"
                            "time_to_liquides: %" PRIu16 "\n"
                            "peak_temp: %" PRIu16 "\n"
                            "time_to_peak: %" PRIu16 "\n"
                            "cool_rate: %" PRIu16 "\n"
                            "pid_p: %" PRIu16 "\n"
                            "pid_i: %" PRIu16 "\n"
                            "pid_d: %" PRIu16 "\n",
          profile.preheat_rate, profile.soak_temp1, profile.soak_temp2, profile.soak_time,
          profile.liquides_temp, profile.time_to_liquides, profile.peak_temp, profile.time_to_peak,
          profile.cool_rate, profile.pid_p, profile.pid_i, profile.pid_d);
}

/** Read a new profile from the host computer and replace the current profile in EEPROM and SRAM. */
void write_profile(void)
{
  uint8_t new_profile[24];
  int rx;
  for (int8_t i=0; i<24; ++i) {
    if ((rx = fgetc(&USBSerialStream)) == EOF) {
      fprintf(&USBSerialStream, "An EOF occurred while reading data for a new profile.\n");
      return;
    }
    new_profile[i] = (uint8_t)rx;
  }
  /** Write data to SRAM. */
  profile.preheat_rate = (((uint16_t)new_profile[0])<<8) | (uint16_t)new_profile[1];
  profile.soak_temp1 = (((uint16_t)new_profile[2])<<8) | (uint16_t)new_profile[3];
  profile.soak_temp2 = (((uint16_t)new_profile[4])<<8) | (uint16_t)new_profile[5];
  profile.soak_time = (((uint16_t)new_profile[6])<<8) | (uint16_t)new_profile[7];
  profile.liquides_temp = (((uint16_t)new_profile[8])<<8) | (uint16_t)new_profile[9];
  profile.time_to_liquides = (((uint16_t)new_profile[10])<<8) | (uint16_t)new_profile[11];
  profile.peak_temp = (((uint16_t)new_profile[12])<<8) | (uint16_t)new_profile[13];
  profile.time_to_peak = (((uint16_t)new_profile[14])<<8) | (uint16_t)new_profile[15];
  profile.cool_rate = (((uint16_t)new_profile[16])<<8) | (uint16_t)new_profile[17];
  profile.pid_p = (((uint16_t)new_profile[18])<<8) | (uint16_t)new_profile[19];
  profile.pid_i = (((uint16_t)new_profile[20])<<8) | (uint16_t)new_profile[21];
  profile.pid_d = (((uint16_t)new_profile[22])<<8) | (uint16_t)new_profile[23];

  /** Write data to EEPROM. */
  eeprom_update_block(&profile, &eeprom_profile, sizeof(profile));
}

/** Event handler that is called when the USB is connected. */
void EVENT_USB_Device_Connect(void)
{
  usb_connected = 1;
}

/** Event handler that is called when the USB is disconnected. */
void EVENT_USB_Device_Disconnect(void)
{
  usb_connected = 0;
}

/** Register the interrupt. The LUFA macro ISR is needed to properly handle the interrupt. */
ISR(TIMER1_COMPB_vect)
{
  /** When TCNT1 reaches the top value, raise flag to call `update_pid()` to update OCR1A. */
  update_pid = 1;
}

/** Event handler for when the USB configuration changes. Not sure why we need this. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
  CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for if a control request was made. Not sure why we need this. */
void EVENT_USB_Device_ControlRequest(void)
{
  CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}
