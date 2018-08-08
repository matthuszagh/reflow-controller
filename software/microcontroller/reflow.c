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
static int8_t PID_debug = 0;

/** Standard file stream for the CDC interface when set up, so that the virtual CDC COM port can be
 *  used like any regular character stream in the C APIs.
 */
static FILE USBSerialStream;

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
{
  .Config =
  {
    .ControlInterfaceNumber   = 0,
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
  },
};

void set_profile(void) {
  /* Read profile from EEPROM to RAM */
  eeprom_read_block(&profile, &eeprom_profile, sizeof(profile));
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

uint16_t get_target(uint16_t temp, uint16_t* timer)
{
  switch (reflow_state) {
    case STOP:
      target = ROOM_TEMP;
      *timer = 0;
      integral = 0;
      break;
    case PREHEAT:
      if (*timer >= (profile.soak_temp1 - ROOM_TEMP)/profile.preheat_rate) {
        reflow_state = SOAK;
        *timer = 0;
      } else {
        target = ROOM_TEMP + ((*timer)*profile.preheat_rate);
      }
      break;
    case SOAK:
      if (*timer >= profile.soak_time) {
        reflow_state = REFLOW_RAMP;
        *timer = 0;
      } else {
        target = profile.soak_temp1 + ((profile.soak_temp2-profile.soak_temp1)*(*timer)/profile.soak_time);
      }
      break;
    case REFLOW_RAMP:
      if (*timer >= profile.time_to_liquides) {
        reflow_state = REFLOW_TAL;
        *timer = 0;
      } else {
        target = profile.soak_temp2 + ((profile.liquides_temp-profile.soak_temp2)*(*timer)/profile.time_to_liquides);
      }
      break;
    case REFLOW_TAL:
      if (*timer >= profile.time_to_peak) {
        reflow_state = COOL;
        *timer = 0;
      } else {
        target = profile.liquides_temp + ((profile.peak_temp-profile.liquides_temp)*(*timer)/profile.time_to_peak);
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

void usb_rx(void) {
  /*  Handle messages from host */
  char ReceivedChar;
  int ReceivedByte;
  /* Start commands with '!' */
  if ( (ReceivedChar = fgetc(&USBSerialStream)) != '!') {
    return;
  }
  /* Get the real command */
  while((ReceivedByte = fgetc(&USBSerialStream)) == EOF);
  ReceivedChar = (char)ReceivedByte;
  /* PID debugging, prints PID term values */
  if (ReceivedChar == 'D') {
    PID_debug = 1;
  }
  if (ReceivedChar == 'd') {
    PID_debug = 0;
  }
  /* Write temperature profile and PID settings */
  if (ReceivedChar == 'W') {
    write_profile();
  }
  /* Start reflow */
  if (ReceivedChar == 'S') {
    reflow_state = PREHEAT;
  }
  /* Stop reflow */
  if (ReceivedChar == 'H') {
    reflow_state = STOP;
  }
  /* Output current profile */
  if (ReceivedChar == 'O') {
    print_profile();
  }
  return;
}

uint16_t read_sensor(void) {
  /* Bits:
   * 31 : sign,
   * 30 - 18 : thermocouple temperature,
   * 17 : reserved(0),
   * 16 : 1 if fault,
   * 15 - 4 : cold junction temperature,
   * 3 : reserved(0),
   * 2 : 1 if thermocouple is shorted to Vcc,
   * 1 : 1 if thermocouple is shorted to ground,
   * 0 : 1 if thermocouple is open circuit */

  /* Enable slave */
  uint8_t sensor[4];
  uint16_t temp;
  int8_t i;
  /* SS = 0 */
  PORTB = (0<<SS);

  /* Wait for the device */
  _NOP();
  _NOP();
  /* Transmit nothing */
  for(i=0;i<4;i++) {
    SPDR = 0x00;
    /* Wait for transmission to complete */
    while (!(SPSR & _BV(SPIF)));
    sensor[i] = SPDR;
  }

  /* Thermocouple temperature */
  if (sensor[0]&(1<<7)) {
    /* Negative temperature, clamp it to zero */
    temp = 0;
  } else {
    temp = (((uint16_t)sensor[0])<<6)+(sensor[1]>>2);
  }

  /* Room temperature */
  if (sensor[2]&(1<<7)) {
    /* Negative temperature, clamp it to zero */
    room_temp = 0;
  } else {
    room_temp = (((uint16_t)sensor[2])<<4)+(sensor[3]>>4);
    /* Sensor gives room temp as sixteenths of celsius,
     * divide it by four to get quarters of celsius. */
    room_temp = room_temp / 4;
  }

  if (sensor[1]&0x01) {
    /* Fault */
    fprintf(&USBSerialStream,"Fault:%u\n",sensor[3]&0b00000111);
  }

  /* Disable slave */
  PORTB = (1<<SS);
  return temp;
}

void setupHardware(void) {

  /* Disable wtachdog */
  MCUSR &= ~(1 << WDRF);
  wdt_disable();
  /* Disable prescaler */
  clock_prescale_set(clock_div_1);

  /* Set !SS and SCK output, all others input */
  DDRB = (1<<SS)|(1<<SCK);
  //bit_set(DDRB,SS);
  //bit_set(DDRB,SCK);
  bit_set(PORTB, SS);/* Set !SS high (slave not enabled) */
  bit_clear(PORTB, SCK);

  /* Set timer1 to count 1 second */
  TCNT1 = 0x00;
  /* PWM output to channel A, pin PC6 */
  TCCR1A = 0b10000010;
  /*  Set prescaler to divide by 256 for TMR1 */
  TCCR1B = 0b11011100;
  TIMSK1 = (1<<2);
  ICR1 = _ICR1;
  OCR1B = 1;
  OCR1A = _ICR1;

  /* PC6 = Relay */
  DDRC   = 0b01000000;
  PORTC  = 0x00;
  DDRD   = 0x00;

  /* Enable SPI, Master, set clock rate fck/2 */
  SPCR = (1<<SPE) | (1<<MSTR);
  SPSR = (1<<SPI2X);

  /* Initialize USB */
  USB_Init();
  CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

  return;
}

/* Get PWM frequency from target temperature */
uint16_t approx_pwm(uint16_t target)
{
  int32_t t;
  t = ((_ICR1*(target-room_temp)) / (MAXTEMP*4));
  return (uint16_t)CLAMP(t,0,_ICR1);
}


uint16_t pid(uint16_t target, uint16_t temp) {
  int32_t error = (int32_t)target - (int32_t)temp;
  if (target == 0) {
    integral = 0;
    prev_error = error;
    return 0;
  } else {

    int32_t p_term = profile.pid_p * error;
    int32_t i_term = integral * profile.pid_i;
    int32_t d_term = (prev_error - error) * profile.pid_d;

    int16_t new_integral = integral + error;
    /* Clamp integral to a reasonable value */
    new_integral = CLAMP(new_integral,-4*100,4*100);

    prev_error = error;

    int32_t result = approx_pwm(target) + p_term + i_term + d_term;

    /* Avoid integral buildup */
    if ((result >= _ICR1 && new_integral < integral) || (result < 0 && new_integral > integral) || (result <= _ICR1 && result >= 0)) {
      integral = new_integral;
    }

    /* Clamp the output value */
    return (uint16_t)(CLAMP(result,0,_ICR1));
  }
}

int main(void) {

  uint8_t target_update = 0;
  bool tx_flag = 0;
  uint16_t temp = 0;
  uint16_t target = 0;
  uint16_t timer = 0;/* Timer for various reflow stages */

  setupHardware();
  set_profile();
  GlobalInterruptEnable();

  reflow_state = PREHEAT;
  temp = read_sensor();
  target = get_target(temp, &timer);
  timer++;

  while(1)
  {
    if (usb_connected) {
      /*  Check mail */
      usb_rx();
      CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
      USB_USBTask();
    }

    if (usb_connected && tx_flag) {
      tx_flag = 0;
      /* Send temp temperature */
      fprintf(&USBSerialStream, "temp:%u,room:%u,target:%u,PWM:%u,state:%d", temp, room_temp, target, OCR1A, reflow_state);
      if (PID_debug)
        fprintf(&USBSerialStream, ",I:%d", integral);
      fprintf(&USBSerialStream, "\n");
    }
    if (update_pid) {
      /* Update target once per second */
      if (target_update++ == 5) {
        target_update = 0;
        target = get_target(temp, &timer);
        timer++;
        tx_flag = 1;
      }
      /* Read the current temperature, updates temp and room_temp */
      temp = read_sensor();
      update_pid = 0;
      OCR1A = pid(target, temp);
    }
  }
}


ISR(TIMER1_COMPB_vect) {
  /* Set PWM */
  update_pid = 1;
}


/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
  usb_connected = 1;
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
  usb_connected = 0;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
  CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
  CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}
