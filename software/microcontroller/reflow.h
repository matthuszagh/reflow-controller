#ifndef _REFLOW_H
#define _REFLOW_H

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <util/delay.h>

#include "Descriptors.h"
#include <LUFA/Drivers/Peripheral/Serial.h>
#include <LUFA/Drivers/USB/USB.h>

#define _NOP() do { __asm__ __volatile__ ("nop"); } while (0)
#define bit_set(p,m) ((p) |= (1<<m))
#define bit_clear(p,m) ((p) &= ~(1<<m))
#define _ICR1 6250 /* Counts 0.2 seconds */
/* Clamp x between l and h */
#define CLAMP(x, l, h) (((x) > (h)) ? (h) : (((x) < (l)) ? (l) : (x)))
#define SIGN(x) ((x > 0) ? 1 : ((x < 0) ? -1 : 0))
#define MAXTEMP 340
//#define MIN(x,y) (((x)<(y) ? (x) : (y)))
void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);

typedef struct {
  uint16_t preheat_rate;
  uint16_t soak_temp1;
  uint16_t soak_temp2;
  uint16_t soak_time;
  uint16_t liquides_temp;
  uint16_t time_to_liquides;
  uint16_t peak_temp;
  uint16_t time_to_peak;
  uint16_t cool_rate;
  uint16_t pid_p;
  uint16_t pid_i;
  uint16_t pid_d;
} profile_t;

typedef enum {
  STOP = 0,
  PREHEAT,
  SOAK,
  REFLOW_RAMP,
  REFLOW_TAL,
  COOL
} state;

uint16_t get_target(uint16_t temp, uint16_t *timer);
uint16_t pid(uint16_t, uint16_t);
void set_profile(void);
void write_profile(void);
void output_profile(void);

#endif
