#ifndef REFLOW_H
#define REFLOW_H

// AVR Libc headers
#include <stdio.h>
#include <avr/power.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <inttypes.h>

// LUFA headers
#include "Descriptors.h"
#include <LUFA/Drivers/USB/USB.h>

static void setup_hardware(void);
static uint16_t read_temp(void);
static uint16_t get_target(uint16_t, uint16_t*);
static void usb_rx(void);
static void write_profile(void);
static void print_profile(void);
static uint16_t pid(uint16_t, uint16_t);

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

#endif
