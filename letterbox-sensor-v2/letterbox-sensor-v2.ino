// 
// letterbox-sensor-v2.ino
//

#include <Arduino.h>
#include "config.h"
#include "pins.h"
#include "lorawan.h"

#ifndef TIMERDRIFT_SECONDS_DAY
#define PERIOD_MS (PERIOD * 60 * 1000)
#else
#define PERIOD_MS (PERIOD * 60 * 1000 - (TIMERDRIFT_SECONDS_DAY * 1000) / 24)
#endif

void setup() {

  // set up serial line
  Serial.begin(115200);
  delay(1000);

  // set pinmodes
  pinMode(statusled, OUTPUT);
  pinMode(irled1, OUTPUT);
  pinMode(irled2, OUTPUT);
  pinMode(irdiode1, OUTPUT);
  pinMode(irdiode2, OUTPUT);
  pinMode(irsens1, INPUT);
  pinMode(irsens2, INPUT);

  // init lorawan
  lorawan_init();

  // set low power mode
  if (!api.system.lpm.set(1)) {	Serial.println("Set low power mode failed!\r\n");}

  // ADC mode
  //udrv_adc_set_mode(UDRV_ADC_MODE_3_3);

#ifdef EXTRAINFO
  // set resolution (uses 10 bit for Sensor and Battery)
  analogReadResolution(10);
#endif

  // blink led once on startup
  digitalWrite(statusled,HIGH);
  delay(50);
  digitalWrite(statusled,LOW);

  Serial.println("-------------------------");
  Serial.println("Letterbox sensor v2");
  Serial.printf("Firmware Version: %s\r\n", api.system.firmwareVersion.get().c_str());
  Serial.printf("CLI Version: %s\r\n", api.system.cliVersion.get().c_str());
  Serial.printf("API Version: %s\r\n", api.system.apiVersion.get().c_str());
  Serial.printf("Model ID: %s\r\n", api.system.modelId.get().c_str());
  Serial.printf("Hardware ID: %s\r\n", api.system.chipId.get().c_str());
  Serial.println("-------------------------");

	// create timer
  Serial.printf("Create periodic timer with period(ms): %d\r\n", PERIOD_MS);
  api.system.timer.create(RAK_TIMER_0, sensor_handler, RAK_TIMER_PERIODIC);
  // start timer
	api.system.timer.start(RAK_TIMER_0, PERIOD_MS, NULL);

  // get and send sensor values on boot
	sensor_handler(NULL);

#ifdef SERIALQUIET
  Serial.println("Compiled with SERIALQUIET to save battery");
#endif
  Serial.println("-------------------------");
}

// read sensor values and send lorawan package
void sensor_handler(void *) {

  // variables
  uint16_t s1 = 0; // sensor#1
  uint16_t s2 = 0; // sensor#2
  uint16_t bat = 0; // battery
  uint16_t voltage = 0; // voltage of battery

  static uint8_t status = 0; // status

  // data buffer
  uint8_t data_length = 8;
  uint8_t data[data_length];

#ifndef EXTRAINFO
  // set resolution
  analogReadResolution(10);
#endif

  // measure sensor #1
  digitalWrite(irled1,HIGH);
  digitalWrite(irdiode1,HIGH);
  delay(10);
  for(int i = 0 ; i <3 ; i++){
    delay(25);
    s1 += analogRead(irsens1);
  }
  digitalWrite(irled1,LOW);
  digitalWrite(irdiode1,LOW);
  s1=s1/3;
#ifndef SERIALQUIET
  Serial.printf("Sensor#1 raw AD 10 bit: %d\r\n", s1);
#endif

  // measure sensor #2
  digitalWrite(irled2,HIGH);
  digitalWrite(irdiode2,HIGH);
  delay(10);
  for(int i = 0 ; i <3 ; i++){
    delay(25);
    s2 += analogRead(irsens2);
  }
  digitalWrite(irled2,LOW);
  digitalWrite(irdiode2,LOW);
  s2=s2/3;
#ifndef SERIALQUIET
  Serial.printf("Sensor#2 raw AD 10 bit: %d\r\n", s2);
#endif

  // battery voltage from external voltage divider
#ifndef EXTRAINFO
  analogReadResolution(12);
#endif
  delay(25);
  bat=analogRead(vdiv);
#ifndef EXTRAINFO
  // voltage = 1750 + (bat - 1235) * 10; // 2x 3.3MOhm and 12 bit
  voltage = 1550 + (bat - 1040) * 8; // 2x 10MOhm and 12 bit
#else
  voltage = 1550 + (bat * 4 - 1040) * 8; // 2x 10MOhm and 10 bit
#endif

#ifndef SERIALQUIET
#ifndef EXTRAINFO
  Serial.printf("Voltage  raw AD 12 bit: %d\r\n", bat);
#else
  Serial.printf("Voltage  raw AD 10 bit: %d\r\n", bat);
#endif
  Serial.printf("Voltage (mV): %d\r\n", voltage);
#endif

// sensor value adjustment depending on battery voltage
#if defined s1_voltage_offset && defined s1_voltage_factor
  s1 = max(0, s1 - (voltage - s1_voltage_offset) * s1_voltage_factor / 1000);
#ifndef SERIALQUIET
  Serial.printf("Sensor#1 compensated: %d\r\n", s1);
#endif
#endif

#if defined s2_voltage_offset && defined s2_voltage_factor
  s2 = max(0, s2 - (voltage - s2_voltage_offset) * s2_voltage_factor / 1000);
#ifndef SERIALQUIET
  Serial.printf("Sensor#2 compensated: %d\r\n", s2);
#endif
#endif

#ifndef EXTRAINFO
  // letterbox full/empty
  if((s1 > THRESHOLD)||(s2 > THRESHOLD)) { data[0] = 0xFF; }
  else { data[0] = 0x00; }

  // battery
  data[1] = (voltage & 0xFF);
  data[2] = ((voltage >> 8) & 0xFF);

  // sensor#1
  data[3] = (s1 & 0xFF);
  data[4] = ((s1 >> 8) & 0xFF);

  // sensor#2
  data[5] = (s2 & 0xFF);
  data[6] = ((s2 >> 8) & 0xFF);

  // threshold
  data[7] = THRESHOLD;

  // temperature N/A for now
  //data[8] = readTemp();

#else
#define EXTRAINFO_VERSION 0x01 // 0:status+indicator+version 1:batt 2:PERIOD 3+4:sensor1+DATARATE 5+6:sensor2+TXPOWER 7:THRESHOLD
#define PRESET_DATARATE ((DATARATE & 0xF) << 4)
#define PRESET_TXPOWER  ((TXPOWER  & 0xF) << 4)
#define PRESET_DATA_0  ((EXTRAINFO_VERSION) | 0x10)
  // 7 6 5 4 3 2 1 0
  // s c 0 1 v v v v
  // s                status 1:full 0:empty
  //   c              status changed 1:yes 0:no
  //     0 1          flag for EXTRAINFO
  //         v v v v  version EXTRAINFO
  //                   1:  DATARATE in sensor #1 MSB
  //                       TXPOWER  in sensor #2 MSB
  //                       PERIOD   in data[2]
  //                       bat      only 8-bit
  data[0] = PRESET_DATA_0;
  if((s1 > THRESHOLD)||(s2 > THRESHOLD)) {
    if (status == 0) {
      status = 1;
      data[0] |= 0xC0; // status has changed
    } else {
      data[0] |= 0x80;
    }
  } else {
    if (status == 1) {
      status = 0;
      data[0] |= 0x40; // status has changed
    }
  }

  // recalc battery 1500V + 0…256*10mV and store in 8-bit
  // TODO let TTN do the work
  //bat = 1550 + (bat - 1040)*8; // 2x 10MOhm
  //data[1] = ((1550 + (bat - 1040)*8) - 1500) / 10;
  data[1] = (bat << 4) / 5 - 827;

  // configured PERIOD in minutes, max 255
  #if PERIOD > 255
  #define PRESET_PERIOD 0xFF
  #else
  #define PRESET_PERIOD PERIOD
  #endif
  data[2] = PRESET_PERIOD;

  // encode configured DATARATE (0-15) into sensor (10-bit) #1 MSB 15-12
  data[3] = (s1 & 0xFF);
  data[4] = ((s1 >> 8) & 0x0F) | PRESET_DATARATE;

  // encode configured TXPOWER (0-15) into sensor (10-bit) #2 MSB 15-12
  data[5] = (s2 & 0xFF);
  data[6] = ((s2 >> 8) & 0x0F) | PRESET_TXPOWER;

  // threshold
  data[7] = THRESHOLD;
#endif

  // send data via lorawan
  lorawan_send(data, data_length);

#ifndef SERIALQUIET
  Serial.println("-------------------------");
#endif
}


void loop(){
  // destroy this busy loop and use a timer instead,
  // so that the system thread can auto enter low power mode by api.system.lpm.set(1)
  api.system.scheduler.task.destroy();
}

