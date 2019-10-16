
/* flash.h
 * Description: 
 *Flash header file from Bryce Himebaugh's code with some minor changes
 * Author: Bryce Himebaugh
 * Contact: bhimebau@indiana.edu
 * Edited by: Nathan Schellink
 * Date: 04. 7.2019
 * Copyright (C) 2019
 *
 */
#include <stdint.h>
#ifndef FLASH_H
#define FLASH_H

// STM32L432 Flash Starts at 0x8000000 and ends at 0x803FFFFF. 
#define FLASH_START        0x08000000
#define FLASH_END          0x0803FFF8
#define SENTINEL_MARK      0xDEADBEEFDEADBEEF

typedef struct sensor_data {
  uint8_t watermark;               // 0x01=populated, 0xFF=unpopulated 
  uint8_t type;                  // record type, 01=sensor data, 02=error data;
  uint16_t record_number;          // Which number is this particular record  
  uint16_t battery_voltage;        // 16 bit battery voltage
  uint16_t temperature;            // STM32 Temperature sensor reading 
  uint32_t timestamp;              // Time, bit packed into 32 bits
  float lux;                       // Reading from the light sensor
} sensor_data_t;

typedef struct log_data {
  uint8_t watermark;               // 0x01=populated, 0xFF=unpopulated 
  uint8_t type;                  // record type, 01=sensor data, 02=error data;
  uint16_t record_number;          // Which number is this particular record  
  uint8_t msg[12];                 // 
} logdata_t;

typedef struct raw {
  uint64_t data0;
  uint64_t data1;
} raw_t;

typedef struct flash_status {
  uint64_t * data_start;
  uint32_t total_records;
  uint32_t max_possible_records;
  uint64_t * next_address;
  uint32_t next_record_number;
} flash_status_t;
 
#endif

uint64_t* find_sentinel(void);
int flash_write_init(flash_status_t *);
int write_record(flash_status_t *, void *);
int read_all_records(flash_status_t * );
int write_sensor_data(flash_status_t *,uint16_t,uint16_t,float, uint32_t);
int report_flash_status(flash_status_t *);