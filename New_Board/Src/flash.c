//Nathan Schellink
//flash.c
//c file for reading and writing to flash
//Credit to: Bryce Himebaugh
#include <math.h>
#include "flash.h"
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "sys/types.h"
extern int _edata;
extern int _sdata;
extern int __fini_array_end;
uint64_t* find_sentinel(){   
  uint64_t * mem = (uint64_t *)&__fini_array_end + ((uint64_t *)&_edata - (uint64_t *)&_sdata); // Dynamically finds the next available address after code and data in flash 
  mem+=2;   // Increment pointer by 16. 
  mem= (uint64_t *) ((uintptr_t) mem & ~(uintptr_t)0xF); // Align pointer on a 16 byte boundary
  while (mem<=((uint64_t*)FLASH_END)) {
    if (*mem==SENTINEL_MARK) {
      return(mem);                 // Return the address of the sentinel
    }
    mem++;
  }
  return (0);                    // If the sentinel was note located, return a null pointer
} 

int flash_write_init(flash_status_t * fs) {
  uint64_t *p = find_sentinel();
  sensor_data_t *sd = 0;
  uint16_t record_counter = 0;
  raw_t sentinel = {SENTINEL_MARK,0};
  if (p) {
    // Previously Inialized Data Storage, Sentinel located. Point at first open data slot. 
    p+=2;  // Point at first data location.
    fs->data_start = p;
    fs->max_possible_records = ((uint32_t)0x0803FFF0 - (uint32_t) p) >> 4;
    sd = (sensor_data_t *) p;
    while (sd->watermark!=0xFF) {
      record_counter++;
      sd++;
    }
    fs->next_address = (uint64_t *) sd;
    fs->next_record_number = record_counter;
    fs->total_records = record_counter;
    return(0);
  }
  else {
    // The data storage area is uninitialized. Write the sentinel to the first data slot. 
    p = (uint64_t *)&__fini_array_end + ((uint64_t *)&_edata - (uint64_t *)&_sdata); // Compute last address of the flashed program. 
    p+=2;   // Increment pointer by 16. 
    p = (uint64_t *) ((uintptr_t) p & ~(uintptr_t)0xF); // Align pointer on a 16 byte boundary
    if (p) {
      // write the sentinel mark to denote the start of the data area 
      fs->next_address = p;
      if (write_record(fs,(void *) &sentinel)) {
        return (-1);
      }
      fs->next_record_number = 0; // Set the first record number to 0, the sentinel does not count. 
      p+=2;
      fs->data_start = p;  // The data will start at one location beyond the sentinel.
      fs->max_possible_records = ((uint32_t)0x0803FFF0 - (uint32_t) p) >> 4;
      fs->total_records = 0;
      return (0);
    }
    else {
      // Flash data area needs to be erased. Cannot write any data
      return (-1);
    }
  }
}
int write_record(flash_status_t * fs, void * record) {
  raw_t * write_data;
  HAL_StatusTypeDef status = 0;

  write_data = (raw_t *) record;
  
  HAL_FLASH_Unlock();
  if ((status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,(int) fs->next_address++, write_data->data0))) {
    HAL_FLASH_Lock();
    return (-1);
  }
  if ((status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,(int) fs->next_address++, write_data->data1))) {
    HAL_FLASH_Lock();
    return (-1);
  }
  HAL_FLASH_Lock();
  fs->next_record_number++;
  fs->total_records++;
  return (0);
}

int read_all_records(flash_status_t * fs) {
  sensor_data_t * p = (sensor_data_t *) fs->data_start;
  if (p->watermark == 0xFF) {
    printf("Data Storage is empty\n\r");
    return (0);
  }
  else {
    while (p->watermark!=0xFF) {
      printf("Found record number %d\n\r",p->record_number);
      p++;
    }
    printf("End of records\n\r");
  }
  return(0);
}
int read_all_sensor_data(flash_status_t * fs) {
  sensor_data_t * p = (sensor_data_t *) fs->data_start;
  if (p->watermark == 0xFF) {
    printf("Data Storage is empty\n\r");
    return (0);
  }
  else {
    while (p->watermark!=0xFF) {
      if (p->type==01){
        printf("%d\t%f\n\r", p->timestamp, (float) p->lux);
      }
      p++;
    }
    printf("End of records\n\r");
  }
  return(0);
}
int write_sensor_data(flash_status_t *fs,
                      uint16_t battery_voltage,
                      uint16_t temperature,
                      float lux, uint32_t time) {
  sensor_data_t p = {0x01,
                    0x01,
                    fs->next_record_number,
                    battery_voltage,
                    temperature,
                    time,
                    lux};
  write_record(fs,&p);
  return(0);
}

int report_flash_status(flash_status_t *fs) {
  printf("Starting Location of Flash Data = %p\n\r",fs->data_start);
  printf("Total Number of Records in the Flash Now = %d\n\r",(int) fs->total_records);
  printf("Maximum possible records = %d\n\r",(int) fs->max_possible_records);
  printf("Next Address to Write = %p\n\r",fs->next_address);
  printf("Next Record Number = %d\n\r",(int) fs->next_record_number);
  return(0);
}
void flash_erase() {
    FLASH_EraseInitTypeDef pEraseInit;
    uint32_t pageError;
    pEraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
    uint64_t begin = find_sentinel();
    printf("Begin: %d\n\r", &begin);
    if (begin != NULL) { // because find_sentinel() can return a null pointer
    int page = ceil((begin - FLASH_START)/2000);
    printf("Page: %d\n\r", page);
    pEraseInit.Page = page;
    pEraseInit.NbPages = 127-page;
    HAL_FLASH_Unlock();
    HAL_FLASHEx_Erase(&pEraseInit, &pageError);
    if (pageError == 0xFFFFFFFF){
        printf("Successfully Erased\n\r");
    }
    else{
        printf("\n\r Erase failed\n\r");
    }
    HAL_FLASH_Lock();
    }
}