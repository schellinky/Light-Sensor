#include "main.h"
#include <stdio.h>
#include "tsl25911.h"

#define LIGHT_SENSOR_ADDRESS = 0x29
#define CHANNEL_0_HIGH = 0x0D
#define CHANNEL_1_HIGH = 0x0F 
//The below code is all declared in tsl25911.h
// #define TSL25911_ADDR        (0x29)
// #define TSL25911_ID          (0x50)

// #define TSL25911_LUX_DF      (408.0F)
// #define TSL25911_LUX_COEFB   (1.64F)  // CH0 coefficient 
// #define TSL25911_LUX_COEFC   (0.59F)  // CH1 coefficient A
// #define TSL25911_LUX_COEFD   (0.86F)  // CH2 coefficient B

// enum {
//     TSL25911_REG_ENABLE          = 0x00,
//     TSL25911_REG_CONTROL         = 0x01,
//     TSL25911_REG_THRES_AILTL     = 0x04,
//     TSL25911_REG_THRES_AILTH     = 0x05,
//     TSL25911_REG_THRES_AIHTL     = 0x06,
//     TSL25911_REG_THRES_AIHTH     = 0x07,
//     TSL25911_REG_THRES_NPAILTL   = 0x08,
//     TSL25911_REG_THRES_NPAILTH   = 0x09,
//     TSL25911_REG_THRES_NPAIHTL   = 0x0A,
//     TSL25911_REG_THRES_NPAIHTH   = 0x0B,
//     TSL25911_REG_PERSIST         = 0x0C,
//     TSL25911_REG_PID             = 0x11,
//     TSL25911_REG_ID              = 0x12,
//     TSL25911_REG_STATUS          = 0x13,
//     TSL25911_REG_CHAN0_L         = 0x14,
//     TSL25911_REG_CHAN0_H         = 0x15,
//     TSL25911_REG_CHAN1_L         = 0x16,
//     TSL25911_REG_CHAN1_H         = 0x17,
// };

// typedef enum {
//     TSL25911_GAIN_LOW    = 0x00,
//     TSL25911_GAIN_MED    = 0x01,
//     TSL25911_GAIN_HIGH   = 0x02,
//     TSL25911_GAIN_MAX    = 0x03,
// } tsl25911Gain_t;

// typedef enum {
//     TSL25911_INTT_100MS  = 0x00,
//     TSL25911_INTT_200MS  = 0x01,
//     TSL25911_INTT_300MS  = 0x02,
//     TSL25911_INTT_400MS  = 0x03,
//     TSL25911_INTT_500MS  = 0x04,
//     TSL25911_INTT_600MS  = 0x05,
// } tsl25911IntegrationTime_t;

// typedef struct tsl25911_shadow {
//   I2C_HandleTypeDef           *i2c_port;
//   tsl25911Gain_t               gain;
//   tsl25911IntegrationTime_t    integration;
//   uint32_t rawALS;
//   uint32_t full;
//   uint32_t ir;
//   uint32_t visible;
//   float lux;
//   int saturated;
// } tsl25911_shadow_t;

int tsl25911_writereg(I2C_HandleTypeDef *hi2c1, uint8_t addr, uint8_t * data, uint16_t size) {
  HAL_StatusTypeDef status;
  status = HAL_I2C_Mem_Write(hi2c1,
                            (TSL25911_ADDR<<1),
                            0xA0|addr,
                            1,
                            data,
                            size,
                            HAL_MAX_DELAY
                            );
  return((int) status);
}

int tsl25911_readreg(I2C_HandleTypeDef *hi2c1, uint8_t addr, uint8_t * data, uint16_t size) {
  HAL_StatusTypeDef status;
  status = HAL_I2C_Mem_Read(hi2c1,
                   (TSL25911_ADDR<<1),
                   0xA0|addr,
                   1,
                   data,
                   size,
                   HAL_MAX_DELAY
                   );
  return((int) status);
}

int tsl25911_init(tsl25911_shadow_t *shadow,
                   I2C_HandleTypeDef *port,
                   tsl25911Gain_t gain,
                   tsl25911IntegrationTime_t integration) {
  if ((gain<0)||(gain>3)) return (-1);
  if ((integration<0)||(integration>5)) return (-1);
  
  shadow->i2c_port = port;
  shadow->gain = gain;
  shadow->integration = integration;
  if (tsl25911_readID(shadow) != 0x50) {
    return (-1);
  }
  tsl25911_setGain(shadow,gain); 
  tsl25911_setIntg(shadow,integration);
  tsl25911_disable(shadow);
  return (0);
}

uint8_t tsl25911_readID(tsl25911_shadow_t *shadow) {
  uint8_t rxbuf;
  tsl25911_readreg(shadow->i2c_port,TSL25911_REG_ID,&rxbuf,1);
  return rxbuf;
}

uint8_t tsl25911_readControl(tsl25911_shadow_t *shadow) {
  uint8_t rxbuf;
  tsl25911_readreg(shadow->i2c_port,TSL25911_REG_CONTROL,&rxbuf,1);
  return rxbuf;
}

int tsl25911_setGain(tsl25911_shadow_t *shadow, tsl25911Gain_t gain) {
  uint8_t rxbuf;
  if ((gain<0) || (gain>0x03)) {
    return (-1);
  }
  shadow->gain = gain;
  tsl25911_readreg(shadow->i2c_port,TSL25911_REG_CONTROL,&rxbuf,1);
  rxbuf &= ~0x30;
  rxbuf |= (gain<<4);
  tsl25911_writereg(shadow->i2c_port,TSL25911_REG_CONTROL,&rxbuf,1);
  return (0);
}

int tsl25911_setIntg(tsl25911_shadow_t *shadow, tsl25911IntegrationTime_t integration) {
  uint8_t rxbuf;
  if ((integration<0) || (integration>0x05)) {
    return (-1);
  }
  shadow->integration = integration;
  tsl25911_readreg(shadow->i2c_port,TSL25911_REG_CONTROL,&rxbuf,1);
  rxbuf &= ~0x07;
  rxbuf |= integration;
  tsl25911_writereg(shadow->i2c_port,TSL25911_REG_CONTROL,&rxbuf,1);
  return (0);
}

void tsl25911_enable(tsl25911_shadow_t *shadow) {
  uint8_t rxbuf;
  tsl25911_readreg(shadow->i2c_port,TSL25911_REG_ENABLE,&rxbuf,1);
  rxbuf |= TSL25911_EN_PON;
  rxbuf |= TSL25911_EN_AEN;
  tsl25911_writereg(shadow->i2c_port,TSL25911_REG_ENABLE,&rxbuf,1);
}

void tsl25911_disable(tsl25911_shadow_t *shadow) {
  uint8_t rxbuf;
  tsl25911_readreg(shadow->i2c_port,TSL25911_REG_ENABLE,&rxbuf,1);
  rxbuf &= ~TSL25911_EN_PON;
  rxbuf &= ~TSL25911_EN_AEN;
  tsl25911_writereg(shadow->i2c_port,TSL25911_REG_ENABLE,&rxbuf,1);
}

// ALS stands for ambient light sensor ...
void tsl25911_getALS(tsl25911_shadow_t *shadow) {
  uint8_t sensor_data[4];
  uint32_t start_time = uwTick; // uwTick increments every 10mS in systick
  uint32_t end_time;
  switch (shadow->integration) {
  case TSL25911_INTT_100MS:
    end_time = start_time + 10;
    break;
  case TSL25911_INTT_200MS:
    end_time = start_time + 20;
    break;
  case TSL25911_INTT_300MS:
    end_time = start_time + 30;
    break;
  case TSL25911_INTT_400MS:
    end_time = start_time + 40;
    break;
  case TSL25911_INTT_500MS:
    end_time = start_time + 50;
    break;
  case TSL25911_INTT_600MS:
    end_time = start_time + 60;
    break;
  default:
    end_time = start_time + 60;
  }
  tsl25911_enable(shadow);
  while (uwTick <= (end_time+5)); 
  tsl25911_readreg(shadow->i2c_port,TSL25911_REG_CHAN0_L,sensor_data,4);
  tsl25911_disable(shadow);
  shadow->rawALS = 0;
  shadow->rawALS = ((((sensor_data[3]<<8)|sensor_data[2])<<16)|((sensor_data[1]<<8)|sensor_data[0]));
  shadow->full = shadow->rawALS & 0xFFFF;
  shadow->ir = shadow->rawALS >> 16;
  shadow->visible = shadow->full - shadow->ir;
}

void tsl25911_calcLux(tsl25911_shadow_t *shadow) {
  float atime, again, cpl, lux1, lux2;
  shadow->saturated = 0;
  if((shadow->full == 0xFFFF)|(shadow->ir == 0xFFFF)) {
    shadow->saturated = 1;
    shadow->lux = 0;
    return;
  }
  switch(shadow->integration) {
  case TSL25911_INTT_100MS:
    atime = 100.0F;
    break;
  case TSL25911_INTT_200MS:
    atime = 200.0F;
    break;
  case TSL25911_INTT_300MS:
    atime = 300.0F;
    break;
  case TSL25911_INTT_400MS:
    atime = 400.0F;
    break;
  case TSL25911_INTT_500MS:
    atime = 500.0F;
    break;
  case TSL25911_INTT_600MS:
    atime = 600.0F;
    break;
  default:
    atime = 100.0F;
    break;
  }
  switch(shadow->gain) {
  case TSL25911_GAIN_LOW:
    again = 1.0F;
    break;
  case TSL25911_GAIN_MED:
    again = 25.0F;
    break;
  case TSL25911_GAIN_HIGH:
    again = 428.0F;
    break;
  case TSL25911_GAIN_MAX:
    again = 9876.0F;
    break;
  default:
    again = 1.0F;
    break;
  }
  cpl = (atime * again) / TSL25911_LUX_DF;
  lux1 = ((float)shadow->full - (TSL25911_LUX_COEFB * (float)shadow->ir)) / cpl;
  lux2 = (( TSL25911_LUX_COEFC * (float)shadow->full ) - ( TSL25911_LUX_COEFD * (float)shadow->ir)) / cpl;
  shadow->lux = lux1 > lux2 ? lux1 : lux2;
  shadow->saturated = 0;
}

float tsl25911_readsensor(I2C_HandleTypeDef *i2c_port) {
  tsl25911_shadow_t s;
  tsl25911_init(&s,i2c_port,TSL25911_GAIN_MAX,TSL25911_INTT_600MS);
  tsl25911_getALS(&s);
  tsl25911_calcLux(&s);
  while (s.saturated) {
    switch (s.gain) {
    case TSL25911_GAIN_MAX:
      tsl25911_init(&s,i2c_port,TSL25911_GAIN_HIGH,TSL25911_INTT_600MS);
      tsl25911_getALS(&s);
      tsl25911_calcLux(&s);
      break;
    case TSL25911_GAIN_HIGH:
      tsl25911_init(&s,i2c_port,TSL25911_GAIN_MED,TSL25911_INTT_600MS);
      tsl25911_getALS(&s);
      tsl25911_calcLux(&s);
      break;
    case TSL25911_GAIN_MED:
      tsl25911_init(&s,i2c_port,TSL25911_GAIN_LOW,TSL25911_INTT_600MS);
      tsl25911_getALS(&s);
      tsl25911_calcLux(&s);
      break;
    case TSL25911_GAIN_LOW:
      return (-1);
      break;
    }
  }
  return s.lux;
}

// uint16_t lightData[2];
// uint8_t tsl25911_readid(I2C_HandleTypeDef *hi2c1){
// 	uint8_t rxbuf;
// 	HAL_I2C_Mem_Read(hi2c1, (0x29<<1), 0xA0|0x12, 1, (uint8_t *) &rxbuf, 1, HAL_MAX_DELAY);
// 	return rxbuf;
// }
uint8_t tsl25911_power_on(I2C_HandleTypeDef *hi2c1){
	uint8_t rxbuf;
	HAL_I2C_Mem_Read(hi2c1, 0x29<<1, 0xA0|0x00, 1, (uint8_t *) &rxbuf, 1, HAL_MAX_DELAY);
	uint8_t turn_on= (0x01|0x02|0x10|0x80);
	HAL_I2C_Mem_Write(hi2c1, (0x29<<1), 0xA0|0x00, 1, (uint8_t *) &turn_on, 2, HAL_MAX_DELAY);	uint8_t status;
	// HAL_I2C_Mem_Read(hi2c1, 0x29<<1, 0xA0|0x13, 1, (uint8_t *) &status, 1, HAL_MAX_DELAY);
	// status = (status & 0x1);
	// printf("Status: %d\n\r", status);
	// setGain(&hi2c1);
	return rxbuf;
}
	
// uint16_t* tsl25911_readLightData(I2C_HandleTypeDef *hi2c1){
// 	uint8_t data0[2];//data low and data high
// 	uint8_t data1[2];
// 	uint16_t channel0;
// 	uint16_t channel1;
// 	uint8_t lightData2[4];
// 	uint8_t dataLow;
// 	uint8_t dataHigh;
// 	uint8_t dataLow1;
// 	uint8_t dataHigh1;
// 	HAL_I2C_Mem_Read(hi2c1, (0x29<<1), (0xA0|0x14), 1, (uint8_t *) data0, 2, HAL_MAX_DELAY);
// 	channel0 = 256*data0[1] + data0[0];//combine 8 bit fields to form 16 bit word
// 	HAL_I2C_Mem_Read(hi2c1, (0x29<<1), (0xA0|0x16), 1, (uint8_t *) data1, 2, HAL_MAX_DELAY);
// 	channel1 = 256*data1[1] + data1[0];
// 	lightData[0] = channel0;
// 	lightData[1] = channel1;
// 	return lightData;
// }
// void setGain(I2C_HandleTypeDef *hi2c1){
// 	uint8_t gain = 0x03;//max gain (11)
// 	uint8_t int_t = 0x05;//max integration time (101)
// 	uint8_t gain_intt = gain|int_t;
// 	HAL_I2C_Mem_Read(hi2c1, 0x29<<1, 0xA0|0x01, 1, (uint8_t *) &gain_intt, 1, HAL_MAX_DELAY);
// }
// uint16_t calculateLux(uint32_t iGain, uint32_t tInt, uint32_t ch0, uint32_t ch1, uint8_t iType) {
// uint32_t chScale;
// uint32_t channel1;
// uint32_t channel0;
// switch (tInt) {
// 	case 0:    // 13.7 msec
// 	chScale = CHSCALE_TINT0;
// 	break;
// 	case 1:    // 101 msec
// 	chScale = CHSCALE_TINT1;
// 	break;
// 	default:   // assume no scaling
// 	chScale = (1 << CH_SCALE);
// 	break;
// 	}// scale if gain is NOT 16X
// 	if (!iGain) chScale = chScale << 4;   // scale 1X to 16X// scale the channel values
// 	channel0 = (ch0 * chScale) >> CH_SCALE;
// 	channel1 = (ch1 * chScale) >> CH_SCALE;//−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−−// find the ratio of the channel values (Channel1/Channel0)// protect against divide by zero
// 	uint32_t ratio1 = 0;
// 	if (channel0 != 0) ratio1 = (channel1 << (RATIO_SCALE+1)) / channel0;// round the ratio value
// 	uint32_t ratio = (ratio1 + 1) >> 1;// is ratio <= eachBreak ?
// 	uint32_t b, m;
// 	switch (iType) {
// 		case 0: // T, FN and CL package
// 		if ((ratio >= 0) && (ratio <= K1T)) {
// 			b=B1T;
// 			m=M1T;
// 			}
// 		else if (ratio <= K2T) {
// 			b=B2T;
// 			m=M2T;
// 			}
// 		else if (ratio <= K3T) {
// 			b=B3T;
// 			m=M3T;
// 			}
// 		else if (ratio <= K4T) {
// 			b=B4T;
// 			m=M4T;
// 			}
// 		else if (ratio <= K5T) {
// 			b=B5T;
// 			m=M5T;
// 			}
// 		else if (ratio <= K6T) {
// 			b=B6T;
// 			m=M6T;
// 			}
// 		else if (ratio <= K7T) {
// 			b=B7T;
// 			m=M7T;
// 			}
// 		else if (ratio > K8T) {
// 			b=B8T;
// 			m=M8T;
// 			}
// 			break;
// 		case 1:// CS package
// 		if ((ratio >= 0) && (ratio <= K1C)) {
// 			b=B1C;
// 			m=M1C;
// 			}
// 		else if (ratio <= K2C) {
// 			b=B2C;
// 			m=M2C;
// 			}
// 		else if (ratio <= K3C) {
// 			b=B3C;
// 			m=M3C;
// 			}
// 		else if (ratio <= K4C) {
// 			b=B4C;
// 			m=M4C;
// 			}
// 		else if (ratio <= K5C) {
// 			b=B5C;
// 			m=M5C;
// 			}
// 		else if (ratio <= K6C) {
// 			b=B6C;
// 			m=M6C;
// 			}
// 		else if (ratio <= K7C) {
// 			b=B7C;
// 			m=M7C;
// 			}
// 		else if (ratio > K8C) {
// 			b=B8C;
// 			m=M8C;
// 			}
// 			break;
// 		}
// 		uint32_t temp;
// 		temp = ((channel0 * b) - (channel1 * m));// do not allow negative lux value
// 		if (temp < 0) temp = 0;// round lsb (2^(LUX_SCALE−1))
// 		temp += (1 << (LUX_SCALE-1));// strip off fractional portion
// 		uint32_t lux = temp >> LUX_SCALE;
// 		return(lux);
// }


	
	
