#ifndef _SHT3X_H_
#define	_SHT3X_H_

#include <stdint.h>

#define  	ADDR_READ				0x89
#define  	ADDR_WRITE			0x88
#define		CMD_PERI_READ		0xE000		
#define		CMD_PERI_MODE		0x2130		
#define		CMD_SHOT_MODE		0x2400
#define		CMD_ART					0x2B32
#define		CMD_RESET				0x30A2
#define		CMD_STA_READ		0xF32D
#define		CMD_STA_CLR			0x3041
#define		CMD_HEAT_EN			0x306D
#define		CMD_HEAT_DIS		0x3066
#define		CMD_NRST				0x0006


#define		POLYNOMIAL			0X31


//uint8_t SHT3x_CMD_Send(uint16_t cmd);
uint8_t cal_crc_SHT3x(uint8_t *data, uint8_t len);
uint8_t  SHT3x_Init(void);
uint8_t SHT3x_Read_Statu(uint16_t *statu);
uint8_t Periodic_Read(float *Tem_f, float *Hum_f);
#endif /* __I2C_EE_H */
