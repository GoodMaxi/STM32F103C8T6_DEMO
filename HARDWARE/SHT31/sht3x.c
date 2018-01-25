#include <stdio.h>
#include "sht3x.h"
#include "bsp_i2c_gpio.h"
#include "delay.h"

uint8_t SHT3x_CMD_Send(uint16_t cmd)
{		
		i2c_Start();
		i2c_SendByte(ADDR_WRITE);
		if (i2c_WaitAck() != 0)
		{
				printf("ADDR WRITE NO ACK. \r\n");		
				return 1;
		}		
		i2c_SendByte(cmd >> 8);//(PERIODIC_CMD >> 8);
		if (i2c_WaitAck() != 0)
		{
				printf("CMD MSB NO ACK. \r\n");		
				return 2;		
		}
		i2c_SendByte((uint8_t)cmd);//(PERIODIC_CMD & 0xFF);
		if (i2c_WaitAck() != 0)
		{
				printf("CMD LSB NO ACK. \r\n");	
				return 3;					
		}
		return 0;		
}

uint8_t  SHT3x_Init(void)
{
		uint8_t i = 0;
		i2c_CfgGpio();
		while(SHT3x_CMD_Send(CMD_PERI_MODE))
		{
				i++;
				if(i > 5)
				{
						printf("periodic set fail!\r\n");
						return 1;						
				}	
				delay_ms(100);
		}
		return 0;
}

uint8_t SHT3x_Read_Statu(uint16_t *statu)
{
		uint8_t sta[2], CRC_t, i=0;
	
		while(SHT3x_CMD_Send(CMD_STA_READ))
		{
				i++;
				if(i > 5)
				{
						printf("status read cmd fail!\r\n");
						return 1;						
				}	
				delay_ms(100);
		}	
		
		i2c_Start();
		i2c_SendByte(ADDR_READ);	
		while(i2c_WaitAck())
		{
				delay_ms(100);			
				i2c_Start();
				i2c_SendByte(ADDR_READ);				
				i++;
				if(i > 5)
				{
						printf("statu read addr err!\r\n");
						return 2;						
				}	
		}		
		sta[0] = i2c_ReadByte();
		i2c_Ack();
		sta[1] = i2c_ReadByte();
		i2c_Ack();	
		CRC_t = i2c_ReadByte();
		i2c_NAck();
		i2c_Stop();
		if(CRC_t != cal_crc_SHT3x(sta, 2))
		{
				printf("statu read crc err! \r\n");				
				return 3;		
		}
		statu[0] = (sta[0]<<8) + sta[1];
		return 0;		
}

uint8_t Periodic_Read(float *Tem_f, float *Hum_f)
{
		uint16_t Tem, Hum;
		uint8_t  CRC_t, CRC_h, i=0, t_val[2], h_val[2];

		while(SHT3x_CMD_Send(CMD_PERI_READ))
		{
				i++;
				if(i > 5)
				{
						printf("periodic read cmd fail!\r\n");
						return 1;						
				}	
				delay_ms(100);
		}		

		i2c_Start();
		i2c_SendByte(ADDR_READ);	
		while(i2c_WaitAck())
		{
				delay_ms(100);			
				i2c_Start();
				i2c_SendByte(ADDR_READ);				
				i++;
				if(i > 5)
				{
						printf("peri read addr err!\r\n");
						return 2;						
				}	
		}		

		t_val[0] = i2c_ReadByte();
		i2c_Ack();
		t_val[1] = i2c_ReadByte();
		i2c_Ack();	
		CRC_t = i2c_ReadByte();
		i2c_Ack();
		h_val[0] = i2c_ReadByte();
		i2c_Ack();
		h_val[1] = i2c_ReadByte();
		i2c_Ack();	
		CRC_h = i2c_ReadByte();
		i2c_NAck();
		i2c_Stop();
		if(CRC_t != cal_crc_SHT3x(t_val, 2))
		{
				printf("t_val read crc err! \r\n");				
				return 3;		
		}		
		if(CRC_h != cal_crc_SHT3x(h_val, 2))
		{
				printf("h_val read crc err! \r\n");				
				return 4;		
		}				
		
		Tem = (t_val[0]<<8) + t_val[1];
		Hum = (h_val[0]<<8) + h_val[1];		
		Tem_f[0] = (float)(175*Tem)/(float)65535 - 45;
		Hum_f[0] = (float)(100*Hum)/(float)65535;		

		return 0;
}




uint8_t cal_crc_SHT3x(uint8_t *data, uint8_t len)
{
	uint8_t  crc = 0xff, bit, byteCtr;

	for (byteCtr = 0; byteCtr < len; byteCtr++)
	{
			crc ^= data[byteCtr];
			for (bit = 8; bit>0; --bit)
			{
				if (crc & 0x80) crc = (crc << 1) ^ POLYNOMIAL;
				else crc = (crc << 1);
			}

	}

	return crc;
}

