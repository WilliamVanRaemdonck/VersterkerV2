#include "stdio.h"
#include "i2c-lcd.h"

extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart1;

#define SLAVE_ADDRESS_LCD 0b01000000 // change this according to ur setup 01000000


void lcd_send_cmd (char cmd)
{
	unsigned char data_u, data_l;
	uint8_t data_t[4];

	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);

	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0

	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, HAL_MAX_DELAY);
}

void lcd_send_data (char data)
{
	if(data == 'Y')
		data = 0xff;

	unsigned char data_u, data_l;
	uint8_t data_t[4];

	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);

	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1

	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x00);
	for (int i=0; i<100; i++)
	{
		lcd_send_data (' ');
	}
}

void lcd_init (void)
{
	// 4 bit initialisation
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (swapNibble(0b00110000));
	HAL_Delay(10);  // wait for >4.1ms
	lcd_send_cmd (swapNibble(0b00110000));
	HAL_Delay(10);  // wait for >100us
	lcd_send_cmd (swapNibble(0b00110000));
	HAL_Delay(10);
	lcd_send_cmd (swapNibble(0b00100000));  // 4bit mode
	HAL_Delay(10);

	lcd_send_cmd (swapNibble(0b00101000));  // Config
	HAL_Delay(10);
	lcd_send_cmd (swapNibble(0b00001100));  // Config
	HAL_Delay(10);
	lcd_send_cmd (swapNibble(0b00001111));  // Config
	HAL_Delay(10);
	lcd_send_cmd (swapNibble(0b00000110));  // Config
	HAL_Delay(10);
}

void lcd_send_string(char *str) {
    while (*str) {
        lcd_send_data(*str++);
    }
}

char swapNibble(char x) {
    return ((x & 0x0F) << 4 | (x & 0xF0) >> 4);
}

void print_bin(unsigned char value)
{
	for (int i = sizeof(char) * 7; i >= 0; i--)
		printf("%d", (value & (1 << i)) >> i );
	printf("\n");
}

