#include <string.h>

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/task.h"



#define OLED_I2C_ADDRESS   0x3C

// Control byte
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40

// Fundamental commands (pg.28)
#define OLED_CMD_SET_CONTRAST           0x81    // follow with 0x7F
#define OLED_CMD_DISPLAY_RAM            0xA4
#define OLED_CMD_DISPLAY_ALLON          0xA5
#define OLED_CMD_DISPLAY_NORMAL         0xA6
#define OLED_CMD_DISPLAY_INVERTED       0xA7
#define OLED_CMD_DISPLAY_OFF            0xAE
#define OLED_CMD_DISPLAY_ON             0xAF

// Addressing Command Table (pg.30)
#define OLED_CMD_SET_MEMORY_ADDR_MODE   0x20    // follow with 0x00 = HORZ mode = Behave like a KS108 graphic LCD
#define OLED_CMD_SET_COLUMN_RANGE       0x21    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x7F = COL127
#define OLED_CMD_SET_PAGE_RANGE         0x22    // can be used only in HORZ/VERT mode - follow with 0x00 and 0x07 = PAGE7

// Hardware Config (pg.31)
#define OLED_CMD_SET_DISPLAY_START_LINE 0x40
#define OLED_CMD_SET_SEGMENT_REMAP      0xA1    
#define OLED_CMD_SET_MUX_RATIO          0xA8    // follow with 0x3F = 64 MUX
#define OLED_CMD_SET_COM_SCAN_MODE      0xC8    
#define OLED_CMD_SET_DISPLAY_OFFSET     0xD3    // follow with 0x00
#define OLED_CMD_SET_COM_PIN_MAP        0xDA    // follow with 0x12
#define OLED_CMD_NOP                    0xE3    // NOP

// Timing and Driving Scheme (pg.32)
#define OLED_CMD_SET_DISPLAY_CLK_DIV    0xD5    // follow with 0x80
#define OLED_CMD_SET_PRECHARGE          0xD9    // follow with 0xF1
#define OLED_CMD_SET_VCOMH_DESELCT      0xDB    // follow with 0x30

// Charge Pump (pg.62)
#define OLED_CMD_SET_CHARGE_PUMP        0x8D    // follow with 0x14


#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#define tag "SSD1306"

uint8_t pattern1[] = {
	0x00,0x7E,0x42,0x42,0x42,0x42,0x7E,0x00,
	0x00,0x7E,0x42,0x42,0x42,0x42,0x7E,0x00
	};

void i2c_master_init()
{
	i2c_config_t i2c_config = 
	{
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void ssd1306_init() {
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	
	i2c_master_write_byte(cmd,OLED_CONTROL_BYTE_CMD_STREAM,true);
	i2c_master_write_byte(cmd,OLED_CMD_DISPLAY_OFF,true);
	i2c_master_write_byte(cmd,OLED_CMD_SET_MUX_RATIO,true);
	i2c_master_write_byte(cmd,0x3F,true);
	i2c_master_write_byte(cmd,OLED_CMD_SET_DISPLAY_OFFSET,true);
	i2c_master_write_byte(cmd,0x00,true);
	i2c_master_write_byte(cmd,OLED_CMD_SET_DISPLAY_START_LINE,true);
	i2c_master_write_byte(cmd,OLED_CMD_SET_SEGMENT_REMAP,true);
    i2c_master_write_byte(cmd,OLED_CMD_SET_COM_SCAN_MODE,true);
	i2c_master_write_byte(cmd,OLED_CMD_SET_COM_PIN_MAP,true);
	i2c_master_write_byte(cmd,0x12,true);
	i2c_master_write_byte(cmd,OLED_CMD_SET_CONTRAST,true);
	i2c_master_write_byte(cmd,0x7F,true);
	i2c_master_write_byte(cmd,OLED_CMD_DISPLAY_RAM,true);
	i2c_master_write_byte(cmd,OLED_CMD_DISPLAY_NORMAL,true);
	i2c_master_write_byte(cmd,OLED_CMD_SET_DISPLAY_CLK_DIV,true);
	i2c_master_write_byte(cmd,0x80,true);
	i2c_master_write_byte(cmd,OLED_CMD_SET_CHARGE_PUMP,true);
	i2c_master_write_byte(cmd,0x14,true);
	i2c_master_write_byte(cmd,OLED_CMD_SET_PRECHARGE,true);
	i2c_master_write_byte(cmd,0x22,true);
	i2c_master_write_byte(cmd,OLED_CMD_SET_VCOMH_DESELCT,true);
	i2c_master_write_byte(cmd,0x30,true);
	i2c_master_write_byte(cmd,OLED_CMD_SET_MEMORY_ADDR_MODE,true);
	i2c_master_write_byte(cmd,0x00,true);
	i2c_master_write_byte(cmd,OLED_CMD_DISPLAY_ON,true);
    i2c_master_stop(cmd);

	espRc =i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		ESP_LOGI(tag, "OLED configured successfully");
	} else {
		ESP_LOGE(tag, "OLED configuration failed. code: 0x%.2X", espRc);
	}
	i2c_cmd_link_delete(cmd);
}



void app_main(void)
{
	i2c_master_init();
	ssd1306_init();

	esp_err_t espRc;
	
	i2c_cmd_handle_t cmd;

	cmd  = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd,OLED_CONTROL_BYTE_CMD_STREAM, true);
	i2c_master_write_byte(cmd,OLED_CMD_SET_COLUMN_RANGE, true);
	i2c_master_write_byte(cmd,0x00, true);
	i2c_master_write_byte(cmd,0x7F, true);
	i2c_master_write_byte(cmd,OLED_CMD_SET_PAGE_RANGE, true);
	i2c_master_write_byte(cmd,0, true);
	i2c_master_write_byte(cmd,0x07, true);
	i2c_master_stop(cmd);

	espRc =i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) 
	{
		ESP_LOGI(tag, "OLED cc successfully");
	} 
	else 
	{
		ESP_LOGE(tag, "OLED cc failed. code: 0x%.2X", espRc);
	}
	i2c_cmd_link_delete(cmd);

	for(uint16_t i=0;i<1024;i++)
	{
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
	    i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	    i2c_master_write_byte(cmd,OLED_CONTROL_BYTE_DATA_STREAM, true);
		for(uint8_t x=0; x<16 ; x++)
		{
			i2c_master_write(cmd , pattern1[x],8 ,true);
			i++;
		}
		i--;
	}
	i2c_master_stop(cmd);  
	espRc =i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
		if (espRc == ESP_OK) {
			ESP_LOGI(tag, "OLED dd successfully");
		} else {
			ESP_LOGE(tag, "OLED ddfailed. code: 0x%.2X", espRc);
		}
     i2c_cmd_link_delete(cmd);

	 vTaskDelay(1000/portTICK_PERIOD_MS);
}



	


