#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>
#include <font8x8_basic.h>

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

#define mpuAddress 0x68
#define mpuPwrMngmnt 0x6B
#define mpuAccX 0x3B
#define mpuGyroX 0x43
#define mpuTemp 0x41

#define ACCEL_TRANSFORMATION_NUMBER     0.00006103515 // (1 / 16384) precalculated
#define GYRO_TRANSFORMATION_NUMBER      131.0 // (131.0) precalculated

#define ACK_VAL    0x0
#define NACK_VAL   0x1



#define TAG "MPU6050"
#define tag "SSD1306"
#define TaG "I2C"
#define taG "semaphore"
#define Tag "queue"

float AccX , AccY , AccZ;

SemaphoreHandle_t semaphore ;
QueueHandle_t queue ;


void masterEsp32Init(void)
{
     i2c_config_t i2c_config = 
    {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000000,
        .clk_flags = 0
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
    printf("- i2c controller configured\r\n");

    //  install the driver
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));
    printf("- i2c driver installed\r\n\r\n");
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



void mpu6050Init()
{
    esp_err_t espRC ;

    i2c_cmd_handle_t cmd;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd,(mpuAddress << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd , mpuPwrMngmnt , true);
    i2c_master_write_byte(cmd , 0x0 , true);
    i2c_master_stop(cmd);

    espRC = i2c_master_cmd_begin(I2C_NUM_0 ,cmd ,10/portTICK_PERIOD_MS);
    if (espRC == ESP_OK)
    {
        ESP_LOGI(TAG ," mpu initialized");
    }
    else
    {
        ESP_LOGI(TAG , "MPU is not connected");
    }
    i2c_cmd_link_delete(cmd);

}



void i2cScan(int address1 , int address2)
{
    esp_err_t espRC ;

    ESP_LOGI(TaG , "SCAnnING ........\r\n\r\n");

    i2c_cmd_handle_t cmd ;
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd , (address1 << 1) | I2C_MASTER_WRITE , true);
    i2c_master_write_byte(cmd , (address2 << 1) | I2C_MASTER_WRITE , true);
    i2c_master_stop(cmd);

    espRC = i2c_master_cmd_begin(I2C_NUM_0 , cmd , 10/portTICK_PERIOD_MS);

    if(espRC == ESP_OK)
    {
        ESP_LOGI(TaG , "found device with address 0x%02x\r\n" , address1);
        ESP_LOGI(TaG , "found device with address 0x%02x\r\n" , address2);
       
    }
        
    i2c_cmd_link_delete(cmd);

}

void floatToString()
{
    esp_err_t espRC;
    espRC = xSemaphoreGive(semaphore);
    if(espRC == ESP_OK)
    {
        ESP_LOGI(taG , "semaphore given ");
    }
    else
    {
        ESP_LOGI(taG , "not given ");
    }

    espRC = xQueueReceive(queue ,&AccX , (TickType_t)1000/portTICK_PERIOD_MS);
    if(espRC == ESP_OK)
    {
    ESP_LOGI(Tag , "value received on queue %f\r\n " , AccX);
    }
    else
    {
        ESP_LOGI(Tag , "not recvd");
    }
}



void readAcc()
{
    esp_err_t espRC;
    i2c_cmd_handle_t cmd;

    uint8_t AccXH , AccXL ,AccYH ,AccYL ,AccZH ,AccZL ;

    while(1)
    {

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd , (mpuAddress <<1)|I2C_MASTER_WRITE , true);
    i2c_master_write_byte(cmd , mpuAccX , true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd , (mpuAddress <<1)|I2C_MASTER_READ , true);

    i2c_master_read_byte(cmd ,&AccXH , ACK_VAL );
    i2c_master_read_byte(cmd ,&AccXL , ACK_VAL );
    i2c_master_read_byte(cmd ,&AccYH , ACK_VAL ); 
    i2c_master_read_byte(cmd ,&AccYL , ACK_VAL ); 
    i2c_master_read_byte(cmd ,&AccZH , ACK_VAL );
    i2c_master_read_byte(cmd ,&AccZL ,  NACK_VAL);
    i2c_master_stop(cmd);

    espRC = i2c_master_cmd_begin(I2C_NUM_0 , cmd , 10/portTICK_PERIOD_MS);
    if(espRC == ESP_OK)
    {
        ESP_LOGI(TAG , " got Accln values");
    }
    else
    {
        ESP_LOGI(TAG , "FAiled (accltn)");
    }

    i2c_cmd_link_delete(cmd);

    int16_t AccRawX , AccRawY , AccRawZ;
    

    AccRawX = (AccXH << 8 | AccXL);
    AccRawY = (AccYH << 8 | AccYL);
    AccRawZ = (AccZH << 8 | AccZL);

    AccX = AccRawX * ACCEL_TRANSFORMATION_NUMBER ;
    AccY = AccRawY * ACCEL_TRANSFORMATION_NUMBER ;
    AccZ = AccRawZ * ACCEL_TRANSFORMATION_NUMBER ;


    esp_log_level_set ("ACC" , ESP_LOG_INFO);
    ESP_LOGI("ACC" , "AccX : %f" , AccX);
    ESP_LOGI("ACC" , "AccY : %f" , AccY);
    ESP_LOGI("ACC" , "AccZ : %f" , AccZ);

    xSemaphoreTake(semaphore , 1000/portMAX_DELAY);
    xQueueSend(queue , &AccX , (TickType_t )1000/portTICK_PERIOD_MS);

   

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    
    }

}


void readGyro()
{
    esp_err_t espRC;
    i2c_cmd_handle_t cmd;

    uint8_t GyroXH , GyroXL ,GyroYH ,GyroYL ,GyroZH ,GyroZL ;


    while(1)
    {

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd , (mpuAddress <<1)|I2C_MASTER_WRITE , true);
    i2c_master_write_byte(cmd , mpuGyroX , true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd , (mpuAddress <<1)|I2C_MASTER_READ , true);

    
    i2c_master_read_byte(cmd ,&GyroXH , ACK_VAL );
    i2c_master_read_byte(cmd ,&GyroXL , ACK_VAL );
    i2c_master_read_byte(cmd ,&GyroYH , ACK_VAL ); 
    i2c_master_read_byte(cmd ,&GyroYL , ACK_VAL ); 
    i2c_master_read_byte(cmd ,&GyroZH , ACK_VAL );
    i2c_master_read_byte(cmd ,&GyroZL , NACK_VAL);

    i2c_master_stop(cmd);

    espRC = i2c_master_cmd_begin(I2C_NUM_0 , cmd , 10/portTICK_PERIOD_MS);
    if(espRC == ESP_OK)
    {
        ESP_LOGI(TAG , " got gyro values");
    }
    else
    {
        ESP_LOGI(TAG , "FAiled (gyro)");
    }

    i2c_cmd_link_delete(cmd);

   

    int16_t gyroRawX , gyroRawY , gyroRawZ; 
    float GYROX , GYROY ,GYROZ ;

    gyroRawX = (GyroXH << 8 | GyroXL);
    gyroRawY = (GyroYH << 8 | GyroYL);
    gyroRawZ = (GyroZH << 8 | GyroZL);

    GYROX = gyroRawX * GYRO_TRANSFORMATION_NUMBER ;
    GYROY = gyroRawY * GYRO_TRANSFORMATION_NUMBER ;
    GYROZ = gyroRawZ * GYRO_TRANSFORMATION_NUMBER ;

    
    esp_log_level_set ("GYRO" , ESP_LOG_INFO);
    ESP_LOGI("GYRO" , "GyroX : %f" , GYROX);
    ESP_LOGI("GYRO" , "GyroY : %f" , GYROY);
    ESP_LOGI("GYRO" , "GyroZ : %f" , GYROZ);

    vTaskDelay(1000 / portTICK_PERIOD_MS);


    }

}

void readTemp()
{
    esp_err_t espRC;
    i2c_cmd_handle_t cmd;

    uint8_t TempH , TempL  ;


    while(1)
    {

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd , (mpuAddress <<1)|I2C_MASTER_WRITE , true);
    i2c_master_write_byte(cmd , mpuTemp , true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd , (mpuAddress <<1)|I2C_MASTER_READ , true);

    i2c_master_read_byte(cmd ,&TempH , ACK_VAL );
    i2c_master_read_byte(cmd ,&TempL , NACK_VAL );
    
    i2c_master_stop(cmd);

    espRC = i2c_master_cmd_begin(I2C_NUM_0 , cmd , 10/portTICK_PERIOD_MS);
    if(espRC == ESP_OK)
    {
        ESP_LOGI(TAG , " got temp values");
    }
    else
    {
        ESP_LOGI(TAG , "FAiled (temp)");
    }

    i2c_cmd_link_delete(cmd);

   

    int16_t tempRaw ;
    float Temp ;

    tempRaw = (TempH << 8 | TempL);

    Temp = tempRaw / 340 + 36.53  ;
    
    esp_log_level_set ("TEMP" , ESP_LOG_INFO);
    ESP_LOGI("TEMP" , "TEMP : %f" , Temp);
    

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}


void app_main()
{
   masterEsp32Init();
   mpu6050Init();
   ssd1306_init();
   i2cScan(0x3C , 0x68);
   
   semaphore = xSemaphoreCreateCounting(14 , 0 );
//    esp_err_t espRC ;
   queue = xQueueCreate(14 , sizeof(float));

//    if(espRC == ESP_OK)
//    {
//     ESP_LOGI(Tag , "queue created");

//    }

   xTaskCreate(&readAcc , "accclrtn task" , 2048 , NULL , 5 , NULL);
   
   
   xTaskCreate(&readGyro , "GYRO task " , 2048 , NULL , 5 , NULL);

   xTaskCreate(&readTemp , " TEmp task " , 2048 , NULL , 5 , NULL);
   vTaskDelay(1000 / portTICK_PERIOD_MS);
}



