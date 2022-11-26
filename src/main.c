#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>


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

    // if(size > 1)
    // {
    //     i2c_master_read(cmd , data ,size -1 , ACK_VAL);
      
    // }
    i2c_master_read_byte(cmd ,&AccXH , ACK_VAL );
    i2c_master_read_byte(cmd ,&AccXL , ACK_VAL );
    i2c_master_read_byte(cmd ,&AccYH , ACK_VAL ); 
    i2c_master_read_byte(cmd ,&AccYL , ACK_VAL ); 
    i2c_master_read_byte(cmd ,&AccZH , ACK_VAL);
    i2c_master_read_byte(cmd , &AccZL, NACK_VAL);
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
    float AccX , AccY , AccZ;

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
    i2c_master_read_byte(cmd ,&GyroZH , ACK_VAL);
    i2c_master_read_byte(cmd ,&GyroZL, NACK_VAL);

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

    // GYROX = (data[6] << 8 | data[7]);
    // GYROY = (data[8] << 8 | data[9]);
    // GYROZ = (data[10] << 8 | data[11]);

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

   xTaskCreate(&readAcc , "accclrtn task" , 2048 , NULL , 5 , NULL);
   
   
   xTaskCreate(&readGyro , "GYRO task " , 2048 , NULL , 5 , NULL);

   xTaskCreate(&readTemp , " TEmp task " , 2048 , NULL , 5 , NULL);
   vTaskDelay(1000 / portTICK_PERIOD_MS);
}



