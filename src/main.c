#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2c.h>
#include <esp_err.h>
#include <esp_log.h>


#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22

#define mpuAddress 0x68
#define mpuSmplRate 0x19
#define whoAmI 0x75
#define mpuPwrMngmnt 0x6B
#define mpuConfig 0x1A

#define mpuAccConfig 0x1C 
#define mpuAccXH 0x3B
#define mpuAccXL 0x3C
#define mpuAccYH 0x3D
#define mpuAccYL 0x3E
#define mpuAccZH 0x3F
#define mpuAccZL 0x50

#define mpuGyroConfig 0x1B
#define mpuGyroXH 0x43
#define mpuGyroXL 0x44
#define mpuGyroYH 0x45
#define mpuGyroYL 0x46
#define mpuGyroZH 0x47
#define mpuGyroZL 0x48

#define mpuTemp 0x41

#define ACCEL_TRANSFORMATION_NUMBER     0.00006103515 // (1 / 16384) precalculated
// #define GYRO_TRANSFORMATION_NUMBER      131.0 // (131.0) precalculated

#define ACK_VAL    0x0
#define NACK_VAL   0x1

#define PCA9548ADDR   0x70
#define  PCA9548_CH1  0x01


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
//     i2c_param_config(I2C_NUM_0, &i2c_config);
//     i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
       ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &i2c_config));
       printf("- i2c controller configured\r\n");

    // install the driver
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
    i2c_master_write_byte(cmd , mpuGyroConfig ,true);
    i2c_master_write_byte(cmd , 0x00000000 ,true);
    i2c_master_write_byte(cmd , mpuAccConfig , true);
    i2c_master_write_byte(cmd ,0b00000000 ,true);
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

    while(1)
    {

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd , (mpuAddress <<1)|I2C_MASTER_WRITE , true);
    i2c_master_write_byte(cmd , mpuAccXH , true);
    i2c_master_write_byte(cmd , mpuAccXL , true);
    i2c_master_write_byte(cmd , mpuAccYH , true);
    i2c_master_write_byte(cmd , mpuAccYL , true);
    i2c_master_write_byte(cmd , mpuAccZH , true);
    i2c_master_write_byte(cmd , mpuAccZL , true);
    i2c_master_stop(cmd);

    i2c_master_cmd_begin(I2C_NUM_0 , cmd , 10/portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    uint8_t AccXH ,AccXL , AccYH ,AccYL , AccZH ,AccZL ;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd , (mpuAddress <<1)|I2C_MASTER_READ , true);
    i2c_master_read_byte(cmd , &AccXH , ACK_VAL);
    i2c_master_read_byte(cmd , &AccXL , ACK_VAL);
    i2c_master_read_byte(cmd , &AccYH , ACK_VAL);
    i2c_master_read_byte(cmd , &AccYL , ACK_VAL);
    i2c_master_read_byte(cmd , &AccZH , ACK_VAL);
    i2c_master_read_byte(cmd , &AccZL ,NACK_VAL);
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

    int16_t AccX , AccY , AccZ;

    AccX = (AccXH << 8 | AccXL);
    AccY = (AccYH << 8 | AccYL);
    AccZ = (AccZH << 8 | AccZL);

    esp_log_level_set ("ACC" , ESP_LOG_INFO);
    ESP_LOGI("ACC" , "AccX : %d" , AccX);
    ESP_LOGI("ACC" , "AccY : %d" , AccY);
    ESP_LOGI("ACC" , "AccZ : %d" , AccZ);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

}

void readGyro()
{
    esp_err_t espRC ;
    i2c_cmd_handle_t cmd ;

    while(1)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd , (mpuAddress << 1) | I2C_MASTER_WRITE , true);
        i2c_master_write_byte(cmd , mpuGyroXH , true);
        i2c_master_write_byte(cmd , mpuGyroXL , true);
        i2c_master_write_byte(cmd , mpuGyroYH , true);
        i2c_master_write_byte(cmd , mpuGyroYL , true);
        i2c_master_write_byte(cmd , mpuGyroZH , true);
        i2c_master_write_byte(cmd , mpuGyroZL , true);
        // i2c_master_stop(cmd);

        // ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0 , cmd , 10/portTICK_PERIOD_MS));
        // printf("gyro configured \n");

        // i2c_cmd_link_delete(cmd);

        uint8_t GyroXH , GyroXL , GyroYH , GyroYL , GyroZH ,GyroZL ;

        // cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd , (mpuAddress << 1) | I2C_MASTER_READ , true);
        i2c_master_read_byte(cmd , &GyroXH , ACK_VAL);
        i2c_master_read_byte(cmd , &GyroXL , ACK_VAL);
        i2c_master_read_byte(cmd , &GyroYH , ACK_VAL);
        i2c_master_read_byte(cmd , &GyroYL , ACK_VAL);
        i2c_master_read_byte(cmd , &GyroZH , ACK_VAL);
        i2c_master_read_byte(cmd , &GyroZL , NACK_VAL);
        i2c_master_stop(cmd);

        espRC = i2c_master_cmd_begin(I2C_NUM_0 , cmd , 10/portTICK_PERIOD_MS);

        if(espRC == ESP_OK)
        {
            ESP_LOGI(TAG , "got gyro vaalues" );
        }
        else
        {
            ESP_LOGI(TAG , "failed (gyro)");
        }

        i2c_cmd_link_delete(cmd);

        int16_t GYROX , GYROY ,GYROZ ;

        GYROX = (GyroXH << 8 | GyroXL);
        GYROY = (GyroYH << 8 | GyroYL);
        GYROZ = (GyroZH << 8 | GyroZL);

        esp_log_level_set ("GYRO" , ESP_LOG_INFO);
        ESP_LOGI("GYRO" , "GyroX : %d" , GYROX);
        ESP_LOGI("GYRO" , "GyroY : %d" , GYROY);
        ESP_LOGI("GYRO" , "GyroZ : %d" , GYROZ);

        vTaskDelay(1000 / portTICK_PERIOD_MS);




    }

}

void app_main()
{
   masterEsp32Init();
   mpu6050Init();

   xTaskCreate(&readAcc , "accclrtn task" , 2048 , NULL , 5 , NULL);
   vTaskDelay(1000 / portTICK_PERIOD_MS);
   
   xTaskCreate(&readGyro , "GYRO task " , 2048 , NULL , 5 , NULL);
}

