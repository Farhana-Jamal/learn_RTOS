#include <freertos/FreeRTOS.h>
#include <driver/gpio.h>
#include<freertos/task.h>


#define sensor GPIO_NUM_4

#define ESP_INTR_FLAG_DEFAULT 0

static int count = 0;

void IRAM_ATTR isr_handler(void *arg)
{
    count++;
    printf("%d" ,count);
        
}

void app_main()
{
    gpio_config_t io_conf =
    {
        .pin_bit_mask = (1ULL<<sensor),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_POSEDGE
    };

    gpio_config(&io_conf);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);        /*flag used to allocate isr*/

    gpio_isr_handler_add(sensor , isr_handler ,(void*)sensor);

     int cnt = 0;
     while(1)
     {
        printf("cnt : %d \n ",cnt++);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(sensor , cnt % 2);
     }
    
      
}