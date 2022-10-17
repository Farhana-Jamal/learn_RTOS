#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>


#define LEDPIN 2

void configureLED(void)
{
    gpio_reset_pin(LEDPIN);
    gpio_set_direction(LEDPIN,GPIO_MODE_OUTPUT);
}

void blinkLED(void)
{
    while (1)
    {
        gpio_set_level(LEDPIN,0);
        vTaskDelay(1000/portTICK_RATE_MS);
        gpio_set_level(LEDPIN,1);
        vTaskDelay(1000/portTICK_RATE_MS);
        
    }
    
}

void app_main() 
{
    configureLED();
    blinkLED();
    xTaskCreate(&blinkLED,"LED_BLINK",512,NULL,5,NULL);

}