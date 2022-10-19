#include <freertos/FreeRTOS.h>
#include <driver/pcnt.h>
#include<freertos/task.h>

#define PCNT_GPIO 4
#define H_LIM_VALUE 1000
#define L_LIM_VALUE -10
 
int unit;

static void PCNT_init(int unit)
{
    pcnt_config_t pcnt_config = 
    {
        .pulse_gpio_num = PCNT_GPIO,
        .channel = PCNT_CHANNEL_0,
        .unit = unit,
        .pos_mode = PCNT_COUNT_INC,
        .neg_mode = PCNT_COUNT_DIS,
        .counter_h_lim = H_LIM_VALUE,
        .counter_l_lim = L_LIM_VALUE

    };
    pcnt_unit_config(&pcnt_config);

    pcnt_set_filter_value(unit ,10);
    pcnt_filter_enable(unit);
     
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);
    pcnt_counter_resume(unit);
}

void app_main()
{
    int pcnt_unit = PCNT_UNIT_0;
    PCNT_init(pcnt_unit);
    

    int16_t count = 0;
    while(1)
    {
        pcnt_get_counter_value(pcnt_unit , &count);
        printf("%d \n", count );
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}