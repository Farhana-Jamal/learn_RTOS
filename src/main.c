#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <string.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include <nvs_flash.h>




#define wifi_SSID "Farhana"
#define wifi_PASS "1234567890"
#define maximumRetry 15

static EventGroupHandle_t wifiEventGroup;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char* TAG = "wifi_station";

static void event_handler(void *arg, esp_event_base_t eventBase, int32_t eventId, void *eventDAta)
{
    static int retryNum = 0;
    if(eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if(eventBase == WIFI_EVENT && eventId == WIFI_EVENT_STA_DISCONNECTED)
    {
        if(retryNum < maximumRetry)
        {
            esp_wifi_connect();
            retryNum++;
            ESP_LOGI(TAG, "retry to connect to wifi AP");
        }
        else
        {
            xEventGroupSetBits(wifiEventGroup,WIFI_FAIL_BIT);
              
        }
        ESP_LOGI(TAG, "connection fails");
    }
    else if(eventBase == IP_EVENT && eventId == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t* ipEvent = (ip_event_got_ip_t*)eventDAta;
        ESP_LOGI(TAG, "Got IP: " IPSTR ,IP2STR(&ipEvent->ip_info.ip));
        retryNum = 0;
        xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_BIT);
    }
    
}

void wifi_init_STA()
{
    wifiEventGroup = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());         /*esp_netif_init() to create an LwIP core task and initialize LwIP-related work*/
    ESP_ERROR_CHECK(esp_event_loop_create_default());      /*create event task*/

    esp_netif_create_default_wifi_sta();          /*to create default network interface instance binding stationmode (create or init wifi task)*/

    wifi_init_config_t configure = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&configure));           /*to create the Wi-Fi driver task and initialize the Wi-Fi driver*/

    esp_event_handler_instance_t any_id;
    esp_event_handler_instance_t got_ip;

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &any_id));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler , NULL , &got_ip));


    wifi_config_t wifiConfig =
    {
        .sta = 
        {
            .ssid = wifi_SSID,
            .password = wifi_PASS,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));          /*to configure the Wi-Fi mode as station*/
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA , &wifiConfig));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    EventBits_t bits = xEventGroupWaitBits(wifiEventGroup , WIFI_CONNECTED_BIT|WIFI_FAIL_BIT ,pdFALSE ,pdFALSE ,portMAX_DELAY);

    if (bits && WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG , "connected to ssid:farhana password 1234567890");
        
    }
    else if (bits && WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG , "failed to connect");
        
    }
    else
    {
        ESP_LOGI(TAG, "unexpectedEvent");
    }
    
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG , "WIFI_STA_mode");
    wifi_init_STA();
           
}