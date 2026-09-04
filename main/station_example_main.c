/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include <esp_http_server.h>
#include "driver/gpio.h"
#include "driver/ledc.h"

/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      CONFIG_ESP_WIFI_SSID
#define EXAMPLE_ESP_WIFI_PASS      CONFIG_ESP_WIFI_PASSWORD
#define EXAMPLE_ESP_MAXIMUM_RETRY  CONFIG_ESP_MAXIMUM_RETRY

#if CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_STATION_EXAMPLE_WPA3_SAE_PWE_BOTH
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_BOTH
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#endif
#if CONFIG_ESP_WIFI_AUTH_OPEN
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN
#elif CONFIG_ESP_WIFI_AUTH_WEP
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WEP
#elif CONFIG_ESP_WIFI_AUTH_WPA_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA_WPA2_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA_WPA2_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WPA2_WPA3_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WPA2_WPA3_PSK
#elif CONFIG_ESP_WIFI_AUTH_WAPI_PSK
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_WAPI_PSK
#endif

/*
 GPIO
*/

static const gpio_num_t LED = GPIO_NUM_2;

static const gpio_num_t TB6612_PWMA = GPIO_NUM_22;
static const gpio_num_t TB6612_AIN2 = GPIO_NUM_23;
static const gpio_num_t TB6612_AIN1 = GPIO_NUM_21;
static const gpio_num_t TB6612_STBY = GPIO_NUM_19;

#define TB6612_PWM_FREQUENCY_HZ 20000
#define TB6612_PWM_RESOLUTION  LEDC_TIMER_10_BIT //1024 = 100%
#define TB6612_PWM_TIMER       LEDC_TIMER_0
#define TB6612_PWM_CHANNEL     LEDC_CHANNEL_0

static const int TB6612_PWM_DUTY_PERCENT = 100;

static uint32_t tb6612_duty_from_percent(int percent)
{
    if (percent < 0) {
        percent = 0;
    } else if (percent > 100) {
        percent = 100;
    }
    uint32_t val = (uint32_t)percent * ((1U << TB6612_PWM_RESOLUTION) - 1U) / 100U;
    ESP_LOGI("MOTOR","pwm duty cycle: %d -> %ul", TB6612_PWM_DUTY_PERCENT, val);
    return val;
}
/*
 WIFI
*/

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;

static void tb6612_gpio_init(void)
{
    gpio_num_t output_gpios[] = {TB6612_AIN1, TB6612_AIN2, TB6612_STBY};
    gpio_config_t output_config = {
        .pin_bit_mask = (1ULL << TB6612_AIN1) | (1ULL << TB6612_AIN2) | (1ULL << TB6612_STBY),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    ESP_ERROR_CHECK(gpio_config(&output_config));
    for (size_t index = 0; index < sizeof(output_gpios) / sizeof(output_gpios[0]); index++) {
        ESP_ERROR_CHECK(gpio_set_level(output_gpios[index], 0));
    }

    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = TB6612_PWM_TIMER,
        .duty_resolution = TB6612_PWM_RESOLUTION,
        .freq_hz = TB6612_PWM_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    ledc_channel_config_t pwm_channel = {
        .gpio_num = TB6612_PWMA,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = TB6612_PWM_CHANNEL,
        .timer_sel = TB6612_PWM_TIMER,
        .duty = tb6612_duty_from_percent(TB6612_PWM_DUTY_PERCENT),
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_channel));
}


static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

int wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    // 1. Create the default station interface and capture the pointer
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();

    // 2. Stop the DHCP client
    ESP_ERROR_CHECK(esp_netif_dhcpc_stop(sta_netif));

    // 3. Define the static IP, Gateway, and Netmask
    esp_netif_ip_info_t ip_info;
    esp_netif_set_ip4_addr(&ip_info.ip, 192, 168, 1, 100);      // Set desired IP
    esp_netif_set_ip4_addr(&ip_info.gw, 192, 168, 1, 1);        // Set Gateway
    esp_netif_set_ip4_addr(&ip_info.netmask, 255, 255, 255, 0); // Set Netmask

    // 4. Apply the static IP configuration to the interface
    ESP_ERROR_CHECK(esp_netif_set_ip_info(sta_netif, &ip_info));
    
    // 5. (Optional) Set a primary DNS server if you need internet access
    esp_netif_dns_info_t dns_info;
    esp_netif_set_ip4_addr(&dns_info.ip.u_addr.ip4, 8, 8, 8, 8); // Google DNS
    dns_info.ip.type = ESP_IPADDR_TYPE_V4;
    ESP_ERROR_CHECK(esp_netif_set_dns_info(sta_netif, ESP_NETIF_DNS_MAIN, &dns_info));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Authmode threshold resets to WPA2 as default if auth mode threshold equals WIFI_AUTH_OPEN
             * and password matches WPA2 standards (password len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
#ifdef CONFIG_ESP_WIFI_WPA3_COMPATIBLE_SUPPORT
            .disable_wpa3_compatible_mode = 0,
#endif
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        return true;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
        return false;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        return false;
    }
}

/*
 SERVER
*/

/* HTTP GET handler for /open */
static esp_err_t open_get_handler(httpd_req_t *req)
{
    gpio_set_level(LED, 1);
    gpio_set_level(TB6612_AIN1, 1);
    gpio_set_level(TB6612_AIN2, 0);
    gpio_set_level(TB6612_STBY, 1);
    const char* resp_str = "Action: OPEN triggered!";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "/open endpoint hit");

    return ESP_OK;
}

static const httpd_uri_t open_uri = {
    .uri       = "/open",
    .method    = HTTP_GET,
    .handler   = open_get_handler,
    .user_ctx  = NULL
};

/* HTTP GET handler for /close */
static esp_err_t close_get_handler(httpd_req_t *req)
{
    gpio_set_level(LED, 0);
    gpio_set_level(TB6612_AIN1, 0);
    gpio_set_level(TB6612_AIN2, 1);
    gpio_set_level(TB6612_STBY, 1);
    const char* resp_str = "Action: CLOSE triggered!";
    httpd_resp_send(req, resp_str, HTTPD_RESP_USE_STRLEN);
    ESP_LOGI(TAG, "/close endpoint hit");

    return ESP_OK;
}

static const httpd_uri_t close_uri = {
    .uri       = "/close",
    .method    = HTTP_GET,
    .handler   = close_get_handler,
    .user_ctx  = NULL
};

/* Function to start the web server */
static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &open_uri);
        httpd_register_uri_handler(server, &close_uri);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void app_main(void)
{
    gpio_reset_pin(LED);
    gpio_set_direction(LED, GPIO_MODE_OUTPUT);
    gpio_set_level(LED, 0);
    ESP_LOGI(TAG, "Initializing gpios");
    tb6612_gpio_init();

    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (CONFIG_LOG_MAXIMUM_LEVEL > CONFIG_LOG_DEFAULT_LEVEL) {
        /* If you only want to open more logs in the wifi module, you need to make the max level greater than the default level,
         * and call esp_log_level_set() before esp_wifi_init() to improve the log level of the wifi module. */
        esp_log_level_set("wifi", CONFIG_LOG_MAXIMUM_LEVEL);
    }

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    if (wifi_init_sta())
    {
        start_webserver();
    }
}
