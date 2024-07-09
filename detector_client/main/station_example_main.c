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
#include <lwip/sockets.h>
#include "lwip/err.h"
#include "lwip/sys.h"
#include <driver/gpio.h>
#include <stdio.h>
#include <stdlib.h>
#include "driver/ledc.h"
#include "driver/gpio.h"


/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_ESP_WIFI_SSID      	"528A"				//Wi-Fi名字
#define EXAMPLE_ESP_WIFI_PASS      	"13790387087"		//Wi-Fi密码
#define EXAMPLE_ESP_MAXIMUM_RETRY  	6					//Wi-Fi最大重连次数
#define SERVER_IP_ADDRESS			"192.168.3.254"		//服务器IP地址
#define SERVER_SOCKET_PORT			8888				//服务器端口
#define SOCKET_MAXIMUM_RETRY		5					//链接socket server的重试次数
#define DEVICE_ID					"2"			//设备序列号

#define SENSOR_RECV_PIN GPIO_NUM_0
#define TRIGGER_THRESHOLD 4
#define TIMEOUT 100

#if CONFIG_ESP_WPA3_SAE_PWE_HUNT_AND_PECK
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HUNT_AND_PECK
#define EXAMPLE_H2E_IDENTIFIER ""
#elif CONFIG_ESP_WPA3_SAE_PWE_HASH_TO_ELEMENT
#define ESP_WIFI_SAE_MODE WPA3_SAE_PWE_HASH_TO_ELEMENT
#define EXAMPLE_H2E_IDENTIFIER CONFIG_ESP_WIFI_PW_ID
#elif CONFIG_ESP_WPA3_SAE_PWE_BOTH
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

*/
// 这里是定义控制舵机的相关宏
#define LEDC_TIMER           LEDC_TIMER_0
#define LEDC_MODE            LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_GPIO     (1) // GPIO connected to the servo
#define LEDC_CHANNEL         LEDC_CHANNEL_0
#define LEDC_FREQ_HZ         50   // Frequency for PWM signal
#define LEDC_RESOLUTION      LEDC_TIMER_13_BIT
ledc_channel_config_t motor_init(){//初始化电机配置
	// 配置定时器
	ledc_timer_config_t ledc_timer = {
	    .duty_resolution = LEDC_RESOLUTION,
	    .freq_hz = LEDC_FREQ_HZ,
	    .speed_mode = LEDC_MODE,
	    .timer_num = LEDC_TIMER,
	    .clk_cfg = LEDC_AUTO_CLK
	};
	ledc_timer_config(&ledc_timer);

	// 配置通道
	ledc_channel_config_t ledc_channel = {
	    .channel    = LEDC_CHANNEL,
	    .duty       = 0, // 设置占空比
	    .gpio_num   = LEDC_OUTPUT_GPIO,
	    .speed_mode = LEDC_MODE,
	    .hpoint     = 0,
	    .timer_sel  = LEDC_TIMER
	};
	ledc_channel_config(&ledc_channel);
	return ledc_channel;
}
// 输入目标角度
int getPWMDuty(float targetAngle) {
    // PWM占空比的范围 (2^LEDC_RESOLUTION)
    int range = 8191;
    int pwmFrequency = 50;
    float cycleLength = 1000.0 / pwmFrequency; // 周期长度 (ms)

    // 计算脉宽时间 (ms)
    // 0度对应1ms, 180度对应2ms
    float minPulseWidth = 1.0; // 最小脉宽 (ms)
    float maxPulseWidth = 2.0; // 最大脉宽 (ms)
    float pulseWidth = minPulseWidth + (maxPulseWidth - minPulseWidth) * targetAngle / 180.0;

    // 将脉宽时间转换为占空比值
    int dutyCycle = (int)((pulseWidth / cycleLength) * range);

    return dutyCycle;
}
// 设置舵机角度
void setServoAngle(float targetAngle, ledc_channel_config_t ledc_channel_config) {
    // 计算运转到目标角度需要的PWM占空比
    int duty = getPWMDuty(targetAngle);
    // 设置LEDC通道以输出计算得到的占空比
    ledc_set_duty(ledc_channel_config.speed_mode, ledc_channel_config.channel, duty);
    ledc_update_duty(ledc_channel_config.speed_mode, ledc_channel_config.channel);
}

void motor_release(ledc_channel_config_t ledc_channel){
	for (int i = 0; i<160; i+=40){
		setServoAngle(i, ledc_channel);
		vTaskDelay(50);
	}

}

void motor_lock(ledc_channel_config_t ledc_channel){
	setServoAngle(0, ledc_channel);
	vTaskDelay(50);
}
/*

*/


static const char *TAG3 = "SENSOR";
void sensor_init(){
	gpio_config_t io_conf = {
			.pin_bit_mask = (1ULL<<SENSOR_RECV_PIN),
			.mode = GPIO_MODE_INPUT,
			.intr_type = GPIO_INTR_DISABLE,
			.pull_up_en = GPIO_PULLUP_DISABLE,
			.pull_down_en = GPIO_PULLDOWN_ENABLE,
	};
	gpio_config(&io_conf);
}
void Alarm(){
	ESP_LOGE(TAG3, "Detect timeout! (>%d)", TIMEOUT);
}
int check() {
	//初始化引脚
		sensor_init();

		int counter = 0;

	    while (true) {
	        int level = gpio_get_level(SENSOR_RECV_PIN);
	        if (level == 1) {
	        	if (counter < TRIGGER_THRESHOLD) {//液体滴落时会连续触发多个程序循环周期的高电平，需要数据除重
	        		counter = 0;
	        	}else {
	        		ESP_LOGI(TAG3, "detected! Last trigger: %d", counter);
	        		counter = 0;
	        	}
	        }
	        counter++;
	        if (counter > TIMEOUT){
	        	Alarm();
	        	return 666;
	        }
	        vTaskDelay(100 / portTICK_PERIOD_MS);
	    }
}

/*

*/


/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";	//用于在监视器中打印信息显示的标签
static const char *TAG2 = "Socket Client";

static int s_retry_num = 0;

static int sock;

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

#define LED_PIN GPIO_NUM_8 // led对应的gpio口
void led_init()
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1 << LED_PIN,
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg)); // 对gpio进行初始化配置
}
void led_on()
{
    ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 1));
}
void led_off()
{
    ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 0));
}

void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

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
            /* Authmode threshold resets to WPA2 as default if password matches WPA2 standards (pasword len => 8).
             * If you want to connect the device to deprecated WEP/WPA networks, Please set the threshold value
             * to WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK and set the password with length and format matching to
             * WIFI_AUTH_WEP/WIFI_AUTH_WPA_PSK standards.
             */
            .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
            .sae_pwe_h2e = ESP_WIFI_SAE_MODE,
            .sae_h2e_identifier = EXAMPLE_H2E_IDENTIFIER,
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
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

int client_send(){
    sock = socket(AF_INET, SOCK_STREAM, 0);
       if (sock < 0)
       {
           ESP_LOGE(TAG2, "create socket failed!");
           return -1;
       }
       ESP_LOGI(TAG2, "create socket successfully!");

       // 初始化server的地址结构体sockaddr_in
       struct sockaddr_in destaddr = {};
       destaddr.sin_family = AF_INET;
       destaddr.sin_port = htons(SERVER_SOCKET_PORT);                      // 填写网络调试助手服务端实际端口
       destaddr.sin_addr.s_addr = inet_addr(SERVER_IP_ADDRESS); // 填写网络调试助手服务端实际IP地址
       ESP_LOGI(TAG2, "Try to connect server at IP-%s PORT-%d", SERVER_IP_ADDRESS, SERVER_SOCKET_PORT);
       // 建立socket连接：
       socklen_t len = sizeof(struct sockaddr);
       if (connect(sock, (struct sockaddr *)&destaddr, len) < 0)
       {
           ESP_LOGE(TAG2, "connect to server failed!");
           close(sock);
           return -2;
       }
       ESP_LOGI(TAG2, "connect to server successfully!");

       // 发送数据给服务端：send();
       char buff[64];
       sprintf(buff,"%s", DEVICE_ID);
       send(sock, buff, strlen(buff), 0);
       printf("\nmessage send\n");

       //接收服务器返回的数据并对比
       char buff_recv[64];
       int length = recv(sock, buff_recv, sizeof(buff_recv) - 1, 0);
       if (length < 0) {//没收到消息或者发生错误
    	   ESP_LOGE(TAG2, "recv failed: errno %d", errno);
    	   return -3;
       }
       // Data received
       else {
    	   buff_recv[length] = 0; // Null-terminate whatever we received and treat like a string
    	   ESP_LOGI(TAG2, "Received %d bytes from %s:%s", length, SERVER_IP_ADDRESS, buff_recv);
       }
       if (strcmp(buff, buff_recv) != 0){
    	   ESP_LOGE(TAG2, "Data verify error!");
    	   return -4;
       }
       shutdown(sock, 0);
       close(sock);
       return 0;
}

void app_main(void)
{
	led_init();
	ledc_channel_config_t ledc_channel = motor_init();
	motor_release(ledc_channel);//电机解锁输液管
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();//初始化Wi-Fi并连接
    if(check() == 666){// 返回值为666代表检测到液滴停止
    	motor_lock(ledc_channel);
        for (int i = 0;i < SOCKET_MAXIMUM_RETRY; i++){
        	if(client_send()!=0){
        		ESP_LOGE(TAG2,"connect to server failed! Retry: %d",i);
        	}
        	else break;
        }

    }

}
