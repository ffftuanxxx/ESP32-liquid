/* Static IP Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/semphr.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include <netdb.h>
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

/* The examples use configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/
#define EXAMPLE_WIFI_SSID             "528A"
#define EXAMPLE_WIFI_PASS             "13790387087"
#define EXAMPLE_MAXIMUM_RETRY         6
#define EXAMPLE_STATIC_IP_ADDR        "192.168.3.254"
#define EXAMPLE_STATIC_NETMASK_ADDR   "255.255.255.0"
#define EXAMPLE_STATIC_GW_ADDR        "192.168.3.1"
#ifdef CONFIG_EXAMPLE_STATIC_DNS_AUTO
#define EXAMPLE_MAIN_DNS_SERVER       "192.168.3.1"
#define EXAMPLE_BACKUP_DNS_SERVER     "8.8.8.8"
#else
#define EXAMPLE_MAIN_DNS_SERVER       CONFIG_EXAMPLE_STATIC_DNS_SERVER_MAIN
#define EXAMPLE_BACKUP_DNS_SERVER     CONFIG_EXAMPLE_STATIC_DNS_SERVER_BACKUP
#endif
#ifdef CONFIG_EXAMPLE_STATIC_DNS_RESOLVE_TEST
#define EXAMPLE_RESOLVE_DOMAIN        CONFIG_EXAMPLE_STATIC_RESOLVE_DOMAIN
#endif

#define ESP_INTR_FLAG_DEFAULT 0
SemaphoreHandle_t xSemaphore = NULL;

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT 			BIT0
#define WIFI_FAIL_BIT      			BIT1

#define BUZZER_PIN						GPIO_NUM_0
#define BUTTON_PIN						GPIO_NUM_20

#define QUEUE_LENGTH					10

#define DATA_PIN GPIO_NUM_1	//数据引脚，向数码管发送数据,链接DIO
#define CLOCK_PIN GPIO_NUM_2	//时钟引脚，和数码管同步时钟频率，链接SCLK
#define LATCH_PIN GPIO_NUM_3	//锁存引脚，该引脚电平上升时，74HC595移位寄存器数据移动一位，链接RCLK
#define LOW 0
#define HIGH 1
#define LSBFIRST 0
#define MSBFIRST 1

const uint8_t symbol[] = {
	    0b11111100, // 0
	    0b01100000, // 1
	    0b11011010, // 2
	    0b11110010, // 3
	    0b01100110, // 4
	    0b10110110, // 5
	    0b10111110, // 6
	    0b11100000, // 7
	    0b11111110, // 8
	    0b11110110, // 9
		0b00000001, // Dot
		0b00000000, // Space
		0b11101110  // A
};

void initialize_gpio_pins() {		//数码管初始化引脚
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LATCH_PIN) | (1ULL << CLOCK_PIN) | (1ULL << DATA_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(CLOCK_PIN, LOW);// 先拉低电平
    gpio_set_level(LATCH_PIN, LOW);
}

void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value) {//将1Byte的数据一位一位地移出寄存器
    for (int i = 0; i < 8; i++) {
        if (bitOrder == LSBFIRST) {//最低位优先
            gpio_set_level(dataPin, !!(value & (1 << i)));
        } else if (bitOrder == MSBFIRST){//最高位优先
            gpio_set_level(dataPin, !!(value & (1 << (7 - i))));
        }

        gpio_set_level(clockPin, HIGH);
        gpio_set_level(clockPin, LOW);
    }
}

void digit_display(int num){	//显示一位字符（字符位置，要显示的字符）
	shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, ~symbol[num]);//字符内容
	shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, 0b00000001);//第几位字符
    gpio_set_level(LATCH_PIN, HIGH);
    gpio_set_level(LATCH_PIN, LOW);
}

typedef struct {
    int queue[QUEUE_LENGTH];
    int front;
    int rear;
} Queue; //这里的queue是用于储存传感器发送过来的设备编号

Queue QUEUE = { .front = -1, .rear = -1 };

bool isFull(Queue *q) {
    return ((q->rear + 1) % QUEUE_LENGTH == q->front);
}

bool isEmpty(Queue *q) {
    return (q->front == -1 && q->rear == -1);
}

void enqueue(Queue *q, int item) {
    if (isFull(q)) {
        printf("Queue is full.\n");
        return;
    } else if (isEmpty(q)) {
        q->front = q->rear = 0;
    } else {
        q->rear = (q->rear + 1) % QUEUE_LENGTH;
    }
    q->queue[q->rear] = item;
}

int dequeue(Queue *q) {
    int item;
    if (isEmpty(q)) {
        printf("Queue is empty.\n");
        return -1;
    } else if (q->front == q->rear) {
        item = q->queue[q->front];
        q->front = q->rear = -1;
    } else {
        item = q->queue[q->front];
        q->front = (q->front + 1) % QUEUE_LENGTH;
    }
    return item;
}

void displayQueue(Queue *q) {
    if (isEmpty(q)) {
        printf("Queue is empty.\n");
        return;
    }
    int i = q->front;
    printf("Queue: ");
    do {
        printf("%d ", q->queue[i]);
        i = (i + 1) % QUEUE_LENGTH;
    } while (i != (q->rear + 1) % QUEUE_LENGTH);
    printf("\n");
}

static const char *TAG = "static_ip";

static int s_retry_num = 0;

static esp_err_t example_set_dns_server(esp_netif_t *netif, uint32_t addr, esp_netif_dns_type_t type)
{
    if (addr && (addr != IPADDR_NONE)) {
        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = addr;
        dns.ip.type = IPADDR_TYPE_V4;
        ESP_ERROR_CHECK(esp_netif_set_dns_info(netif, type, &dns));
    }
    return ESP_OK;
}

static void example_set_static_ip(esp_netif_t *netif)
{
    if (esp_netif_dhcpc_stop(netif) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop dhcp client");
        return;
    }
    esp_netif_ip_info_t ip;
    memset(&ip, 0 , sizeof(esp_netif_ip_info_t));
    ip.ip.addr = ipaddr_addr(EXAMPLE_STATIC_IP_ADDR);
    ip.netmask.addr = ipaddr_addr(EXAMPLE_STATIC_NETMASK_ADDR);
    ip.gw.addr = ipaddr_addr(EXAMPLE_STATIC_GW_ADDR);
    if (esp_netif_set_ip_info(netif, &ip) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ip info");
        return;
    }
    ESP_LOGD(TAG, "Success to set static ip: %s, netmask: %s, gw: %s", EXAMPLE_STATIC_IP_ADDR, EXAMPLE_STATIC_NETMASK_ADDR, EXAMPLE_STATIC_GW_ADDR);
    ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(EXAMPLE_MAIN_DNS_SERVER), ESP_NETIF_DNS_MAIN));
    ESP_ERROR_CHECK(example_set_dns_server(netif, ipaddr_addr(EXAMPLE_BACKUP_DNS_SERVER), ESP_NETIF_DNS_BACKUP));
}

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_CONNECTED) {
        example_set_static_ip(arg);
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "static ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}
void wifi_conn_succ();
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        sta_netif,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        sta_netif,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_WIFI_SSID,
            .password = EXAMPLE_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,
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
                 EXAMPLE_WIFI_SSID, EXAMPLE_WIFI_PASS);
        xTaskCreate(wifi_conn_succ, "wifi_succ_ala", 4096, NULL, 9, NULL);//wifi链接成功后快速响三声
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_WIFI_SSID, EXAMPLE_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
#ifdef CONFIG_EXAMPLE_STATIC_DNS_RESOLVE_TEST
    struct addrinfo *address_info;
    struct addrinfo hints;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;

    int res = getaddrinfo(EXAMPLE_RESOLVE_DOMAIN, NULL, &hints, &address_info);
    if (res != 0 || address_info == NULL) {
        ESP_LOGE(TAG, "couldn't get hostname for :%s: "
                      "getaddrinfo() returns %d, addrinfo=%p", EXAMPLE_RESOLVE_DOMAIN, res, address_info);
    } else {
        if (address_info->ai_family == AF_INET) {
            struct sockaddr_in *p = (struct sockaddr_in *)address_info->ai_addr;
            ESP_LOGI(TAG, "Resolved IPv4 address: %s", ipaddr_ntoa((const ip_addr_t*)&p->sin_addr.s_addr));
        }
#if CONFIG_LWIP_IPV6
        else if (address_info->ai_family == AF_INET6) {
            struct sockaddr_in6 *p = (struct sockaddr_in6 *)address_info->ai_addr;
            ESP_LOGI(TAG, "Resolved IPv6 address: %s", ip6addr_ntoa((const ip6_addr_t*)&p->sin6_addr));
        }
#endif
    }
#endif
    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}
/*
 *
 *
 */
#define PORT                        	8888		//绑定端口
#define KEEPALIVE_IDLE              	3			//链接空闲时间
#define KEEPALIVE_INTERVAL          	3			//保持链接间隔时间，每隔这段时间会重发保活报文
#define KEEPALIVE_COUNT             	2			//保活报文重发次数

static const char *TAG2 = "ESP_SERVER";
void Alarm();
static void do_retransmit(const int sock)
{
    int len;
    char rx_buffer[128];
    int Device_ID;
    do {
        len = recv(sock, rx_buffer, sizeof(rx_buffer) - 1, 0);
        if (len < 0) {
            ESP_LOGE(TAG2, "Error occurred during receiving: errno %d", errno);
        } else if (len == 0) {
            ESP_LOGW(TAG2, "Connection closed");
        } else {
            rx_buffer[len] = 0; // Null-terminate whatever is received and treat it like a string
            ESP_LOGI(TAG2, "Received %d bytes: %s", len, rx_buffer);
            Device_ID = rx_buffer[0]-48;
            enqueue(&QUEUE, Device_ID);
            displayQueue(&QUEUE);
            digit_display(Device_ID);
            xTaskCreate(Alarm, "buzzer_alarm", 4096, NULL, 9, NULL);
            // send() can return less bytes than supplied length.
            // Walk-around for robust implementation.
            int to_write = len;
            while (to_write > 0) {
                int written = send(sock, rx_buffer + (len - to_write), to_write, 0);
                if (written < 0) {
                    ESP_LOGE(TAG2, "Error occurred during sending: errno %d", errno);
                    // Failed to retransmit, giving up
                    return;
                }
                to_write -= written;
            }
        }
    } while (len > 0);
}

static void tcp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;


    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }



    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG2, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(listen_sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));


    ESP_LOGI(TAG, "Socket created");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG2, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG2, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG2, "Socket bound, port %d", PORT);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG2, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {

        ESP_LOGI(TAG2, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG2, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(sock, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(sock, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string

        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }


        ESP_LOGI(TAG2, "Socket accepted ip address: %s", addr_str);

        do_retransmit(sock);

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}
/*
 *
 */
void buzzer_init(){// 初始化蜂鸣器引脚
	gpio_config_t io_conf = {
	        .pin_bit_mask = (1ULL << BUZZER_PIN),
	        .mode = GPIO_MODE_OUTPUT,
	        .intr_type = GPIO_INTR_DISABLE,
	        .pull_up_en = GPIO_PULLUP_DISABLE,
	        .pull_down_en = GPIO_PULLDOWN_DISABLE,
	    };
	    gpio_config(&io_conf);
	    gpio_set_level(BUZZER_PIN, 0);// 先拉低电平
}

void Start_upALA(){//启动时快速响两声
	gpio_set_level(BUZZER_PIN, 1);
	vTaskDelay(pdMS_TO_TICKS(150));
	gpio_set_level(BUZZER_PIN, 0);
	vTaskDelay(pdMS_TO_TICKS(50));
	gpio_set_level(BUZZER_PIN, 1);
	vTaskDelay(pdMS_TO_TICKS(150));
	gpio_set_level(BUZZER_PIN, 0);
}

void wifi_conn_succ(){//成功链接wifi快速响三声
	gpio_set_level(BUZZER_PIN, 1);
	vTaskDelay(pdMS_TO_TICKS(150));
	gpio_set_level(BUZZER_PIN, 0);
	vTaskDelay(pdMS_TO_TICKS(50));
	gpio_set_level(BUZZER_PIN, 1);
	vTaskDelay(pdMS_TO_TICKS(150));
	gpio_set_level(BUZZER_PIN, 0);
	vTaskDelay(pdMS_TO_TICKS(50));
	gpio_set_level(BUZZER_PIN, 1);
	vTaskDelay(pdMS_TO_TICKS(150));
	gpio_set_level(BUZZER_PIN, 0);
	vTaskDelete(NULL);
}

void Alarm(){// 蜂鸣器三连响
	for (int i=0;i<3;i++){
		gpio_set_level(BUZZER_PIN, 1);
		vTaskDelay(pdMS_TO_TICKS(250));
		gpio_set_level(BUZZER_PIN, 0);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	vTaskDelete(NULL);
}



void button_init(){// 初始化按键
	gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_POSEDGE;  // Interrupt on positive edge
    io_conf.pin_bit_mask = (1ULL << BUTTON_PIN);
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
}

void check_button(){
	if (gpio_get_level(BUTTON_PIN) == 0){// 按键按下
		return;
	}else {// 重复警报
		Alarm();
	}
}



void app_main(void)
{
	buzzer_init();
	button_init();
	initialize_gpio_pins();
    //Initialize NVS
	Start_upALA();
	digit_display(10);
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    wifi_init_sta();
    xTaskCreate(tcp_server_task, "tcp_server", 4096, (void*)AF_INET, 5, NULL);
}
