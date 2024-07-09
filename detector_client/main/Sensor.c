#include <stdio.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "Sensor.h"
#include "Motor.h"


#define SENSOR_RECV_PIN GPIO_NUM_0
#define TRIGGER_THRESHOLD 4
#define TIMEOUT 300
static const char *TAG = "SENSOR";

void Alarm(){
	ESP_LOGE(TAG, "Detect timeout! (>%d)", TIMEOUT);
}

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
	        		ESP_LOGI(TAG, "detected! Last trigger: %d", counter);
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


