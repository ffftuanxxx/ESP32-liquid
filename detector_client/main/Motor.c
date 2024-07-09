#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "Motor.h"




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
