#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "NeoPixel.hpp"
#include "MPU6500.hpp"
#include "PCA9632.hpp"
#include "Buzzer.hpp"
#include "Motor.hpp"

extern "C" void app_main(void)
{
    spi_bus_config_t bus_imu_adc;
    memset(&bus_imu_adc, 0, sizeof(bus_imu_adc));
    bus_imu_adc.miso_io_num = GPIO_NUM_2;
    bus_imu_adc.mosi_io_num = GPIO_NUM_3;
    bus_imu_adc.sclk_io_num = GPIO_NUM_4;
    bus_imu_adc.quadwp_io_num = -1;
    bus_imu_adc.quadhd_io_num = -1;
    bus_imu_adc.max_transfer_sz = 16;
    bus_imu_adc.intr_flags = 0;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_imu_adc, SPI_DMA_CH_AUTO));

    MPU6500 imu(SPI2_HOST, GPIO_NUM_14);

    i2c_config_t led_conf;
    memset(&led_conf, 0, sizeof(led_conf));
    led_conf.mode = I2C_MODE_MASTER;
    led_conf.sda_io_num = GPIO_NUM_38;
    led_conf.scl_io_num = GPIO_NUM_39;
    led_conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    led_conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    led_conf.master.clk_speed = 1000000;
    led_conf.clk_flags = 0;

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &led_conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

    PCA9632 led(I2C_NUM_0, 0x62);

    led.set(0b1001);

    BUZZER buzzer(GPIO_NUM_13);
    static BUZZER::buzzer_score_t pc98[] = {
        {2000,100},{1000,100}
    };
    buzzer.play_melody(pc98, 2);

    NeoPixel np(GPIO_NUM_15, 1);
    np.set_hsv({0, 0, 0}, 0, 1);
    np.show();

    Motor motor(GPIO_NUM_41, GPIO_NUM_42, GPIO_NUM_45, GPIO_NUM_46, GPIO_NUM_11, GPIO_NUM_40);

    uint32_t h = 0;
    float t = 0.0;
    while (1)
    {
        h = imu.accelZ() * 360;
        np.set_hsv({h, 100, 10}, 0, 1);
        np.show();

        motor.setMotorSpeed(1.0 * sin(t), 1.0 * sin(t));
        t = t + 0.01;
        if (t > 2 * M_PI)
            t = 0.0;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
