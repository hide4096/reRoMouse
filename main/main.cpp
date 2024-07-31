#include <iostream>
#include <memory>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/spi_master.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "structs.hpp"
#include "drivers.hpp"
#include "micromouse.hpp"

std::shared_ptr<t_drivers> driver = std::make_shared<t_drivers>();

static SemaphoreHandle_t wallCharged;

static void timer_chargeCompleted(void *arg)
{
    BaseType_t _highPriorityTask = pdFALSE;
    xSemaphoreGiveFromISR(wallCharged, &_highPriorityTask);
    portYIELD_FROM_ISR(_highPriorityTask);
}

void myTaskAdc(void *pvpram)
{
    // pvpram が指す既存の ADS7066 ポインタを std::shared_ptr に変換
    ADS7066* raw_adc_ptr = static_cast<ADS7066 *>(pvpram);
    std::shared_ptr<ADS7066> adc(raw_adc_ptr);

    // t_drivers 構造体を作成し、adc ポインタを設定
    std::shared_ptr<t_drivers> driver = std::make_shared<t_drivers>();
    driver->adc = adc;

    ESP_LOGI("ADC", "ADC Task Start");

    for (int i = 0; i < 4; i++)
    {
        gpio_set_level(driver->adc->LED[i], 1);
    }
    driver->adc->_off = driver->adc->readOnTheFly(4);

    esp_timer_create_args_t chargeTimerSetting = {
        .callback = &timer_chargeCompleted,
        .name = "wallCharge"};
    esp_timer_handle_t chargeTimer;
    ESP_ERROR_CHECK(esp_timer_create(&chargeTimerSetting, &chargeTimer));

    wallCharged = xSemaphoreCreateBinary();

    uint16_t charge_us = 60;
    uint16_t rise_us = 15;

    std::shared_ptr<t_sens_data> sens = std::make_shared<t_sens_data>();

    while(1)
    {
        sens->BatteryVoltage = driver->adc->BatteryVoltage();
        for (int i = 0; i < 4; i++)
        {
            if (i > 0)
            {
                driver->adc->_on = driver->adc->readOnTheFly(driver->adc->SENS[i]);
                gpio_set_level(driver->adc->LED[i], 0);
                esp_timer_start_once(chargeTimer, charge_us);
                xSemaphoreTake(wallCharged, portMAX_DELAY);
                gpio_set_level(driver->adc->LED[i], 1);
                esp_rom_delay_us(rise_us);
                if (i > 0)
                    driver->adc->value[i - 1] = driver->adc->_on - driver->adc->_off;
                driver->adc->_off = driver->adc->readOnTheFly(driver->adc->SENS[i]);
            }
                
        }
        driver->adc->_on = driver->adc->readOnTheFly(4);
        driver->adc->value[3] = driver->adc->_on - driver->adc->_off;

        sens->wall.val.fr = driver->adc->value[0];
        sens->wall.val.l = driver->adc->value[2];
        sens->wall.val.r = driver->adc->value[1];
        sens->wall.val.fl = driver->adc->value[3];

        vTaskDelay(1 /portTICK_PERIOD_MS);
    }
}

extern "C" void app_main(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =
        (1ULL << 10) | (1ULL << 17) | (1ULL << 18) | (1ULL << 21);
    io_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;

    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // IMU SPIバスの設定
    spi_bus_config_t bus_imu_adc;
    memset(&bus_imu_adc, 0, sizeof(bus_imu_adc));
    bus_imu_adc.miso_io_num = GPIO_NUM_4;
    bus_imu_adc.mosi_io_num = GPIO_NUM_2;
    bus_imu_adc.sclk_io_num = GPIO_NUM_3;
    bus_imu_adc.quadwp_io_num = -1;
    bus_imu_adc.quadhd_io_num = -1;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_imu_adc, SPI_DMA_CH_AUTO));

    driver->adc = std::make_shared<ADS7066>(SPI2_HOST, GPIO_NUM_5);
    driver->imu = std::make_shared<MPU6500>(SPI2_HOST, GPIO_NUM_14);

    // Encoder SPIバスの設定
    spi_bus_config_t bus_enc;
    memset(&bus_enc, 0, sizeof(bus_enc));
    bus_enc.mosi_io_num = GPIO_NUM_9;
    bus_enc.miso_io_num = GPIO_NUM_8;
    bus_enc.sclk_io_num = GPIO_NUM_7;
    bus_enc.quadwp_io_num = -1;
    bus_enc.quadhd_io_num = -1;
    bus_enc.max_transfer_sz = 4;
    bus_enc.flags = SPICOMMON_BUSFLAG_MASTER;
    bus_enc.intr_flags = 0;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus_enc, SPI_DMA_DISABLED));

    driver->encL = std::make_shared<MA730>(SPI3_HOST, GPIO_NUM_6, 1);
    driver->encR = std::make_shared<MA730>(SPI3_HOST, GPIO_NUM_1, 0);

    // LED driver I2Cバスの設定
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

    driver->led = std::make_shared<PCA9632>(I2C_NUM_0, 0x62);

    driver->led->set(0b1111);

    // Buzzer GPIOの設定
    driver->bz = std::make_shared<BUZZER>(GPIO_NUM_13);
    static BUZZER::buzzer_score_t pc98[] = {
        {2000,100},{1000,100}
    };
    driver->bz->play_melody(pc98, 2);

    // NeoPixel GPIOの設定
    driver->np = std::make_shared<NeoPixel>(GPIO_NUM_15, 1);
    driver->np->set_hsv({0, 0, 0}, 0, 1);
    driver->np->show();

    // Motor driver Fan Moter GPIOの設定
    driver->mot = std::make_shared<Motor>(GPIO_NUM_41, GPIO_NUM_42, GPIO_NUM_45, GPIO_NUM_46, GPIO_NUM_11, GPIO_NUM_40);

    ADS7066 *adc = driver->adc.get();

    xTaskCreatePinnedToCore(myTaskAdc,
                            "adc", 8192, &adc, configMAX_PRIORITIES - 2, NULL, APP_CPU_NUM);

    //uint32_t h = 0, h1 = 0;
    //float t = 0.0;
    while (1)
    {
        MICROMOUSE(driver);

        //motor.setMotorSpeed((-0.3), -(0.3));
        /*h = imu.accelZ() * 360;
        np.set_hsv({h, 100, 10}, 0, 1);
        np.show();

        printf("Z:%ld\n", h);*/
        /*
        motor.setMotorSpeed(1.0 * sin(t), 1.0 * sin(t));
        t = t + 0.01;
        if (t > 2 * M_PI)
            t = 0.0;
        */
        
        /*h = EncL.readAngle();
        h1 = EncR.readAngle();

        float WheelAngle_L = 2.0 * M_PI * h / 16384.0;
        float WheelAngle_R = 2.0 * M_PI * h1 / 16384.0;

        float WeeelDegree_L = WheelAngle_L * 180.0 / M_PI;
        float WeeelDegree_R = WheelAngle_R * 180.0 / M_PI;

        //printf("L:%ld    R:%ld\n", h, h1);
        //printf("L:%f    R:%f\n", WheelAngle_L, WheelAngle_R);
        printf("L:%f    R:%f\n", WeeelDegree_L, WeeelDegree_R);*/
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
