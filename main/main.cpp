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
extern "C"
{
#include "nimble-nordic-uart.h"
}

#include "main.hpp"
#include "sdkconfig.h"

QueueHandle_t notify;
QueueHandle_t routemap;
const char *base_path = "/storage";
static wl_handle_t wl_handle = WL_INVALID_HANDLE;

void control_1ms_task(void *pvparam)
{
    driver_t *driver = (driver_t *)pvparam;

    odometry_t odom = {0, 0, 0};
    float vel = 0;
    float angvel = 0;
    uint16_t past_encL = driver->encL->readAngle();
    uint16_t past_encR = driver->encR->readAngle();

    float integral_velError = 0;
    float integral_angvelError = 0;
    float past_vel = 0;
    float past_angvel = 0;

    const uint16_t RESOLUTION = 0x4000;
    const uint16_t HALF_RES = RESOLUTION / 2;
    const float TIRE_DIAM = 0.01368;
    const float PULSE2RAD = TIRE_DIAM * M_PI / RESOLUTION;

    float voltage = 4.0;
    voltage_t volt = {0, 0};
    voltage_t ff_volt = {0, 0};

    notify_t notifyMsg;

    float ramp_tgt_vel = 0;
    float ramp_tgt_angvel = 0;

    float xi = 0;
    uint8_t direction = NORTH;
    float dxPast = 0;
    float dyPast = 0;
    float _origin_x = 0;
    float _origin_y = 0;
    int tickTraject = 0;

    float Kx1 = 10;
    float Kx2 = 10;
    float Ky1 = 10;
    float Ky2 = 10;

    driver->tick = 0;
    routemap_t nextRoute;
    nextRoute.path = NULL;

    while (1)
    {
        if (driver->enable == false)
        {
            driver->motor->setMotorSpeed(0, 0);
            integral_angvelError = 0;
            integral_velError = 0;
            past_vel = 0;
            past_angvel = 0;
            vel = 0;
            angvel = 0;
            ramp_tgt_vel = 0;
            ramp_tgt_angvel = 0;
            driver->tick = 0;
            driver->tgt_vel = 0;
            driver->tgt_angvel = 0;
            _origin_x = 0;
            _origin_y = 0;
            xi = 0;
            dxPast = 0;
            dyPast = 0;
            tickTraject = 0;
            nextRoute.path = NULL;
            direction = NORTH;
            xQueueReset(routemap);
            xQueueReset(notify);

            memset(&odom, 0, sizeof(odometry_t));
            while (driver->enable == false)
            {
                vTaskDelay(1);
            }
        }

        /*
        if(fabs(vel) >= 2.0 || fabs(angvel) >= 30.0)
        {
            driver->enable = false;
        }
        */

        switch (driver->mode)
        {
        case GENERAL:
            break;
        case TRAJECT:
            if (nextRoute.path == NULL)
            {
                if (xQueueReceive(routemap, &nextRoute, 0) != pdTRUE)
                {
                    driver->enable = false;
                    break;
                }
            }

            orbit_t _ob = nextRoute.path->orbit[tickTraject];
            orbit_t _obFuture = nextRoute.path->orbit[tickTraject + 1];

            float ob_x = _origin_x;
            float ob_y = _origin_y;
            float obFuture_x = _origin_x;
            float obFuture_y = _origin_y;

            float reverse = 1;
            if (nextRoute.isReverse)
                reverse = -1;

            switch (direction)
            {
            default:
            case NORTH:
                ob_x += _ob.x;
                ob_y += _ob.y * reverse;
                obFuture_x += _obFuture.x;
                obFuture_y += _obFuture.y * reverse;
                break;
            case EAST:
                ob_x += _ob.y * reverse;
                ob_y -= _ob.x;
                obFuture_x += _obFuture.y * reverse;
                obFuture_y -= _obFuture.x;
                break;
            case SOUTH:
                ob_x -= _ob.x;
                ob_y -= _ob.y * reverse;
                obFuture_x -= _obFuture.x;
                obFuture_y -= _obFuture.y * reverse;
                break;
            case WEST:
                ob_x -= _ob.y * reverse;
                ob_y += _ob.x;
                obFuture_x -= _obFuture.y * reverse;
                obFuture_y += _obFuture.x;
                break;
            }

            float _dxTgt = (obFuture_x - ob_x) / 0.001;
            float _dyTgt = (obFuture_y - ob_y) / 0.001;
            float _ddxTgt = (_dxTgt - dxPast) / 0.001;
            float _ddyTgt = (_dyTgt - dyPast) / 0.001;

            float _dx = vel * cos(odom.yaw);
            float _dy = vel * sin(odom.yaw);

            float ux = _ddxTgt + Kx1 * (_dxTgt - _dx) + Kx2 * (ob_x - odom.x);
            float uy = _ddyTgt + Ky1 * (_dyTgt - _dy) + Ky2 * (ob_y - odom.y);
            float dxi = ux * cos(odom.yaw) + uy * sin(odom.yaw);

            dxPast = _dxTgt;
            dyPast = _dyTgt;

            xi += dxi * 0.001;

            driver->tgt_vel = xi;
            if (xi > 0.01)
                driver->tgt_angvel = (uy * cos(odom.yaw) - ux * sin(odom.yaw)) / xi;
            else
                driver->tgt_angvel = 0;

            // ESP_LOGI("traject", "%.3f %.3f %.3f %.3f %.3f", ux, uy, sin(odom.yaw), cos(odom.yaw), tracking.xi);
            notifyMsg.direction = direction;
            tickTraject++;
            if (tickTraject >= nextRoute.path->size - 1)
            {
                tickTraject = 0;
                _origin_x = obFuture_x;
                _origin_y = obFuture_y;
                if (nextRoute.isReverse)
                {
                    direction = (direction - nextRoute.path->turn + 4) % 4;
                }
                else
                {
                    direction = (direction + nextRoute.path->turn + 4) % 4;
                }
                nextRoute.path = NULL;
            }
            break;
        }

        // エンコーダーの値を取得
        uint16_t encL = driver->encL->readAngle();
        uint16_t encR = driver->encR->readAngle();

        int16_t diff_encL = encL - past_encL;
        int16_t diff_encR = encR - past_encR;

        past_encL = encL;
        past_encR = encR;

        // 速度計算
        if (diff_encL > HALF_RES)
            diff_encL -= RESOLUTION;
        if (diff_encR > HALF_RES)
            diff_encR -= RESOLUTION;
        if (diff_encL < -HALF_RES)
            diff_encL += RESOLUTION;
        if (diff_encR < -HALF_RES)
            diff_encR += RESOLUTION;

        float lenL = diff_encL * PULSE2RAD;
        float lenR = diff_encR * PULSE2RAD;

        float velL = lenL / 0.001;
        float velR = lenR / 0.001;

        vel = (velL + velR) / 2;

        // 角度計算
        angvel = (driver->imu->gyroZ() * (M_PI / 180.));
        odom.yaw += angvel * 0.001;

        odom.x += vel * cos(odom.yaw) * 0.001;
        odom.y += vel * sin(odom.yaw) * 0.001;

        // モーター制御
        // ステップで渡されるターゲット速度と角度をランプ入力にする
        float _angaccel = 0;
        float _accel = 0;

        float _demand_accel = (driver->tgt_vel - ramp_tgt_vel) / 0.001;
        float _demand_angaccel = (driver->tgt_angvel - ramp_tgt_angvel) / 0.001;

        if (_demand_accel > driver->accelMax)
            _accel = driver->accelMax * 0.001;
        else if (_demand_accel < -driver->accelMax)
            _accel = -driver->accelMax * 0.001;
        else
            ramp_tgt_vel = driver->tgt_vel;

        if (_demand_angaccel > driver->angaccelMax)
            _angaccel = driver->angaccelMax * 0.001;
        else if (_demand_angaccel < -driver->angaccelMax)
            _angaccel = -driver->angaccelMax * 0.001;
        else
            ramp_tgt_angvel = driver->tgt_angvel;

        if (_accel != 0)
            ramp_tgt_vel += _accel;
        if (_angaccel != 0)
            ramp_tgt_angvel += _angaccel;

        // FF制御
        ff_volt.voltageL = (_accel - _angaccel * 0.0105) * driver->ff_gain;
        ff_volt.voltageR = (_accel + _angaccel * 0.0105) * driver->ff_gain;

        // PI-D制御
        float velError = ramp_tgt_vel - vel;
        float diff_vel = (past_vel - vel) / 0.001;
        integral_velError += velError * 0.001;

        float angvelError = angvel - ramp_tgt_angvel;
        float diff_angvel = (past_angvel - angvel) / 0.001;
        integral_angvelError += angvelError * 0.001;

        pid_gain_t vel_gain = driver->vel_gain;
        pid_gain_t angvel_gain = driver->angvel_gain;

        volt.voltageL = vel_gain.kp * velError + vel_gain.ki * integral_velError + vel_gain.kd * diff_vel;
        volt.voltageR = vel_gain.kp * velError + vel_gain.ki * integral_velError + vel_gain.kd * diff_vel;

        volt.voltageL -= angvel_gain.kp * angvelError + angvel_gain.ki * integral_angvelError + angvel_gain.kd * diff_angvel;
        volt.voltageR += angvel_gain.kp * angvelError + angvel_gain.ki * integral_angvelError + angvel_gain.kd * diff_angvel;

        past_vel = vel;
        past_angvel = angvel;

        // データをキューに送信
        notifyMsg.output_voltage = volt;
        notifyMsg.ff_voltage = ff_volt;
        notifyMsg.odom = odom;
        notifyMsg.tgt_vel = driver->tgt_vel;
        notifyMsg.tgt_angvel = driver->tgt_angvel;
        xQueueSend(notify, &notifyMsg, 0);

        driver->motor->setMotorSpeed((volt.voltageL + ff_volt.voltageL) / voltage, (volt.voltageR + ff_volt.voltageR) / voltage);

        driver->tick++;
        vTaskDelay(1);
    }
}

driver_t driver;

inline void parse_command(char *input, char *output, int size)
{
    for (int i = 0; i < size; i++)
    {
        if (input[i] == ' ' || input[i] == '\0' || input[i] == '\n')
        {
            memcpy(output, input, i);
            output[i] = '\0';
            return;
        }
    }
    output[0] = '\0';
}

static void onRecieved(struct ble_gatt_access_ctxt *ctxt)
{
    char buf[64];
    char *recv = (char *)ctxt->om->om_data;

    char command[16] = {0};
    char option[16] = {0};
    char argument[16] = {0};

    parse_command(recv, command, sizeof(command));
    recv += strlen(command) + 1;
    parse_command(recv, option, sizeof(option));
    recv += strlen(option) + 1;
    parse_command(recv, argument, sizeof(argument));

    if (memcmp(command, "run", 3) == 0)
    {
        if (memcmp(option, "traject", 7) == 0)
        {
            routemap_t route;
            route.path = &driver.start;
            route.isReverse = false;
            xQueueSend(routemap, &route, 0);
            route.path = &driver.slalom;
            xQueueSend(routemap, &route, 0);
            route.path = &driver.straight;
            xQueueSend(routemap, &route, 0);
            route.path = &driver.slalom;
            xQueueSend(routemap, &route, 0);
            route.path = &driver.stop;
            xQueueSend(routemap, &route, 0);

            driver.mode = TRAJECT;
            nordic_uart_sendln("traject");
        }
        else
        {
            driver.mode = GENERAL;
        }
        driver.enable = true;
        nordic_uart_sendln("run");
        return;
    }
    else if (memcmp(command, "stop", 4) == 0)
    {
        driver.enable = false;
        nordic_uart_sendln("stop");
        return;
    }

    if (memcmp(command, "radio", 5) == 0)
    {
        int _vel = atoi(option);
        int _angvel = atoi(argument);
        driver.tgt_vel = (_vel - 50) / 50.0 * 0.2;
        driver.tgt_angvel = (_angvel - 50) / 50.0 * 2.0;
        return;
    }

    if (driver.enable)
    {
        nordic_uart_sendln("stop before setting");
        return;
    }

    if (memcmp(command, "push", 4) == 0)
    {
        nvs_handle nvsHandle;
        esp_err_t ret = nvs_open("setting", NVS_READWRITE, &nvsHandle);
        if (ret == ESP_OK)
        {
            nvs_set_blob(nvsHandle, "vel_gain", &driver.vel_gain, sizeof(pid_gain_t));
            nvs_set_blob(nvsHandle, "angvel_gain", &driver.angvel_gain, sizeof(pid_gain_t));
            nvs_commit(nvsHandle);
            nvs_close(nvsHandle);
            nordic_uart_sendln("success");
        }
        else
        {
            nordic_uart_sendln("failed");
        }
        return;
    }
    else if (memcmp(command, "pull", 4) == 0)
    {
        nvs_handle nvsHandle;
        esp_err_t ret = nvs_open("setting", NVS_READONLY, &nvsHandle);
        if (ret == ESP_OK)
        {
            size_t size = sizeof(pid_gain_t);
            nvs_get_blob(nvsHandle, "vel_gain", &driver.vel_gain, &size);
            nvs_get_blob(nvsHandle, "angvel_gain", &driver.angvel_gain, &size);
            nvs_close(nvsHandle);
            nordic_uart_sendln("success");
        }
        else
        {
            nordic_uart_sendln("failed");
        }
        return;
    }
    else if (memcmp(command, "show", 4) == 0)
    {
        sprintf(buf, "vel_gain: %.3f %.3f %.3f", driver.vel_gain.kp, driver.vel_gain.ki, driver.vel_gain.kd);
        nordic_uart_sendln(buf);
        sprintf(buf, "angvel_gain: %.3f %.3f %.3f", driver.angvel_gain.kp, driver.angvel_gain.ki, driver.angvel_gain.kd);
        nordic_uart_sendln(buf);
        return;
    }

    if (memcmp(command, "vel", 3) == 0)
    {
        if (memcmp(option, "kp", 2) == 0)
        {
            driver.vel_gain.kp = atof(argument);
        }
        else if (memcmp(option, "ki", 2) == 0)
        {
            driver.vel_gain.ki = atof(argument);
        }
        else if (memcmp(option, "kd", 2) == 0)
        {
            driver.vel_gain.kd = atof(argument);
        }
        else if (memcmp(option, "vel", 3) == 0)
        {
            driver.tgt_vel = atof(argument);
        }
        else
        {
            nordic_uart_sendln("invalid option");
            return;
        }
        nordic_uart_sendln("success");
        return;
    }
    else if (memcmp(command, "angvel", 6) == 0)
    {
        if (memcmp(option, "kp", 2) == 0)
        {
            driver.angvel_gain.kp = atof(argument);
        }
        else if (memcmp(option, "ki", 2) == 0)
        {
            driver.angvel_gain.ki = atof(argument);
        }
        else if (memcmp(option, "kd", 2) == 0)
        {
            driver.angvel_gain.kd = atof(argument);
        }
        else if (memcmp(option, "vel", 3) == 0)
        {
            driver.tgt_angvel = atof(argument);
        }
        else
        {
            nordic_uart_sendln("invalid option");
            return;
        }
        nordic_uart_sendln("success");
        return;
    }
    else if (memcmp(command, "ff", 2) == 0)
    {
        driver.ff_gain = atof(option);
        nordic_uart_sendln("success");
        return;
    }

    if (memcmp(command, "led", 3) == 0)
    {
        driver.led->set(atoi(option));
        nordic_uart_sendln("success");
        return;
    }
    else if (memcmp(command, "neopixel", 8) == 0)
    {
        if (memcmp(option, "off", 3) == 0)
        {
            driver.np->set_hsv({0, 0, 0}, 0, 1);
        }
        else if (memcmp(option, "hue", 3) == 0)
        {
            uint8_t hue = atoi(argument);
            driver.np->set_hsv({hue, 100, 5}, 0, 1);
        }
        driver.np->show();
        nordic_uart_sendln("success");
    }

    nordic_uart_sendln("invalid command");
}

inline void loadTraject(orbitBase_t *orbit, char *filename, Turn turn)
{
    FILE *file = fopen(filename, "r");
    if (file != NULL)
    {
        char _buf[64];
        fgets(_buf, sizeof(_buf), file);
        orbit->size = atoi(_buf);
        orbit->orbit = (orbit_t *)malloc(sizeof(orbit_t) * orbit->size);
        ESP_LOGI("FAT", "orbit_size: %d", orbit->size);
        if (orbit->orbit == NULL)
        {
            ESP_LOGE("FAT", "malloc failed");
        }
        else
        {
            for (int i = 0; i < orbit->size; i++)
            {
                fgets(_buf, sizeof(_buf), file);
                char *p = strtok(_buf, ",");
                orbit->orbit[i].x = atof(p) / 1000;
                p = strtok(NULL, ",");
                orbit->orbit[i].y = atof(p) / 1000;
            }
            ESP_LOGI("FAT", "traject loaded");
            orbit->turn = turn;
        }
    }
    else
    {
        ESP_LOGE("FAT", "Failed to open %s", filename);
        orbit->orbit = NULL;
        orbit->size = 0;
    }
}

extern "C" void app_main(void)
{
    esp_err_t ret;

    // NVSの初期化
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    const esp_vfs_fat_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 10,
        .allocation_unit_size = CONFIG_WL_SECTOR_SIZE,
        .disk_status_check_enable = true,
    };

    ret = esp_vfs_fat_spiflash_mount_rw_wl(base_path, "storage", &mount_config, &wl_handle);
    ESP_ERROR_CHECK(ret);

    // PIDゲインの設定
    nvs_handle nvsHandle;
    ret = nvs_open("setting", NVS_READONLY, &nvsHandle);
    size_t size = sizeof(pid_gain_t);
    if (ret == ESP_OK)
    {
        nvs_get_blob(nvsHandle, "vel_gain", &driver.vel_gain, &size);
        nvs_get_blob(nvsHandle, "angvel_gain", &driver.angvel_gain, &size);
        nvs_close(nvsHandle);
    }
    else
    {
        driver.vel_gain = {0.0, 0.0, 0.0};
        driver.angvel_gain = {0.0, 0.0, 0.0};
    }

    // BLEの初期化
    nordic_uart_start("reRoMouse", NULL);
    nordic_uart_yield(onRecieved);

    // IMU SPIバスの設定
    spi_bus_config_t bus_imu_adc;
    memset(&bus_imu_adc, 0, sizeof(bus_imu_adc));
    bus_imu_adc.miso_io_num = GPIO_NUM_2;
    bus_imu_adc.mosi_io_num = GPIO_NUM_3;
    bus_imu_adc.sclk_io_num = GPIO_NUM_4;
    bus_imu_adc.quadwp_io_num = -1;
    bus_imu_adc.quadhd_io_num = -1;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_imu_adc, SPI_DMA_CH_AUTO));

    driver.imu = new MPU6500(SPI2_HOST, GPIO_NUM_14);

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

    driver.encR = new MA730(SPI3_HOST, GPIO_NUM_1, 0);
    driver.encL = new MA730(SPI3_HOST, GPIO_NUM_6, 1);

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

    driver.led = new PCA9632(I2C_NUM_0, 0x62);
    driver.led->set(0b0000);

    // Buzzerの設定
    driver.buzzer = new BUZZER(GPIO_NUM_13);
    static BUZZER::buzzer_score_t pc98[] = {
        {2000, 100}, {1000, 100}};

    driver.buzzer->play_melody(pc98, 2);

    // NeoPixelの設定
    driver.np = new NeoPixel(GPIO_NUM_15, 1);
    driver.np->show();

    // モーター関係の設定
    notify = xQueueCreate(1, sizeof(notify_t));
    driver.motor = new Motor(GPIO_NUM_41, GPIO_NUM_42, GPIO_NUM_45, GPIO_NUM_46, GPIO_NUM_11, GPIO_NUM_40);

    driver.np->set_hsv({0, 0, 0}, 0, 1);
    driver.np->show();

    driver.enable = false;
    xTaskCreatePinnedToCore(control_1ms_task, "control_1ms_task", 8192, &driver, configMAX_PRIORITIES - 1, NULL, APP_CPU_NUM);
    driver.tgt_vel = 0;
    driver.tgt_angvel = 0;

    char buf[64];

    // 軌道データの読み込み
    routemap = xQueueCreate(5, sizeof(routemap_t));
    loadTraject(&driver.slalom, "/storage/slalom90.csv", LEFT);
    loadTraject(&driver.start, "/storage/start.csv", STRAIGHT);
    loadTraject(&driver.stop, "/storage/stop.csv", STRAIGHT);
    loadTraject(&driver.straight, "/storage/straight.csv", STRAIGHT);

    while (1)
    {
        if (driver.enable)
        {
            notify_t notifyMsg;
            xQueueReceive(notify, &notifyMsg, portMAX_DELAY);
            sprintf(buf, "%3.3f %3.3f %d", notifyMsg.odom.x, notifyMsg.odom.y, notifyMsg.direction);
            nordic_uart_sendln(buf);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        else
        {
            vTaskDelay(1);
        }
    }
}
