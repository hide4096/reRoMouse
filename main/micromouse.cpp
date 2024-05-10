#include "include/Interrupt.hpp"
#include "include/UI/fast.hpp"
#include "include/UI/log.hpp"
#include "include/UI/search.hpp"
#include "include/UI/test.hpp"
#include "include/Motion/Adachi.hpp"
#include "include/micromouse.hpp"
#include <functional>

std::vector<std::shared_ptr<UI>> ui;

void MICROMOUSE(MCP3464 &adc, MA730 &enc_R, MA730 &enc_L, BUZZER &buzzer, MPU6500 &imu, PCA9632 &led, Motor &motor);
void set_interface();
void call_task(UI *task, Adachi &motion);
void set_param(Micromouse *task, t_sens_data *_sen, t_mouse_motion_val *_val, t_control *_control, t_map *_map);
void mode_select(uint8_t *_mode_num, Adachi &adachi, t_sens_data *sens, t_mouse_motion_val *val, t_control *control, t_map *map);

void myTaskInterrupt(void *pvpram)
{
    Interrupt *interrupt = static_cast<Interrupt *>(pvpram);
    interrupt->interrupt();
}

/*void myTaskAdc(void *pvpram)
{
    MCP3464 *adc = static_cast<MCP3464 *>(pvpram);
    adc->adc_loop();
}*/

void myTaskLog(void *pvpram)
{
    Interrupt *log = static_cast<Interrupt *>(pvpram);
    log->logging();
}

/* 基本的に全ての処理のをここにまとめ、mainで呼び出す。 */

void MICROMOUSE(MCP3464 &adc, MA730 &enc_R, MA730 &enc_L, BUZZER &buzzer, MPU6500 &imu, PCA9632 &led, Motor &motor)
{
    // printf("start MICROMOUSE\n");

    /* 構造体のインスタンス生成 */
    t_sens_data sens;
    t_mouse_motion_val val;
    t_control control;
    t_map map;
    t_file_pid_gain pid_gain;
    t_file_wall_th wall_threshold;
    t_file_center_sens_value center_sens_val;

    printf("finish struct\n");

    /* ログ取得用ハンドルの設定 */
    SemaphoreHandle_t on_logging = xSemaphoreCreateBinary();

    /* ポインタの設定・構造体の共有 */

    // 制御系
    Interrupt interrupt;
    interrupt.set_device(adc, enc_R, enc_L, buzzer, imu, led, motor);
    interrupt.ptr_by_sensor(&sens);
    interrupt.ptr_by_motion(&val);
    interrupt.ptr_by_control(&control);
    interrupt.ptr_by_map(&map);
    interrupt.GetSemphrHandle(&on_logging);

    printf("finish interrupt struct\n");

    // モーション系
    Adachi motion;
    motion.set_device(adc, enc_R, enc_L, buzzer, imu, led, motor);
    motion.ptr_by_sensor(&sens);
    motion.ptr_by_motion(&val);
    motion.ptr_by_control(&control);
    motion.ptr_by_map(&map);
    motion.GetSemphrHandle(&on_logging);

    printf("finish motion struct\n");

    // センサ系
    //adc.GetData(&sens);
    imu.GetData(&sens);
    enc_R.GetData(&sens);
    enc_L.GetData(&sens);

    printf("finish sensor struct\n");

    /* パラメータの設定 */
    //pid_gain = read_file_pid();
    //wall_threshold = read_file_wall_th();
    //center_sens_val = read_file_center_sens_val();

    // 距離
    val.tar.len = 0.09;
    val.tar.len_half = 0.045;

    // 角度
    val.tar.rad = M_PI/2.0;

    // 速度
    //val.tar.acc = 0.5;
    val.max.acc = 1.0;
    //val.tar.vel = 0.3;
    val.max.vel = 0.3;
    val.min.vel = 0.05;
    val.end.vel = 0.3;

    // 角速度
    val.tar.ang_acc = 0.0;
    val.max.ang_acc = M_PI*5.0;
    val.tar.ang_vel = 0.0;
    val.max.ang_vel = M_PI;
    val.min.ang_vel = M_PI/5.0;
    val.end.ang_vel = 0.0;

    // 速度制御
    //control.v.Kp = pid_gain.speed_Kp;
    //control.v.Ki = pid_gain.speed_Ki;
    //control.v.Kd = pid_gain.speed_Kd;
    control.v.Kp = 5.0;
    control.v.Ki = 500.0;
    control.v.Kd = 0.0;

    // 角速度制御
    //control.o.Kp = pid_gain.ang_vel_Kp;
    //control.o.Ki = pid_gain.ang_vel_Ki;
    //control.o.Kd = pid_gain.ang_vel_Kd;
    control.o.Kp = 0.50;
    control.o.Ki = 100.0;
    control.o.Kd = 0.0;

    // 壁制御
    //control.wall.Kp = pid_gain.wall_Kp;
    //control.wall.Ki = pid_gain.wall_Ki;
    //control.wall.Kd = pid_gain.wall_Kd;
    control.wall.Kp = 0.005;
    control.wall.Ki = 0.0;
    control.wall.Kd = 0.0;

    // 壁センサ閾値
    //sens.wall.th_wall.fl = wall_threshold.th_wall_fl;
    //sens.wall.th_wall.fr = wall_threshold.th_wall_fr;
    //sens.wall.th_wall.l = wall_threshold.th_wall_l;
    //sens.wall.th_wall.r = wall_threshold.th_wall_r;
    //sens.wall.th_control.l = wall_threshold.th_control_l;
    //sens.wall.th_control.r = wall_threshold.th_control_r;
    //sens.wall.ref.l = wall_threshold.ref_l;
    //sens.wall.ref.r = wall_threshold.ref_r;
    sens.wall.th_wall.fl = 42;
    sens.wall.th_wall.fr = 47;
    sens.wall.th_wall.l = 42;
    sens.wall.th_wall.r = 55;
    sens.wall.th_control.l = 100;
    sens.wall.th_control.r = 100;
    sens.wall.ref.l = 141;
    sens.wall.ref.r = 170;

    // ゴール座標
    map.GOAL_X = 4;
    map.GOAL_Y = 4;

    printf("finish parameter\n");
    // タスク優先順位 1 ~ 25    25が最高優先度
    xTaskCreatePinnedToCore(myTaskInterrupt,
                            "interrupt", 8192, &interrupt, configMAX_PRIORITIES, NULL, PRO_CPU_NUM);
    /*xTaskCreatePinnedToCore(myTaskAdc,
                            "adc", 8192, &adc, configMAX_PRIORITIES - 1, NULL, APP_CPU_NUM);*/
    xTaskCreatePinnedToCore(myTaskLog,
                            "log", 8192, &interrupt, configMAX_PRIORITIES - 2, NULL, APP_CPU_NUM);
    printf("finish task\n");

    /*char buffer[512];
    vTaskList(buffer);
    printf("Task execution statistics:\n%s", buffer);*/

    uint8_t mode = 0;
    uint16_t time_count = 0;
    const int MODE_MAX = 0b1111;
    const int MODE_MIN = 0;

    /* メインループ */
    while (1)
    {

        led.set(mode + 1);

        /*vTaskList(buffer);
        printf("Task execution statistics:\n%s", buffer);*/

        /*if (sens.wall.val.fl + sens.wall.val.l + sens.wall.val.r + sens.wall.val.fr > 3000)
        {

            led.set(0b1111);
            sens.gyro.ref = imu.surveybias(2000);
            mode_select(&mode, motion, &sens, &val, &control, &map);
            control.flag = FALSE;
        }*/
        if (time_count > 3000)
        {
            led.set(0b1111);
            sens.gyro.ref = imu.surveybias(2000);
            mode_select(&mode, motion, &sens, &val, &control, &map);
            control.flag = FALSE;
            time_count = 0;
        }

        if (val.current.vel > 0.04)
        {
            if (mode >= MODE_MAX)
            {
                mode = MODE_MIN;
            }
            else
            {
                mode++;
            }
            time_count = 0;
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        if (val.current.vel < -0.04)
        {
            if (mode <= MODE_MIN)
            {
                mode = MODE_MAX;
            }
            else
            {
                mode--;
            }
            time_count = 0;
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        //printf("time:%d\n", control.time_count);
        //printf("vel:%f\n", val.current.vel);
        //printf("rad:%f\n", val.current.rad);
        //printf("BatteryVoltage:%f\n", sens.BatteryVoltage);
        //printf("sens.wall.val.fl:%d sens.wall.val.l:%d sens.wall.val.r:%d sens.wall.val.fr:%d\n", sens.wall.val.fl, sens.wall.val.l, sens.wall.val.r, sens.wall.val.fr);
        time_count++;
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
    //vTaskDelay(pdMS_TO_TICKS(10));
}

void set_interface()
{
    /* クラスのポインタを配列に保持*/

    ui.push_back(std::make_shared<Search>());
    ui.push_back(std::make_shared<All_Search>());
    ui.push_back(std::make_shared<Fast>());
    ui.push_back(std::make_shared<Fast2>());
    ui.push_back(std::make_shared<Fast3>());
    ui.push_back(std::make_shared<Fast4>());
    ui.push_back(std::make_shared<Test>());
    ui.push_back(std::make_shared<Test2>());
    ui.push_back(std::make_shared<Test3>());
    ui.push_back(std::make_shared<Test4>());
    ui.push_back(std::make_shared<Test5>());
    ui.push_back(std::make_shared<Test6>());
    ui.push_back(std::make_shared<Test7>());
    ui.push_back(std::make_shared<Log>());
    ui.push_back(std::make_shared<Log1>());

    //std::cout << "set_interface" << std::endl;
}

void call_task(UI *task, Adachi &motion)
{
    task->ref_by_motion(motion);
    task->main_task();
    //std::cout << "call_task" << std::endl;
}

void set_param(Micromouse *task, t_sens_data *_sen, t_mouse_motion_val *_val, t_control *_control, t_map *_map)
{
    task->ptr_by_sensor(_sen);
    task->ptr_by_motion(_val);
    task->ptr_by_control(_control);
    task->ptr_by_map(_map);
    //std::cout << "set_param" << std::endl;
}

void mode_select(uint8_t *_mode_num, Adachi &adachi, t_sens_data *sens, t_mouse_motion_val *val, t_control *control, t_map *map)
{
    set_interface();
    set_param(ui[*_mode_num].get(), sens, val, control, map);
    call_task(ui[*_mode_num].get(), adachi);
    //std::cout << "mode_select" << std::endl;
}
