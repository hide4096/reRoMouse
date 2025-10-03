#include "include/Motion/Motion.hpp"

#define MODE_MAX 15
#define MODE_MIN 0
#define SECTION 0.09
#define SECTION_HALF 0.045
#define TURN_HALF M_PI
#define TURN_QUARTER M_PI / 2.0
#define OFFSET_DISTANCE 0.011
#define FRONT_WALL_LIMIT_FL 25100
#define FRONT_WALL_LIMIT_FR 14600
#define DONE 1
#define NOT_YET 0
#define PRE_DISTANCE 0.0054
#define FOL_DISTANCE 0.0051

static BUZZER::buzzer_score_t pc98[] = {{2000, 100}, {1000, 100}};
static BUZZER::buzzer_score_t pc98_2[] = {{1000, 100}, {2000, 100}};

Motion::Motion()
{ /*std::cout << "Motion" << std::endl;*/
}

Motion::~Motion() {}

void Motion::ptr_by_sensor(t_sens_data *_sens) { sens = _sens; }

void Motion::ptr_by_motion(t_mouse_motion_val *_val) { val = _val; }

void Motion::ptr_by_control(t_control *_control) { control = _control; }

void Motion::ptr_by_map(t_map *_map) { map = _map; }

void Motion::set_device_driver(std::shared_ptr<t_drivers> driver)
{
    np = driver->np;
    imu = driver->imu;
    led = driver->led;
    bz = driver->bz;
    mot = driver->mot;
    encL = driver->encL;
    encR = driver->encR;
    adc = driver->adc;
    // std::cout << "set_device_driver" << std::endl;
}

void Motion::GetSemphrHandle(SemaphoreHandle_t *_on_logging) { on_logging = _on_logging; }

void Motion::run()
{
    control->delta_run_time = 0;
    control->start_run_time = esp_timer_get_time();

    control->flag = TRUE;      // 制御ON
    sens->wall.control = TRUE; // 壁制御OFF

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    val->current.len = 0.0;
    val->tar.acc = val->max.acc;
    val->tar.len = SECTION;
    // val->tar.vel = 0.0;

    /*if (len_count == 7)
    {
        // val->tar.len = 45;
        val->tar.len = 0.045;
    }*/
    // np->set_hsv({240, 100, 100}, 0, 1);
    // np->show();
    led->set(0b0000);

    while (((val->tar.len - 0.01) - val->current.len) > (((val->tar.vel) * (val->tar.vel) - (val->end.vel) * (val->end.vel)) / (2.0 *
                                                                                                                                val->tar.acc)))
    {
        if (val->tar.len - 0.01 <= val->current.len)
        {
            break;
        }
        if (val->tar.vel >= val->max.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->max.vel;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    // val->tar.acc = -(val->max.acc);

    while ((val->tar.len - 0.001) > val->current.len)
    {
        if (val->tar.vel >= val->max.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->max.vel;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // val->tar.vel = val->tar.vel;
    val->tar.acc = 0.0;
    // np->set_hsv({0, 100, 100}, 0, 1);
    // np->show();

    // control->flag = FALSE;
    led->set(0b1111);

    control->end_run_time = esp_timer_get_time();
    control->delta_run_time = control->end_run_time - control->start_run_time;

    // std::cout << "run" << std::endl;
}

void Motion::run2()
{
    map->thinking_flag = FALSE;
    control->flag = TRUE;      // 制御ON
    sens->wall.control = TRUE; // 壁制御OFF

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    val->current.len = 0.0;
    val->tar.acc = val->max.acc;
    val->tar.len = SECTION;
    // val->tar.vel = 0.0;

    /*if (len_count == 7)
    {
        // val->tar.len = 45;
        val->tar.len = 0.045;
    }*/

    bool l_wall_check = sens->wall.exist.l;
    bool r_wall_check = sens->wall.exist.r;
    bool hosei_flag = FALSE;

    while (((val->tar.len - 0.01) - val->current.len) > (((val->tar.vel) * (val->tar.vel) - (val->end.vel) * (val->end.vel)) / (2.0 *
                                                                                                                                val->tar.acc)))
    {
        // 壁あり->壁なし
        if (sens->wall.exist.l == FALSE && l_wall_check == TRUE && hosei_flag == FALSE)
        {
            bz->play_melody(pc98, 2);
            val->current.len = 0.057;
            hosei_flag = TRUE;
        }

        if (sens->wall.exist.r == FALSE && r_wall_check == TRUE && hosei_flag == FALSE)
        {
            bz->play_melody(pc98, 2);
            val->current.len = 0.057;
            hosei_flag = TRUE;
        }

        // 壁なし->壁あり
        if (sens->wall.exist.l == TRUE && l_wall_check == FALSE && hosei_flag == FALSE)
        {
            bz->play_melody(pc98_2, 2);
            val->current.len = 0.035; // 補正後の距離を伸ばしたい場合は、値を小さく
            hosei_flag = TRUE;
        }

        if (sens->wall.exist.r == TRUE && r_wall_check == FALSE && hosei_flag == FALSE)
        {
            bz->play_melody(pc98_2, 2);
            val->current.len = 0.033;
            hosei_flag = TRUE;
        }

        if (val->tar.len - 0.01 <= val->current.len)
        {
            break;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    // val->tar.acc = -(val->max.acc);

    while ((val->tar.len - 0.001) > val->current.len)
    {
        if (val->tar.vel >= val->max.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->max.vel;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // val->tar.vel = val->tar.vel;
    val->tar.acc = 0.0;

    // control->flag = FALSE;

    map->thinking_flag = TRUE;

    // std::cout << "run" << std::endl;
}

void Motion::run_half()
{
    control->flag = TRUE;      // 制御ON
    sens->wall.control = TRUE; // 壁制御OFF

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    val->tar.len = SECTION_HALF;
    val->current.len = 0.0;
    val->tar.acc = val->max.acc;
    // val->tar.vel = 0.0;

    /*if (len_count == 7)
    {
        // val->tar.len = 45;
        val->tar.len = 0.045;
    }*/

    // 制御式に欠陥を見つけた。加速度が一定以下だと、while文を抜け出せず、走行距離が延びてしまう。
    while (((val->tar.len - 0.01) - val->current.len) > (((val->tar.vel) * (val->tar.vel) - (val->end.vel) * (val->end.vel)) / (2.0 *
                                                                                                                                val->tar.acc)))
    {
        if (val->tar.len - 0.01 <= val->current.len)
        {
            break;
        }
        if (val->tar.vel >= val->max.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->max.vel;
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    // val->tar.acc = -(val->tar.acc);

    while ((val->tar.len - 0.001) > val->current.len)
    {
        if (val->tar.vel >= val->max.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->max.vel;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // val->tar.vel = val->tar.vel;
    val->tar.acc = 0.0;
    // control->flag = FALSE;

    // std::cout << "run" << std::endl;
}

void Motion::turn_left()
{
    vTaskDelay(200);

    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF
    val->current.flag = LEFT;   // 左旋回

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;

    val->tar.ang_acc = val->max.ang_acc;
    val->max.ang_vel = val->max.ang_vel;

    val->tar.rad = TURN_QUARTER;

    // std::cout << "turn_left" << std::endl;
    int turn_count = 0;

    local_rad = val->current.rad; // 現在の角度を保存

    while (val->tar.rad > (val->current.rad - local_rad))
    {
        turn_count++;
        // printf("turn_count : %d\n", turn_count);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    control->flag = FALSE;

    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    // std::cout << "turn" << std::endl;
}

void Motion::turn_right()
{
    vTaskDelay(200);

    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF
    val->current.flag = RIGHT;  // 右旋回

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;

    val->tar.ang_acc = -(val->max.ang_acc);
    // val->max.ang_vel = -(val->max.ang_vel);

    val->tar.rad = -(TURN_QUARTER);

    // std::cout << "turn_left" << std::endl;
    // int turn_count = 0;

    local_rad = val->current.rad; // 現在の角度を保存

    while (val->tar.rad < (val->current.rad - local_rad))
    {
        // turn_count++;
        // printf("val->tar.ang_vel : %f\n", val->tar.ang_vel);
        //  printf("turn_count : %d\n", turn_count);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    control->flag = FALSE;

    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    // std::cout << "turn" << std::endl;
}

void Motion::turn_half()
{
    vTaskDelay(200);

    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF
    val->current.flag = LEFT;   // 左旋回

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;

    val->tar.ang_acc = val->max.ang_acc;
    val->max.ang_vel = val->max.ang_vel;

    val->tar.rad = TURN_HALF;

    // std::cout << "turn_left" << std::endl;
    int turn_count = 0;

    local_rad = val->current.rad; // 現在の角度を保存

    while (val->tar.rad > (val->current.rad - local_rad))
    {
        turn_count++;
        // printf("turn_count : %d\n", turn_count);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    while (val->current.ang_vel >= 0.01 || val->current.ang_vel <= -0.01)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    control->flag = FALSE;

    // std::cout << "turn" << std::endl;
}

void Motion::stop()
{
    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    val->tar.len = SECTION_HALF;
    val->current.len = 0.0;
    val->tar.acc = val->max.acc;
    // val->tar.vel = 0.0;

    // bool hosei_flag = NOT_YET;
    // uint8_t hosei_dist = 0.050;

    np->set_hsv({240, 100, 100}, 0, 1);
    np->show();

    while (((val->tar.len - 0.01) - val->current.len) > (((val->tar.vel) * (val->tar.vel)) / (2.0 *
                                                                                              val->tar.acc)))
    {
        if (val->tar.vel >= val->max.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->max.vel;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    val->tar.acc = -(val->max.acc);

    while ((val->tar.len - 0.001) > val->current.len)
    {
        if (val->tar.vel <= val->min.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->min.vel;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.acc = 0.0;
    val->tar.vel = 0.0;

    while (val->current.vel >= 0.0)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    control->flag = FALSE; // 制御OFF

    val->current.len = 0.0;

    // np->set_hsv({240, 100, 100}, 0, 1);
    // np->show();

    // std::cout << "stop" << std::endl;
}

void ::Motion::stop2()
{
    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    val->tar.len = SECTION_HALF;
    val->current.len = 0.0;
    val->tar.acc = val->max.acc;
    // val->tar.vel = 0.0;

    bool hosei_flag = NOT_YET;
    uint8_t hosei_dist = 0.050;

    while (((val->tar.len - 0.01) - val->current.len) > (((val->tar.vel) * (val->tar.vel)) / (2.0 *
                                                                                              val->tar.acc)))
    {
        if (sens->wall.exist.fl == TRUE && sens->wall.exist.fr == TRUE)
        {
            if (sens->wall.val.fl > FRONT_WALL_LIMIT_FL && sens->wall.val.fr > FRONT_WALL_LIMIT_FR)
            {
                bz->play_melody(pc98, 2);
                // control->flag = FALSE;
                break;
            }
            if (sens->wall.val.fl < FRONT_WALL_LIMIT_FL && sens->wall.val.fr < FRONT_WALL_LIMIT_FR && hosei_flag == NOT_YET)
            {
                // bz->play_melody(pc98, 2);
                // while(sens->wall.val.fl < FRONT_WALL_LIMIT_FL && sens->wall.val.fr < FRONT_WALL_LIMIT_FR){vTaskDelay(1/portTICK_PERIOD_MS);}
                val->tar.len = hosei_dist;
                // bz->play_melody(pc98, 2);
                hosei_flag = DONE;
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    val->tar.acc = -(val->max.acc);

    while ((val->tar.len - 0.001) > val->current.len)
    {
        if (val->tar.vel <= val->min.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->min.vel;
            if (sens->wall.exist.fl == TRUE && sens->wall.exist.fr == TRUE)
            {
                if (sens->wall.val.fl > FRONT_WALL_LIMIT_FL && sens->wall.val.fr > FRONT_WALL_LIMIT_FR)
                {
                    bz->play_melody(pc98, 2);
                    // control->flag = FALSE;
                    break;
                }
                /*if (sens->wall.val.fl < FRONT_WALL_LIMIT_FL && sens->wall.val.fr < FRONT_WALL_LIMIT_FR && hosei_flag == NOT_YET)
                {
                    bz->play_melody(pc98, 2);
                    //while(sens->wall.val.fl < FRONT_WALL_LIMIT_FL && sens->wall.val.fr < FRONT_WALL_LIMIT_FR){vTaskDelay(1/portTICK_PERIOD_MS);}
                    val->tar.len = hosei_dist;
                    bz->play_melody(pc98, 2);
                    hosei_flag = DONE;
                }*/
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.acc = 0.0;
    val->tar.vel = 0.0;

    while (val->current.vel >= 0.0)
    {
        if (sens->wall.exist.fl == TRUE && sens->wall.exist.fr == TRUE)
        {
            if (sens->wall.val.fl > FRONT_WALL_LIMIT_FL && sens->wall.val.fr > FRONT_WALL_LIMIT_FR)
            {
                bz->play_melody(pc98, 2);
                // control->flag = FALSE;
                break;
            }
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    control->flag = FALSE; // 制御OFF

    // std::cout << "stop" << std::endl;
}

void Motion::back()
{
    vTaskDelay(100);

    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;
    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    val->tar.len = -OFFSET_DISTANCE - 0.006;
    val->current.len = 0.0;
    val->tar.acc = -(0.4);
    // val->tar.vel = 0.0;

    // control->test_flag = TRUE;

    uint16_t count = 0;

    while (val->current.len > val->tar.len)
    {
        if (val->current.len < val->tar.len + 0.011)
        {
            control->flag = FALSE; // 制御ON
            control->test_flag = TRUE;
        }

        if (val->current.vel >= -0.01 && count > 400)
        {
            break;
        }
        count++;
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    // static BUZZER::buzzer_score_t pc98[] = {
    //{2000, 100}, {1000, 100}};

    // control->test_flag = TRUE;

    /*while (val->current.len > val->tar.len)
    {
         control->test_Duty_l = -0.25;
         control->test_Duty_r = -0.28;
         if (count > 1000) // 300だとダメ（8）
         {
             //bz->play_melody(pc98, 2);
             break;
         }
         count++;
         vTaskDelay(1/portTICK_PERIOD_MS);
    }*/

    // bz->play_melody(pc98, 2);

    // control->flag = TRUE;
    // control->test_flag = FALSE;

    val->tar.acc = 0.0;
    val->tar.vel = 0.0;

    while (val->current.vel <= 0.0)
    {
        if (count > 400)
        {
            break;
        }
        count++;
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    vTaskDelay(100);

    control->test_flag = FALSE;

    control->flag = FALSE; // 制御OFF

    std::cout << "back" << std::endl;
}

void Motion::slalom_left()
{
    map->thinking_flag = FALSE;
    control->flag = TRUE;         // 制御ON
    sens->wall.control = FALSE;   // 壁制御OFF
    val->current.flag = SLA_LEFT; // 左旋回

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    // val->tar.vel = 0.0;
    val->tar.acc = 0.0;
    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    val->current.len = 0.0;

    // val->tar.ang_acc = val->max.ang_acc;
    //  val->max.ang_vel = val->max.ang_vel;

    val->tar.rad = TURN_QUARTER;

    // std::cout << "turn_left" << std::endl;

    local_rad = val->current.rad; // 現在の角度を保存

    val->tar.len = PRE_DISTANCE;
    val->tar.vel = val->max.vel;

    // 前距離
    while ((val->tar.len) > val->current.len)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.ang_acc = val->sla.ang_acc;
    // 旋回
    while (val->tar.rad - (val->current.rad - local_rad) > (val->tar.ang_vel * val->tar.ang_vel) / (2.0 * val->tar.ang_acc))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.ang_acc = -(val->sla.ang_acc);

    while (val->tar.rad > (val->current.rad - local_rad))
    {
        if (val->tar.ang_vel <= 0)
        {
            val->tar.ang_acc = 0;
            val->tar.ang_vel = 0;
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;
    val->current.len = 0.0;
    val->tar.len = FOL_DISTANCE;

    // 後距離
    while ((val->tar.len) > val->current.len)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // control->flag = FALSE;

    map->thinking_flag = TRUE;

    // std::cout << "turn" << std::endl;
}

void Motion::slalom_right()
{
    map->thinking_flag = FALSE;
    control->flag = TRUE;          // 制御ON
    sens->wall.control = FALSE;    // 壁制御OFF
    val->current.flag = SLA_RIGHT; // 左旋回

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    // val->tar.vel = 0.0;
    val->tar.acc = 0.0;
    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    val->current.len = 0.0;

    // val->tar.ang_acc = val->max.ang_acc;
    //  val->max.ang_vel = val->max.ang_vel;

    val->tar.rad = -(TURN_QUARTER);

    // std::cout << "turn_left" << std::endl;

    local_rad = val->current.rad; // 現在の角度を保存

    val->tar.len = PRE_DISTANCE;
    val->tar.vel = val->max.vel;

    // 前距離
    while ((val->tar.len) > val->current.len)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.ang_acc = -(val->sla.ang_acc);
    // 旋回
    while (-(val->tar.rad - (val->current.rad - local_rad)) > (val->tar.ang_vel * val->tar.ang_vel) / (2.0 * -(val->tar.ang_acc)))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.ang_acc = val->sla.ang_acc;

    while ((val->tar.rad) < (val->current.rad - local_rad))
    {
        if (val->tar.ang_vel >= 0)
        {
            val->tar.ang_acc = 0;
            val->tar.ang_vel = 0;
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;
    val->current.len = 0.0;
    val->tar.len = FOL_DISTANCE;

    // 後距離
    while ((val->tar.len) > val->current.len)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // control->flag = FALSE;

    map->thinking_flag = TRUE;

    // std::cout << "turn" << std::endl;
}

void Motion::check_enkaigei()
{
    sens->wall.control = FALSE;
    control->flag = TRUE;

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;
    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    while (1)
    {
        printf("Duty_L:%f    Duty_R:%f\n", control->Duty_l, control->Duty_r);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    std::cout << "check_enkaigei" << std::endl;
}

void Motion::turn_left_2()
{
    vTaskDelay(200);

    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF
    val->current.flag = LEFT;   // 左旋回

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;

    val->tar.ang_acc = val->max.ang_acc;
    // val->max.ang_vel = val->max.ang_vel;

    val->tar.rad = TURN_QUARTER;

    // std::cout << "turn_left" << std::endl;

    local_rad = val->current.rad; // 現在の角度を保存

    while (val->tar.rad - (val->current.rad - local_rad) > (val->tar.ang_vel * val->tar.ang_vel) / (2.0 * val->tar.ang_acc))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.ang_acc = -(val->max.ang_acc);

    while (val->tar.rad > (val->current.rad - local_rad))
    {
        if (val->tar.ang_vel < val->min.ang_vel)
        {
            val->tar.ang_acc = 0;
            val->tar.ang_vel = val->min.ang_vel;
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    while (val->current.ang_vel >= 0.01 || val->current.ang_vel <= -0.01)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    control->flag = FALSE;

    // std::cout << "turn" << std::endl;
}

void Motion::turn_right_2()
{
    vTaskDelay(200);

    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF
    val->current.flag = RIGHT;  // 右旋回

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;

    val->tar.ang_acc = -(val->max.ang_acc);
    // val->max.ang_vel = -(val->max.ang_vel);

    val->tar.rad = -(TURN_QUARTER);

    // std::cout << "turn_left" << std::endl;
    // int turn_count = 0;

    local_rad = val->current.rad; // 現在の角度を保存

    while (-(val->tar.rad - (val->current.rad - local_rad)) > (val->tar.ang_vel * val->tar.ang_vel) / (2.0 * -(val->tar.ang_acc)))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.ang_acc = val->max.ang_acc;

    while ((val->tar.rad) < (val->current.rad - local_rad))
    {
        if (val->tar.ang_vel > -(val->min.ang_vel))
        {
            val->tar.ang_acc = 0;
            val->tar.ang_vel = -(val->min.ang_vel);
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    // std::cout << "##### deceleration #####" << std::endl;
    /*while (val->current.ang_vel >= 0.01 || val->current.ang_vel <= -0.01)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }*/

    control->flag = FALSE;

    // std::cout << "turn" << std::endl;
}

void Motion::wall_check()
{
    sens->wall.control = FALSE;
    control->flag = FALSE;

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;
    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    while (1)
    {
        // 設定したsens->wall.th_wallの値によって光るLEDが変わる
        printf("sens.wall.val.fl:%d    sens.wall.val.l:%d    sens.wall.val.r:%d    sens.wall.val.fr:%d\n", sens->wall.val.fl, sens->wall.val.l, sens->wall.val.r, sens->wall.val.fr);
        led->set(sens->wall.exist.fl + (sens->wall.exist.l << 1) + (sens->wall.exist.r << 2) + (sens->wall.exist.fr << 3));
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    std::cout << "check_enkaigei" << std::endl;
}

void Motion::adjust_pid(const char *gain, float *pid, float step, uint8_t mode_num)
{

    while (1)
    {
        led->set(mode_num + 1);

        if (sens->wall.val.fl + sens->wall.val.l + sens->wall.val.r + sens->wall.val.fr > 3000)
        {
            led->set(0b1111);
            printf("adjust %s\n", gain);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            return;
        }

        if (val->current.vel > 0.01)
        {

            *pid += step;

            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (val->current.vel < -0.01)
        {

            *pid -= step;

            vTaskDelay(pdMS_TO_TICKS(50));
        }

        printf("%s : %f\n", gain, *pid);

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void Motion::set_pid_gain()
{
    uint8_t mode = 0;

    const char *speed_Kp = "speed_Kp";
    const char *speed_Ki = "speed_Ki";
    const char *speed_Kd = "speed_Kd";
    const char *ang_vel_Kp = "ang_vel_Kp";
    const char *ang_vel_Ki = "ang_vel_Ki";
    const char *ang_vel_Kd = "ang_vel_Kd";
    const char *wall_Kp = "wall_Kp";
    const char *wall_Ki = "wall_Ki";
    const char *wall_Kd = "wall_Kd";

    t_file_pid_gain pid_gain = read_file_pid();

    adjust_pid(speed_Kp, &pid_gain.speed_Kp, 0.1, mode);
    adjust_pid(speed_Ki, &pid_gain.speed_Ki, 10, mode + 1);
    adjust_pid(speed_Kd, &pid_gain.speed_Kd, 0.1, mode + 2);
    adjust_pid(ang_vel_Kp, &pid_gain.ang_vel_Kp, 0.01, mode + 3);
    adjust_pid(ang_vel_Ki, &pid_gain.ang_vel_Ki, 1, mode + 4);
    adjust_pid(ang_vel_Kd, &pid_gain.ang_vel_Kd, 0.1, mode + 5);
    adjust_pid(wall_Kp, &pid_gain.wall_Kp, 0.001, mode + 6);
    adjust_pid(wall_Ki, &pid_gain.wall_Ki, 0.01, mode + 7);
    adjust_pid(wall_Kd, &pid_gain.wall_Kd, 0.01, mode + 8);

    write_file_pid(&pid_gain);

    printf("set_pid_gain\n");
}

void Motion::adjust_wall_threshold(const char *threshold, uint16_t *th_value, uint8_t step, uint8_t mode_num)
{

    while (1)
    {
        led->set(mode_num + 1);

        if (sens->wall.val.fl + sens->wall.val.l + sens->wall.val.r + sens->wall.val.fr > 3000)
        {
            led->set(0b1111);
            printf("adjust %s\n", threshold);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            return;
        }

        if (val->current.vel > 0.01)
        {

            *th_value += step;

            vTaskDelay(pdMS_TO_TICKS(50));
        }
        if (val->current.vel < -0.01)
        {

            *th_value -= step;

            vTaskDelay(pdMS_TO_TICKS(50));
        }

        printf("%s : %d\n", threshold, *th_value);

        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}

void Motion::set_wall_threshold()
{
    uint8_t mode = 0;

    const char *th_wall_fl = "th_wall_fl";
    const char *th_wall_l = "th_wall_l";
    const char *th_wall_r = "th_wall_r";
    const char *th_wall_fr = "th_wall_fr";
    const char *th_control_l = "th_control_l";
    const char *th_control_r = "th_control_r";
    const char *ref_l = "ref_l";
    const char *ref_r = "ref_r";

    t_file_wall_th th_value = read_file_wall_th();

    adjust_wall_threshold(th_wall_fl, &th_value.th_wall_fl, 1, mode);
    adjust_wall_threshold(th_wall_l, &th_value.th_wall_l, 1, mode + 1);
    adjust_wall_threshold(th_wall_r, &th_value.th_wall_r, 1, mode + 2);
    adjust_wall_threshold(th_wall_fr, &th_value.th_wall_fr, 1, mode + 3);
    adjust_wall_threshold(th_control_l, &th_value.th_control_l, 1, mode + 4);
    adjust_wall_threshold(th_control_r, &th_value.th_control_r, 1, mode + 5);
    adjust_wall_threshold(ref_l, &th_value.ref_l, 10, mode + 6);
    adjust_wall_threshold(ref_r, &th_value.ref_r, 10, mode + 7);

    write_file_wall_th(&th_value);

    printf("set_pid_gain\n");
}

void Motion::offset()
{
    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;
    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    val->tar.len = OFFSET_DISTANCE;
    val->current.len = 0.0;
    val->tar.acc = val->max.acc;
    // val->tar.vel = 0.0;

    while (((val->tar.len - 0.01) - val->current.len) > (((val->tar.vel) * (val->tar.vel)) / (2.0 *
                                                                                              val->tar.acc)))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    val->tar.acc = -(val->max.acc);

    while ((val->tar.len - 0.001) > val->current.len)
    {
        if (val->tar.vel <= val->min.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->min.vel;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.acc = 0.0;
    val->tar.vel = 0.0;

    while (val->current.vel >= 0.0)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    control->flag = FALSE; // 制御OFF

    // std::cout << "offset" << std::endl;
}

void Motion::calibrate_wall_th()
{
    t_file_center_sens_value center_val;

    // 壁当てで中央に移動
    offset();
    turn_right_2();
    back();
    offset();

    // 右壁見る（前壁センサ）
    center_val.right_fl = sens->wall.val.fl;
    center_val.right_fr = sens->wall.val.fr;

    turn_right_2();

    // 後ろ壁見る（前壁センサ）
    center_val.rear_fl = sens->wall.val.fl;
    center_val.rear_fr = sens->wall.val.fr;

    turn_right_2();

    // 左壁見る（前壁センサ）
    center_val.left_fl = sens->wall.val.fl;
    center_val.left_fr = sens->wall.val.fr;

    turn_right_2();

    // 正面を向く（左右センサ）
    center_val.front_l = sens->wall.val.l;
    center_val.front_r = sens->wall.val.r;

    write_file_center_sens_val(&center_val);
}

void Motion::offset2()
{
    control->flag = TRUE;       // 制御ON
    sens->wall.control = FALSE; // 壁制御OFF

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;
    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    val->tar.len = OFFSET_DISTANCE;
    val->current.len = 0.0;
    val->tar.acc = val->max.acc;
    // val->tar.vel = 0.0;

    while (((val->tar.len - 0.01) - val->current.len) > (((val->tar.vel) * (val->tar.vel)) / (2.0 *
                                                                                              val->tar.acc)))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    // val->tar.acc = -(val->max.acc);

    while ((val->tar.len - 0.001) > val->current.len) // offsetにこのwhile文の処理いらないかも
    {
        if (val->tar.vel <= val->min.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->min.vel;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // val->tar.acc = 0.0;
    // val->tar.vel = 0.0;

    // std::cout << "offset" << std::endl;
}

void Motion::fast_straight(uint8_t straight_count)
{
    map->thinking_flag = FALSE;
    control->flag = TRUE;      // 制御ON
    sens->wall.control = TRUE; // 壁制御OFF

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    val->tar.len = SECTION * straight_count;
    val->current.len = 0.0;
    val->tar.acc = val->max.acc;

    val->max.vel = CalcVelocity((val->tar.len - 0.09), val->fast_ref.vel, val->max.acc); // 一区画以上の場合に加速区間を設定
    val->end.vel = val->fast_ref.vel;

    bool l_wall_check = sens->wall.exist.l;
    bool r_wall_check = sens->wall.exist.r;
    bool hosei_flag = FALSE;
    uint8_t section_cnt = 0;

    while (((val->tar.len - 0.01) - val->current.len) > (((val->tar.vel) * (val->tar.vel) - (val->end.vel) * (val->end.vel)) / (2.0 *
                                                                                                                                val->tar.acc)))
    {
        // 壁あり->壁なし
        if (sens->wall.exist.l == FALSE && l_wall_check == TRUE && hosei_flag == FALSE)
        {
            bz->play_melody(pc98, 2);
            val->current.len = 0.055 + SECTION * (section_cnt);
            hosei_flag = TRUE;
        }

        if (sens->wall.exist.r == FALSE && r_wall_check == TRUE && hosei_flag == FALSE)
        {
            bz->play_melody(pc98, 2);
            val->current.len = 0.055 + SECTION * (section_cnt);
            hosei_flag = TRUE;
        }

        // 壁なし->壁あり
        if (sens->wall.exist.l == TRUE && l_wall_check == FALSE && hosei_flag == FALSE)
        {
            bz->play_melody(pc98_2, 2);
            val->current.len = 0.038 + SECTION * (section_cnt); // 補正後の距離を伸ばしたい場合は、値を小さく
            hosei_flag = TRUE;
        }

        if (sens->wall.exist.r == TRUE && r_wall_check == FALSE && hosei_flag == FALSE)
        {
            bz->play_melody(pc98_2, 2);
            val->current.len = 0.040 + SECTION * (section_cnt);
            hosei_flag = TRUE;
        }

        section_cnt = static_cast<uint8_t>(std::floor(val->current.len / SECTION));

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    val->tar.acc = -(val->max.acc);

    while ((val->tar.len) > val->current.len)
    {
        if (val->tar.vel <= val->fast_ref.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->fast_ref.vel;
        }

        // 減速時壁切れ補正(ただし、連続する直線区間の距離によってかなり変動する)

        // 壁あり->壁なし
        if (sens->wall.exist.l == FALSE && l_wall_check == TRUE && hosei_flag == FALSE)
        {
            bz->play_melody(pc98, 2);
            val->current.len = 0.055 + SECTION * (section_cnt);
            hosei_flag = TRUE;
        }

        if (sens->wall.exist.r == FALSE && r_wall_check == TRUE && hosei_flag == FALSE)
        {
            bz->play_melody(pc98, 2);
            val->current.len = 0.055 + SECTION * (section_cnt);
            hosei_flag = TRUE;
        }

        // 壁なし->壁あり
        if (sens->wall.exist.l == TRUE && l_wall_check == FALSE && hosei_flag == FALSE)
        {
            bz->play_melody(pc98_2, 2);
            val->current.len = 0.038 + SECTION * (section_cnt); // 補正後の距離を伸ばしたい場合は、値を小さく
            hosei_flag = TRUE;
        }

        if (sens->wall.exist.r == TRUE && r_wall_check == FALSE && hosei_flag == FALSE)
        {
            bz->play_melody(pc98_2, 2);
            val->current.len = 0.040 + SECTION * (section_cnt);
            hosei_flag = TRUE;
        }

        section_cnt = static_cast<uint8_t>(std::floor(val->current.len / SECTION));

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.acc = 0.0;
    val->tar.vel = val->fast_ref.vel;
    val->max.vel = val->fast_ref.vel;

    map->thinking_flag = TRUE;
}

void Motion::fast_stop(uint8_t straight_count)
{
    map->thinking_flag = FALSE;
    control->flag = TRUE;      // 制御ON
    sens->wall.control = TRUE; // 壁制御OFF

    val->I.vel_error = 0.0;
    val->I.ang_error = 0.0;
    val->I.wall_error = 0.0;

    val->tar.ang_acc = 0.0;
    val->tar.ang_vel = 0.0;

    val->tar.len = SECTION * straight_count;
    val->current.len = 0.0;
    val->tar.acc = val->max.acc;

    val->max.vel = CalcVelocity(val->tar.len, val->fast_ref.vel, val->max.acc);
    val->end.vel = 0;

    while (((val->tar.len - 0.01) - val->current.len) > (((val->tar.vel) * (val->tar.vel)) / (2.0 *
                                                                                              val->tar.acc)))
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // std::cout << "##### deceleration #####" << std::endl;
    val->tar.acc = -(val->max.acc);

    while ((val->tar.len - 0.001) > val->current.len)
    {
        if (val->tar.vel <= val->min.vel)
        {
            val->tar.acc = 0;
            val->tar.vel = val->min.vel;
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    val->tar.acc = 0.0;
    val->tar.vel = 0.0;
    val->max.vel = val->fast_ref.vel;

    while (val->current.vel >= 0.0)
    {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    control->flag = FALSE; // 制御OFF

    map->thinking_flag = TRUE;

    // std::cout << "stop" << std::endl;
}

void Motion::CheckMotorDuty(float Duty_l, float Duty_r, uint32_t time)
{
    control->flag = FALSE; // 制御ON
    control->test_flag = TRUE;

    control->Duty_l = Duty_l;
    control->Duty_r = Duty_r;

    uint32_t count = 0;

    while (count < time)
    {
        mot->setMotorSpeed(Duty_l, Duty_r);
        printf("Duty_L:%f    Duty_R:%f\n", control->Duty_l, control->Duty_r);
        count++;
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    control->flag = FALSE; // 制御OFF
    control->test_flag = FALSE;
}

float Motion::CalcVelocity(float dist, float vel, float acc)
{
    return (vel + sqrt((vel * vel) + 2 * acc * dist)) / 2;
}

void Motion::DetectDeadZone(float step_size, float max_duty, uint32_t update_rate, uint32_t settle_time)
{
    control->flag = FALSE; // 制御OFF（手動モータ制御）
    control->test_flag = TRUE;
    
    float current_duty = 0.0;

    vTaskDelay(settle_time / portTICK_PERIOD_MS);

    while (current_duty <= max_duty) {
        control->Duty_l = current_duty;
        control->Duty_r = current_duty;
        // モーターに指令値を出力
        mot->setMotorSpeed(current_duty, current_duty);

        current_duty += step_size;
        vTaskDelay(update_rate / portTICK_PERIOD_MS); // 1秒待機
    }
    
    mot->setMotorSpeed(0.0, 0.0); // 最終的にモーター停止
    control->flag = FALSE; // 制御OFF維持
    control->test_flag = FALSE;
}

void Motion::DetectSaturationRegion(float start_duty, float step_size, float max_duty, uint32_t update_rate, uint32_t settle_time)
{
    control->flag = FALSE; // 制御OFF（手動モータ制御）
    control->test_flag = TRUE;
    
    float current_duty = start_duty;
    
    vTaskDelay(settle_time / portTICK_PERIOD_MS);

    while (current_duty <= max_duty) {
        control->Duty_l = current_duty;
        control->Duty_r = current_duty;
        // モーターに指令値を出力
        mot->setMotorSpeed(current_duty, current_duty);

        current_duty += step_size;
        vTaskDelay(update_rate / portTICK_PERIOD_MS); // 1秒待機
    }


    mot->setMotorSpeed(0.0, 0.0); // 最終的にモーター停止
    control->flag = FALSE; // 制御OFF維持
    control->test_flag = FALSE;
}

void Motion::ApplySystemIdentificationSignal(const float* signal_left, const float* signal_right, int num_samples, int sampling_period_ms)
{
    control->flag = FALSE;
    control->test_flag = TRUE;

    static const float DUTY_L = 0.15;
    static const float DUTY_R = 0.15;

    val->tar.vel = 0.0;
    val->tar.acc = 0.0;
    val->tar.ang_vel = 0.0;
    val->tar.ang_acc = 0.0;

    printf("=== Starting M-sequence Signal Application ===\n");
    printf("Number of samples: %d\n", num_samples);
    printf("Sampling period: %d ms\n", sampling_period_ms);

    for (int i = 0; i < num_samples; i++) {
        float duty_left = signal_left[i];
        float duty_right = signal_right[i];

        // duty値を安全な範囲に制限 (-0.5 to 0.5)
        if (duty_left > DUTY_L) duty_left = DUTY_L;
        if (duty_left < -DUTY_L) duty_left = -DUTY_L;
        if (duty_right > DUTY_R) duty_right = DUTY_R;
        if (duty_right < -DUTY_R) duty_right = -DUTY_R;

        control->Duty_l = duty_left;
        control->Duty_r = duty_right;

        mot->setMotorSpeed(duty_left, duty_right);

        if (i % 10 == 0) {
            //printf("Progress: %d/%d samples\n", i + 1, num_samples);
        }

        vTaskDelay(sampling_period_ms / portTICK_PERIOD_MS);
    }

    mot->setMotorSpeed(0.0, 0.0);
    control->Duty_l = 0.0;
    control->Duty_r = 0.0;

    printf("M-sequence signal application completed.\n");

    control->flag = FALSE;
    control->test_flag = FALSE;
}

void Motion::RunTranslationIdentification(const float* signal_left, const float* signal_right, int num_samples, int sampling_period_ms)
{
    printf("--- Starting Translation Model Identification ---\n");
    printf("Robot will perform parallel wheel motion for system identification.\n");

    ApplySystemIdentificationSignal(signal_left, signal_right, num_samples, sampling_period_ms);

    printf("Translation model identification completed.\n");
}

void Motion::RunRotationIdentification(const float* signal_left, const float* signal_right, int num_samples, int sampling_period_ms)
{
    printf("--- Starting Rotation Model Identification ---\n");
    printf("Robot will perform differential wheel motion for system identification.\n");

    ApplySystemIdentificationSignal(signal_left, signal_right, num_samples, sampling_period_ms);

    printf("Rotation model identification completed.\n");
}