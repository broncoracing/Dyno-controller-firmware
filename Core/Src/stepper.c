#include "stepper.h"

extern TIM_HandleTypeDef htim1;

volatile uint32_t speed = 0;
volatile int32_t position = 0;
volatile uint32_t last_update_tick;
int32_t pos_target = 0;
uint8_t moving = 0;

enum DIRECTION {
    FWD,
    REV
} direction = FWD;

int32_t get_pos(void){
    return position;
}

void stop_movement(void) {
    HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_2);
    moving = 0;
}

void set_arr(void) {
    uint32_t autoreload = 65535 / speed;
    __HAL_TIM_SET_AUTORELOAD(&htim1, (uint16_t) autoreload);
}

void start_movement(void) {
    speed = MIN_SPEED;
    if(direction == FWD){
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 1);
    } else {
        HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, 0);
    }
    set_arr();
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    last_update_tick = HAL_GetTick();
    HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
}

void stepper_pulse_finished(void) {
    uint32_t distance_remaining;
    uint32_t time = HAL_GetTick();
    // todo
    if(direction == FWD) {
        position++;
        if(position >= pos_target) {
            stop_movement();
            return;
        }
        distance_remaining = pos_target - position;
    } else {
        position--;

        if(position <= pos_target) {
            stop_movement();
            moving = 0;
            return;
        }
        distance_remaining = position - pos_target;
    }

    uint32_t dt = time - last_update_tick;
    speed += dt * ACCEL;

    if(speed > MAX_SPEED) {
        speed = MAX_SPEED;
    }
    uint32_t decel_speed = (distance_remaining * DECEL) >> 8;
    if(speed > decel_speed) {
        speed = decel_speed;
    }
    if(speed < MIN_SPEED) {
        speed = MIN_SPEED;
    }
    set_arr();
    last_update_tick = time;
}


void move_to(int32_t target){
    if(moving) {
        enum DIRECTION new_dir;
        if(pos_target > position){
            new_dir = FWD;
        } else if(pos_target < position){
            new_dir = REV;
        }

        pos_target = target;
        if(new_dir == direction) {
            return;
        } else {
            direction = new_dir;
            start_movement();
        }
    } else {
        moving = 1;
        pos_target = target;
        if(pos_target > position){
            direction = FWD;
        } else if(pos_target < position){
            direction = REV;
        } else {
            // already at target
            stop_movement();
            return;
        }

        start_movement();
    }   
}

void zero(void) {
    stop_movement();
    position = 0;
}
uint8_t is_moving(void) {
    return moving;
}
