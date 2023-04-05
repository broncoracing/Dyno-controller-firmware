#include "segment.h"
#define NUM_SEGMENTS 4

uint8_t segments[NUM_SEGMENTS];

uint8_t current_seg;

struct GPIO_Pin_t {
    GPIO_TypeDef * port;
    uint16_t pin;
};

struct GPIO_Pin_t digit_pins[NUM_SEGMENTS] = {
    {DIG_0_GPIO_Port, DIG_0_Pin},
    {DIG_1_GPIO_Port, DIG_1_Pin},
    {DIG_2_GPIO_Port, DIG_2_Pin},
    {DIG_3_GPIO_Port, DIG_3_Pin},
};

struct GPIO_Pin_t segment_pins[8] = {
    {SEG_DP_GPIO_Port, SEG_DP_Pin},
    {SEG_G_GPIO_Port, SEG_G_Pin},
    {SEG_F_GPIO_Port, SEG_F_Pin},
    {SEG_E_GPIO_Port, SEG_E_Pin},
    {SEG_D_GPIO_Port, SEG_D_Pin},
    {SEG_C_GPIO_Port, SEG_C_Pin},
    {SEG_B_GPIO_Port, SEG_B_Pin},
    {SEG_A_GPIO_Port, SEG_A_Pin},
};

const uint8_t segment_map[10] = {
    0b11111100,
    0b01100000,
    0b11011010,
    0b11110010,
    0b01100110,
    0b10110110,
    0b10111110,
    0b11100000,
    0b11111110,
    0b11110110
};
const uint8_t segment_dp = 1;

void seg_set_value(uint32_t val){
    segments[0] = val % 10;
    val /= 10;
    segments[1] = val % 10;
    val /= 10;
    segments[2] = val % 10;
    val /= 10;
    segments[3] = val % 10;
}

void seg_update(void){
    HAL_GPIO_WritePin(digit_pins[current_seg].port, digit_pins[current_seg].pin, 1);

    current_seg = (current_seg + 1) % NUM_SEGMENTS;

    HAL_GPIO_WritePin(digit_pins[current_seg].port, digit_pins[current_seg].pin, 0);
    
    uint8_t digit_val = segments[current_seg];
    uint8_t digit_segments = segment_map[digit_val];
    if(current_seg == 2){
        digit_segments |= segment_dp;
    }

    for(uint8_t i = 0; i < 8; ++i){
        uint8_t seg_on = 0;
        if((digit_segments >> i) & 1) {
            seg_on = 1;
        }
        HAL_GPIO_WritePin(segment_pins[i].port, segment_pins[i].pin, seg_on);
    }
}