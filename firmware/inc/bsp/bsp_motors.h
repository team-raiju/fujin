#ifndef BSP_MOTORS_H
#define BSP_MOTORS_H

#include <stdint.h>

typedef enum motor_rot_dir {
    MOT_ROT_CW,
    MOT_ROT_CCW,
} motor_rot_dir_t;

void BSP_motorsInit(void);
void BSP_motors(int16_t vel_left, int16_t vel_right);

motor_rot_dir_t BSP_motor_left_dir_pin();
motor_rot_dir_t BSP_motor_right_dir_pin();

#endif /* BSP_MOTORS_H */
