#ifndef Mona_ESP_motor_speed_h
#define Mona_ESP_motor_speed_h

/** Closed-loop wheel speed control (encoder ticks/s → PWM). */
void motor_speed_init(void);

/** Set left/right wheel target speed in signed encoder ticks per second. */
void motor_set_left(int ticks_per_s);
void motor_set_right(int ticks_per_s);

/**
 * PI+FF divisors: PWM ≈ target/ff + error/kp + integral/ki
 * Larger divisor = weaker term. Values < 1 are clamped to 1.
 * set_gains applies the same divisors to both wheels.
 */
void motor_speed_set_gains(int ff_div, int kp_div, int ki_div);
void motor_speed_set_left_gains(int ff_div, int kp_div, int ki_div);
void motor_speed_set_right_gains(int ff_div, int kp_div, int ki_div);
void motor_speed_get_left_gains(int *ff_div, int *kp_div, int *ki_div);
void motor_speed_get_right_gains(int *ff_div, int *kp_div, int *ki_div);

typedef struct {
	int left_target_tps;
	int left_meas_tps;
	int left_pwm;
	int right_target_tps;
	int right_meas_tps;
	int right_pwm;
} motor_speed_state_t;

void motor_speed_get_state(motor_speed_state_t *out);

#endif
