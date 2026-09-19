#include "motor_speed.h"
#include "Mona_ESP_lib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const int kControlPeriodMs = 20;
static const int kMaxTicksPerSec = 10000;
/* Feedforward: ~3600 ticks/s ≈ PWM 255 → divide by 14. Tune with Motor_speed_PID_tune. */
static const int kDefaultLeftFfDiv = 30;
static const int kDefaultLeftKpDiv = 8;
static const int kDefaultLeftKiDiv = 100;
static const int kDefaultRightFfDiv = 30;
static const int kDefaultRightKpDiv = 8;
static const int kDefaultRightKiDiv = 100;
static const int kIMax = 4000;

static volatile int left_cmd_tps = 0;
static volatile int right_cmd_tps = 0;
static volatile int left_meas_tps = 0;
static volatile int right_meas_tps = 0;
static volatile int left_pwm_out = 0;
static volatile int right_pwm_out = 0;
static volatile int left_ff_div = kDefaultLeftFfDiv;
static volatile int left_kp_div = kDefaultLeftKpDiv;
static volatile int left_ki_div = kDefaultLeftKiDiv;
static volatile int right_ff_div = kDefaultRightFfDiv;
static volatile int right_kp_div = kDefaultRightKpDiv;
static volatile int right_ki_div = kDefaultRightKiDiv;

static int32_t last_left_ticks = 0;
static int32_t last_right_ticks = 0;
static int left_integral = 0;
static int right_integral = 0;

static int clamp_pwm(int v) {
	if (v > 255) {
		return 255;
	}
	if (v < -255) {
		return -255;
	}
	return v;
}

static int clamp_tps(int v) {
	if (v > kMaxTicksPerSec) {
		return kMaxTicksPerSec;
	}
	if (v < -kMaxTicksPerSec) {
		return -kMaxTicksPerSec;
	}
	return v;
}

static int clamp_div(int v) {
	if (v < 1) {
		return 1;
	}
	return v;
}

static void apply_left(int signed_pwm) {
	if (signed_pwm > 0) {
		analogWrite(Mot_left_forward, signed_pwm);
		analogWrite(Mot_left_backward, 0);
	} else {
		analogWrite(Mot_left_forward, 0);
		analogWrite(Mot_left_backward, -signed_pwm);
	}
}

static void apply_right(int signed_pwm) {
	if (signed_pwm > 0) {
		analogWrite(Mot_right_forward, signed_pwm);
		analogWrite(Mot_right_backward, 0);
	} else {
		analogWrite(Mot_right_forward, 0);
		analogWrite(Mot_right_backward, -signed_pwm);
	}
}

static int pi_to_pwm(int target_tps, int measured_tps, int *integral,
		     int ff_div, int kp_div, int ki_div) {
	if (target_tps == 0) {
		*integral = 0;
		return 0;
	}

	int err = target_tps - measured_tps;
	*integral += err;
	if (*integral > kIMax) {
		*integral = kIMax;
	} else if (*integral < -kIMax) {
		*integral = -kIMax;
	}

	int u = target_tps / ff_div + err / kp_div + (*integral) / ki_div;
	return clamp_pwm(u);
}

static void motor_speed_step(void) {
	int left_target = left_cmd_tps;
	int right_target = right_cmd_tps;
	int32_t left_now = Encoder_left();
	int32_t right_now = Encoder_right();

	int left_meas = (int)(left_now - last_left_ticks) * (1000 / kControlPeriodMs);
	int right_meas = (int)(right_now - last_right_ticks) * (1000 / kControlPeriodMs);
	last_left_ticks = left_now;
	last_right_ticks = right_now;
	left_meas_tps = left_meas;
	right_meas_tps = right_meas;

	int left_u = pi_to_pwm(left_target, left_meas, &left_integral,
			      left_ff_div, left_kp_div, left_ki_div);
	int right_u = pi_to_pwm(right_target, right_meas, &right_integral,
			       right_ff_div, right_kp_div, right_ki_div);
	left_pwm_out = left_u;
	right_pwm_out = right_u;
	apply_left(left_u);
	apply_right(right_u);
}

static void motor_speed_task(void *arg) {
	(void)arg;
	last_left_ticks = Encoder_left();
	last_right_ticks = Encoder_right();
	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(kControlPeriodMs));
		motor_speed_step();
	}
}

void motor_set_left(int ticks_per_s) {
	int target = clamp_tps(ticks_per_s);
	left_cmd_tps = target;
	if (target == 0) {
		left_integral = 0;
		left_pwm_out = 0;
		apply_left(0);
	}
}

void motor_set_right(int ticks_per_s) {
	int target = clamp_tps(ticks_per_s);
	right_cmd_tps = target;
	if (target == 0) {
		right_integral = 0;
		right_pwm_out = 0;
		apply_right(0);
	}
}

void motor_speed_set_left_gains(int new_ff_div, int new_kp_div, int new_ki_div) {
	left_ff_div = clamp_div(new_ff_div);
	left_kp_div = clamp_div(new_kp_div);
	left_ki_div = clamp_div(new_ki_div);
	left_integral = 0;
}

void motor_speed_set_right_gains(int new_ff_div, int new_kp_div, int new_ki_div) {
	right_ff_div = clamp_div(new_ff_div);
	right_kp_div = clamp_div(new_kp_div);
	right_ki_div = clamp_div(new_ki_div);
	right_integral = 0;
}

void motor_speed_set_gains(int new_ff_div, int new_kp_div, int new_ki_div) {
	motor_speed_set_left_gains(new_ff_div, new_kp_div, new_ki_div);
	motor_speed_set_right_gains(new_ff_div, new_kp_div, new_ki_div);
}

void motor_speed_get_left_gains(int *out_ff_div, int *out_kp_div, int *out_ki_div) {
	*out_ff_div = left_ff_div;
	*out_kp_div = left_kp_div;
	*out_ki_div = left_ki_div;
}

void motor_speed_get_right_gains(int *out_ff_div, int *out_kp_div, int *out_ki_div) {
	*out_ff_div = right_ff_div;
	*out_kp_div = right_kp_div;
	*out_ki_div = right_ki_div;
}

void motor_speed_get_state(motor_speed_state_t *out) {
	out->left_target_tps = left_cmd_tps;
	out->left_meas_tps = left_meas_tps;
	out->left_pwm = left_pwm_out;
	out->right_target_tps = right_cmd_tps;
	out->right_meas_tps = right_meas_tps;
	out->right_pwm = right_pwm_out;
}

void motor_speed_init(void) {
	xTaskCreate(motor_speed_task, "mot_spd", 2048, NULL, 1, NULL);
}
