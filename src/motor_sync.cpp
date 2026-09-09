#include "motor_sync.h"
#include "Mona_ESP_lib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const int kSyncPeriodMs = 20;
static const int kKp = 1;
static const int kKiDiv = 16;
static const int kIMax = 2000;

static volatile int left_cmd = 0;
static volatile int right_cmd = 0;
static int32_t last_left = 0;
static int32_t last_right = 0;
static int integral = 0;
static int prev_left_cmd = 0;
static int prev_right_cmd = 0;

static int clamp_mag(int v) {
	if (v < 0) {
		return 0;
	}
	if (v > 255) {
		return 255;
	}
	return v;
}

static int clamp_signed(int v) {
	if (v > 255) {
		return 255;
	}
	if (v < -255) {
		return -255;
	}
	return v;
}

static int abs_i(int v) {
	return v < 0 ? -v : v;
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

static void motor_sync_step(void) {
	int lc = left_cmd;
	int rc = right_cmd;
	int32_t left_now = Encoder_left();
	int32_t right_now = Encoder_right();

	if (lc != prev_left_cmd || rc != prev_right_cmd) {
		prev_left_cmd = lc;
		prev_right_cmd = rc;
		last_left = left_now;
		last_right = right_now;
		if (lc == 0 && rc == 0) {
			integral = 0;
		}
		return;
	}

	int32_t dl = left_now - last_left;
	int32_t dr = right_now - last_right;
	last_left = left_now;
	last_right = right_now;

	if (lc == 0 || rc == 0 || abs_i(lc) != abs_i(rc)) {
		integral = 0;
		return;
	}

	int sl = (int)dl;
	int sr = (int)dr;
	if (lc < 0) {
		sl = -sl;
	}
	if (rc < 0) {
		sr = -sr;
	}

	int err = sl - sr;
	integral += err;
	if (integral > kIMax) {
		integral = kIMax;
	} else if (integral < -kIMax) {
		integral = -kIMax;
	}

	int adj = kKp * err + integral / kKiDiv;
	int base = abs_i(lc);
	int left_mag = clamp_mag(base - adj);
	int right_mag = clamp_mag(base + adj);
	apply_left(lc > 0 ? left_mag : -left_mag);
	apply_right(rc > 0 ? right_mag : -right_mag);
}

static void motor_sync_task(void *arg) {
	(void)arg;
	last_left = Encoder_left();
	last_right = Encoder_right();
	for (;;) {
		vTaskDelay(pdMS_TO_TICKS(kSyncPeriodMs));
		motor_sync_step();
	}
}

void motor_set_left(int signed_pwm) {
	left_cmd = clamp_signed(signed_pwm);
	apply_left(left_cmd);
}

void motor_set_right(int signed_pwm) {
	right_cmd = clamp_signed(signed_pwm);
	apply_right(right_cmd);
}

void motor_sync_init(void) {
	xTaskCreate(motor_sync_task, "mot_sync", 2048, NULL, 1, NULL);
}
