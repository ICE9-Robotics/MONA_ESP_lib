#include "encoder.h"
#include <Arduino.h>
#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

// Quadrature hall encoders counted by the ESP32 PCNT peripheral (not GPIO ISRs).
#define ENC_PCNT_HIGH 32767
#define ENC_PCNT_LOW  (-32768)
#define ENC_GLITCH_NS 1000

typedef struct {
	pcnt_unit_handle_t unit;
	volatile int32_t overflow;
} encoder_channel_t;

static encoder_channel_t enc_left = {};
static encoder_channel_t enc_right = {};
static portMUX_TYPE encoder_mux = portMUX_INITIALIZER_UNLOCKED;

static bool IRAM_ATTR encoder_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx) {
	(void)unit;
	encoder_channel_t *ch = (encoder_channel_t *)user_ctx;
	portENTER_CRITICAL_ISR(&encoder_mux);
	ch->overflow += edata->watch_point_value;
	portEXIT_CRITICAL_ISR(&encoder_mux);
	return false;
}

static bool encoder_setup_unit(encoder_channel_t *ch, int gpio_a, int gpio_b) {
	pcnt_unit_config_t unit_config = {};
	unit_config.high_limit = ENC_PCNT_HIGH;
	unit_config.low_limit = ENC_PCNT_LOW;
	if (pcnt_new_unit(&unit_config, &ch->unit) != ESP_OK) {
		ch->unit = NULL;
		return false;
	}

	pcnt_glitch_filter_config_t filter_config = {};
	filter_config.max_glitch_ns = ENC_GLITCH_NS;
	if (pcnt_unit_set_glitch_filter(ch->unit, &filter_config) != ESP_OK) {
		return false;
	}

	pcnt_chan_config_t chan_a_config = {};
	chan_a_config.edge_gpio_num = gpio_a;
	chan_a_config.level_gpio_num = gpio_b;
	pcnt_channel_handle_t chan_a = NULL;
	if (pcnt_new_channel(ch->unit, &chan_a_config, &chan_a) != ESP_OK) {
		return false;
	}

	pcnt_chan_config_t chan_b_config = {};
	chan_b_config.edge_gpio_num = gpio_b;
	chan_b_config.level_gpio_num = gpio_a;
	pcnt_channel_handle_t chan_b = NULL;
	if (pcnt_new_channel(ch->unit, &chan_b_config, &chan_b) != ESP_OK) {
		return false;
	}

	pcnt_channel_set_edge_action(chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
	pcnt_channel_set_level_action(chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
	pcnt_channel_set_edge_action(chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
	pcnt_channel_set_level_action(chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

	pcnt_unit_add_watch_point(ch->unit, ENC_PCNT_HIGH);
	pcnt_unit_add_watch_point(ch->unit, ENC_PCNT_LOW);

	pcnt_event_callbacks_t cbs = {};
	cbs.on_reach = encoder_on_reach;
	if (pcnt_unit_register_event_callbacks(ch->unit, &cbs, ch) != ESP_OK) {
		return false;
	}

	if (pcnt_unit_enable(ch->unit) != ESP_OK) {
		return false;
	}
	pcnt_unit_clear_count(ch->unit);
	ch->overflow = 0;
	return pcnt_unit_start(ch->unit) == ESP_OK;
}

static int32_t encoder_read(encoder_channel_t *ch) {
	int pulse = 0;
	int32_t overflow;
	if (ch->unit == NULL) {
		return 0;
	}
	portENTER_CRITICAL(&encoder_mux);
	pcnt_unit_get_count(ch->unit, &pulse);
	overflow = ch->overflow;
	portEXIT_CRITICAL(&encoder_mux);
	return overflow + pulse;
}

void encoder_init(int right_a, int right_b, int left_a, int left_b) {
	if (!encoder_setup_unit(&enc_right, right_a, right_b)) {
		Serial.println("Unable to initialize right wheel encoder");
	}
	if (!encoder_setup_unit(&enc_left, left_a, left_b)) {
		Serial.println("Unable to initialize left wheel encoder");
	}
}

int32_t Encoder_left(void) {
	return encoder_read(&enc_left);
}

int32_t Encoder_right(void) {
	return encoder_read(&enc_right);
}
void Encoder_reset(void) {
	portENTER_CRITICAL(&encoder_mux);
	if (enc_left.unit != NULL) {
		pcnt_unit_clear_count(enc_left.unit);
		enc_left.overflow = 0;
	}
	if (enc_right.unit != NULL) {
		pcnt_unit_clear_count(enc_right.unit);
		enc_right.overflow = 0;
	}
	portEXIT_CRITICAL(&encoder_mux);
}

