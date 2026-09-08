#ifndef Mona_ESP_encoder_h
#define Mona_ESP_encoder_h

#include <stdint.h>

void encoder_init(int right_a, int right_b, int left_a, int left_b);
int32_t Encoder_left(void);
int32_t Encoder_right(void);
void Encoder_reset(void);

#endif
