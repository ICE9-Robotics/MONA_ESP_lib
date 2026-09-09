#ifndef Mona_ESP_motor_sync_h
#define Mona_ESP_motor_sync_h

void motor_sync_init(void);
void motor_set_left(int signed_pwm);
void motor_set_right(int signed_pwm);

#endif
