#ifndef LSM303_H_
#define LSM303_H_

#include "sensors.h"

extern void (*sensors_states_listener)(sensor_state_t* sensors_states, uint8_t sensors_count);

void movement_sensor_disable(void);
void movement_sensor_enable(void);

void movement_sensor_milisecond_listener(void);

void add_sensors_states_listener(void (*listener)(sensor_state_t* sensors_states, uint8_t sensors_count));

bool LSM303_init(void);
void LSM303_poll(void);

bool waiting_movement_sensor_enable_timeout(void);

void disable_movement(void);
void enable_movement(void);

int16_t GetX();
int16_t GetY();
int16_t GetZ();

int16_t GetAcceleration();

#endif /* LSM303_H_ */
