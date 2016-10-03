#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "clock.h"
#include "RTC.h"
#include "twi.h"
#include "chrono.h"
#include "clock.h"
#include "test.h"
#include "logger.h"
#include "sensors.h"
#include "config.h"
#include "Sensors/LSM303.h"

#include "config/conf_os.h"

#include "event_buffer.h"

#define MOVEMENT_SENSOR_DISABLED_TIME 2000 // ms
#define X_AXIS_MASK	0x03
#define Y_AXIS_MASK	0x0C
#define Z_AXIS_MASK	0x30

#define ADDRESS_MASK 0x80

static volatile uint16_t movement_sensor_disabled_timeout = 0;

static bool sensor_detected = false;
static bool sensor_ready = false;

static uint8_t twi_buff[8];
static char tmp_data[20];

static volatile int	compass_X;
static volatile int	compass_Y;
static volatile int	compass_Z;

void (*sensors_states_listener)(sensor_state_t* sensors_states, uint8_t sensors_count) = NULL;

static char I2C_ReadRegisterLSM(char rAddr) {
	tmp_data[0] = rAddr;
	bool  timeout=false;

	TWI_MasterWriteRead(&sensor_twi, 0x19, tmp_data, 1, 1);
	start_auxTimeout(10);
	while ((sensor_twi.status != TWIM_STATUS_READY) && !timeout) {timeout=read_auxTimeout();}
	if(timeout || (sensor_twi.result != TWIM_RESULT_OK));
	return sensor_twi.readData[0];
}

int16_t GetX()
{
	uint8_t xlow = I2C_ReadRegisterLSM(0xA8);
	uint8_t xhigh = I2C_ReadRegisterLSM(0xA9);

	int16_t xaxis = (int16_t)((xlow | (xhigh << 8)));

	//LOG_PRINT(1,PSTR("X axis high : 0x%x\n"),xhigh);
	//LOG_PRINT(1,PSTR("X axis low : 0x%x\n"),xlow);

	xaxis = ((xaxis - 60) >> 14) + 1;

	LOG_PRINT(1,PSTR("X axis : %d\n"),xaxis);

	return xaxis;
}

int16_t GetY()
{
	uint8_t ylow = I2C_ReadRegisterLSM(0xAA);
	uint8_t yhigh = I2C_ReadRegisterLSM(0xAB);

	int16_t yaxis = (int16_t)((ylow | (yhigh << 8)));

	//LOG_PRINT(1,PSTR("Y axis high : 0x%x\n"),yhigh);
	//LOG_PRINT(1,PSTR("Y axis low : 0x%x\n"),ylow);

	yaxis = ((yaxis - 60) >> 14) + 1;

	LOG_PRINT(1,PSTR("Y axis : %d\n"),yaxis);

	return yaxis;
}

int16_t GetZ()
{
	uint8_t zlow = I2C_ReadRegisterLSM(0xAC);
	uint8_t zhigh = I2C_ReadRegisterLSM(0xAD);

	int16_t zaxis = (int16_t)((zlow | (zhigh << 8)));

	//LOG_PRINT(1,PSTR("Z axis high : 0x%x\n"),zhigh);
	//LOG_PRINT(1,PSTR("Z axis low : 0x%x\n"),zlow);

	zaxis = ((zaxis - 60) >> 14) + 1;

	LOG_PRINT(1,PSTR("Z axis : %d\n"),zaxis);

	return zaxis;
}

bool LSM303_init(void) {
	bool  timeout=false;
	start_auxTimeout(10);
	while ((sensor_twi.status != TWIM_STATUS_READY) && !timeout) {timeout=read_auxTimeout();}
	if(timeout) return false;

	LOG(3,"LSM303 - 1");
	twi_buff[0] = 0x80 + 0x20;
	twi_buff[1] = 0b01110111;		// REG1, 400Hz, Normal Mode, ZYX enabled
	twi_buff[2] = 0b00000000;		// REG2,
	twi_buff[3] = 0b01000000;		// REG3, AOI1 INT1 enabled
	twi_buff[4] = 0b10001000;		// REG4, BDU enabled, High Resolution
	twi_buff[5] = 0b00001100;		// REG5, LIR INT1, D4D INT1 enabled
	twi_buff[6] = 0b00000000;		// REG6. interrupt active high
	twi_buff[7] = 0b00000001;		// REFERENCE_A
	TWI_MasterWriteRead(&sensor_twi, 0x19, twi_buff, 8, 0);
	start_auxTimeout(10);
	while ((sensor_twi.status != TWIM_STATUS_READY) && !timeout) {timeout=read_auxTimeout();}
	if(timeout || (sensor_twi.result != TWIM_RESULT_OK)) return false;

	uint8_t reg1 = I2C_ReadRegisterLSM(0x20);
	LOG_PRINT(1,PSTR("REG1 : 0x%x\n\n"),reg1);

	uint8_t reg2 = I2C_ReadRegisterLSM(0x21);
	LOG_PRINT(1,PSTR("REG2 : 0x%x\n\n"),reg2);

	uint8_t reg3 = I2C_ReadRegisterLSM(0x22);
	LOG_PRINT(1,PSTR("REG3 : 0x%x\n\n"),reg3);

	uint8_t reg4 = I2C_ReadRegisterLSM(0x23);
	LOG_PRINT(1,PSTR("REG4 : 0x%x\n\n"),reg4);

	uint8_t reg5 = I2C_ReadRegisterLSM(0x24);
	LOG_PRINT(1,PSTR("REG5 : 0x%x\n\n"),reg5);

	uint8_t reg6 = I2C_ReadRegisterLSM(0x25);
	LOG_PRINT(1,PSTR("REG6B : 0x%x\n\n"),reg6);

	LOG(3,"LSM303 - 2");
	twi_buff[0] = 0x80 + 0x30;
	twi_buff[1] = 0b11111111;		// INT1_CFG_A, ZYX high interrupt enabled
	TWI_MasterWriteRead(&sensor_twi, 0x19, twi_buff, 2, 0);
	start_auxTimeout(10);
	while ((sensor_twi.status != TWIM_STATUS_READY) && !timeout) {timeout=read_auxTimeout();}
	if(timeout || (sensor_twi.result != TWIM_RESULT_OK)) return false;

	LOG(3,"LSM303 - 3");
	twi_buff[0] = 0x80 + 0x32;
	twi_buff[1] = 0b11001101;		// INT1_THS, threshold
	twi_buff[2] = 0b00000001;		// INT1_DURATION, duration
	TWI_MasterWriteRead(&sensor_twi, 0x19, twi_buff, 3, 0);
	start_auxTimeout(10);
	while ((sensor_twi.status != TWIM_STATUS_READY) && !timeout) {timeout=read_auxTimeout();}
	if(timeout || (sensor_twi.result != TWIM_RESULT_OK)) return false;

	sensor_detected = true;

	uint8_t cfg_reg = I2C_ReadRegisterLSM(0x30);
	LOG_PRINT(1,PSTR("CFG : 0x%x\n"),cfg_reg);

	// enable external rising edge interrupt for LSM303
	PORTB.DIR &= 0b11111011;
	PORTB.INT0MASK = 0b00000100;		// LSM303 interrupt is in int0 group
	PORTB.PIN2CTRL = 0b00010001;		// pull-down, detect rising edge
	PORTB.INTFLAGS = 0b00000001;		// clear pending int0 interrupts
	PORTB.INTCTRL |= 0b00000011;		// interrupt 0, high level
	LOG(1,"LSM303 detected");

	GetX();
	GetY();
	GetZ();

	movement_sensor_enable();

	return true;
}


void LSM303_poll(void) 
{
	LOG(2,"LSM poll start");

	if (!sensor_detected){
		LOG(1,"LSM not processed");
		return;
	}

	bool  timeout=false;
	start_auxTimeout(10);
	while ((sensor_twi.status != TWIM_STATUS_READY) && !timeout) {timeout=read_auxTimeout();}
	if(timeout) return;

	LOG(2,"LSM poll 1");

	uint8_t src_reg = I2C_ReadRegisterLSM(0x31);
	LOG_PRINT(1,PSTR("INT1_SRC_REG : 0x%x\n\n"),src_reg);

	LOG_PRINT(1,PSTR("X interrupts : 0x%x\n"),(src_reg & X_AXIS_MASK));
	LOG_PRINT(1,PSTR("Y interrupts : 0x%x\n"),(src_reg & Y_AXIS_MASK));
	LOG_PRINT(1,PSTR("Z interrupts : 0x%x\n"),(src_reg & Z_AXIS_MASK));

	LOG(1,"LSM303 ready");

	uint8_t status_reg = I2C_ReadRegisterLSM(0x27);
	LOG_PRINT(1,PSTR("Status : 0x%x\n"),status_reg);

	GetX();
	GetY();
	GetZ();

	sensor_ready = true;

}

ISR(PORTB_INT0_vect) 
{
	LOG(2,"LSM303 interrupt !!!!");
	if (sensor_detected && sensor_ready)
	{
		LOG(1,"LSM303 moved !!!!");
		
		movement_sensor_disable();
		
		movement_sensor_disabled_timeout = MOVEMENT_SENSOR_DISABLED_TIME;
		
		if(movement_status)
		{
			sensor_state_t movement_sensor_state;
			movement_sensor_state.id = 'M';
			movement_sensor_state.value = 1;
			
			if(sensors_states_listener) sensors_states_listener(&movement_sensor_state, 1);
		}
	}
}

void movement_sensor_disable(void)
{
	sensor_ready = false;
}

void movement_sensor_enable(void)
{
	LSM303_poll();
}

void movement_sensor_milisecond_listener(void)
{
	if(movement_sensor_disabled_timeout)
	{
		if((--movement_sensor_disabled_timeout == 0))
		{
			LOG(1, "Movement sensor enable");
			
			movement_sensor_enable();
		}
	}
}

void add_sensors_states_listener(void (*listener)(sensor_state_t* sensors_states, uint8_t sensors_count))
{
	sensors_states_listener = listener;
}

bool waiting_movement_sensor_enable_timeout(void)
{
	return movement_status && (movement_sensor_disabled_timeout > 0);
}


void disable_movement(void)
{
	bool  timeout=false;
	
	start_auxTimeout(10);
	while ((sensor_twi.status != TWIM_STATUS_READY) && !timeout) {timeout=read_auxTimeout();}
	if(timeout) return false;
	
	LOG(1,"Disable movement sensor");
	twi_buff[0] = 0x80 + 0x20;
	twi_buff[1] = 0b00000000;	
	TWI_MasterWriteRead(&sensor_twi, 0x19, twi_buff, 2, 0);
}

void enable_movement(void)
{
	bool  timeout=false;
	
	start_auxTimeout(10);
	while ((sensor_twi.status != TWIM_STATUS_READY) && !timeout) {timeout=read_auxTimeout();}
	if(timeout) return false;
	
	LOG(1,"Enable movement sensor");
	twi_buff[0] = 0x80 + 0x20;
	twi_buff[1] = 0b01010111;	
	TWI_MasterWriteRead(&sensor_twi, 0x19, twi_buff, 2, 0);
}