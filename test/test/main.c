#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ch.h"
#include "hal.h"
#include "memory_protection.h"
#include <usbcfg.h>
#include <main.h>
#include <chprintf.h>
#include "sensors/proximity.h"
#include <ir_collision_detector.h>
#include "leds.h"
#include "spi_comm.h"
#include "motors.h"

messagebus_t bus;
MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void serial_start(void)
{
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};

	sdStart(&SD3, &ser_cfg); // UART3.
}

//static void timer12_start(void){
//    //General Purpose Timer configuration
//    //timer 12 is a 16 bit timer so we can measure time
//    //to about 65ms with a 1Mhz counter
//    static const GPTConfig gpt12cfg = {
//        1000000,        /* 1MHz timer clock in order to measure uS.*/
//        NULL,           /* Timer callback.*/
//        0,
//        0
//    };
//
//    gptStart(&GPTD12, &gpt12cfg);
//    //let the timer count to max value
//    gptStartContinuous(&GPTD12, 0xFFFF);
//}


void LED_update(uint8_t collision_states){
	for(int i=0;i<PROXIMITY_NB_CHANNELS;i++){
		uint8_t led_value = 0;
		if((collision_states & (1<<i)) == (1<<i)){
			led_value = 1;
		}
		switch(i){
		case 0:
			set_body_led(led_value);
			break;
		case 1:
			set_rgb_led(LED2,0,0,led_value*RGB_MAX_INTENSITY);
			break;
		case 2:
			set_led(LED3,led_value);
			break;
		case 3:
			set_rgb_led(LED4,led_value*RGB_MAX_INTENSITY,0,led_value*RGB_MAX_INTENSITY);
			break;
		case 4:
			set_rgb_led(LED6,0,led_value*RGB_MAX_INTENSITY,led_value*RGB_MAX_INTENSITY);
			break;
		case 5:
			set_led(LED7,led_value);
			break;
		case 6:
			set_rgb_led(LED8,led_value*RGB_MAX_INTENSITY,led_value*RGB_MAX_INTENSITY,0);
			break;
		case 7:
			set_led(LED1,led_value);
			break;
		}
	}
}

int main(void)
{
	halInit();
	chSysInit();
	mpu_init();
	spi_comm_start();

	//starts the serial communication
	serial_start();
	//starts the USB communication

	usb_start();
	motors_init();

	proximity_start();


	messagebus_init(&bus, &bus_lock, &bus_condvar);

	messagebus_topic_t *prox_topic = messagebus_find_topic_blocking(&bus, "/proximity");
	proximity_msg_t prox_values;
	calibrate_ir();

	left_motor_set_speed(MOTOR_SPEED_LIMIT);
	right_motor_set_speed(-MOTOR_SPEED_LIMIT);


    while(1){
    	messagebus_topic_wait(prox_topic, &prox_values, sizeof(prox_values));
    	chprintf((BaseSequentialStream *)&SD3,"%Proximity=%-7d Calib. Proximity=%-7d Ambient light=%-7d \r\n"
    			,prox_values.delta[2],prox_values.delta[2]-prox_values.initValue[2],prox_values.ambient[2]);
    	uint8_t collision_states = collision_detection(&prox_values);
    	LED_update(collision_states);
    	chThdSleepMilliseconds(10);
    }
}




#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
