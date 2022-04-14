


#include "ir_collision_detector.h"

#define DEFAULT_IR_TRESHOLD 100
//Front sensors
#define IR0_THRESHOLD DEFAULT_IR_TRESHOLD
#define IR7_THRESHOLD IR0_THRESHOLD

//Front 45deg sensors
#define IR1_THRESHOLD DEFAULT_IR_TRESHOLD
#define IR6_THRESHOLD IR1_THRESHOLD

//Side sensors
#define IR2_THRESHOLD DEFAULT_IR_TRESHOLD
#define IR5_THRESHOLD IR2_THRESHOLD

//Back sensors
#define IR3_THRESHOLD DEFAULT_IR_TRESHOLD
#define IR4_THRESHOLD IR3_THRESHOLD


uint8_t collision_detection(proximity_msg_t *prox_values){
	uint8_t collision_state = 0;


	//creating a static table to make a more compact function
	static const int threshold_table[PROXIMITY_NB_CHANNELS] = {IR0_THRESHOLD,IR1_THRESHOLD,IR2_THRESHOLD,IR3_THRESHOLD,IR4_THRESHOLD,IR5_THRESHOLD,IR6_THRESHOLD,IR7_THRESHOLD};


	for(int i=0;i<PROXIMITY_NB_CHANNELS;i++){
		if(((int)prox_values->delta[i] - (int)prox_values->initValue[i]) >= threshold_table[i]){
			collision_state = (collision_state | (1 << i));
		}
	}

	return collision_state;
}


