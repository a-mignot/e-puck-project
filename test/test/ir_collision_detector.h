#ifndef IR_COLLISION_DETECTOR_H
#define IR_COLLISION_DETECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "sensors/proximity.h"


/*
 * input : valeurs des capteurs de proximités
 * output: nombre binaire de 8 bits.
 *
 * Un 1 signifie qu'il y a une collision imminente
 * sur le capteur correspondant au rang du bit associé. Un 0 signifie qu'il n'y a pas
 * de détection de collision *
 */


uint8_t collision_detection(proximity_msg_t *prox_values);

#ifdef __cplusplus
}
#endif

#endif
