/*
 * fuzzy.h
 *
 *  Created on: Nov 23, 2024
 *      Author: lamanhvu
 */

#ifndef ALGORITHM_FUZZY_FUZZY_H_
#define ALGORITHM_FUZZY_FUZZY_H_

#include "stdint.h"
#include "math.h"
#include "errorCode.h"
#include "compilerSwitch.h"
#include <string.h>

typedef struct {
    float NL;
    float NS;
    float ZE;
    float PS;
    float PL;
} fuzzy_input_variable_t;

float membership_NL(float x);
float membership_NS(float x);
float membership_ZE(float x);
float membership_PS(float x);
float membership_PL(float x);
fuzzy_input_variable_t get_fuzzy_label(float x) ;

#endif /* ALGORITHM_FUZZY_FUZZY_H_ */
