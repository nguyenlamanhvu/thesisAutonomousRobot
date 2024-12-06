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

#define NL_error	-40
#define NS_error	-5
#define PS_error	5
#define PL_error	40

#define NL_cerror	-3.11
#define NS_cerror	-0.18
#define PS_cerror	0.18
#define PL_cerror	3.11

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
float fuzzy_calculate_tamgiac(float x, float left, float right, float c);
fuzzy_input_variable_t Kp_calculate_e(float error);
fuzzy_input_variable_t Kp_calculate_ce(float cerror);
fuzzy_input_variable_t Ki_calculate_e(float error);
fuzzy_input_variable_t Ki_calculate_ce(float cerror);
fuzzy_input_variable_t Kd_calculate_e(float error);
fuzzy_input_variable_t Kd_calculate_ce(float cerror);
float Kp_calculate(fuzzy_input_variable_t e, fuzzy_input_variable_t ce);
float Ki_calculate(fuzzy_input_variable_t e, fuzzy_input_variable_t ce);
float Kd_calculate(fuzzy_input_variable_t e, fuzzy_input_variable_t ce);

#endif /* ALGORITHM_FUZZY_FUZZY_H_ */
