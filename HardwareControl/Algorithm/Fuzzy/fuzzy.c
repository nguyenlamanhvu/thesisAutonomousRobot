/*
 * fuzzy.c
 *
 *  Created on: Nov 23, 2024
 *      Author: lamanhvu
 */

#include "fuzzy.h"

float membership_NL(float x) {
    if (x <= -1.0f) return 1.0f;
    else if (x < -0.5f) return 1.0f - 2.0f * (x + 1.0f);
    return 0.0f;
}

float membership_NS(float x) {
    if (x >= -1.0f && x < 0.0f) return x + 1.0f;
    else if (x >= 0.0f && x <= 0.5f) return 0.5f - x;
    return 0.0f;
}

float membership_ZE(float x) {
    if (x >= -0.5f && x <= 0.5f) return 1.0f - fabsf(x * 2.0f);
    return 0.0f;
}

float membership_PS(float x) {
    if (x >= -0.5f && x < 0.0f) return x + 0.5f;
    else if (x >= 0.0f && x <= 1.0f) return 1.0f - x;
    return 0.0f;
}

float membership_PL(float x) {
    if (x >= 0.5f) return 1.0f;
    else if (x > 0.0f) return 2.0f * x;
    return 0.0f;
}

fuzzy_input_variable_t get_fuzzy_label(float x) {
    fuzzy_input_variable_t x_value;
    x_value.NL = membership_NL(x);
    x_value.NS = membership_NS(x);
    x_value.ZE = membership_ZE(x);
    x_value.PS = membership_PS(x);
    x_value.PL = membership_PL(x);
    return x_value;
}

float fuzzy_calculate_tamgiac(float x, float left, float right, float c)
{
	if(x < left)		return 0;
	else if(x < c)		return ((x - left) / (c - left));
	else if(x < right)	return ((right - x) / (right - c));
	else				return 0;
}


