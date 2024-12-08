#include "fuzzy.h"

#define PMS_weight 	8.899152
#define PM_weight 	9.397153
#define PS_weight 	8.010561
#define PVS_weight	7.351712

typedef struct {
    float PMS;
    float PS;
    float PM;
    float PVS;
} fuzzy_output_variable_t;

const char *rule_base_Ki[5][5] = {
    {"PM", "PM", "PM", "PM", "PM"},
    {"PMS", "PMS", "PMS", "PMS", "PMS"},
    {"PS", "PS", "PVS", "PS", "PS"},
    {"PMS", "PMS", "PMS", "PMS", "PMS"},
    {"PM", "PM", "PM", "PM", "PM"}
};

fuzzy_input_variable_t Ki_calculate_e(float error)
{
	fuzzy_input_variable_t KiError;

	if(error <= NL_error)
	{
		KiError.NL = 1;
		KiError.NS = 0;
		KiError.ZE = 0;
		KiError.PS = 0;
		KiError.PL = 0;
	}
	else if(error > NL_error || error <= PL_error)
	{
		KiError.NL = fuzzy_calculate_tamgiac(error, NL_error - 100, NS_error, NL_error);
		KiError.NS = fuzzy_calculate_tamgiac(error, NL_error, 0, NS_error);
		KiError.ZE = fuzzy_calculate_tamgiac(error, NS_error, PS_error, 0);
		KiError.PS = fuzzy_calculate_tamgiac(error, 0, PL_error, PS_error);
		KiError.PL = fuzzy_calculate_tamgiac(error, PS_error, PL_error + 100, PL_error);
	}
	else
	{
		KiError.NL = 0;
		KiError.NS = 0;
		KiError.ZE = 0;
		KiError.PS = 0;
		KiError.PL = 1;
	}

	return KiError;
}

fuzzy_input_variable_t Ki_calculate_ce(float cerror)
{
	fuzzy_input_variable_t KiCError;

	if(cerror <= NL_cerror)
	{
		KiCError.NL = 1;
		KiCError.NS = 0;
		KiCError.ZE = 0;
		KiCError.PS = 0;
		KiCError.PL = 0;
	}
	else if(cerror > NL_cerror || cerror <= PL_cerror)
	{
		KiCError.NL = fuzzy_calculate_tamgiac(cerror, NL_cerror - 100, NS_cerror, NL_cerror);
		KiCError.NS = fuzzy_calculate_tamgiac(cerror, NL_cerror, 0, NS_cerror);
		KiCError.ZE = fuzzy_calculate_tamgiac(cerror, NS_cerror, PS_cerror, 0);
		KiCError.PS = fuzzy_calculate_tamgiac(cerror, 0, PL_cerror, PS_cerror);
		KiCError.PL = fuzzy_calculate_tamgiac(cerror, PS_cerror, PL_cerror + 100, PL_cerror);
	}
	else
	{
		KiCError.NL = 0;
		KiCError.NS = 0;
		KiCError.ZE = 0;
		KiCError.PS = 0;
		KiCError.PL = 1;
	}

	return KiCError;
}


float Ki_calculate(fuzzy_input_variable_t e, fuzzy_input_variable_t ce) {
    fuzzy_output_variable_t out_variable = {0};

    float memberships[5] = {e.NL, e.NS, e.ZE, e.PS, e.PL};
    float ce_memberships[5] = {ce.NL, ce.NS, ce.ZE, ce.PS, ce.PL};

    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            float rule_strength = fminf(memberships[i], ce_memberships[j]);
            const char *output = rule_base_Ki[i][j];

            if (strcmp(output, "PM") == 0) out_variable.PM = fmaxf(out_variable.PM, rule_strength);
            else if (strcmp(output, "PMS") == 0) out_variable.PMS = fmaxf(out_variable.PMS, rule_strength);
            else if (strcmp(output, "PS") == 0) out_variable.PS = fmaxf(out_variable.PS, rule_strength);
            else if (strcmp(output, "PVS") == 0) out_variable.PVS = fmaxf(out_variable.PVS, rule_strength);
        }
    }

    float Ki = 0;
    Ki = (out_variable.PM * PM_weight + out_variable.PMS * PMS_weight + out_variable.PS * PS_weight + out_variable.PVS * PVS_weight) /
         (out_variable.PM + out_variable.PMS + out_variable.PS + out_variable.PVS);
    return Ki;
}
