#include "fuzzy.h"

#define PVS_weight 	0.0000193
#define PMS_weight 	0.0000251
#define PM_weight 	0.0000302
#define PML_weight	0.0000451
#define PL_weight 	0.0000518
#define PVL_weight 	0.0000651

typedef struct {
    float PVS;
    float PMS;
    float PM;
    float PML;
    float PL;
    float PVL;
} fuzzy_output_variable_t;

const char *rule_base_Kd[5][5] = {
    {"PVS", "PMS", "PM", "PL", "PVL"},
    {"PMS", "PML", "PL", "PVL", "PVL"},
    {"PM",  "PL",  "PL", "PVL", "PVL"},
    {"PML", "PVL", "PVL", "PVL", "PVL"},
    {"PVL", "PVL", "PVL", "PVL", "PVL"}
};

fuzzy_input_variable_t Kd_calculate_e(float error)
{
	fuzzy_input_variable_t KdError;

	if(error <= NL_error)
	{
		KdError.NL = 1;
		KdError.NS = 0;
		KdError.ZE = 0;
		KdError.PS = 0;
		KdError.PL = 0;
	}
	else if(error > NL_error || error <= PL_error)
	{
		KdError.NL = fuzzy_calculate_tamgiac(error, NL_error - 100, NS_error, NL_error);
		KdError.NS = fuzzy_calculate_tamgiac(error, NL_error, 0, NS_error);
		KdError.ZE = fuzzy_calculate_tamgiac(error, NS_error, PS_error, 0);
		KdError.PS = fuzzy_calculate_tamgiac(error, 0, PL_error, PS_error);
		KdError.PL = fuzzy_calculate_tamgiac(error, PS_error, PL_error + 100, PL_error);
	}
	else
	{
		KdError.NL = 0;
		KdError.NS = 0;
		KdError.ZE = 0;
		KdError.PS = 0;
		KdError.PL = 1;
	}

	return KdError;
}

fuzzy_input_variable_t Kd_calculate_ce(float cerror)
{
	fuzzy_input_variable_t KdCError;

	if(cerror <= NL_cerror)
	{
		KdCError.NL = 1;
		KdCError.NS = 0;
		KdCError.ZE = 0;
		KdCError.PS = 0;
		KdCError.PL = 0;
	}
	else if(cerror > NL_cerror || cerror <= PL_cerror)
	{
		KdCError.NL = fuzzy_calculate_tamgiac(cerror, NL_cerror - 100, NS_cerror, NL_cerror);
		KdCError.NS = fuzzy_calculate_tamgiac(cerror, NL_cerror, 0, NS_cerror);
		KdCError.ZE = fuzzy_calculate_tamgiac(cerror, NS_cerror, PS_cerror, 0);
		KdCError.PS = fuzzy_calculate_tamgiac(cerror, 0, PL_cerror, PS_cerror);
		KdCError.PL = fuzzy_calculate_tamgiac(cerror, PS_cerror, PL_cerror + 100, PL_cerror);
	}
	else
	{
		KdCError.NL = 0;
		KdCError.NS = 0;
		KdCError.ZE = 0;
		KdCError.PS = 0;
		KdCError.PL = 1;
	}

	return KdCError;
}

float Kd_calculate(fuzzy_input_variable_t e, fuzzy_input_variable_t ce) {
    fuzzy_output_variable_t out_variable = {0};

    float memberships[5] = {e.NL, e.NS, e.ZE, e.PS, e.PL};
    float ce_memberships[5] = {ce.NL, ce.NS, ce.ZE, ce.PS, ce.PL};

    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            float rule_strength = fminf(memberships[i], ce_memberships[j]);
            const char *output = rule_base_Kd[i][j];

            if (strcmp(output, "PVS") == 0) out_variable.PVS = fmaxf(out_variable.PVS, rule_strength);
            else if (strcmp(output, "PMS") == 0) out_variable.PMS = fmaxf(out_variable.PMS, rule_strength);
            else if (strcmp(output, "PM") == 0) out_variable.PM = fmaxf(out_variable.PM, rule_strength);
            else if (strcmp(output, "PML") == 0) out_variable.PML = fmaxf(out_variable.PML, rule_strength);
            else if (strcmp(output, "PL") == 0) out_variable.PL = fmaxf(out_variable.PL, rule_strength);
            else if (strcmp(output, "PVL") == 0) out_variable.PVL = fmaxf(out_variable.PVL, rule_strength);
        }
    }

    float Kd = (out_variable.PVS * PVS_weight + out_variable.PMS * PMS_weight +
                out_variable.PM * PM_weight + out_variable.PL * PL_weight +
                out_variable.PVL * PVL_weight + out_variable.PML * PML_weight) /
               (out_variable.PVS + out_variable.PMS + out_variable.PM +
                out_variable.PL + out_variable.PVL + out_variable.PML);
    return Kd;
}
