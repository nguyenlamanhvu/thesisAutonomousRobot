#include "fuzzy.h"

#define PVL_weight 2.1845
#define PML_weight 1.734
#define PVS_weight 0.978
#define PMS_weight 1.067
#define PL_weight 2.0775
#define PS_weight 1.09175

typedef struct {
    float PVL;
    float PML;
    float PVS;
    float PMS;
    float PL;
    float PS;
} fuzzy_output_variable_t;

const char *rule_base[5][5] = {
    {"PVL", "PVL", "PVL", "PVL", "PVL"},
    {"PML", "PML", "PML", "PL", "PVL"},
    {"PVS", "PVS", "PS", "PMS", "PMS"},
    {"PML", "PML", "PML", "PL", "PVL"},
    {"PVL", "PVL", "PVL", "PVL", "PVL"}
};

fuzzy_input_variable_t Kp_calculate_e(float error)
{
	fuzzy_input_variable_t KpError;

	if(error <= NL_error)
	{
		KpError.NL = 1;
		KpError.NS = 0;
		KpError.ZE = 0;
		KpError.PS = 0;
		KpError.PL = 0;
	}
	else if(error > NL_error || error <= PL_error)
	{
		KpError.NL = fuzzy_calculate_tamgiac(error, NL_error - 100, NS_error, NL_error);
		KpError.NS = fuzzy_calculate_tamgiac(error, NL_error, 0, NS_error);
		KpError.ZE = fuzzy_calculate_tamgiac(error, NS_error, PS_error, 0);
		KpError.PS = fuzzy_calculate_tamgiac(error, 0, PL_error, PS_error);
		KpError.PL = fuzzy_calculate_tamgiac(error, PS_error, PL_error + 100, PL_error);
	}
	else
	{
		KpError.NL = 0;
		KpError.NS = 0;
		KpError.ZE = 0;
		KpError.PS = 0;
		KpError.PL = 1;
	}

	return KpError;
}

fuzzy_input_variable_t Kp_calculate_ce(float cerror)
{
	fuzzy_input_variable_t KpCError;

	if(cerror <= NL_cerror)
	{
		KpCError.NL = 1;
		KpCError.NS = 0;
		KpCError.ZE = 0;
		KpCError.PS = 0;
		KpCError.PL = 0;
	}
	else if(cerror > NL_cerror || cerror <= PL_cerror)
	{
		KpCError.NL = fuzzy_calculate_tamgiac(cerror, NL_cerror - 100, NS_cerror, NL_cerror);
		KpCError.NS = fuzzy_calculate_tamgiac(cerror, NL_cerror, 0, NS_cerror);
		KpCError.ZE = fuzzy_calculate_tamgiac(cerror, NS_cerror, PS_cerror, 0);
		KpCError.PS = fuzzy_calculate_tamgiac(cerror, 0, PL_cerror, PS_cerror);
		KpCError.PL = fuzzy_calculate_tamgiac(cerror, PS_cerror, PL_cerror + 100, PL_cerror);
	}
	else
	{
		KpCError.NL = 0;
		KpCError.NS = 0;
		KpCError.ZE = 0;
		KpCError.PS = 0;
		KpCError.PL = 1;
	}

	return KpCError;
}

float Kp_calculate(fuzzy_input_variable_t e, fuzzy_input_variable_t ce) {
    fuzzy_output_variable_t out_variable = {0};

    float memberships[5] = {e.NL, e.NS, e.ZE, e.PS, e.PL};
    float ce_memberships[5] = {ce.NL, ce.NS, ce.ZE, ce.PS, ce.PL};

    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 5; ++j) {
            float rule_strength = fminf(memberships[i], ce_memberships[j]);
            const char *output = rule_base[i][j];

            if (strcmp(output, "PVL") == 0) out_variable.PVL = fmaxf(out_variable.PVL, rule_strength);
            else if (strcmp(output, "PML") == 0) out_variable.PML = fmaxf(out_variable.PML, rule_strength);
            else if (strcmp(output, "PVS") == 0) out_variable.PVS = fmaxf(out_variable.PVS, rule_strength);
            else if (strcmp(output, "PMS") == 0) out_variable.PMS = fmaxf(out_variable.PMS, rule_strength);
            else if (strcmp(output, "PL") == 0) out_variable.PL = fmaxf(out_variable.PL, rule_strength);
            else if (strcmp(output, "PS") == 0) out_variable.PS = fmaxf(out_variable.PS, rule_strength);
        }
    }
    float Kp = 0;
    Kp = (out_variable.PVL*PVL_weight + out_variable.PML*PML_weight + out_variable.PVS*PVS_weight + \
          out_variable.PMS*PMS_weight + out_variable.PL*PL_weight + out_variable.PS*PS_weight) / \
         (out_variable.PVL + out_variable.PML + out_variable.PVS + out_variable.PMS + out_variable.PL + out_variable.PS);
    return Kp;
}
