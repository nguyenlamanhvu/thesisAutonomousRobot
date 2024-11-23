#include "fuzzy.h"

#define PVS_weight 1
#define PMS_weight 1
#define PM_weight 1
#define PL_weight 1
#define PVL_weight 1

typedef struct {
    float PVS;
    float PMS;
    float PM;
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
            else if (strcmp(output, "PL") == 0) out_variable.PL = fmaxf(out_variable.PL, rule_strength);
            else if (strcmp(output, "PVL") == 0) out_variable.PVL = fmaxf(out_variable.PVL, rule_strength);
        }
    }

    float Kd = (out_variable.PVS * PVS_weight + out_variable.PMS * PMS_weight +
                out_variable.PM * PM_weight + out_variable.PL * PL_weight +
                out_variable.PVL * PVL_weight) /
               (out_variable.PVS + out_variable.PMS + out_variable.PM +
                out_variable.PL + out_variable.PVL);
    return Kd;
}
