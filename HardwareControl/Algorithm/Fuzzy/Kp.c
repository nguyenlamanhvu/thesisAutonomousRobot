#include "fuzzy.h"

#define PVL_weight 1
#define PML_weight 1
#define PVS_weight 1
#define PMS_weight 1
#define PL_weight 1
#define PS_weight 1

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
