#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8989729873251700153);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5494395439268003082);
void pose_H_mod_fun(double *state, double *out_4818367292979666309);
void pose_f_fun(double *state, double dt, double *out_5283112497490037971);
void pose_F_fun(double *state, double dt, double *out_4041536850565338045);
void pose_h_4(double *state, double *unused, double *out_2536598407606234528);
void pose_H_4(double *state, double *unused, double *out_2430636927818656010);
void pose_h_10(double *state, double *unused, double *out_1438571128295234105);
void pose_H_10(double *state, double *unused, double *out_5878179630925345156);
void pose_h_13(double *state, double *unused, double *out_3737352406153045413);
void pose_H_13(double *state, double *unused, double *out_5642910753150988811);
void pose_h_14(double *state, double *unused, double *out_1881635978908402004);
void pose_H_14(double *state, double *unused, double *out_652151504476716286);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}