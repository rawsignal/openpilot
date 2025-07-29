#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_3543435702888968549);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2416974275063884114);
void pose_H_mod_fun(double *state, double *out_8476558897598068373);
void pose_f_fun(double *state, double dt, double *out_4641425989573363532);
void pose_F_fun(double *state, double dt, double *out_8960990753670273174);
void pose_h_4(double *state, double *unused, double *out_6682348964168201837);
void pose_H_4(double *state, double *unused, double *out_261637334484920765);
void pose_h_10(double *state, double *unused, double *out_5527329664152594749);
void pose_H_10(double *state, double *unused, double *out_1417237105866251151);
void pose_h_13(double *state, double *unused, double *out_7750677098364519715);
void pose_H_13(double *state, double *unused, double *out_3473911159817253566);
void pose_h_14(double *state, double *unused, double *out_4559486557289584562);
void pose_H_14(double *state, double *unused, double *out_4224878190824405294);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}