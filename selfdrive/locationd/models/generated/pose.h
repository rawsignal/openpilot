#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_6666156999559179096);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6383049424053173820);
void pose_H_mod_fun(double *state, double *out_7560663382806042444);
void pose_f_fun(double *state, double dt, double *out_4734719999270252373);
void pose_F_fun(double *state, double dt, double *out_7949110505019825749);
void pose_h_4(double *state, double *unused, double *out_3083781484230159567);
void pose_H_4(double *state, double *unused, double *out_4286441304015093191);
void pose_h_10(double *state, double *unused, double *out_1431314934865780613);
void pose_H_10(double *state, double *unused, double *out_4520041178384004627);
void pose_h_13(double *state, double *unused, double *out_1948613078602992934);
void pose_H_13(double *state, double *unused, double *out_6549671561377757496);
void pose_h_14(double *state, double *unused, double *out_7821491744597387773);
void pose_H_14(double *state, double *unused, double *out_8249682160354577720);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}