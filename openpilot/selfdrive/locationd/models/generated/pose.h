#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_8365414901728243703);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_2264373635515068274);
void pose_H_mod_fun(double *state, double *out_9199109663248748241);
void pose_f_fun(double *state, double dt, double *out_465988394318989602);
void pose_F_fun(double *state, double dt, double *out_58762011279446305);
void pose_h_4(double *state, double *unused, double *out_2902652897630841860);
void pose_H_4(double *state, double *unused, double *out_7379410937342451380);
void pose_h_10(double *state, double *unused, double *out_2167868639468176046);
void pose_H_10(double *state, double *unused, double *out_476627251863467283);
void pose_h_13(double *state, double *unused, double *out_3170648018346895167);
void pose_H_13(double *state, double *unused, double *out_7855059311034767435);
void pose_h_14(double *state, double *unused, double *out_5521880719452377363);
void pose_H_14(double *state, double *unused, double *out_7104092280027615707);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}