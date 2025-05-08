#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_5959269884630241271);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7281047554654548610);
void pose_H_mod_fun(double *state, double *out_6722182246202922039);
void pose_f_fun(double *state, double dt, double *out_5905758100805214049);
void pose_F_fun(double *state, double dt, double *out_8803043244110166487);
void pose_h_4(double *state, double *unused, double *out_4745763628231792235);
void pose_H_4(double *state, double *unused, double *out_4796039147815908771);
void pose_h_10(double *state, double *unused, double *out_6977620553933084530);
void pose_H_10(double *state, double *unused, double *out_3695487615244328658);
void pose_h_13(double *state, double *unused, double *out_7820235432546942220);
void pose_H_13(double *state, double *unused, double *out_1583765322483575970);
void pose_h_14(double *state, double *unused, double *out_2348946808153196977);
void pose_H_14(double *state, double *unused, double *out_832798291476424242);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}