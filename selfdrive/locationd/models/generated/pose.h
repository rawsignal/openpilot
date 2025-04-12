#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_2107472926716733093);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7606941711378929712);
void pose_H_mod_fun(double *state, double *out_2474305163268538487);
void pose_f_fun(double *state, double dt, double *out_6855082133400824491);
void pose_F_fun(double *state, double dt, double *out_5712260729693103006);
void pose_h_4(double *state, double *unused, double *out_7382434891101514169);
void pose_H_4(double *state, double *unused, double *out_2678213221770542248);
void pose_h_10(double *state, double *unused, double *out_3695414532195514518);
void pose_H_10(double *state, double *unused, double *out_4693822457708176897);
void pose_h_13(double *state, double *unused, double *out_1948308378304519843);
void pose_H_13(double *state, double *unused, double *out_5890487047102875049);
void pose_h_14(double *state, double *unused, double *out_3950222992602798817);
void pose_H_14(double *state, double *unused, double *out_6641454078110026777);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}