#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_72747944530161733);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7058365865363402022);
void pose_H_mod_fun(double *state, double *out_6187177415367410681);
void pose_f_fun(double *state, double dt, double *out_8681455416555502143);
void pose_F_fun(double *state, double dt, double *out_3542904472886764692);
void pose_h_4(double *state, double *unused, double *out_4697645595739621983);
void pose_H_4(double *state, double *unused, double *out_2757184250690410661);
void pose_h_10(double *state, double *unused, double *out_8842851692336888185);
void pose_H_10(double *state, double *unused, double *out_5833963184882541613);
void pose_h_13(double *state, double *unused, double *out_7087867461883945740);
void pose_H_13(double *state, double *unused, double *out_455089574641922140);
void pose_h_14(double *state, double *unused, double *out_2045247785068682723);
void pose_H_14(double *state, double *unused, double *out_5839972682985782957);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}