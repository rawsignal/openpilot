#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_455832145760308814);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6833442360289991349);
void pose_H_mod_fun(double *state, double *out_4443372719987989996);
void pose_f_fun(double *state, double dt, double *out_7271509597229759153);
void pose_F_fun(double *state, double dt, double *out_8398366488676661931);
void pose_h_4(double *state, double *unused, double *out_1122190748750831614);
void pose_H_4(double *state, double *unused, double *out_8215179929831897568);
void pose_h_10(double *state, double *unused, double *out_7012370241110020883);
void pose_H_10(double *state, double *unused, double *out_5502004992437324718);
void pose_h_13(double *state, double *unused, double *out_354175508569982951);
void pose_H_13(double *state, double *unused, double *out_7019290318545321247);
void pose_h_14(double *state, double *unused, double *out_8860411994425898009);
void pose_H_14(double *state, double *unused, double *out_6268323287538169519);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}