#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_5394071911806177632);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_390924532607337478);
void pose_H_mod_fun(double *state, double *out_9013755718701150594);
void pose_f_fun(double *state, double dt, double *out_559878194513068250);
void pose_F_fun(double *state, double dt, double *out_3177690484241037676);
void pose_h_4(double *state, double *unused, double *out_3640963725510673355);
void pose_H_4(double *state, double *unused, double *out_3584906548256442301);
void pose_h_10(double *state, double *unused, double *out_2998629541313996374);
void pose_H_10(double *state, double *unused, double *out_1479014506561746056);
void pose_h_13(double *state, double *unused, double *out_7662457042295327166);
void pose_H_13(double *state, double *unused, double *out_6797180373588775102);
void pose_h_14(double *state, double *unused, double *out_9220208786383067342);
void pose_H_14(double *state, double *unused, double *out_7548147404595926830);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}