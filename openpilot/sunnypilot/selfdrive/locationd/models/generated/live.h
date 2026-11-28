#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_1578792126544681646);
void live_err_fun(double *nom_x, double *delta_x, double *out_5185604539909899997);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_8880148546557296848);
void live_H_mod_fun(double *state, double *out_3604788338706602481);
void live_f_fun(double *state, double dt, double *out_1491323239337397566);
void live_F_fun(double *state, double dt, double *out_1118098057884224731);
void live_h_4(double *state, double *unused, double *out_8709295013198719306);
void live_H_4(double *state, double *unused, double *out_5285986568993536268);
void live_h_9(double *state, double *unused, double *out_5899991510039587046);
void live_H_9(double *state, double *unused, double *out_2001232366270911202);
void live_h_10(double *state, double *unused, double *out_6373726741231044412);
void live_H_10(double *state, double *unused, double *out_6910203520008974495);
void live_h_12(double *state, double *unused, double *out_5459334715550524371);
void live_H_12(double *state, double *unused, double *out_2381141744688914224);
void live_h_35(double *state, double *unused, double *out_3703575679580372683);
void live_H_35(double *state, double *unused, double *out_1919324511620928892);
void live_h_32(double *state, double *unused, double *out_2918673097889252669);
void live_H_32(double *state, double *unused, double *out_1788159726680188423);
void live_h_13(double *state, double *unused, double *out_6133788781912484363);
void live_H_13(double *state, double *unused, double *out_963062708851720368);
void live_h_14(double *state, double *unused, double *out_5899991510039587046);
void live_H_14(double *state, double *unused, double *out_2001232366270911202);
void live_h_33(double *state, double *unused, double *out_991424358781111646);
void live_H_33(double *state, double *unused, double *out_1231232493017928712);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}