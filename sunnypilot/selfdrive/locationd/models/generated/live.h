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
void live_H(double *in_vec, double *out_5180415625229588290);
void live_err_fun(double *nom_x, double *delta_x, double *out_7194095639053651938);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_9161606507576483223);
void live_H_mod_fun(double *state, double *out_3127665888108330922);
void live_f_fun(double *state, double dt, double *out_2709373319494978951);
void live_F_fun(double *state, double dt, double *out_2969287643924474768);
void live_h_4(double *state, double *unused, double *out_6203496495406399804);
void live_H_4(double *state, double *unused, double *out_4420160522342671805);
void live_h_9(double *state, double *unused, double *out_3460053025642728910);
void live_H_9(double *state, double *unused, double *out_4178970875713081160);
void live_h_10(double *state, double *unused, double *out_2734909127549699712);
void live_H_10(double *state, double *unused, double *out_295343665185742427);
void live_h_12(double *state, double *unused, double *out_6565070252257529383);
void live_H_12(double *state, double *unused, double *out_599295885689289990);
void live_h_35(double *state, double *unused, double *out_6044413596240552461);
void live_H_35(double *state, double *unused, double *out_1053498464970064429);
void live_h_32(double *state, double *unused, double *out_3076508861503339695);
void live_H_32(double *state, double *unused, double *out_922333680029323960);
void live_h_13(double *state, double *unused, double *out_1999510100200081848);
void live_H_13(double *state, double *unused, double *out_8162527565365289399);
void live_h_14(double *state, double *unused, double *out_3460053025642728910);
void live_H_14(double *state, double *unused, double *out_4178970875713081160);
void live_h_33(double *state, double *unused, double *out_8676181226143291805);
void live_H_33(double *state, double *unused, double *out_2097058539668793175);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}