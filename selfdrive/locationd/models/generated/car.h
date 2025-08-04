#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_6466759103940957948);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6301621981866168795);
void car_H_mod_fun(double *state, double *out_5475158372291201128);
void car_f_fun(double *state, double dt, double *out_1322705351199537539);
void car_F_fun(double *state, double dt, double *out_3283955945887505467);
void car_h_25(double *state, double *unused, double *out_7334748793472295430);
void car_H_25(double *state, double *unused, double *out_3718029853621036109);
void car_h_24(double *state, double *unused, double *out_8075960681212006981);
void car_H_24(double *state, double *unused, double *out_2936175604487149648);
void car_h_30(double *state, double *unused, double *out_3348730023416583745);
void car_H_30(double *state, double *unused, double *out_8245726183748644307);
void car_h_26(double *state, double *unused, double *out_4455149410379091890);
void car_H_26(double *state, double *unused, double *out_7459533172495092333);
void car_h_27(double *state, double *unused, double *out_5626911248896423731);
void car_H_27(double *state, double *unused, double *out_6022132112564701090);
void car_h_29(double *state, double *unused, double *out_8574637239251908692);
void car_H_29(double *state, double *unused, double *out_7735494839434252123);
void car_h_28(double *state, double *unused, double *out_4269015409183433725);
void car_H_28(double *state, double *unused, double *out_5628850217205768919);
void car_h_31(double *state, double *unused, double *out_7171839259411449720);
void car_H_31(double *state, double *unused, double *out_3687383891744075681);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}