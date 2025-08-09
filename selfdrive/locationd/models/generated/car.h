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
void car_err_fun(double *nom_x, double *delta_x, double *out_1831389753275826188);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4290228637053937851);
void car_H_mod_fun(double *state, double *out_2619348439350555686);
void car_f_fun(double *state, double dt, double *out_849567269053238190);
void car_F_fun(double *state, double dt, double *out_9107872014031309504);
void car_h_25(double *state, double *unused, double *out_5454240522955423473);
void car_H_25(double *state, double *unused, double *out_9116371399240585637);
void car_h_24(double *state, double *unused, double *out_8007148282328552359);
void car_H_24(double *state, double *unused, double *out_8859915192425627255);
void car_h_30(double *state, double *unused, double *out_9091690051209445823);
void car_H_30(double *state, double *unused, double *out_9201033727325725909);
void car_h_26(double *state, double *unused, double *out_148850902318785726);
void car_H_26(double *state, double *unused, double *out_5588869355594909755);
void car_h_27(double *state, double *unused, double *out_7990830283093282498);
void car_H_27(double *state, double *unused, double *out_7026270415525300998);
void car_h_29(double *state, double *unused, double *out_4129146806664780846);
void car_H_29(double *state, double *unused, double *out_5312907688655749965);
void car_h_28(double *state, double *unused, double *out_727006636775409828);
void car_H_28(double *state, double *unused, double *out_230508671586219391);
void car_h_31(double *state, double *unused, double *out_5531944786505239818);
void car_H_31(double *state, double *unused, double *out_9085725437363625209);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}