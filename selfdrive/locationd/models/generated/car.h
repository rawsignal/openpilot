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
void car_err_fun(double *nom_x, double *delta_x, double *out_2891105964222538946);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6776165792917660510);
void car_H_mod_fun(double *state, double *out_9150571167475995922);
void car_f_fun(double *state, double dt, double *out_3162943793073791679);
void car_F_fun(double *state, double dt, double *out_1510888866567458713);
void car_h_25(double *state, double *unused, double *out_1634459234320078510);
void car_H_25(double *state, double *unused, double *out_6659615981218129500);
void car_h_24(double *state, double *unused, double *out_4880598876319921497);
void car_H_24(double *state, double *unused, double *out_2559062906422226891);
void car_h_30(double *state, double *unused, double *out_1477272565968949365);
void car_H_30(double *state, double *unused, double *out_6530277034074889430);
void car_h_26(double *state, double *unused, double *out_873956681292471848);
void car_H_26(double *state, double *unused, double *out_2918112662344073276);
void car_h_27(double *state, double *unused, double *out_7347509937686762255);
void car_H_27(double *state, double *unused, double *out_4355513722274464519);
void car_h_29(double *state, double *unused, double *out_7948928095300125809);
void car_H_29(double *state, double *unused, double *out_2642150995404913486);
void car_h_28(double *state, double *unused, double *out_2345836670025916610);
void car_H_28(double *state, double *unused, double *out_2440248021664617088);
void car_h_31(double *state, double *unused, double *out_4585917948526935459);
void car_H_31(double *state, double *unused, double *out_6690261943095089928);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}