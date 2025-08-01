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
void car_err_fun(double *nom_x, double *delta_x, double *out_4865063124797718752);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_142399555576515117);
void car_H_mod_fun(double *state, double *out_7845543819884407850);
void car_f_fun(double *state, double dt, double *out_2675724449816003272);
void car_F_fun(double *state, double dt, double *out_3848783217205820649);
void car_h_25(double *state, double *unused, double *out_2109339641109411266);
void car_H_25(double *state, double *unused, double *out_9149882930076379470);
void car_h_24(double *state, double *unused, double *out_1220963728804744999);
void car_H_24(double *state, double *unused, double *out_764637571092593037);
void car_h_30(double *state, double *unused, double *out_1752343835319090386);
void car_H_30(double *state, double *unused, double *out_9167522196489932076);
void car_h_26(double *state, double *unused, double *out_7237427866659922841);
void car_H_26(double *state, double *unused, double *out_8493028865966067566);
void car_h_27(double *state, double *unused, double *out_1127255547774113154);
void car_H_27(double *state, double *unused, double *out_6992758884689507165);
void car_h_29(double *state, double *unused, double *out_6754017102979936031);
void car_H_29(double *state, double *unused, double *out_8768990532905227356);
void car_h_28(double *state, double *unused, double *out_6089635545455406386);
void car_H_28(double *state, double *unused, double *out_6805360261339901105);
void car_h_31(double *state, double *unused, double *out_8680365730020570467);
void car_H_31(double *state, double *unused, double *out_9119236968199419042);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}