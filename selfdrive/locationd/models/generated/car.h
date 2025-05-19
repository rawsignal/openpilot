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
void car_err_fun(double *nom_x, double *delta_x, double *out_4320380359544859819);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1859743066527118969);
void car_H_mod_fun(double *state, double *out_5439349944232446224);
void car_f_fun(double *state, double dt, double *out_1730559994120671036);
void car_F_fun(double *state, double dt, double *out_7120253859189811687);
void car_h_25(double *state, double *unused, double *out_4764996439537147967);
void car_H_25(double *state, double *unused, double *out_535658119047781384);
void car_h_24(double *state, double *unused, double *out_2789116603674463020);
void car_H_24(double *state, double *unused, double *out_6039913687543736717);
void car_h_30(double *state, double *unused, double *out_3874881037181393917);
void car_H_30(double *state, double *unused, double *out_6381032222443835371);
void car_h_26(double *state, double *unused, double *out_1610202994159438538);
void car_H_26(double *state, double *unused, double *out_4277161437921837608);
void car_h_27(double *state, double *unused, double *out_1728210388092815282);
void car_H_27(double *state, double *unused, double *out_4206268910643410460);
void car_h_29(double *state, double *unused, double *out_1270009025934253620);
void car_H_29(double *state, double *unused, double *out_2492906183773859427);
void car_h_28(double *state, double *unused, double *out_2331526272769292799);
void car_H_28(double *state, double *unused, double *out_2589492833295671147);
void car_h_31(double *state, double *unused, double *out_4489802377252642078);
void car_H_31(double *state, double *unused, double *out_505012157170820956);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}