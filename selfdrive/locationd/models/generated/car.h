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
void car_err_fun(double *nom_x, double *delta_x, double *out_327747029822113705);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7888979340653327288);
void car_H_mod_fun(double *state, double *out_4913543922609770225);
void car_f_fun(double *state, double dt, double *out_2453800355351253396);
void car_F_fun(double *state, double dt, double *out_2543684986104582723);
void car_h_25(double *state, double *unused, double *out_7132489347614403493);
void car_H_25(double *state, double *unused, double *out_5418878364810066937);
void car_h_24(double *state, double *unused, double *out_6600611664260682252);
void car_H_24(double *state, double *unused, double *out_9026153630268976046);
void car_h_30(double *state, double *unused, double *out_4543419499744797187);
void car_H_30(double *state, double *unused, double *out_6111175367407867924);
void car_h_26(double *state, double *unused, double *out_5486929352646459622);
void car_H_26(double *state, double *unused, double *out_1677375045936010713);
void car_h_27(double *state, double *unused, double *out_2130816079953557848);
void car_H_27(double *state, double *unused, double *out_8285938679208292835);
void car_h_29(double *state, double *unused, double *out_1506633448300464502);
void car_H_29(double *state, double *unused, double *out_5600944023093475740);
void car_h_28(double *state, double *unused, double *out_2870857187707287797);
void car_H_28(double *state, double *unused, double *out_7763401033546545302);
void car_h_31(double *state, double *unused, double *out_630775909206268948);
void car_H_31(double *state, double *unused, double *out_5449524326687027365);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}