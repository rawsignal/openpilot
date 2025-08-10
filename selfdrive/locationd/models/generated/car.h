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
void car_err_fun(double *nom_x, double *delta_x, double *out_7581984428922518495);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7458254433530772005);
void car_H_mod_fun(double *state, double *out_4805202653021023192);
void car_f_fun(double *state, double dt, double *out_603278022302126708);
void car_F_fun(double *state, double dt, double *out_2142672846904298435);
void car_h_25(double *state, double *unused, double *out_47232026210041201);
void car_H_25(double *state, double *unused, double *out_1691820306869006759);
void car_h_24(double *state, double *unused, double *out_8495854549207496621);
void car_H_24(double *state, double *unused, double *out_6560635171896713611);
void car_h_30(double *state, double *unused, double *out_9184949310893892383);
void car_H_30(double *state, double *unused, double *out_6219516636996614957);
void car_h_26(double *state, double *unused, double *out_6996661668192902125);
void car_H_26(double *state, double *unused, double *out_5433323625743062983);
void car_h_27(double *state, double *unused, double *out_3084018077654373886);
void car_H_27(double *state, double *unused, double *out_3995922565812671740);
void car_h_29(double *state, double *unused, double *out_8591890769704826794);
void car_H_29(double *state, double *unused, double *out_5709285292682222773);
void car_h_28(double *state, double *unused, double *out_2480702192977896369);
void car_H_28(double *state, double *unused, double *out_7655059763957798269);
void car_h_31(double *state, double *unused, double *out_322426088494547090);
void car_H_31(double *state, double *unused, double *out_6059531727976414459);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}