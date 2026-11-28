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
void car_err_fun(double *nom_x, double *delta_x, double *out_8105623026857581998);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7503404914069652669);
void car_H_mod_fun(double *state, double *out_4783738824564572493);
void car_f_fun(double *state, double dt, double *out_6401182990301164404);
void car_F_fun(double *state, double dt, double *out_5658938656973252260);
void car_h_25(double *state, double *unused, double *out_2658415674908657098);
void car_H_25(double *state, double *unused, double *out_1260758174575080816);
void car_h_24(double *state, double *unused, double *out_3386724067472285602);
void car_H_24(double *state, double *unused, double *out_2297167424227242320);
void car_h_30(double *state, double *unused, double *out_3786195456112836173);
void car_H_30(double *state, double *unused, double *out_5655932166916535939);
void car_h_26(double *state, double *unused, double *out_1439711759722488774);
void car_H_26(double *state, double *unused, double *out_603904110464768912);
void car_h_27(double *state, double *unused, double *out_2343257592752188547);
void car_H_27(double *state, double *unused, double *out_3564860433518745797);
void car_h_29(double *state, double *unused, double *out_2618451655036694436);
void car_H_29(double *state, double *unused, double *out_879865777403928702);
void car_h_28(double *state, double *unused, double *out_4435271162866397930);
void car_H_28(double *state, double *unused, double *out_1083764494161397549);
void car_h_31(double *state, double *unused, double *out_6295865203162679448);
void car_H_31(double *state, double *unused, double *out_1230112212698120388);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}