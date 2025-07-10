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
void car_err_fun(double *nom_x, double *delta_x, double *out_8939131331752933554);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_4134102941266011107);
void car_H_mod_fun(double *state, double *out_8366129814821909934);
void car_f_fun(double *state, double dt, double *out_3833305672384494666);
void car_F_fun(double *state, double dt, double *out_2882009860667348310);
void car_h_25(double *state, double *unused, double *out_3318328831680801391);
void car_H_25(double *state, double *unused, double *out_6557633608898150005);
void car_h_24(double *state, double *unused, double *out_2312509804207382212);
void car_H_24(double *state, double *unused, double *out_7142601137844892577);
void car_h_30(double *state, double *unused, double *out_3593522893965307280);
void car_H_30(double *state, double *unused, double *out_359056732593466750);
void car_h_26(double *state, double *unused, double *out_2049206977115649555);
void car_H_26(double *state, double *unused, double *out_8147607145937345387);
void car_h_27(double *state, double *unused, double *out_3080712685733047755);
void car_H_27(double *state, double *unused, double *out_1815706579206958161);
void car_h_29(double *state, double *unused, double *out_556736842520974595);
void car_H_29(double *state, double *unused, double *out_869288076907858934);
void car_h_28(double *state, double *unused, double *out_4024222538634710190);
void car_H_28(double *state, double *unused, double *out_4213110940161671640);
void car_h_31(double *state, double *unused, double *out_7235475747144226866);
void car_H_31(double *state, double *unused, double *out_6526987647021189577);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}