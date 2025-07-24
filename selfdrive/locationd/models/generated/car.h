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
void car_err_fun(double *nom_x, double *delta_x, double *out_4586029367640539685);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5378054743951964721);
void car_H_mod_fun(double *state, double *out_8109768655304479947);
void car_f_fun(double *state, double dt, double *out_8436141603802697156);
void car_F_fun(double *state, double dt, double *out_2266778645478433972);
void car_h_25(double *state, double *unused, double *out_6676757261096667859);
void car_H_25(double *state, double *unused, double *out_1143787192492834432);
void car_h_24(double *state, double *unused, double *out_547609794561793546);
void car_H_24(double *state, double *unused, double *out_7797992650601020189);
void car_h_30(double *state, double *unused, double *out_7908303336184382105);
void car_H_30(double *state, double *unused, double *out_1014448245349594362);
void car_h_26(double *state, double *unused, double *out_5312533521689844564);
void car_H_26(double *state, double *unused, double *out_1800641256603146336);
void car_h_27(double *state, double *unused, double *out_3052149892744191431);
void car_H_27(double *state, double *unused, double *out_1160315066450830549);
void car_h_29(double *state, double *unused, double *out_2906630068523536460);
void car_H_29(double *state, double *unused, double *out_1524679589663986546);
void car_h_28(double *state, double *unused, double *out_2696461356750672739);
void car_H_28(double *state, double *unused, double *out_3488309861229312797);
void car_h_31(double *state, double *unused, double *out_4107799479400003493);
void car_H_31(double *state, double *unused, double *out_1174433154369794860);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}