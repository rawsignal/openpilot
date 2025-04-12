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
void car_err_fun(double *nom_x, double *delta_x, double *out_2788712361531455797);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_5243626477297637109);
void car_H_mod_fun(double *state, double *out_7309627436752179880);
void car_f_fun(double *state, double dt, double *out_5495602249935962846);
void car_F_fun(double *state, double dt, double *out_6467568700425071091);
void car_h_25(double *state, double *unused, double *out_7832645508647381886);
void car_H_25(double *state, double *unused, double *out_1943928411045134499);
void car_h_24(double *state, double *unused, double *out_1226025131367495892);
void car_H_24(double *state, double *unused, double *out_2802580915921374535);
void car_h_30(double *state, double *unused, double *out_3843263338711818794);
void car_H_30(double *state, double *unused, double *out_8860618752536751254);
void car_h_26(double *state, double *unused, double *out_8396322947517111767);
void car_H_26(double *state, double *unused, double *out_1797574907828921725);
void car_h_27(double *state, double *unused, double *out_1917014754913519550);
void car_H_27(double *state, double *unused, double *out_6685855440736326343);
void car_h_29(double *state, double *unused, double *out_1030711235441965411);
void car_H_29(double *state, double *unused, double *out_4972492713866775310);
void car_h_28(double *state, double *unused, double *out_6247245530635606257);
void car_H_28(double *state, double *unused, double *out_109906303202755264);
void car_h_31(double *state, double *unused, double *out_69380251826709445);
void car_H_31(double *state, double *unused, double *out_1974574372922094927);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}