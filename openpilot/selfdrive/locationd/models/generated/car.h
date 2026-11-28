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
void car_err_fun(double *nom_x, double *delta_x, double *out_285877354863972642);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_865842154645548932);
void car_H_mod_fun(double *state, double *out_2321332579323854249);
void car_f_fun(double *state, double dt, double *out_8939805825965589821);
void car_F_fun(double *state, double dt, double *out_6594188117642331652);
void car_h_25(double *state, double *unused, double *out_3622607962420453030);
void car_H_25(double *state, double *unused, double *out_2582359245860810591);
void car_h_24(double *state, double *unused, double *out_8587894906978392811);
void car_H_24(double *state, double *unused, double *out_3427355854644505990);
void car_h_30(double *state, double *unused, double *out_8008059677881605864);
void car_H_30(double *state, double *unused, double *out_8947694486357124270);
void car_h_26(double *state, double *unused, double *out_7730774847579258584);
void car_H_26(double *state, double *unused, double *out_1159144073013245633);
void car_h_27(double *state, double *unused, double *out_6803325635591764793);
void car_H_27(double *state, double *unused, double *out_7324286275552002435);
void car_h_29(double *state, double *unused, double *out_6159966003405800524);
void car_H_29(double *state, double *unused, double *out_5610923548682451402);
void car_h_28(double *state, double *unused, double *out_1801652367930919148);
void car_H_28(double *state, double *unused, double *out_528524531612920828);
void car_h_31(double *state, double *unused, double *out_6526040805289417513);
void car_H_31(double *state, double *unused, double *out_2613005207737771019);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}