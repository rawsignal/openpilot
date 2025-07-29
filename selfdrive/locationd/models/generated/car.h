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
void car_err_fun(double *nom_x, double *delta_x, double *out_1752089738275979241);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1045694545913765962);
void car_H_mod_fun(double *state, double *out_3061534902965190884);
void car_f_fun(double *state, double dt, double *out_5345978839819676021);
void car_F_fun(double *state, double dt, double *out_4711065191693970871);
void car_h_25(double *state, double *unused, double *out_3031350575474952037);
void car_H_25(double *state, double *unused, double *out_8276550673710960652);
void car_h_24(double *state, double *unused, double *out_1555058544644953461);
void car_H_24(double *state, double *unused, double *out_4718625074908637516);
void car_h_30(double *state, double *unused, double *out_8644558271884248643);
void car_H_30(double *state, double *unused, double *out_8405889620854200722);
void car_h_26(double *state, double *unused, double *out_8634442000749161236);
void car_H_26(double *state, double *unused, double *out_6428690081124534740);
void car_h_27(double *state, double *unused, double *out_5989192836965524405);
void car_H_27(double *state, double *unused, double *out_7866091141054925983);
void car_h_29(double *state, double *unused, double *out_5832006168614395260);
void car_H_29(double *state, double *unused, double *out_7895658276539808538);
void car_h_28(double *state, double *unused, double *out_1445950597463176408);
void car_H_28(double *state, double *unused, double *out_5932028004974482287);
void car_h_31(double *state, double *unused, double *out_4731914681345720404);
void car_H_31(double *state, double *unused, double *out_8245904711834000224);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}