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
void car_err_fun(double *nom_x, double *delta_x, double *out_5230351242871143245);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_7994685084327859477);
void car_H_mod_fun(double *state, double *out_2156622796639092135);
void car_f_fun(double *state, double dt, double *out_4839858682589434933);
void car_F_fun(double *state, double dt, double *out_6752875690228243553);
void car_h_25(double *state, double *unused, double *out_3012118820978252315);
void car_H_25(double *state, double *unused, double *out_5764715236334767670);
void car_h_24(double *state, double *unused, double *out_4008937272930415502);
void car_H_24(double *state, double *unused, double *out_3592065637329268104);
void car_h_30(double *state, double *unused, double *out_4189849330954229981);
void car_H_30(double *state, double *unused, double *out_1237018906207159472);
void car_h_26(double *state, double *unused, double *out_7594326007187440774);
void car_H_26(double *state, double *unused, double *out_2023211917460711446);
void car_h_27(double *state, double *unused, double *out_7069448714047433521);
void car_H_27(double *state, double *unused, double *out_3460612977391102689);
void car_h_29(double *state, double *unused, double *out_8498909828761583679);
void car_H_29(double *state, double *unused, double *out_1747250250521551656);
void car_h_28(double *state, double *unused, double *out_5619310445668380139);
void car_H_28(double *state, double *unused, double *out_3710880522086877907);
void car_h_31(double *state, double *unused, double *out_7972818683428907302);
void car_H_31(double *state, double *unused, double *out_1397003815227359970);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}