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
void car_err_fun(double *nom_x, double *delta_x, double *out_4680331361908892937);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_3950569831835191548);
void car_H_mod_fun(double *state, double *out_1066722927762349330);
void car_f_fun(double *state, double dt, double *out_1631803966804731977);
void car_F_fun(double *state, double dt, double *out_8461957512847748442);
void car_h_25(double *state, double *unused, double *out_1273494739935375238);
void car_H_25(double *state, double *unused, double *out_8988060960736209135);
void car_h_24(double *state, double *unused, double *out_3673979862628150281);
void car_H_24(double *state, double *unused, double *out_5900757514171019345);
void car_h_30(double *state, double *unused, double *out_998300677650869349);
void car_H_30(double *state, double *unused, double *out_6940350154466093854);
void car_h_26(double *state, double *unused, double *out_6257843039442582360);
void car_H_26(double *state, double *unused, double *out_5246557641862152911);
void car_h_27(double *state, double *unused, double *out_670178855258897721);
void car_H_27(double *state, double *unused, double *out_9115113466266518765);
void car_h_29(double *state, double *unused, double *out_4003372590009976296);
void car_H_29(double *state, double *unused, double *out_6430118810151701670);
void car_h_28(double *state, double *unused, double *out_3274584176067595372);
void car_H_28(double *state, double *unused, double *out_6934226246488319372);
void car_h_31(double *state, double *unused, double *out_4910944268189397588);
void car_H_31(double *state, double *unused, double *out_4620349539628801435);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}