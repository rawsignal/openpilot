#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_2169286582723434542);
void live_err_fun(double *nom_x, double *delta_x, double *out_536698960109903147);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2152944576913117148);
void live_H_mod_fun(double *state, double *out_3572002111564404809);
void live_f_fun(double *state, double dt, double *out_4530714076034402124);
void live_F_fun(double *state, double dt, double *out_242041159689319846);
void live_h_4(double *state, double *unused, double *out_5923016346657677625);
void live_H_4(double *state, double *unused, double *out_5960880512727366596);
void live_h_9(double *state, double *unused, double *out_4750502691446422176);
void live_H_9(double *state, double *unused, double *out_1326338422537080874);
void live_h_10(double *state, double *unused, double *out_323792365937565073);
void live_H_10(double *state, double *unused, double *out_5682984742854511373);
void live_h_12(double *state, double *unused, double *out_243112562556907841);
void live_H_12(double *state, double *unused, double *out_6104605183939452024);
void live_h_35(double *state, double *unused, double *out_6838073987133625945);
void live_H_35(double *state, double *unused, double *out_4451810833280097605);
void live_h_32(double *state, double *unused, double *out_556016054740309196);
void live_H_32(double *state, double *unused, double *out_2463053670414018751);
void live_h_13(double *state, double *unused, double *out_3041389138897665903);
void live_H_13(double *state, double *unused, double *out_8995540261568807652);
void live_h_14(double *state, double *unused, double *out_4750502691446422176);
void live_H_14(double *state, double *unused, double *out_1326338422537080874);
void live_h_33(double *state, double *unused, double *out_3056640867608804978);
void live_H_33(double *state, double *unused, double *out_7602367837918955209);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}