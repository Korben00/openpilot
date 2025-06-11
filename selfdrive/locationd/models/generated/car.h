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
void car_err_fun(double *nom_x, double *delta_x, double *out_5040871400336792087);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2683939322041871654);
void car_H_mod_fun(double *state, double *out_2417772749377803867);
void car_f_fun(double *state, double dt, double *out_7748123811331147502);
void car_F_fun(double *state, double dt, double *out_8471488585784239923);
void car_h_25(double *state, double *unused, double *out_4980528965572761605);
void car_H_25(double *state, double *unused, double *out_2929705555635046470);
void car_h_24(double *state, double *unused, double *out_1529124758116869801);
void car_H_24(double *state, double *unused, double *out_757055956629546904);
void car_h_30(double *state, double *unused, double *out_4705334903288255716);
void car_H_30(double *state, double *unused, double *out_5448038514142295097);
void car_h_26(double *state, double *unused, double *out_8739735902760450255);
void car_H_26(double *state, double *unused, double *out_811797763239009754);
void car_h_27(double *state, double *unused, double *out_7067173590722940865);
void car_H_27(double *state, double *unused, double *out_3273275202341870186);
void car_h_29(double *state, double *unused, double *out_1429487090540799127);
void car_H_29(double *state, double *unused, double *out_5958269858456687281);
void car_h_28(double *state, double *unused, double *out_1725306354836910302);
void car_H_28(double *state, double *unused, double *out_875870841387156707);
void car_h_31(double *state, double *unused, double *out_8617978493826783955);
void car_H_31(double *state, double *unused, double *out_1438005865472361230);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}