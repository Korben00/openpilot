#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_7254673260181494008);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6732371445488088772);
void pose_H_mod_fun(double *state, double *out_2434764514571563776);
void pose_f_fun(double *state, double dt, double *out_4944120091642834946);
void pose_F_fun(double *state, double dt, double *out_7093184401669479338);
void pose_h_4(double *state, double *unused, double *out_4692819888483323179);
void pose_H_4(double *state, double *unused, double *out_8461705635423868277);
void pose_h_10(double *state, double *unused, double *out_1131133617675999593);
void pose_H_10(double *state, double *unused, double *out_5089104592796037999);
void pose_h_13(double *state, double *unused, double *out_8712870422670877138);
void pose_H_13(double *state, double *unused, double *out_2374407229968982410);
void pose_h_14(double *state, double *unused, double *out_3278734857690699026);
void pose_H_14(double *state, double *unused, double *out_5378917203128495981);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}