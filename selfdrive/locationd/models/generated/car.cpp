#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_5040871400336792087) {
   out_5040871400336792087[0] = delta_x[0] + nom_x[0];
   out_5040871400336792087[1] = delta_x[1] + nom_x[1];
   out_5040871400336792087[2] = delta_x[2] + nom_x[2];
   out_5040871400336792087[3] = delta_x[3] + nom_x[3];
   out_5040871400336792087[4] = delta_x[4] + nom_x[4];
   out_5040871400336792087[5] = delta_x[5] + nom_x[5];
   out_5040871400336792087[6] = delta_x[6] + nom_x[6];
   out_5040871400336792087[7] = delta_x[7] + nom_x[7];
   out_5040871400336792087[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_2683939322041871654) {
   out_2683939322041871654[0] = -nom_x[0] + true_x[0];
   out_2683939322041871654[1] = -nom_x[1] + true_x[1];
   out_2683939322041871654[2] = -nom_x[2] + true_x[2];
   out_2683939322041871654[3] = -nom_x[3] + true_x[3];
   out_2683939322041871654[4] = -nom_x[4] + true_x[4];
   out_2683939322041871654[5] = -nom_x[5] + true_x[5];
   out_2683939322041871654[6] = -nom_x[6] + true_x[6];
   out_2683939322041871654[7] = -nom_x[7] + true_x[7];
   out_2683939322041871654[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_2417772749377803867) {
   out_2417772749377803867[0] = 1.0;
   out_2417772749377803867[1] = 0.0;
   out_2417772749377803867[2] = 0.0;
   out_2417772749377803867[3] = 0.0;
   out_2417772749377803867[4] = 0.0;
   out_2417772749377803867[5] = 0.0;
   out_2417772749377803867[6] = 0.0;
   out_2417772749377803867[7] = 0.0;
   out_2417772749377803867[8] = 0.0;
   out_2417772749377803867[9] = 0.0;
   out_2417772749377803867[10] = 1.0;
   out_2417772749377803867[11] = 0.0;
   out_2417772749377803867[12] = 0.0;
   out_2417772749377803867[13] = 0.0;
   out_2417772749377803867[14] = 0.0;
   out_2417772749377803867[15] = 0.0;
   out_2417772749377803867[16] = 0.0;
   out_2417772749377803867[17] = 0.0;
   out_2417772749377803867[18] = 0.0;
   out_2417772749377803867[19] = 0.0;
   out_2417772749377803867[20] = 1.0;
   out_2417772749377803867[21] = 0.0;
   out_2417772749377803867[22] = 0.0;
   out_2417772749377803867[23] = 0.0;
   out_2417772749377803867[24] = 0.0;
   out_2417772749377803867[25] = 0.0;
   out_2417772749377803867[26] = 0.0;
   out_2417772749377803867[27] = 0.0;
   out_2417772749377803867[28] = 0.0;
   out_2417772749377803867[29] = 0.0;
   out_2417772749377803867[30] = 1.0;
   out_2417772749377803867[31] = 0.0;
   out_2417772749377803867[32] = 0.0;
   out_2417772749377803867[33] = 0.0;
   out_2417772749377803867[34] = 0.0;
   out_2417772749377803867[35] = 0.0;
   out_2417772749377803867[36] = 0.0;
   out_2417772749377803867[37] = 0.0;
   out_2417772749377803867[38] = 0.0;
   out_2417772749377803867[39] = 0.0;
   out_2417772749377803867[40] = 1.0;
   out_2417772749377803867[41] = 0.0;
   out_2417772749377803867[42] = 0.0;
   out_2417772749377803867[43] = 0.0;
   out_2417772749377803867[44] = 0.0;
   out_2417772749377803867[45] = 0.0;
   out_2417772749377803867[46] = 0.0;
   out_2417772749377803867[47] = 0.0;
   out_2417772749377803867[48] = 0.0;
   out_2417772749377803867[49] = 0.0;
   out_2417772749377803867[50] = 1.0;
   out_2417772749377803867[51] = 0.0;
   out_2417772749377803867[52] = 0.0;
   out_2417772749377803867[53] = 0.0;
   out_2417772749377803867[54] = 0.0;
   out_2417772749377803867[55] = 0.0;
   out_2417772749377803867[56] = 0.0;
   out_2417772749377803867[57] = 0.0;
   out_2417772749377803867[58] = 0.0;
   out_2417772749377803867[59] = 0.0;
   out_2417772749377803867[60] = 1.0;
   out_2417772749377803867[61] = 0.0;
   out_2417772749377803867[62] = 0.0;
   out_2417772749377803867[63] = 0.0;
   out_2417772749377803867[64] = 0.0;
   out_2417772749377803867[65] = 0.0;
   out_2417772749377803867[66] = 0.0;
   out_2417772749377803867[67] = 0.0;
   out_2417772749377803867[68] = 0.0;
   out_2417772749377803867[69] = 0.0;
   out_2417772749377803867[70] = 1.0;
   out_2417772749377803867[71] = 0.0;
   out_2417772749377803867[72] = 0.0;
   out_2417772749377803867[73] = 0.0;
   out_2417772749377803867[74] = 0.0;
   out_2417772749377803867[75] = 0.0;
   out_2417772749377803867[76] = 0.0;
   out_2417772749377803867[77] = 0.0;
   out_2417772749377803867[78] = 0.0;
   out_2417772749377803867[79] = 0.0;
   out_2417772749377803867[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_7748123811331147502) {
   out_7748123811331147502[0] = state[0];
   out_7748123811331147502[1] = state[1];
   out_7748123811331147502[2] = state[2];
   out_7748123811331147502[3] = state[3];
   out_7748123811331147502[4] = state[4];
   out_7748123811331147502[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_7748123811331147502[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_7748123811331147502[7] = state[7];
   out_7748123811331147502[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8471488585784239923) {
   out_8471488585784239923[0] = 1;
   out_8471488585784239923[1] = 0;
   out_8471488585784239923[2] = 0;
   out_8471488585784239923[3] = 0;
   out_8471488585784239923[4] = 0;
   out_8471488585784239923[5] = 0;
   out_8471488585784239923[6] = 0;
   out_8471488585784239923[7] = 0;
   out_8471488585784239923[8] = 0;
   out_8471488585784239923[9] = 0;
   out_8471488585784239923[10] = 1;
   out_8471488585784239923[11] = 0;
   out_8471488585784239923[12] = 0;
   out_8471488585784239923[13] = 0;
   out_8471488585784239923[14] = 0;
   out_8471488585784239923[15] = 0;
   out_8471488585784239923[16] = 0;
   out_8471488585784239923[17] = 0;
   out_8471488585784239923[18] = 0;
   out_8471488585784239923[19] = 0;
   out_8471488585784239923[20] = 1;
   out_8471488585784239923[21] = 0;
   out_8471488585784239923[22] = 0;
   out_8471488585784239923[23] = 0;
   out_8471488585784239923[24] = 0;
   out_8471488585784239923[25] = 0;
   out_8471488585784239923[26] = 0;
   out_8471488585784239923[27] = 0;
   out_8471488585784239923[28] = 0;
   out_8471488585784239923[29] = 0;
   out_8471488585784239923[30] = 1;
   out_8471488585784239923[31] = 0;
   out_8471488585784239923[32] = 0;
   out_8471488585784239923[33] = 0;
   out_8471488585784239923[34] = 0;
   out_8471488585784239923[35] = 0;
   out_8471488585784239923[36] = 0;
   out_8471488585784239923[37] = 0;
   out_8471488585784239923[38] = 0;
   out_8471488585784239923[39] = 0;
   out_8471488585784239923[40] = 1;
   out_8471488585784239923[41] = 0;
   out_8471488585784239923[42] = 0;
   out_8471488585784239923[43] = 0;
   out_8471488585784239923[44] = 0;
   out_8471488585784239923[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8471488585784239923[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8471488585784239923[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8471488585784239923[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8471488585784239923[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8471488585784239923[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8471488585784239923[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8471488585784239923[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8471488585784239923[53] = -9.8000000000000007*dt;
   out_8471488585784239923[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8471488585784239923[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8471488585784239923[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8471488585784239923[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8471488585784239923[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8471488585784239923[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8471488585784239923[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8471488585784239923[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8471488585784239923[62] = 0;
   out_8471488585784239923[63] = 0;
   out_8471488585784239923[64] = 0;
   out_8471488585784239923[65] = 0;
   out_8471488585784239923[66] = 0;
   out_8471488585784239923[67] = 0;
   out_8471488585784239923[68] = 0;
   out_8471488585784239923[69] = 0;
   out_8471488585784239923[70] = 1;
   out_8471488585784239923[71] = 0;
   out_8471488585784239923[72] = 0;
   out_8471488585784239923[73] = 0;
   out_8471488585784239923[74] = 0;
   out_8471488585784239923[75] = 0;
   out_8471488585784239923[76] = 0;
   out_8471488585784239923[77] = 0;
   out_8471488585784239923[78] = 0;
   out_8471488585784239923[79] = 0;
   out_8471488585784239923[80] = 1;
}
void h_25(double *state, double *unused, double *out_4980528965572761605) {
   out_4980528965572761605[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2929705555635046470) {
   out_2929705555635046470[0] = 0;
   out_2929705555635046470[1] = 0;
   out_2929705555635046470[2] = 0;
   out_2929705555635046470[3] = 0;
   out_2929705555635046470[4] = 0;
   out_2929705555635046470[5] = 0;
   out_2929705555635046470[6] = 1;
   out_2929705555635046470[7] = 0;
   out_2929705555635046470[8] = 0;
}
void h_24(double *state, double *unused, double *out_1529124758116869801) {
   out_1529124758116869801[0] = state[4];
   out_1529124758116869801[1] = state[5];
}
void H_24(double *state, double *unused, double *out_757055956629546904) {
   out_757055956629546904[0] = 0;
   out_757055956629546904[1] = 0;
   out_757055956629546904[2] = 0;
   out_757055956629546904[3] = 0;
   out_757055956629546904[4] = 1;
   out_757055956629546904[5] = 0;
   out_757055956629546904[6] = 0;
   out_757055956629546904[7] = 0;
   out_757055956629546904[8] = 0;
   out_757055956629546904[9] = 0;
   out_757055956629546904[10] = 0;
   out_757055956629546904[11] = 0;
   out_757055956629546904[12] = 0;
   out_757055956629546904[13] = 0;
   out_757055956629546904[14] = 1;
   out_757055956629546904[15] = 0;
   out_757055956629546904[16] = 0;
   out_757055956629546904[17] = 0;
}
void h_30(double *state, double *unused, double *out_4705334903288255716) {
   out_4705334903288255716[0] = state[4];
}
void H_30(double *state, double *unused, double *out_5448038514142295097) {
   out_5448038514142295097[0] = 0;
   out_5448038514142295097[1] = 0;
   out_5448038514142295097[2] = 0;
   out_5448038514142295097[3] = 0;
   out_5448038514142295097[4] = 1;
   out_5448038514142295097[5] = 0;
   out_5448038514142295097[6] = 0;
   out_5448038514142295097[7] = 0;
   out_5448038514142295097[8] = 0;
}
void h_26(double *state, double *unused, double *out_8739735902760450255) {
   out_8739735902760450255[0] = state[7];
}
void H_26(double *state, double *unused, double *out_811797763239009754) {
   out_811797763239009754[0] = 0;
   out_811797763239009754[1] = 0;
   out_811797763239009754[2] = 0;
   out_811797763239009754[3] = 0;
   out_811797763239009754[4] = 0;
   out_811797763239009754[5] = 0;
   out_811797763239009754[6] = 0;
   out_811797763239009754[7] = 1;
   out_811797763239009754[8] = 0;
}
void h_27(double *state, double *unused, double *out_7067173590722940865) {
   out_7067173590722940865[0] = state[3];
}
void H_27(double *state, double *unused, double *out_3273275202341870186) {
   out_3273275202341870186[0] = 0;
   out_3273275202341870186[1] = 0;
   out_3273275202341870186[2] = 0;
   out_3273275202341870186[3] = 1;
   out_3273275202341870186[4] = 0;
   out_3273275202341870186[5] = 0;
   out_3273275202341870186[6] = 0;
   out_3273275202341870186[7] = 0;
   out_3273275202341870186[8] = 0;
}
void h_29(double *state, double *unused, double *out_1429487090540799127) {
   out_1429487090540799127[0] = state[1];
}
void H_29(double *state, double *unused, double *out_5958269858456687281) {
   out_5958269858456687281[0] = 0;
   out_5958269858456687281[1] = 1;
   out_5958269858456687281[2] = 0;
   out_5958269858456687281[3] = 0;
   out_5958269858456687281[4] = 0;
   out_5958269858456687281[5] = 0;
   out_5958269858456687281[6] = 0;
   out_5958269858456687281[7] = 0;
   out_5958269858456687281[8] = 0;
}
void h_28(double *state, double *unused, double *out_1725306354836910302) {
   out_1725306354836910302[0] = state[0];
}
void H_28(double *state, double *unused, double *out_875870841387156707) {
   out_875870841387156707[0] = 1;
   out_875870841387156707[1] = 0;
   out_875870841387156707[2] = 0;
   out_875870841387156707[3] = 0;
   out_875870841387156707[4] = 0;
   out_875870841387156707[5] = 0;
   out_875870841387156707[6] = 0;
   out_875870841387156707[7] = 0;
   out_875870841387156707[8] = 0;
}
void h_31(double *state, double *unused, double *out_8617978493826783955) {
   out_8617978493826783955[0] = state[8];
}
void H_31(double *state, double *unused, double *out_1438005865472361230) {
   out_1438005865472361230[0] = 0;
   out_1438005865472361230[1] = 0;
   out_1438005865472361230[2] = 0;
   out_1438005865472361230[3] = 0;
   out_1438005865472361230[4] = 0;
   out_1438005865472361230[5] = 0;
   out_1438005865472361230[6] = 0;
   out_1438005865472361230[7] = 0;
   out_1438005865472361230[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_5040871400336792087) {
  err_fun(nom_x, delta_x, out_5040871400336792087);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_2683939322041871654) {
  inv_err_fun(nom_x, true_x, out_2683939322041871654);
}
void car_H_mod_fun(double *state, double *out_2417772749377803867) {
  H_mod_fun(state, out_2417772749377803867);
}
void car_f_fun(double *state, double dt, double *out_7748123811331147502) {
  f_fun(state,  dt, out_7748123811331147502);
}
void car_F_fun(double *state, double dt, double *out_8471488585784239923) {
  F_fun(state,  dt, out_8471488585784239923);
}
void car_h_25(double *state, double *unused, double *out_4980528965572761605) {
  h_25(state, unused, out_4980528965572761605);
}
void car_H_25(double *state, double *unused, double *out_2929705555635046470) {
  H_25(state, unused, out_2929705555635046470);
}
void car_h_24(double *state, double *unused, double *out_1529124758116869801) {
  h_24(state, unused, out_1529124758116869801);
}
void car_H_24(double *state, double *unused, double *out_757055956629546904) {
  H_24(state, unused, out_757055956629546904);
}
void car_h_30(double *state, double *unused, double *out_4705334903288255716) {
  h_30(state, unused, out_4705334903288255716);
}
void car_H_30(double *state, double *unused, double *out_5448038514142295097) {
  H_30(state, unused, out_5448038514142295097);
}
void car_h_26(double *state, double *unused, double *out_8739735902760450255) {
  h_26(state, unused, out_8739735902760450255);
}
void car_H_26(double *state, double *unused, double *out_811797763239009754) {
  H_26(state, unused, out_811797763239009754);
}
void car_h_27(double *state, double *unused, double *out_7067173590722940865) {
  h_27(state, unused, out_7067173590722940865);
}
void car_H_27(double *state, double *unused, double *out_3273275202341870186) {
  H_27(state, unused, out_3273275202341870186);
}
void car_h_29(double *state, double *unused, double *out_1429487090540799127) {
  h_29(state, unused, out_1429487090540799127);
}
void car_H_29(double *state, double *unused, double *out_5958269858456687281) {
  H_29(state, unused, out_5958269858456687281);
}
void car_h_28(double *state, double *unused, double *out_1725306354836910302) {
  h_28(state, unused, out_1725306354836910302);
}
void car_H_28(double *state, double *unused, double *out_875870841387156707) {
  H_28(state, unused, out_875870841387156707);
}
void car_h_31(double *state, double *unused, double *out_8617978493826783955) {
  h_31(state, unused, out_8617978493826783955);
}
void car_H_31(double *state, double *unused, double *out_1438005865472361230) {
  H_31(state, unused, out_1438005865472361230);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
