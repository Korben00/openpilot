#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7254673260181494008) {
   out_7254673260181494008[0] = delta_x[0] + nom_x[0];
   out_7254673260181494008[1] = delta_x[1] + nom_x[1];
   out_7254673260181494008[2] = delta_x[2] + nom_x[2];
   out_7254673260181494008[3] = delta_x[3] + nom_x[3];
   out_7254673260181494008[4] = delta_x[4] + nom_x[4];
   out_7254673260181494008[5] = delta_x[5] + nom_x[5];
   out_7254673260181494008[6] = delta_x[6] + nom_x[6];
   out_7254673260181494008[7] = delta_x[7] + nom_x[7];
   out_7254673260181494008[8] = delta_x[8] + nom_x[8];
   out_7254673260181494008[9] = delta_x[9] + nom_x[9];
   out_7254673260181494008[10] = delta_x[10] + nom_x[10];
   out_7254673260181494008[11] = delta_x[11] + nom_x[11];
   out_7254673260181494008[12] = delta_x[12] + nom_x[12];
   out_7254673260181494008[13] = delta_x[13] + nom_x[13];
   out_7254673260181494008[14] = delta_x[14] + nom_x[14];
   out_7254673260181494008[15] = delta_x[15] + nom_x[15];
   out_7254673260181494008[16] = delta_x[16] + nom_x[16];
   out_7254673260181494008[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6732371445488088772) {
   out_6732371445488088772[0] = -nom_x[0] + true_x[0];
   out_6732371445488088772[1] = -nom_x[1] + true_x[1];
   out_6732371445488088772[2] = -nom_x[2] + true_x[2];
   out_6732371445488088772[3] = -nom_x[3] + true_x[3];
   out_6732371445488088772[4] = -nom_x[4] + true_x[4];
   out_6732371445488088772[5] = -nom_x[5] + true_x[5];
   out_6732371445488088772[6] = -nom_x[6] + true_x[6];
   out_6732371445488088772[7] = -nom_x[7] + true_x[7];
   out_6732371445488088772[8] = -nom_x[8] + true_x[8];
   out_6732371445488088772[9] = -nom_x[9] + true_x[9];
   out_6732371445488088772[10] = -nom_x[10] + true_x[10];
   out_6732371445488088772[11] = -nom_x[11] + true_x[11];
   out_6732371445488088772[12] = -nom_x[12] + true_x[12];
   out_6732371445488088772[13] = -nom_x[13] + true_x[13];
   out_6732371445488088772[14] = -nom_x[14] + true_x[14];
   out_6732371445488088772[15] = -nom_x[15] + true_x[15];
   out_6732371445488088772[16] = -nom_x[16] + true_x[16];
   out_6732371445488088772[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_2434764514571563776) {
   out_2434764514571563776[0] = 1.0;
   out_2434764514571563776[1] = 0.0;
   out_2434764514571563776[2] = 0.0;
   out_2434764514571563776[3] = 0.0;
   out_2434764514571563776[4] = 0.0;
   out_2434764514571563776[5] = 0.0;
   out_2434764514571563776[6] = 0.0;
   out_2434764514571563776[7] = 0.0;
   out_2434764514571563776[8] = 0.0;
   out_2434764514571563776[9] = 0.0;
   out_2434764514571563776[10] = 0.0;
   out_2434764514571563776[11] = 0.0;
   out_2434764514571563776[12] = 0.0;
   out_2434764514571563776[13] = 0.0;
   out_2434764514571563776[14] = 0.0;
   out_2434764514571563776[15] = 0.0;
   out_2434764514571563776[16] = 0.0;
   out_2434764514571563776[17] = 0.0;
   out_2434764514571563776[18] = 0.0;
   out_2434764514571563776[19] = 1.0;
   out_2434764514571563776[20] = 0.0;
   out_2434764514571563776[21] = 0.0;
   out_2434764514571563776[22] = 0.0;
   out_2434764514571563776[23] = 0.0;
   out_2434764514571563776[24] = 0.0;
   out_2434764514571563776[25] = 0.0;
   out_2434764514571563776[26] = 0.0;
   out_2434764514571563776[27] = 0.0;
   out_2434764514571563776[28] = 0.0;
   out_2434764514571563776[29] = 0.0;
   out_2434764514571563776[30] = 0.0;
   out_2434764514571563776[31] = 0.0;
   out_2434764514571563776[32] = 0.0;
   out_2434764514571563776[33] = 0.0;
   out_2434764514571563776[34] = 0.0;
   out_2434764514571563776[35] = 0.0;
   out_2434764514571563776[36] = 0.0;
   out_2434764514571563776[37] = 0.0;
   out_2434764514571563776[38] = 1.0;
   out_2434764514571563776[39] = 0.0;
   out_2434764514571563776[40] = 0.0;
   out_2434764514571563776[41] = 0.0;
   out_2434764514571563776[42] = 0.0;
   out_2434764514571563776[43] = 0.0;
   out_2434764514571563776[44] = 0.0;
   out_2434764514571563776[45] = 0.0;
   out_2434764514571563776[46] = 0.0;
   out_2434764514571563776[47] = 0.0;
   out_2434764514571563776[48] = 0.0;
   out_2434764514571563776[49] = 0.0;
   out_2434764514571563776[50] = 0.0;
   out_2434764514571563776[51] = 0.0;
   out_2434764514571563776[52] = 0.0;
   out_2434764514571563776[53] = 0.0;
   out_2434764514571563776[54] = 0.0;
   out_2434764514571563776[55] = 0.0;
   out_2434764514571563776[56] = 0.0;
   out_2434764514571563776[57] = 1.0;
   out_2434764514571563776[58] = 0.0;
   out_2434764514571563776[59] = 0.0;
   out_2434764514571563776[60] = 0.0;
   out_2434764514571563776[61] = 0.0;
   out_2434764514571563776[62] = 0.0;
   out_2434764514571563776[63] = 0.0;
   out_2434764514571563776[64] = 0.0;
   out_2434764514571563776[65] = 0.0;
   out_2434764514571563776[66] = 0.0;
   out_2434764514571563776[67] = 0.0;
   out_2434764514571563776[68] = 0.0;
   out_2434764514571563776[69] = 0.0;
   out_2434764514571563776[70] = 0.0;
   out_2434764514571563776[71] = 0.0;
   out_2434764514571563776[72] = 0.0;
   out_2434764514571563776[73] = 0.0;
   out_2434764514571563776[74] = 0.0;
   out_2434764514571563776[75] = 0.0;
   out_2434764514571563776[76] = 1.0;
   out_2434764514571563776[77] = 0.0;
   out_2434764514571563776[78] = 0.0;
   out_2434764514571563776[79] = 0.0;
   out_2434764514571563776[80] = 0.0;
   out_2434764514571563776[81] = 0.0;
   out_2434764514571563776[82] = 0.0;
   out_2434764514571563776[83] = 0.0;
   out_2434764514571563776[84] = 0.0;
   out_2434764514571563776[85] = 0.0;
   out_2434764514571563776[86] = 0.0;
   out_2434764514571563776[87] = 0.0;
   out_2434764514571563776[88] = 0.0;
   out_2434764514571563776[89] = 0.0;
   out_2434764514571563776[90] = 0.0;
   out_2434764514571563776[91] = 0.0;
   out_2434764514571563776[92] = 0.0;
   out_2434764514571563776[93] = 0.0;
   out_2434764514571563776[94] = 0.0;
   out_2434764514571563776[95] = 1.0;
   out_2434764514571563776[96] = 0.0;
   out_2434764514571563776[97] = 0.0;
   out_2434764514571563776[98] = 0.0;
   out_2434764514571563776[99] = 0.0;
   out_2434764514571563776[100] = 0.0;
   out_2434764514571563776[101] = 0.0;
   out_2434764514571563776[102] = 0.0;
   out_2434764514571563776[103] = 0.0;
   out_2434764514571563776[104] = 0.0;
   out_2434764514571563776[105] = 0.0;
   out_2434764514571563776[106] = 0.0;
   out_2434764514571563776[107] = 0.0;
   out_2434764514571563776[108] = 0.0;
   out_2434764514571563776[109] = 0.0;
   out_2434764514571563776[110] = 0.0;
   out_2434764514571563776[111] = 0.0;
   out_2434764514571563776[112] = 0.0;
   out_2434764514571563776[113] = 0.0;
   out_2434764514571563776[114] = 1.0;
   out_2434764514571563776[115] = 0.0;
   out_2434764514571563776[116] = 0.0;
   out_2434764514571563776[117] = 0.0;
   out_2434764514571563776[118] = 0.0;
   out_2434764514571563776[119] = 0.0;
   out_2434764514571563776[120] = 0.0;
   out_2434764514571563776[121] = 0.0;
   out_2434764514571563776[122] = 0.0;
   out_2434764514571563776[123] = 0.0;
   out_2434764514571563776[124] = 0.0;
   out_2434764514571563776[125] = 0.0;
   out_2434764514571563776[126] = 0.0;
   out_2434764514571563776[127] = 0.0;
   out_2434764514571563776[128] = 0.0;
   out_2434764514571563776[129] = 0.0;
   out_2434764514571563776[130] = 0.0;
   out_2434764514571563776[131] = 0.0;
   out_2434764514571563776[132] = 0.0;
   out_2434764514571563776[133] = 1.0;
   out_2434764514571563776[134] = 0.0;
   out_2434764514571563776[135] = 0.0;
   out_2434764514571563776[136] = 0.0;
   out_2434764514571563776[137] = 0.0;
   out_2434764514571563776[138] = 0.0;
   out_2434764514571563776[139] = 0.0;
   out_2434764514571563776[140] = 0.0;
   out_2434764514571563776[141] = 0.0;
   out_2434764514571563776[142] = 0.0;
   out_2434764514571563776[143] = 0.0;
   out_2434764514571563776[144] = 0.0;
   out_2434764514571563776[145] = 0.0;
   out_2434764514571563776[146] = 0.0;
   out_2434764514571563776[147] = 0.0;
   out_2434764514571563776[148] = 0.0;
   out_2434764514571563776[149] = 0.0;
   out_2434764514571563776[150] = 0.0;
   out_2434764514571563776[151] = 0.0;
   out_2434764514571563776[152] = 1.0;
   out_2434764514571563776[153] = 0.0;
   out_2434764514571563776[154] = 0.0;
   out_2434764514571563776[155] = 0.0;
   out_2434764514571563776[156] = 0.0;
   out_2434764514571563776[157] = 0.0;
   out_2434764514571563776[158] = 0.0;
   out_2434764514571563776[159] = 0.0;
   out_2434764514571563776[160] = 0.0;
   out_2434764514571563776[161] = 0.0;
   out_2434764514571563776[162] = 0.0;
   out_2434764514571563776[163] = 0.0;
   out_2434764514571563776[164] = 0.0;
   out_2434764514571563776[165] = 0.0;
   out_2434764514571563776[166] = 0.0;
   out_2434764514571563776[167] = 0.0;
   out_2434764514571563776[168] = 0.0;
   out_2434764514571563776[169] = 0.0;
   out_2434764514571563776[170] = 0.0;
   out_2434764514571563776[171] = 1.0;
   out_2434764514571563776[172] = 0.0;
   out_2434764514571563776[173] = 0.0;
   out_2434764514571563776[174] = 0.0;
   out_2434764514571563776[175] = 0.0;
   out_2434764514571563776[176] = 0.0;
   out_2434764514571563776[177] = 0.0;
   out_2434764514571563776[178] = 0.0;
   out_2434764514571563776[179] = 0.0;
   out_2434764514571563776[180] = 0.0;
   out_2434764514571563776[181] = 0.0;
   out_2434764514571563776[182] = 0.0;
   out_2434764514571563776[183] = 0.0;
   out_2434764514571563776[184] = 0.0;
   out_2434764514571563776[185] = 0.0;
   out_2434764514571563776[186] = 0.0;
   out_2434764514571563776[187] = 0.0;
   out_2434764514571563776[188] = 0.0;
   out_2434764514571563776[189] = 0.0;
   out_2434764514571563776[190] = 1.0;
   out_2434764514571563776[191] = 0.0;
   out_2434764514571563776[192] = 0.0;
   out_2434764514571563776[193] = 0.0;
   out_2434764514571563776[194] = 0.0;
   out_2434764514571563776[195] = 0.0;
   out_2434764514571563776[196] = 0.0;
   out_2434764514571563776[197] = 0.0;
   out_2434764514571563776[198] = 0.0;
   out_2434764514571563776[199] = 0.0;
   out_2434764514571563776[200] = 0.0;
   out_2434764514571563776[201] = 0.0;
   out_2434764514571563776[202] = 0.0;
   out_2434764514571563776[203] = 0.0;
   out_2434764514571563776[204] = 0.0;
   out_2434764514571563776[205] = 0.0;
   out_2434764514571563776[206] = 0.0;
   out_2434764514571563776[207] = 0.0;
   out_2434764514571563776[208] = 0.0;
   out_2434764514571563776[209] = 1.0;
   out_2434764514571563776[210] = 0.0;
   out_2434764514571563776[211] = 0.0;
   out_2434764514571563776[212] = 0.0;
   out_2434764514571563776[213] = 0.0;
   out_2434764514571563776[214] = 0.0;
   out_2434764514571563776[215] = 0.0;
   out_2434764514571563776[216] = 0.0;
   out_2434764514571563776[217] = 0.0;
   out_2434764514571563776[218] = 0.0;
   out_2434764514571563776[219] = 0.0;
   out_2434764514571563776[220] = 0.0;
   out_2434764514571563776[221] = 0.0;
   out_2434764514571563776[222] = 0.0;
   out_2434764514571563776[223] = 0.0;
   out_2434764514571563776[224] = 0.0;
   out_2434764514571563776[225] = 0.0;
   out_2434764514571563776[226] = 0.0;
   out_2434764514571563776[227] = 0.0;
   out_2434764514571563776[228] = 1.0;
   out_2434764514571563776[229] = 0.0;
   out_2434764514571563776[230] = 0.0;
   out_2434764514571563776[231] = 0.0;
   out_2434764514571563776[232] = 0.0;
   out_2434764514571563776[233] = 0.0;
   out_2434764514571563776[234] = 0.0;
   out_2434764514571563776[235] = 0.0;
   out_2434764514571563776[236] = 0.0;
   out_2434764514571563776[237] = 0.0;
   out_2434764514571563776[238] = 0.0;
   out_2434764514571563776[239] = 0.0;
   out_2434764514571563776[240] = 0.0;
   out_2434764514571563776[241] = 0.0;
   out_2434764514571563776[242] = 0.0;
   out_2434764514571563776[243] = 0.0;
   out_2434764514571563776[244] = 0.0;
   out_2434764514571563776[245] = 0.0;
   out_2434764514571563776[246] = 0.0;
   out_2434764514571563776[247] = 1.0;
   out_2434764514571563776[248] = 0.0;
   out_2434764514571563776[249] = 0.0;
   out_2434764514571563776[250] = 0.0;
   out_2434764514571563776[251] = 0.0;
   out_2434764514571563776[252] = 0.0;
   out_2434764514571563776[253] = 0.0;
   out_2434764514571563776[254] = 0.0;
   out_2434764514571563776[255] = 0.0;
   out_2434764514571563776[256] = 0.0;
   out_2434764514571563776[257] = 0.0;
   out_2434764514571563776[258] = 0.0;
   out_2434764514571563776[259] = 0.0;
   out_2434764514571563776[260] = 0.0;
   out_2434764514571563776[261] = 0.0;
   out_2434764514571563776[262] = 0.0;
   out_2434764514571563776[263] = 0.0;
   out_2434764514571563776[264] = 0.0;
   out_2434764514571563776[265] = 0.0;
   out_2434764514571563776[266] = 1.0;
   out_2434764514571563776[267] = 0.0;
   out_2434764514571563776[268] = 0.0;
   out_2434764514571563776[269] = 0.0;
   out_2434764514571563776[270] = 0.0;
   out_2434764514571563776[271] = 0.0;
   out_2434764514571563776[272] = 0.0;
   out_2434764514571563776[273] = 0.0;
   out_2434764514571563776[274] = 0.0;
   out_2434764514571563776[275] = 0.0;
   out_2434764514571563776[276] = 0.0;
   out_2434764514571563776[277] = 0.0;
   out_2434764514571563776[278] = 0.0;
   out_2434764514571563776[279] = 0.0;
   out_2434764514571563776[280] = 0.0;
   out_2434764514571563776[281] = 0.0;
   out_2434764514571563776[282] = 0.0;
   out_2434764514571563776[283] = 0.0;
   out_2434764514571563776[284] = 0.0;
   out_2434764514571563776[285] = 1.0;
   out_2434764514571563776[286] = 0.0;
   out_2434764514571563776[287] = 0.0;
   out_2434764514571563776[288] = 0.0;
   out_2434764514571563776[289] = 0.0;
   out_2434764514571563776[290] = 0.0;
   out_2434764514571563776[291] = 0.0;
   out_2434764514571563776[292] = 0.0;
   out_2434764514571563776[293] = 0.0;
   out_2434764514571563776[294] = 0.0;
   out_2434764514571563776[295] = 0.0;
   out_2434764514571563776[296] = 0.0;
   out_2434764514571563776[297] = 0.0;
   out_2434764514571563776[298] = 0.0;
   out_2434764514571563776[299] = 0.0;
   out_2434764514571563776[300] = 0.0;
   out_2434764514571563776[301] = 0.0;
   out_2434764514571563776[302] = 0.0;
   out_2434764514571563776[303] = 0.0;
   out_2434764514571563776[304] = 1.0;
   out_2434764514571563776[305] = 0.0;
   out_2434764514571563776[306] = 0.0;
   out_2434764514571563776[307] = 0.0;
   out_2434764514571563776[308] = 0.0;
   out_2434764514571563776[309] = 0.0;
   out_2434764514571563776[310] = 0.0;
   out_2434764514571563776[311] = 0.0;
   out_2434764514571563776[312] = 0.0;
   out_2434764514571563776[313] = 0.0;
   out_2434764514571563776[314] = 0.0;
   out_2434764514571563776[315] = 0.0;
   out_2434764514571563776[316] = 0.0;
   out_2434764514571563776[317] = 0.0;
   out_2434764514571563776[318] = 0.0;
   out_2434764514571563776[319] = 0.0;
   out_2434764514571563776[320] = 0.0;
   out_2434764514571563776[321] = 0.0;
   out_2434764514571563776[322] = 0.0;
   out_2434764514571563776[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_4944120091642834946) {
   out_4944120091642834946[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_4944120091642834946[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_4944120091642834946[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_4944120091642834946[3] = dt*state[12] + state[3];
   out_4944120091642834946[4] = dt*state[13] + state[4];
   out_4944120091642834946[5] = dt*state[14] + state[5];
   out_4944120091642834946[6] = state[6];
   out_4944120091642834946[7] = state[7];
   out_4944120091642834946[8] = state[8];
   out_4944120091642834946[9] = state[9];
   out_4944120091642834946[10] = state[10];
   out_4944120091642834946[11] = state[11];
   out_4944120091642834946[12] = state[12];
   out_4944120091642834946[13] = state[13];
   out_4944120091642834946[14] = state[14];
   out_4944120091642834946[15] = state[15];
   out_4944120091642834946[16] = state[16];
   out_4944120091642834946[17] = state[17];
}
void F_fun(double *state, double dt, double *out_7093184401669479338) {
   out_7093184401669479338[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7093184401669479338[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7093184401669479338[2] = 0;
   out_7093184401669479338[3] = 0;
   out_7093184401669479338[4] = 0;
   out_7093184401669479338[5] = 0;
   out_7093184401669479338[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7093184401669479338[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7093184401669479338[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_7093184401669479338[9] = 0;
   out_7093184401669479338[10] = 0;
   out_7093184401669479338[11] = 0;
   out_7093184401669479338[12] = 0;
   out_7093184401669479338[13] = 0;
   out_7093184401669479338[14] = 0;
   out_7093184401669479338[15] = 0;
   out_7093184401669479338[16] = 0;
   out_7093184401669479338[17] = 0;
   out_7093184401669479338[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7093184401669479338[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7093184401669479338[20] = 0;
   out_7093184401669479338[21] = 0;
   out_7093184401669479338[22] = 0;
   out_7093184401669479338[23] = 0;
   out_7093184401669479338[24] = 0;
   out_7093184401669479338[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7093184401669479338[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_7093184401669479338[27] = 0;
   out_7093184401669479338[28] = 0;
   out_7093184401669479338[29] = 0;
   out_7093184401669479338[30] = 0;
   out_7093184401669479338[31] = 0;
   out_7093184401669479338[32] = 0;
   out_7093184401669479338[33] = 0;
   out_7093184401669479338[34] = 0;
   out_7093184401669479338[35] = 0;
   out_7093184401669479338[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7093184401669479338[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7093184401669479338[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7093184401669479338[39] = 0;
   out_7093184401669479338[40] = 0;
   out_7093184401669479338[41] = 0;
   out_7093184401669479338[42] = 0;
   out_7093184401669479338[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7093184401669479338[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_7093184401669479338[45] = 0;
   out_7093184401669479338[46] = 0;
   out_7093184401669479338[47] = 0;
   out_7093184401669479338[48] = 0;
   out_7093184401669479338[49] = 0;
   out_7093184401669479338[50] = 0;
   out_7093184401669479338[51] = 0;
   out_7093184401669479338[52] = 0;
   out_7093184401669479338[53] = 0;
   out_7093184401669479338[54] = 0;
   out_7093184401669479338[55] = 0;
   out_7093184401669479338[56] = 0;
   out_7093184401669479338[57] = 1;
   out_7093184401669479338[58] = 0;
   out_7093184401669479338[59] = 0;
   out_7093184401669479338[60] = 0;
   out_7093184401669479338[61] = 0;
   out_7093184401669479338[62] = 0;
   out_7093184401669479338[63] = 0;
   out_7093184401669479338[64] = 0;
   out_7093184401669479338[65] = 0;
   out_7093184401669479338[66] = dt;
   out_7093184401669479338[67] = 0;
   out_7093184401669479338[68] = 0;
   out_7093184401669479338[69] = 0;
   out_7093184401669479338[70] = 0;
   out_7093184401669479338[71] = 0;
   out_7093184401669479338[72] = 0;
   out_7093184401669479338[73] = 0;
   out_7093184401669479338[74] = 0;
   out_7093184401669479338[75] = 0;
   out_7093184401669479338[76] = 1;
   out_7093184401669479338[77] = 0;
   out_7093184401669479338[78] = 0;
   out_7093184401669479338[79] = 0;
   out_7093184401669479338[80] = 0;
   out_7093184401669479338[81] = 0;
   out_7093184401669479338[82] = 0;
   out_7093184401669479338[83] = 0;
   out_7093184401669479338[84] = 0;
   out_7093184401669479338[85] = dt;
   out_7093184401669479338[86] = 0;
   out_7093184401669479338[87] = 0;
   out_7093184401669479338[88] = 0;
   out_7093184401669479338[89] = 0;
   out_7093184401669479338[90] = 0;
   out_7093184401669479338[91] = 0;
   out_7093184401669479338[92] = 0;
   out_7093184401669479338[93] = 0;
   out_7093184401669479338[94] = 0;
   out_7093184401669479338[95] = 1;
   out_7093184401669479338[96] = 0;
   out_7093184401669479338[97] = 0;
   out_7093184401669479338[98] = 0;
   out_7093184401669479338[99] = 0;
   out_7093184401669479338[100] = 0;
   out_7093184401669479338[101] = 0;
   out_7093184401669479338[102] = 0;
   out_7093184401669479338[103] = 0;
   out_7093184401669479338[104] = dt;
   out_7093184401669479338[105] = 0;
   out_7093184401669479338[106] = 0;
   out_7093184401669479338[107] = 0;
   out_7093184401669479338[108] = 0;
   out_7093184401669479338[109] = 0;
   out_7093184401669479338[110] = 0;
   out_7093184401669479338[111] = 0;
   out_7093184401669479338[112] = 0;
   out_7093184401669479338[113] = 0;
   out_7093184401669479338[114] = 1;
   out_7093184401669479338[115] = 0;
   out_7093184401669479338[116] = 0;
   out_7093184401669479338[117] = 0;
   out_7093184401669479338[118] = 0;
   out_7093184401669479338[119] = 0;
   out_7093184401669479338[120] = 0;
   out_7093184401669479338[121] = 0;
   out_7093184401669479338[122] = 0;
   out_7093184401669479338[123] = 0;
   out_7093184401669479338[124] = 0;
   out_7093184401669479338[125] = 0;
   out_7093184401669479338[126] = 0;
   out_7093184401669479338[127] = 0;
   out_7093184401669479338[128] = 0;
   out_7093184401669479338[129] = 0;
   out_7093184401669479338[130] = 0;
   out_7093184401669479338[131] = 0;
   out_7093184401669479338[132] = 0;
   out_7093184401669479338[133] = 1;
   out_7093184401669479338[134] = 0;
   out_7093184401669479338[135] = 0;
   out_7093184401669479338[136] = 0;
   out_7093184401669479338[137] = 0;
   out_7093184401669479338[138] = 0;
   out_7093184401669479338[139] = 0;
   out_7093184401669479338[140] = 0;
   out_7093184401669479338[141] = 0;
   out_7093184401669479338[142] = 0;
   out_7093184401669479338[143] = 0;
   out_7093184401669479338[144] = 0;
   out_7093184401669479338[145] = 0;
   out_7093184401669479338[146] = 0;
   out_7093184401669479338[147] = 0;
   out_7093184401669479338[148] = 0;
   out_7093184401669479338[149] = 0;
   out_7093184401669479338[150] = 0;
   out_7093184401669479338[151] = 0;
   out_7093184401669479338[152] = 1;
   out_7093184401669479338[153] = 0;
   out_7093184401669479338[154] = 0;
   out_7093184401669479338[155] = 0;
   out_7093184401669479338[156] = 0;
   out_7093184401669479338[157] = 0;
   out_7093184401669479338[158] = 0;
   out_7093184401669479338[159] = 0;
   out_7093184401669479338[160] = 0;
   out_7093184401669479338[161] = 0;
   out_7093184401669479338[162] = 0;
   out_7093184401669479338[163] = 0;
   out_7093184401669479338[164] = 0;
   out_7093184401669479338[165] = 0;
   out_7093184401669479338[166] = 0;
   out_7093184401669479338[167] = 0;
   out_7093184401669479338[168] = 0;
   out_7093184401669479338[169] = 0;
   out_7093184401669479338[170] = 0;
   out_7093184401669479338[171] = 1;
   out_7093184401669479338[172] = 0;
   out_7093184401669479338[173] = 0;
   out_7093184401669479338[174] = 0;
   out_7093184401669479338[175] = 0;
   out_7093184401669479338[176] = 0;
   out_7093184401669479338[177] = 0;
   out_7093184401669479338[178] = 0;
   out_7093184401669479338[179] = 0;
   out_7093184401669479338[180] = 0;
   out_7093184401669479338[181] = 0;
   out_7093184401669479338[182] = 0;
   out_7093184401669479338[183] = 0;
   out_7093184401669479338[184] = 0;
   out_7093184401669479338[185] = 0;
   out_7093184401669479338[186] = 0;
   out_7093184401669479338[187] = 0;
   out_7093184401669479338[188] = 0;
   out_7093184401669479338[189] = 0;
   out_7093184401669479338[190] = 1;
   out_7093184401669479338[191] = 0;
   out_7093184401669479338[192] = 0;
   out_7093184401669479338[193] = 0;
   out_7093184401669479338[194] = 0;
   out_7093184401669479338[195] = 0;
   out_7093184401669479338[196] = 0;
   out_7093184401669479338[197] = 0;
   out_7093184401669479338[198] = 0;
   out_7093184401669479338[199] = 0;
   out_7093184401669479338[200] = 0;
   out_7093184401669479338[201] = 0;
   out_7093184401669479338[202] = 0;
   out_7093184401669479338[203] = 0;
   out_7093184401669479338[204] = 0;
   out_7093184401669479338[205] = 0;
   out_7093184401669479338[206] = 0;
   out_7093184401669479338[207] = 0;
   out_7093184401669479338[208] = 0;
   out_7093184401669479338[209] = 1;
   out_7093184401669479338[210] = 0;
   out_7093184401669479338[211] = 0;
   out_7093184401669479338[212] = 0;
   out_7093184401669479338[213] = 0;
   out_7093184401669479338[214] = 0;
   out_7093184401669479338[215] = 0;
   out_7093184401669479338[216] = 0;
   out_7093184401669479338[217] = 0;
   out_7093184401669479338[218] = 0;
   out_7093184401669479338[219] = 0;
   out_7093184401669479338[220] = 0;
   out_7093184401669479338[221] = 0;
   out_7093184401669479338[222] = 0;
   out_7093184401669479338[223] = 0;
   out_7093184401669479338[224] = 0;
   out_7093184401669479338[225] = 0;
   out_7093184401669479338[226] = 0;
   out_7093184401669479338[227] = 0;
   out_7093184401669479338[228] = 1;
   out_7093184401669479338[229] = 0;
   out_7093184401669479338[230] = 0;
   out_7093184401669479338[231] = 0;
   out_7093184401669479338[232] = 0;
   out_7093184401669479338[233] = 0;
   out_7093184401669479338[234] = 0;
   out_7093184401669479338[235] = 0;
   out_7093184401669479338[236] = 0;
   out_7093184401669479338[237] = 0;
   out_7093184401669479338[238] = 0;
   out_7093184401669479338[239] = 0;
   out_7093184401669479338[240] = 0;
   out_7093184401669479338[241] = 0;
   out_7093184401669479338[242] = 0;
   out_7093184401669479338[243] = 0;
   out_7093184401669479338[244] = 0;
   out_7093184401669479338[245] = 0;
   out_7093184401669479338[246] = 0;
   out_7093184401669479338[247] = 1;
   out_7093184401669479338[248] = 0;
   out_7093184401669479338[249] = 0;
   out_7093184401669479338[250] = 0;
   out_7093184401669479338[251] = 0;
   out_7093184401669479338[252] = 0;
   out_7093184401669479338[253] = 0;
   out_7093184401669479338[254] = 0;
   out_7093184401669479338[255] = 0;
   out_7093184401669479338[256] = 0;
   out_7093184401669479338[257] = 0;
   out_7093184401669479338[258] = 0;
   out_7093184401669479338[259] = 0;
   out_7093184401669479338[260] = 0;
   out_7093184401669479338[261] = 0;
   out_7093184401669479338[262] = 0;
   out_7093184401669479338[263] = 0;
   out_7093184401669479338[264] = 0;
   out_7093184401669479338[265] = 0;
   out_7093184401669479338[266] = 1;
   out_7093184401669479338[267] = 0;
   out_7093184401669479338[268] = 0;
   out_7093184401669479338[269] = 0;
   out_7093184401669479338[270] = 0;
   out_7093184401669479338[271] = 0;
   out_7093184401669479338[272] = 0;
   out_7093184401669479338[273] = 0;
   out_7093184401669479338[274] = 0;
   out_7093184401669479338[275] = 0;
   out_7093184401669479338[276] = 0;
   out_7093184401669479338[277] = 0;
   out_7093184401669479338[278] = 0;
   out_7093184401669479338[279] = 0;
   out_7093184401669479338[280] = 0;
   out_7093184401669479338[281] = 0;
   out_7093184401669479338[282] = 0;
   out_7093184401669479338[283] = 0;
   out_7093184401669479338[284] = 0;
   out_7093184401669479338[285] = 1;
   out_7093184401669479338[286] = 0;
   out_7093184401669479338[287] = 0;
   out_7093184401669479338[288] = 0;
   out_7093184401669479338[289] = 0;
   out_7093184401669479338[290] = 0;
   out_7093184401669479338[291] = 0;
   out_7093184401669479338[292] = 0;
   out_7093184401669479338[293] = 0;
   out_7093184401669479338[294] = 0;
   out_7093184401669479338[295] = 0;
   out_7093184401669479338[296] = 0;
   out_7093184401669479338[297] = 0;
   out_7093184401669479338[298] = 0;
   out_7093184401669479338[299] = 0;
   out_7093184401669479338[300] = 0;
   out_7093184401669479338[301] = 0;
   out_7093184401669479338[302] = 0;
   out_7093184401669479338[303] = 0;
   out_7093184401669479338[304] = 1;
   out_7093184401669479338[305] = 0;
   out_7093184401669479338[306] = 0;
   out_7093184401669479338[307] = 0;
   out_7093184401669479338[308] = 0;
   out_7093184401669479338[309] = 0;
   out_7093184401669479338[310] = 0;
   out_7093184401669479338[311] = 0;
   out_7093184401669479338[312] = 0;
   out_7093184401669479338[313] = 0;
   out_7093184401669479338[314] = 0;
   out_7093184401669479338[315] = 0;
   out_7093184401669479338[316] = 0;
   out_7093184401669479338[317] = 0;
   out_7093184401669479338[318] = 0;
   out_7093184401669479338[319] = 0;
   out_7093184401669479338[320] = 0;
   out_7093184401669479338[321] = 0;
   out_7093184401669479338[322] = 0;
   out_7093184401669479338[323] = 1;
}
void h_4(double *state, double *unused, double *out_4692819888483323179) {
   out_4692819888483323179[0] = state[6] + state[9];
   out_4692819888483323179[1] = state[7] + state[10];
   out_4692819888483323179[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8461705635423868277) {
   out_8461705635423868277[0] = 0;
   out_8461705635423868277[1] = 0;
   out_8461705635423868277[2] = 0;
   out_8461705635423868277[3] = 0;
   out_8461705635423868277[4] = 0;
   out_8461705635423868277[5] = 0;
   out_8461705635423868277[6] = 1;
   out_8461705635423868277[7] = 0;
   out_8461705635423868277[8] = 0;
   out_8461705635423868277[9] = 1;
   out_8461705635423868277[10] = 0;
   out_8461705635423868277[11] = 0;
   out_8461705635423868277[12] = 0;
   out_8461705635423868277[13] = 0;
   out_8461705635423868277[14] = 0;
   out_8461705635423868277[15] = 0;
   out_8461705635423868277[16] = 0;
   out_8461705635423868277[17] = 0;
   out_8461705635423868277[18] = 0;
   out_8461705635423868277[19] = 0;
   out_8461705635423868277[20] = 0;
   out_8461705635423868277[21] = 0;
   out_8461705635423868277[22] = 0;
   out_8461705635423868277[23] = 0;
   out_8461705635423868277[24] = 0;
   out_8461705635423868277[25] = 1;
   out_8461705635423868277[26] = 0;
   out_8461705635423868277[27] = 0;
   out_8461705635423868277[28] = 1;
   out_8461705635423868277[29] = 0;
   out_8461705635423868277[30] = 0;
   out_8461705635423868277[31] = 0;
   out_8461705635423868277[32] = 0;
   out_8461705635423868277[33] = 0;
   out_8461705635423868277[34] = 0;
   out_8461705635423868277[35] = 0;
   out_8461705635423868277[36] = 0;
   out_8461705635423868277[37] = 0;
   out_8461705635423868277[38] = 0;
   out_8461705635423868277[39] = 0;
   out_8461705635423868277[40] = 0;
   out_8461705635423868277[41] = 0;
   out_8461705635423868277[42] = 0;
   out_8461705635423868277[43] = 0;
   out_8461705635423868277[44] = 1;
   out_8461705635423868277[45] = 0;
   out_8461705635423868277[46] = 0;
   out_8461705635423868277[47] = 1;
   out_8461705635423868277[48] = 0;
   out_8461705635423868277[49] = 0;
   out_8461705635423868277[50] = 0;
   out_8461705635423868277[51] = 0;
   out_8461705635423868277[52] = 0;
   out_8461705635423868277[53] = 0;
}
void h_10(double *state, double *unused, double *out_1131133617675999593) {
   out_1131133617675999593[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_1131133617675999593[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_1131133617675999593[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_5089104592796037999) {
   out_5089104592796037999[0] = 0;
   out_5089104592796037999[1] = 9.8100000000000005*cos(state[1]);
   out_5089104592796037999[2] = 0;
   out_5089104592796037999[3] = 0;
   out_5089104592796037999[4] = -state[8];
   out_5089104592796037999[5] = state[7];
   out_5089104592796037999[6] = 0;
   out_5089104592796037999[7] = state[5];
   out_5089104592796037999[8] = -state[4];
   out_5089104592796037999[9] = 0;
   out_5089104592796037999[10] = 0;
   out_5089104592796037999[11] = 0;
   out_5089104592796037999[12] = 1;
   out_5089104592796037999[13] = 0;
   out_5089104592796037999[14] = 0;
   out_5089104592796037999[15] = 1;
   out_5089104592796037999[16] = 0;
   out_5089104592796037999[17] = 0;
   out_5089104592796037999[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_5089104592796037999[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_5089104592796037999[20] = 0;
   out_5089104592796037999[21] = state[8];
   out_5089104592796037999[22] = 0;
   out_5089104592796037999[23] = -state[6];
   out_5089104592796037999[24] = -state[5];
   out_5089104592796037999[25] = 0;
   out_5089104592796037999[26] = state[3];
   out_5089104592796037999[27] = 0;
   out_5089104592796037999[28] = 0;
   out_5089104592796037999[29] = 0;
   out_5089104592796037999[30] = 0;
   out_5089104592796037999[31] = 1;
   out_5089104592796037999[32] = 0;
   out_5089104592796037999[33] = 0;
   out_5089104592796037999[34] = 1;
   out_5089104592796037999[35] = 0;
   out_5089104592796037999[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_5089104592796037999[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_5089104592796037999[38] = 0;
   out_5089104592796037999[39] = -state[7];
   out_5089104592796037999[40] = state[6];
   out_5089104592796037999[41] = 0;
   out_5089104592796037999[42] = state[4];
   out_5089104592796037999[43] = -state[3];
   out_5089104592796037999[44] = 0;
   out_5089104592796037999[45] = 0;
   out_5089104592796037999[46] = 0;
   out_5089104592796037999[47] = 0;
   out_5089104592796037999[48] = 0;
   out_5089104592796037999[49] = 0;
   out_5089104592796037999[50] = 1;
   out_5089104592796037999[51] = 0;
   out_5089104592796037999[52] = 0;
   out_5089104592796037999[53] = 1;
}
void h_13(double *state, double *unused, double *out_8712870422670877138) {
   out_8712870422670877138[0] = state[3];
   out_8712870422670877138[1] = state[4];
   out_8712870422670877138[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2374407229968982410) {
   out_2374407229968982410[0] = 0;
   out_2374407229968982410[1] = 0;
   out_2374407229968982410[2] = 0;
   out_2374407229968982410[3] = 1;
   out_2374407229968982410[4] = 0;
   out_2374407229968982410[5] = 0;
   out_2374407229968982410[6] = 0;
   out_2374407229968982410[7] = 0;
   out_2374407229968982410[8] = 0;
   out_2374407229968982410[9] = 0;
   out_2374407229968982410[10] = 0;
   out_2374407229968982410[11] = 0;
   out_2374407229968982410[12] = 0;
   out_2374407229968982410[13] = 0;
   out_2374407229968982410[14] = 0;
   out_2374407229968982410[15] = 0;
   out_2374407229968982410[16] = 0;
   out_2374407229968982410[17] = 0;
   out_2374407229968982410[18] = 0;
   out_2374407229968982410[19] = 0;
   out_2374407229968982410[20] = 0;
   out_2374407229968982410[21] = 0;
   out_2374407229968982410[22] = 1;
   out_2374407229968982410[23] = 0;
   out_2374407229968982410[24] = 0;
   out_2374407229968982410[25] = 0;
   out_2374407229968982410[26] = 0;
   out_2374407229968982410[27] = 0;
   out_2374407229968982410[28] = 0;
   out_2374407229968982410[29] = 0;
   out_2374407229968982410[30] = 0;
   out_2374407229968982410[31] = 0;
   out_2374407229968982410[32] = 0;
   out_2374407229968982410[33] = 0;
   out_2374407229968982410[34] = 0;
   out_2374407229968982410[35] = 0;
   out_2374407229968982410[36] = 0;
   out_2374407229968982410[37] = 0;
   out_2374407229968982410[38] = 0;
   out_2374407229968982410[39] = 0;
   out_2374407229968982410[40] = 0;
   out_2374407229968982410[41] = 1;
   out_2374407229968982410[42] = 0;
   out_2374407229968982410[43] = 0;
   out_2374407229968982410[44] = 0;
   out_2374407229968982410[45] = 0;
   out_2374407229968982410[46] = 0;
   out_2374407229968982410[47] = 0;
   out_2374407229968982410[48] = 0;
   out_2374407229968982410[49] = 0;
   out_2374407229968982410[50] = 0;
   out_2374407229968982410[51] = 0;
   out_2374407229968982410[52] = 0;
   out_2374407229968982410[53] = 0;
}
void h_14(double *state, double *unused, double *out_3278734857690699026) {
   out_3278734857690699026[0] = state[6];
   out_3278734857690699026[1] = state[7];
   out_3278734857690699026[2] = state[8];
}
void H_14(double *state, double *unused, double *out_5378917203128495981) {
   out_5378917203128495981[0] = 0;
   out_5378917203128495981[1] = 0;
   out_5378917203128495981[2] = 0;
   out_5378917203128495981[3] = 0;
   out_5378917203128495981[4] = 0;
   out_5378917203128495981[5] = 0;
   out_5378917203128495981[6] = 1;
   out_5378917203128495981[7] = 0;
   out_5378917203128495981[8] = 0;
   out_5378917203128495981[9] = 0;
   out_5378917203128495981[10] = 0;
   out_5378917203128495981[11] = 0;
   out_5378917203128495981[12] = 0;
   out_5378917203128495981[13] = 0;
   out_5378917203128495981[14] = 0;
   out_5378917203128495981[15] = 0;
   out_5378917203128495981[16] = 0;
   out_5378917203128495981[17] = 0;
   out_5378917203128495981[18] = 0;
   out_5378917203128495981[19] = 0;
   out_5378917203128495981[20] = 0;
   out_5378917203128495981[21] = 0;
   out_5378917203128495981[22] = 0;
   out_5378917203128495981[23] = 0;
   out_5378917203128495981[24] = 0;
   out_5378917203128495981[25] = 1;
   out_5378917203128495981[26] = 0;
   out_5378917203128495981[27] = 0;
   out_5378917203128495981[28] = 0;
   out_5378917203128495981[29] = 0;
   out_5378917203128495981[30] = 0;
   out_5378917203128495981[31] = 0;
   out_5378917203128495981[32] = 0;
   out_5378917203128495981[33] = 0;
   out_5378917203128495981[34] = 0;
   out_5378917203128495981[35] = 0;
   out_5378917203128495981[36] = 0;
   out_5378917203128495981[37] = 0;
   out_5378917203128495981[38] = 0;
   out_5378917203128495981[39] = 0;
   out_5378917203128495981[40] = 0;
   out_5378917203128495981[41] = 0;
   out_5378917203128495981[42] = 0;
   out_5378917203128495981[43] = 0;
   out_5378917203128495981[44] = 1;
   out_5378917203128495981[45] = 0;
   out_5378917203128495981[46] = 0;
   out_5378917203128495981[47] = 0;
   out_5378917203128495981[48] = 0;
   out_5378917203128495981[49] = 0;
   out_5378917203128495981[50] = 0;
   out_5378917203128495981[51] = 0;
   out_5378917203128495981[52] = 0;
   out_5378917203128495981[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_7254673260181494008) {
  err_fun(nom_x, delta_x, out_7254673260181494008);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_6732371445488088772) {
  inv_err_fun(nom_x, true_x, out_6732371445488088772);
}
void pose_H_mod_fun(double *state, double *out_2434764514571563776) {
  H_mod_fun(state, out_2434764514571563776);
}
void pose_f_fun(double *state, double dt, double *out_4944120091642834946) {
  f_fun(state,  dt, out_4944120091642834946);
}
void pose_F_fun(double *state, double dt, double *out_7093184401669479338) {
  F_fun(state,  dt, out_7093184401669479338);
}
void pose_h_4(double *state, double *unused, double *out_4692819888483323179) {
  h_4(state, unused, out_4692819888483323179);
}
void pose_H_4(double *state, double *unused, double *out_8461705635423868277) {
  H_4(state, unused, out_8461705635423868277);
}
void pose_h_10(double *state, double *unused, double *out_1131133617675999593) {
  h_10(state, unused, out_1131133617675999593);
}
void pose_H_10(double *state, double *unused, double *out_5089104592796037999) {
  H_10(state, unused, out_5089104592796037999);
}
void pose_h_13(double *state, double *unused, double *out_8712870422670877138) {
  h_13(state, unused, out_8712870422670877138);
}
void pose_H_13(double *state, double *unused, double *out_2374407229968982410) {
  H_13(state, unused, out_2374407229968982410);
}
void pose_h_14(double *state, double *unused, double *out_3278734857690699026) {
  h_14(state, unused, out_3278734857690699026);
}
void pose_H_14(double *state, double *unused, double *out_5378917203128495981) {
  H_14(state, unused, out_5378917203128495981);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
