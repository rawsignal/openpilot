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
void err_fun(double *nom_x, double *delta_x, double *out_4281085523000754016) {
   out_4281085523000754016[0] = delta_x[0] + nom_x[0];
   out_4281085523000754016[1] = delta_x[1] + nom_x[1];
   out_4281085523000754016[2] = delta_x[2] + nom_x[2];
   out_4281085523000754016[3] = delta_x[3] + nom_x[3];
   out_4281085523000754016[4] = delta_x[4] + nom_x[4];
   out_4281085523000754016[5] = delta_x[5] + nom_x[5];
   out_4281085523000754016[6] = delta_x[6] + nom_x[6];
   out_4281085523000754016[7] = delta_x[7] + nom_x[7];
   out_4281085523000754016[8] = delta_x[8] + nom_x[8];
   out_4281085523000754016[9] = delta_x[9] + nom_x[9];
   out_4281085523000754016[10] = delta_x[10] + nom_x[10];
   out_4281085523000754016[11] = delta_x[11] + nom_x[11];
   out_4281085523000754016[12] = delta_x[12] + nom_x[12];
   out_4281085523000754016[13] = delta_x[13] + nom_x[13];
   out_4281085523000754016[14] = delta_x[14] + nom_x[14];
   out_4281085523000754016[15] = delta_x[15] + nom_x[15];
   out_4281085523000754016[16] = delta_x[16] + nom_x[16];
   out_4281085523000754016[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7803285783767947319) {
   out_7803285783767947319[0] = -nom_x[0] + true_x[0];
   out_7803285783767947319[1] = -nom_x[1] + true_x[1];
   out_7803285783767947319[2] = -nom_x[2] + true_x[2];
   out_7803285783767947319[3] = -nom_x[3] + true_x[3];
   out_7803285783767947319[4] = -nom_x[4] + true_x[4];
   out_7803285783767947319[5] = -nom_x[5] + true_x[5];
   out_7803285783767947319[6] = -nom_x[6] + true_x[6];
   out_7803285783767947319[7] = -nom_x[7] + true_x[7];
   out_7803285783767947319[8] = -nom_x[8] + true_x[8];
   out_7803285783767947319[9] = -nom_x[9] + true_x[9];
   out_7803285783767947319[10] = -nom_x[10] + true_x[10];
   out_7803285783767947319[11] = -nom_x[11] + true_x[11];
   out_7803285783767947319[12] = -nom_x[12] + true_x[12];
   out_7803285783767947319[13] = -nom_x[13] + true_x[13];
   out_7803285783767947319[14] = -nom_x[14] + true_x[14];
   out_7803285783767947319[15] = -nom_x[15] + true_x[15];
   out_7803285783767947319[16] = -nom_x[16] + true_x[16];
   out_7803285783767947319[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_4559444642956092070) {
   out_4559444642956092070[0] = 1.0;
   out_4559444642956092070[1] = 0.0;
   out_4559444642956092070[2] = 0.0;
   out_4559444642956092070[3] = 0.0;
   out_4559444642956092070[4] = 0.0;
   out_4559444642956092070[5] = 0.0;
   out_4559444642956092070[6] = 0.0;
   out_4559444642956092070[7] = 0.0;
   out_4559444642956092070[8] = 0.0;
   out_4559444642956092070[9] = 0.0;
   out_4559444642956092070[10] = 0.0;
   out_4559444642956092070[11] = 0.0;
   out_4559444642956092070[12] = 0.0;
   out_4559444642956092070[13] = 0.0;
   out_4559444642956092070[14] = 0.0;
   out_4559444642956092070[15] = 0.0;
   out_4559444642956092070[16] = 0.0;
   out_4559444642956092070[17] = 0.0;
   out_4559444642956092070[18] = 0.0;
   out_4559444642956092070[19] = 1.0;
   out_4559444642956092070[20] = 0.0;
   out_4559444642956092070[21] = 0.0;
   out_4559444642956092070[22] = 0.0;
   out_4559444642956092070[23] = 0.0;
   out_4559444642956092070[24] = 0.0;
   out_4559444642956092070[25] = 0.0;
   out_4559444642956092070[26] = 0.0;
   out_4559444642956092070[27] = 0.0;
   out_4559444642956092070[28] = 0.0;
   out_4559444642956092070[29] = 0.0;
   out_4559444642956092070[30] = 0.0;
   out_4559444642956092070[31] = 0.0;
   out_4559444642956092070[32] = 0.0;
   out_4559444642956092070[33] = 0.0;
   out_4559444642956092070[34] = 0.0;
   out_4559444642956092070[35] = 0.0;
   out_4559444642956092070[36] = 0.0;
   out_4559444642956092070[37] = 0.0;
   out_4559444642956092070[38] = 1.0;
   out_4559444642956092070[39] = 0.0;
   out_4559444642956092070[40] = 0.0;
   out_4559444642956092070[41] = 0.0;
   out_4559444642956092070[42] = 0.0;
   out_4559444642956092070[43] = 0.0;
   out_4559444642956092070[44] = 0.0;
   out_4559444642956092070[45] = 0.0;
   out_4559444642956092070[46] = 0.0;
   out_4559444642956092070[47] = 0.0;
   out_4559444642956092070[48] = 0.0;
   out_4559444642956092070[49] = 0.0;
   out_4559444642956092070[50] = 0.0;
   out_4559444642956092070[51] = 0.0;
   out_4559444642956092070[52] = 0.0;
   out_4559444642956092070[53] = 0.0;
   out_4559444642956092070[54] = 0.0;
   out_4559444642956092070[55] = 0.0;
   out_4559444642956092070[56] = 0.0;
   out_4559444642956092070[57] = 1.0;
   out_4559444642956092070[58] = 0.0;
   out_4559444642956092070[59] = 0.0;
   out_4559444642956092070[60] = 0.0;
   out_4559444642956092070[61] = 0.0;
   out_4559444642956092070[62] = 0.0;
   out_4559444642956092070[63] = 0.0;
   out_4559444642956092070[64] = 0.0;
   out_4559444642956092070[65] = 0.0;
   out_4559444642956092070[66] = 0.0;
   out_4559444642956092070[67] = 0.0;
   out_4559444642956092070[68] = 0.0;
   out_4559444642956092070[69] = 0.0;
   out_4559444642956092070[70] = 0.0;
   out_4559444642956092070[71] = 0.0;
   out_4559444642956092070[72] = 0.0;
   out_4559444642956092070[73] = 0.0;
   out_4559444642956092070[74] = 0.0;
   out_4559444642956092070[75] = 0.0;
   out_4559444642956092070[76] = 1.0;
   out_4559444642956092070[77] = 0.0;
   out_4559444642956092070[78] = 0.0;
   out_4559444642956092070[79] = 0.0;
   out_4559444642956092070[80] = 0.0;
   out_4559444642956092070[81] = 0.0;
   out_4559444642956092070[82] = 0.0;
   out_4559444642956092070[83] = 0.0;
   out_4559444642956092070[84] = 0.0;
   out_4559444642956092070[85] = 0.0;
   out_4559444642956092070[86] = 0.0;
   out_4559444642956092070[87] = 0.0;
   out_4559444642956092070[88] = 0.0;
   out_4559444642956092070[89] = 0.0;
   out_4559444642956092070[90] = 0.0;
   out_4559444642956092070[91] = 0.0;
   out_4559444642956092070[92] = 0.0;
   out_4559444642956092070[93] = 0.0;
   out_4559444642956092070[94] = 0.0;
   out_4559444642956092070[95] = 1.0;
   out_4559444642956092070[96] = 0.0;
   out_4559444642956092070[97] = 0.0;
   out_4559444642956092070[98] = 0.0;
   out_4559444642956092070[99] = 0.0;
   out_4559444642956092070[100] = 0.0;
   out_4559444642956092070[101] = 0.0;
   out_4559444642956092070[102] = 0.0;
   out_4559444642956092070[103] = 0.0;
   out_4559444642956092070[104] = 0.0;
   out_4559444642956092070[105] = 0.0;
   out_4559444642956092070[106] = 0.0;
   out_4559444642956092070[107] = 0.0;
   out_4559444642956092070[108] = 0.0;
   out_4559444642956092070[109] = 0.0;
   out_4559444642956092070[110] = 0.0;
   out_4559444642956092070[111] = 0.0;
   out_4559444642956092070[112] = 0.0;
   out_4559444642956092070[113] = 0.0;
   out_4559444642956092070[114] = 1.0;
   out_4559444642956092070[115] = 0.0;
   out_4559444642956092070[116] = 0.0;
   out_4559444642956092070[117] = 0.0;
   out_4559444642956092070[118] = 0.0;
   out_4559444642956092070[119] = 0.0;
   out_4559444642956092070[120] = 0.0;
   out_4559444642956092070[121] = 0.0;
   out_4559444642956092070[122] = 0.0;
   out_4559444642956092070[123] = 0.0;
   out_4559444642956092070[124] = 0.0;
   out_4559444642956092070[125] = 0.0;
   out_4559444642956092070[126] = 0.0;
   out_4559444642956092070[127] = 0.0;
   out_4559444642956092070[128] = 0.0;
   out_4559444642956092070[129] = 0.0;
   out_4559444642956092070[130] = 0.0;
   out_4559444642956092070[131] = 0.0;
   out_4559444642956092070[132] = 0.0;
   out_4559444642956092070[133] = 1.0;
   out_4559444642956092070[134] = 0.0;
   out_4559444642956092070[135] = 0.0;
   out_4559444642956092070[136] = 0.0;
   out_4559444642956092070[137] = 0.0;
   out_4559444642956092070[138] = 0.0;
   out_4559444642956092070[139] = 0.0;
   out_4559444642956092070[140] = 0.0;
   out_4559444642956092070[141] = 0.0;
   out_4559444642956092070[142] = 0.0;
   out_4559444642956092070[143] = 0.0;
   out_4559444642956092070[144] = 0.0;
   out_4559444642956092070[145] = 0.0;
   out_4559444642956092070[146] = 0.0;
   out_4559444642956092070[147] = 0.0;
   out_4559444642956092070[148] = 0.0;
   out_4559444642956092070[149] = 0.0;
   out_4559444642956092070[150] = 0.0;
   out_4559444642956092070[151] = 0.0;
   out_4559444642956092070[152] = 1.0;
   out_4559444642956092070[153] = 0.0;
   out_4559444642956092070[154] = 0.0;
   out_4559444642956092070[155] = 0.0;
   out_4559444642956092070[156] = 0.0;
   out_4559444642956092070[157] = 0.0;
   out_4559444642956092070[158] = 0.0;
   out_4559444642956092070[159] = 0.0;
   out_4559444642956092070[160] = 0.0;
   out_4559444642956092070[161] = 0.0;
   out_4559444642956092070[162] = 0.0;
   out_4559444642956092070[163] = 0.0;
   out_4559444642956092070[164] = 0.0;
   out_4559444642956092070[165] = 0.0;
   out_4559444642956092070[166] = 0.0;
   out_4559444642956092070[167] = 0.0;
   out_4559444642956092070[168] = 0.0;
   out_4559444642956092070[169] = 0.0;
   out_4559444642956092070[170] = 0.0;
   out_4559444642956092070[171] = 1.0;
   out_4559444642956092070[172] = 0.0;
   out_4559444642956092070[173] = 0.0;
   out_4559444642956092070[174] = 0.0;
   out_4559444642956092070[175] = 0.0;
   out_4559444642956092070[176] = 0.0;
   out_4559444642956092070[177] = 0.0;
   out_4559444642956092070[178] = 0.0;
   out_4559444642956092070[179] = 0.0;
   out_4559444642956092070[180] = 0.0;
   out_4559444642956092070[181] = 0.0;
   out_4559444642956092070[182] = 0.0;
   out_4559444642956092070[183] = 0.0;
   out_4559444642956092070[184] = 0.0;
   out_4559444642956092070[185] = 0.0;
   out_4559444642956092070[186] = 0.0;
   out_4559444642956092070[187] = 0.0;
   out_4559444642956092070[188] = 0.0;
   out_4559444642956092070[189] = 0.0;
   out_4559444642956092070[190] = 1.0;
   out_4559444642956092070[191] = 0.0;
   out_4559444642956092070[192] = 0.0;
   out_4559444642956092070[193] = 0.0;
   out_4559444642956092070[194] = 0.0;
   out_4559444642956092070[195] = 0.0;
   out_4559444642956092070[196] = 0.0;
   out_4559444642956092070[197] = 0.0;
   out_4559444642956092070[198] = 0.0;
   out_4559444642956092070[199] = 0.0;
   out_4559444642956092070[200] = 0.0;
   out_4559444642956092070[201] = 0.0;
   out_4559444642956092070[202] = 0.0;
   out_4559444642956092070[203] = 0.0;
   out_4559444642956092070[204] = 0.0;
   out_4559444642956092070[205] = 0.0;
   out_4559444642956092070[206] = 0.0;
   out_4559444642956092070[207] = 0.0;
   out_4559444642956092070[208] = 0.0;
   out_4559444642956092070[209] = 1.0;
   out_4559444642956092070[210] = 0.0;
   out_4559444642956092070[211] = 0.0;
   out_4559444642956092070[212] = 0.0;
   out_4559444642956092070[213] = 0.0;
   out_4559444642956092070[214] = 0.0;
   out_4559444642956092070[215] = 0.0;
   out_4559444642956092070[216] = 0.0;
   out_4559444642956092070[217] = 0.0;
   out_4559444642956092070[218] = 0.0;
   out_4559444642956092070[219] = 0.0;
   out_4559444642956092070[220] = 0.0;
   out_4559444642956092070[221] = 0.0;
   out_4559444642956092070[222] = 0.0;
   out_4559444642956092070[223] = 0.0;
   out_4559444642956092070[224] = 0.0;
   out_4559444642956092070[225] = 0.0;
   out_4559444642956092070[226] = 0.0;
   out_4559444642956092070[227] = 0.0;
   out_4559444642956092070[228] = 1.0;
   out_4559444642956092070[229] = 0.0;
   out_4559444642956092070[230] = 0.0;
   out_4559444642956092070[231] = 0.0;
   out_4559444642956092070[232] = 0.0;
   out_4559444642956092070[233] = 0.0;
   out_4559444642956092070[234] = 0.0;
   out_4559444642956092070[235] = 0.0;
   out_4559444642956092070[236] = 0.0;
   out_4559444642956092070[237] = 0.0;
   out_4559444642956092070[238] = 0.0;
   out_4559444642956092070[239] = 0.0;
   out_4559444642956092070[240] = 0.0;
   out_4559444642956092070[241] = 0.0;
   out_4559444642956092070[242] = 0.0;
   out_4559444642956092070[243] = 0.0;
   out_4559444642956092070[244] = 0.0;
   out_4559444642956092070[245] = 0.0;
   out_4559444642956092070[246] = 0.0;
   out_4559444642956092070[247] = 1.0;
   out_4559444642956092070[248] = 0.0;
   out_4559444642956092070[249] = 0.0;
   out_4559444642956092070[250] = 0.0;
   out_4559444642956092070[251] = 0.0;
   out_4559444642956092070[252] = 0.0;
   out_4559444642956092070[253] = 0.0;
   out_4559444642956092070[254] = 0.0;
   out_4559444642956092070[255] = 0.0;
   out_4559444642956092070[256] = 0.0;
   out_4559444642956092070[257] = 0.0;
   out_4559444642956092070[258] = 0.0;
   out_4559444642956092070[259] = 0.0;
   out_4559444642956092070[260] = 0.0;
   out_4559444642956092070[261] = 0.0;
   out_4559444642956092070[262] = 0.0;
   out_4559444642956092070[263] = 0.0;
   out_4559444642956092070[264] = 0.0;
   out_4559444642956092070[265] = 0.0;
   out_4559444642956092070[266] = 1.0;
   out_4559444642956092070[267] = 0.0;
   out_4559444642956092070[268] = 0.0;
   out_4559444642956092070[269] = 0.0;
   out_4559444642956092070[270] = 0.0;
   out_4559444642956092070[271] = 0.0;
   out_4559444642956092070[272] = 0.0;
   out_4559444642956092070[273] = 0.0;
   out_4559444642956092070[274] = 0.0;
   out_4559444642956092070[275] = 0.0;
   out_4559444642956092070[276] = 0.0;
   out_4559444642956092070[277] = 0.0;
   out_4559444642956092070[278] = 0.0;
   out_4559444642956092070[279] = 0.0;
   out_4559444642956092070[280] = 0.0;
   out_4559444642956092070[281] = 0.0;
   out_4559444642956092070[282] = 0.0;
   out_4559444642956092070[283] = 0.0;
   out_4559444642956092070[284] = 0.0;
   out_4559444642956092070[285] = 1.0;
   out_4559444642956092070[286] = 0.0;
   out_4559444642956092070[287] = 0.0;
   out_4559444642956092070[288] = 0.0;
   out_4559444642956092070[289] = 0.0;
   out_4559444642956092070[290] = 0.0;
   out_4559444642956092070[291] = 0.0;
   out_4559444642956092070[292] = 0.0;
   out_4559444642956092070[293] = 0.0;
   out_4559444642956092070[294] = 0.0;
   out_4559444642956092070[295] = 0.0;
   out_4559444642956092070[296] = 0.0;
   out_4559444642956092070[297] = 0.0;
   out_4559444642956092070[298] = 0.0;
   out_4559444642956092070[299] = 0.0;
   out_4559444642956092070[300] = 0.0;
   out_4559444642956092070[301] = 0.0;
   out_4559444642956092070[302] = 0.0;
   out_4559444642956092070[303] = 0.0;
   out_4559444642956092070[304] = 1.0;
   out_4559444642956092070[305] = 0.0;
   out_4559444642956092070[306] = 0.0;
   out_4559444642956092070[307] = 0.0;
   out_4559444642956092070[308] = 0.0;
   out_4559444642956092070[309] = 0.0;
   out_4559444642956092070[310] = 0.0;
   out_4559444642956092070[311] = 0.0;
   out_4559444642956092070[312] = 0.0;
   out_4559444642956092070[313] = 0.0;
   out_4559444642956092070[314] = 0.0;
   out_4559444642956092070[315] = 0.0;
   out_4559444642956092070[316] = 0.0;
   out_4559444642956092070[317] = 0.0;
   out_4559444642956092070[318] = 0.0;
   out_4559444642956092070[319] = 0.0;
   out_4559444642956092070[320] = 0.0;
   out_4559444642956092070[321] = 0.0;
   out_4559444642956092070[322] = 0.0;
   out_4559444642956092070[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_819200123031164352) {
   out_819200123031164352[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_819200123031164352[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_819200123031164352[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_819200123031164352[3] = dt*state[12] + state[3];
   out_819200123031164352[4] = dt*state[13] + state[4];
   out_819200123031164352[5] = dt*state[14] + state[5];
   out_819200123031164352[6] = state[6];
   out_819200123031164352[7] = state[7];
   out_819200123031164352[8] = state[8];
   out_819200123031164352[9] = state[9];
   out_819200123031164352[10] = state[10];
   out_819200123031164352[11] = state[11];
   out_819200123031164352[12] = state[12];
   out_819200123031164352[13] = state[13];
   out_819200123031164352[14] = state[14];
   out_819200123031164352[15] = state[15];
   out_819200123031164352[16] = state[16];
   out_819200123031164352[17] = state[17];
}
void F_fun(double *state, double dt, double *out_1797944479821926981) {
   out_1797944479821926981[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1797944479821926981[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1797944479821926981[2] = 0;
   out_1797944479821926981[3] = 0;
   out_1797944479821926981[4] = 0;
   out_1797944479821926981[5] = 0;
   out_1797944479821926981[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1797944479821926981[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1797944479821926981[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_1797944479821926981[9] = 0;
   out_1797944479821926981[10] = 0;
   out_1797944479821926981[11] = 0;
   out_1797944479821926981[12] = 0;
   out_1797944479821926981[13] = 0;
   out_1797944479821926981[14] = 0;
   out_1797944479821926981[15] = 0;
   out_1797944479821926981[16] = 0;
   out_1797944479821926981[17] = 0;
   out_1797944479821926981[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1797944479821926981[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1797944479821926981[20] = 0;
   out_1797944479821926981[21] = 0;
   out_1797944479821926981[22] = 0;
   out_1797944479821926981[23] = 0;
   out_1797944479821926981[24] = 0;
   out_1797944479821926981[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1797944479821926981[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_1797944479821926981[27] = 0;
   out_1797944479821926981[28] = 0;
   out_1797944479821926981[29] = 0;
   out_1797944479821926981[30] = 0;
   out_1797944479821926981[31] = 0;
   out_1797944479821926981[32] = 0;
   out_1797944479821926981[33] = 0;
   out_1797944479821926981[34] = 0;
   out_1797944479821926981[35] = 0;
   out_1797944479821926981[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1797944479821926981[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1797944479821926981[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1797944479821926981[39] = 0;
   out_1797944479821926981[40] = 0;
   out_1797944479821926981[41] = 0;
   out_1797944479821926981[42] = 0;
   out_1797944479821926981[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1797944479821926981[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_1797944479821926981[45] = 0;
   out_1797944479821926981[46] = 0;
   out_1797944479821926981[47] = 0;
   out_1797944479821926981[48] = 0;
   out_1797944479821926981[49] = 0;
   out_1797944479821926981[50] = 0;
   out_1797944479821926981[51] = 0;
   out_1797944479821926981[52] = 0;
   out_1797944479821926981[53] = 0;
   out_1797944479821926981[54] = 0;
   out_1797944479821926981[55] = 0;
   out_1797944479821926981[56] = 0;
   out_1797944479821926981[57] = 1;
   out_1797944479821926981[58] = 0;
   out_1797944479821926981[59] = 0;
   out_1797944479821926981[60] = 0;
   out_1797944479821926981[61] = 0;
   out_1797944479821926981[62] = 0;
   out_1797944479821926981[63] = 0;
   out_1797944479821926981[64] = 0;
   out_1797944479821926981[65] = 0;
   out_1797944479821926981[66] = dt;
   out_1797944479821926981[67] = 0;
   out_1797944479821926981[68] = 0;
   out_1797944479821926981[69] = 0;
   out_1797944479821926981[70] = 0;
   out_1797944479821926981[71] = 0;
   out_1797944479821926981[72] = 0;
   out_1797944479821926981[73] = 0;
   out_1797944479821926981[74] = 0;
   out_1797944479821926981[75] = 0;
   out_1797944479821926981[76] = 1;
   out_1797944479821926981[77] = 0;
   out_1797944479821926981[78] = 0;
   out_1797944479821926981[79] = 0;
   out_1797944479821926981[80] = 0;
   out_1797944479821926981[81] = 0;
   out_1797944479821926981[82] = 0;
   out_1797944479821926981[83] = 0;
   out_1797944479821926981[84] = 0;
   out_1797944479821926981[85] = dt;
   out_1797944479821926981[86] = 0;
   out_1797944479821926981[87] = 0;
   out_1797944479821926981[88] = 0;
   out_1797944479821926981[89] = 0;
   out_1797944479821926981[90] = 0;
   out_1797944479821926981[91] = 0;
   out_1797944479821926981[92] = 0;
   out_1797944479821926981[93] = 0;
   out_1797944479821926981[94] = 0;
   out_1797944479821926981[95] = 1;
   out_1797944479821926981[96] = 0;
   out_1797944479821926981[97] = 0;
   out_1797944479821926981[98] = 0;
   out_1797944479821926981[99] = 0;
   out_1797944479821926981[100] = 0;
   out_1797944479821926981[101] = 0;
   out_1797944479821926981[102] = 0;
   out_1797944479821926981[103] = 0;
   out_1797944479821926981[104] = dt;
   out_1797944479821926981[105] = 0;
   out_1797944479821926981[106] = 0;
   out_1797944479821926981[107] = 0;
   out_1797944479821926981[108] = 0;
   out_1797944479821926981[109] = 0;
   out_1797944479821926981[110] = 0;
   out_1797944479821926981[111] = 0;
   out_1797944479821926981[112] = 0;
   out_1797944479821926981[113] = 0;
   out_1797944479821926981[114] = 1;
   out_1797944479821926981[115] = 0;
   out_1797944479821926981[116] = 0;
   out_1797944479821926981[117] = 0;
   out_1797944479821926981[118] = 0;
   out_1797944479821926981[119] = 0;
   out_1797944479821926981[120] = 0;
   out_1797944479821926981[121] = 0;
   out_1797944479821926981[122] = 0;
   out_1797944479821926981[123] = 0;
   out_1797944479821926981[124] = 0;
   out_1797944479821926981[125] = 0;
   out_1797944479821926981[126] = 0;
   out_1797944479821926981[127] = 0;
   out_1797944479821926981[128] = 0;
   out_1797944479821926981[129] = 0;
   out_1797944479821926981[130] = 0;
   out_1797944479821926981[131] = 0;
   out_1797944479821926981[132] = 0;
   out_1797944479821926981[133] = 1;
   out_1797944479821926981[134] = 0;
   out_1797944479821926981[135] = 0;
   out_1797944479821926981[136] = 0;
   out_1797944479821926981[137] = 0;
   out_1797944479821926981[138] = 0;
   out_1797944479821926981[139] = 0;
   out_1797944479821926981[140] = 0;
   out_1797944479821926981[141] = 0;
   out_1797944479821926981[142] = 0;
   out_1797944479821926981[143] = 0;
   out_1797944479821926981[144] = 0;
   out_1797944479821926981[145] = 0;
   out_1797944479821926981[146] = 0;
   out_1797944479821926981[147] = 0;
   out_1797944479821926981[148] = 0;
   out_1797944479821926981[149] = 0;
   out_1797944479821926981[150] = 0;
   out_1797944479821926981[151] = 0;
   out_1797944479821926981[152] = 1;
   out_1797944479821926981[153] = 0;
   out_1797944479821926981[154] = 0;
   out_1797944479821926981[155] = 0;
   out_1797944479821926981[156] = 0;
   out_1797944479821926981[157] = 0;
   out_1797944479821926981[158] = 0;
   out_1797944479821926981[159] = 0;
   out_1797944479821926981[160] = 0;
   out_1797944479821926981[161] = 0;
   out_1797944479821926981[162] = 0;
   out_1797944479821926981[163] = 0;
   out_1797944479821926981[164] = 0;
   out_1797944479821926981[165] = 0;
   out_1797944479821926981[166] = 0;
   out_1797944479821926981[167] = 0;
   out_1797944479821926981[168] = 0;
   out_1797944479821926981[169] = 0;
   out_1797944479821926981[170] = 0;
   out_1797944479821926981[171] = 1;
   out_1797944479821926981[172] = 0;
   out_1797944479821926981[173] = 0;
   out_1797944479821926981[174] = 0;
   out_1797944479821926981[175] = 0;
   out_1797944479821926981[176] = 0;
   out_1797944479821926981[177] = 0;
   out_1797944479821926981[178] = 0;
   out_1797944479821926981[179] = 0;
   out_1797944479821926981[180] = 0;
   out_1797944479821926981[181] = 0;
   out_1797944479821926981[182] = 0;
   out_1797944479821926981[183] = 0;
   out_1797944479821926981[184] = 0;
   out_1797944479821926981[185] = 0;
   out_1797944479821926981[186] = 0;
   out_1797944479821926981[187] = 0;
   out_1797944479821926981[188] = 0;
   out_1797944479821926981[189] = 0;
   out_1797944479821926981[190] = 1;
   out_1797944479821926981[191] = 0;
   out_1797944479821926981[192] = 0;
   out_1797944479821926981[193] = 0;
   out_1797944479821926981[194] = 0;
   out_1797944479821926981[195] = 0;
   out_1797944479821926981[196] = 0;
   out_1797944479821926981[197] = 0;
   out_1797944479821926981[198] = 0;
   out_1797944479821926981[199] = 0;
   out_1797944479821926981[200] = 0;
   out_1797944479821926981[201] = 0;
   out_1797944479821926981[202] = 0;
   out_1797944479821926981[203] = 0;
   out_1797944479821926981[204] = 0;
   out_1797944479821926981[205] = 0;
   out_1797944479821926981[206] = 0;
   out_1797944479821926981[207] = 0;
   out_1797944479821926981[208] = 0;
   out_1797944479821926981[209] = 1;
   out_1797944479821926981[210] = 0;
   out_1797944479821926981[211] = 0;
   out_1797944479821926981[212] = 0;
   out_1797944479821926981[213] = 0;
   out_1797944479821926981[214] = 0;
   out_1797944479821926981[215] = 0;
   out_1797944479821926981[216] = 0;
   out_1797944479821926981[217] = 0;
   out_1797944479821926981[218] = 0;
   out_1797944479821926981[219] = 0;
   out_1797944479821926981[220] = 0;
   out_1797944479821926981[221] = 0;
   out_1797944479821926981[222] = 0;
   out_1797944479821926981[223] = 0;
   out_1797944479821926981[224] = 0;
   out_1797944479821926981[225] = 0;
   out_1797944479821926981[226] = 0;
   out_1797944479821926981[227] = 0;
   out_1797944479821926981[228] = 1;
   out_1797944479821926981[229] = 0;
   out_1797944479821926981[230] = 0;
   out_1797944479821926981[231] = 0;
   out_1797944479821926981[232] = 0;
   out_1797944479821926981[233] = 0;
   out_1797944479821926981[234] = 0;
   out_1797944479821926981[235] = 0;
   out_1797944479821926981[236] = 0;
   out_1797944479821926981[237] = 0;
   out_1797944479821926981[238] = 0;
   out_1797944479821926981[239] = 0;
   out_1797944479821926981[240] = 0;
   out_1797944479821926981[241] = 0;
   out_1797944479821926981[242] = 0;
   out_1797944479821926981[243] = 0;
   out_1797944479821926981[244] = 0;
   out_1797944479821926981[245] = 0;
   out_1797944479821926981[246] = 0;
   out_1797944479821926981[247] = 1;
   out_1797944479821926981[248] = 0;
   out_1797944479821926981[249] = 0;
   out_1797944479821926981[250] = 0;
   out_1797944479821926981[251] = 0;
   out_1797944479821926981[252] = 0;
   out_1797944479821926981[253] = 0;
   out_1797944479821926981[254] = 0;
   out_1797944479821926981[255] = 0;
   out_1797944479821926981[256] = 0;
   out_1797944479821926981[257] = 0;
   out_1797944479821926981[258] = 0;
   out_1797944479821926981[259] = 0;
   out_1797944479821926981[260] = 0;
   out_1797944479821926981[261] = 0;
   out_1797944479821926981[262] = 0;
   out_1797944479821926981[263] = 0;
   out_1797944479821926981[264] = 0;
   out_1797944479821926981[265] = 0;
   out_1797944479821926981[266] = 1;
   out_1797944479821926981[267] = 0;
   out_1797944479821926981[268] = 0;
   out_1797944479821926981[269] = 0;
   out_1797944479821926981[270] = 0;
   out_1797944479821926981[271] = 0;
   out_1797944479821926981[272] = 0;
   out_1797944479821926981[273] = 0;
   out_1797944479821926981[274] = 0;
   out_1797944479821926981[275] = 0;
   out_1797944479821926981[276] = 0;
   out_1797944479821926981[277] = 0;
   out_1797944479821926981[278] = 0;
   out_1797944479821926981[279] = 0;
   out_1797944479821926981[280] = 0;
   out_1797944479821926981[281] = 0;
   out_1797944479821926981[282] = 0;
   out_1797944479821926981[283] = 0;
   out_1797944479821926981[284] = 0;
   out_1797944479821926981[285] = 1;
   out_1797944479821926981[286] = 0;
   out_1797944479821926981[287] = 0;
   out_1797944479821926981[288] = 0;
   out_1797944479821926981[289] = 0;
   out_1797944479821926981[290] = 0;
   out_1797944479821926981[291] = 0;
   out_1797944479821926981[292] = 0;
   out_1797944479821926981[293] = 0;
   out_1797944479821926981[294] = 0;
   out_1797944479821926981[295] = 0;
   out_1797944479821926981[296] = 0;
   out_1797944479821926981[297] = 0;
   out_1797944479821926981[298] = 0;
   out_1797944479821926981[299] = 0;
   out_1797944479821926981[300] = 0;
   out_1797944479821926981[301] = 0;
   out_1797944479821926981[302] = 0;
   out_1797944479821926981[303] = 0;
   out_1797944479821926981[304] = 1;
   out_1797944479821926981[305] = 0;
   out_1797944479821926981[306] = 0;
   out_1797944479821926981[307] = 0;
   out_1797944479821926981[308] = 0;
   out_1797944479821926981[309] = 0;
   out_1797944479821926981[310] = 0;
   out_1797944479821926981[311] = 0;
   out_1797944479821926981[312] = 0;
   out_1797944479821926981[313] = 0;
   out_1797944479821926981[314] = 0;
   out_1797944479821926981[315] = 0;
   out_1797944479821926981[316] = 0;
   out_1797944479821926981[317] = 0;
   out_1797944479821926981[318] = 0;
   out_1797944479821926981[319] = 0;
   out_1797944479821926981[320] = 0;
   out_1797944479821926981[321] = 0;
   out_1797944479821926981[322] = 0;
   out_1797944479821926981[323] = 1;
}
void h_4(double *state, double *unused, double *out_1556670692920147002) {
   out_1556670692920147002[0] = state[6] + state[9];
   out_1556670692920147002[1] = state[7] + state[10];
   out_1556670692920147002[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_8039217624001500825) {
   out_8039217624001500825[0] = 0;
   out_8039217624001500825[1] = 0;
   out_8039217624001500825[2] = 0;
   out_8039217624001500825[3] = 0;
   out_8039217624001500825[4] = 0;
   out_8039217624001500825[5] = 0;
   out_8039217624001500825[6] = 1;
   out_8039217624001500825[7] = 0;
   out_8039217624001500825[8] = 0;
   out_8039217624001500825[9] = 1;
   out_8039217624001500825[10] = 0;
   out_8039217624001500825[11] = 0;
   out_8039217624001500825[12] = 0;
   out_8039217624001500825[13] = 0;
   out_8039217624001500825[14] = 0;
   out_8039217624001500825[15] = 0;
   out_8039217624001500825[16] = 0;
   out_8039217624001500825[17] = 0;
   out_8039217624001500825[18] = 0;
   out_8039217624001500825[19] = 0;
   out_8039217624001500825[20] = 0;
   out_8039217624001500825[21] = 0;
   out_8039217624001500825[22] = 0;
   out_8039217624001500825[23] = 0;
   out_8039217624001500825[24] = 0;
   out_8039217624001500825[25] = 1;
   out_8039217624001500825[26] = 0;
   out_8039217624001500825[27] = 0;
   out_8039217624001500825[28] = 1;
   out_8039217624001500825[29] = 0;
   out_8039217624001500825[30] = 0;
   out_8039217624001500825[31] = 0;
   out_8039217624001500825[32] = 0;
   out_8039217624001500825[33] = 0;
   out_8039217624001500825[34] = 0;
   out_8039217624001500825[35] = 0;
   out_8039217624001500825[36] = 0;
   out_8039217624001500825[37] = 0;
   out_8039217624001500825[38] = 0;
   out_8039217624001500825[39] = 0;
   out_8039217624001500825[40] = 0;
   out_8039217624001500825[41] = 0;
   out_8039217624001500825[42] = 0;
   out_8039217624001500825[43] = 0;
   out_8039217624001500825[44] = 1;
   out_8039217624001500825[45] = 0;
   out_8039217624001500825[46] = 0;
   out_8039217624001500825[47] = 1;
   out_8039217624001500825[48] = 0;
   out_8039217624001500825[49] = 0;
   out_8039217624001500825[50] = 0;
   out_8039217624001500825[51] = 0;
   out_8039217624001500825[52] = 0;
   out_8039217624001500825[53] = 0;
}
void h_10(double *state, double *unused, double *out_38039999239194626) {
   out_38039999239194626[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_38039999239194626[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_38039999239194626[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_6268676454903036611) {
   out_6268676454903036611[0] = 0;
   out_6268676454903036611[1] = 9.8100000000000005*cos(state[1]);
   out_6268676454903036611[2] = 0;
   out_6268676454903036611[3] = 0;
   out_6268676454903036611[4] = -state[8];
   out_6268676454903036611[5] = state[7];
   out_6268676454903036611[6] = 0;
   out_6268676454903036611[7] = state[5];
   out_6268676454903036611[8] = -state[4];
   out_6268676454903036611[9] = 0;
   out_6268676454903036611[10] = 0;
   out_6268676454903036611[11] = 0;
   out_6268676454903036611[12] = 1;
   out_6268676454903036611[13] = 0;
   out_6268676454903036611[14] = 0;
   out_6268676454903036611[15] = 1;
   out_6268676454903036611[16] = 0;
   out_6268676454903036611[17] = 0;
   out_6268676454903036611[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_6268676454903036611[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_6268676454903036611[20] = 0;
   out_6268676454903036611[21] = state[8];
   out_6268676454903036611[22] = 0;
   out_6268676454903036611[23] = -state[6];
   out_6268676454903036611[24] = -state[5];
   out_6268676454903036611[25] = 0;
   out_6268676454903036611[26] = state[3];
   out_6268676454903036611[27] = 0;
   out_6268676454903036611[28] = 0;
   out_6268676454903036611[29] = 0;
   out_6268676454903036611[30] = 0;
   out_6268676454903036611[31] = 1;
   out_6268676454903036611[32] = 0;
   out_6268676454903036611[33] = 0;
   out_6268676454903036611[34] = 1;
   out_6268676454903036611[35] = 0;
   out_6268676454903036611[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_6268676454903036611[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_6268676454903036611[38] = 0;
   out_6268676454903036611[39] = -state[7];
   out_6268676454903036611[40] = state[6];
   out_6268676454903036611[41] = 0;
   out_6268676454903036611[42] = state[4];
   out_6268676454903036611[43] = -state[3];
   out_6268676454903036611[44] = 0;
   out_6268676454903036611[45] = 0;
   out_6268676454903036611[46] = 0;
   out_6268676454903036611[47] = 0;
   out_6268676454903036611[48] = 0;
   out_6268676454903036611[49] = 0;
   out_6268676454903036611[50] = 1;
   out_6268676454903036611[51] = 0;
   out_6268676454903036611[52] = 0;
   out_6268676454903036611[53] = 1;
}
void h_13(double *state, double *unused, double *out_6341535430578010249) {
   out_6341535430578010249[0] = state[3];
   out_6341535430578010249[1] = state[4];
   out_6341535430578010249[2] = state[5];
}
void H_13(double *state, double *unused, double *out_7195252624375717990) {
   out_7195252624375717990[0] = 0;
   out_7195252624375717990[1] = 0;
   out_7195252624375717990[2] = 0;
   out_7195252624375717990[3] = 1;
   out_7195252624375717990[4] = 0;
   out_7195252624375717990[5] = 0;
   out_7195252624375717990[6] = 0;
   out_7195252624375717990[7] = 0;
   out_7195252624375717990[8] = 0;
   out_7195252624375717990[9] = 0;
   out_7195252624375717990[10] = 0;
   out_7195252624375717990[11] = 0;
   out_7195252624375717990[12] = 0;
   out_7195252624375717990[13] = 0;
   out_7195252624375717990[14] = 0;
   out_7195252624375717990[15] = 0;
   out_7195252624375717990[16] = 0;
   out_7195252624375717990[17] = 0;
   out_7195252624375717990[18] = 0;
   out_7195252624375717990[19] = 0;
   out_7195252624375717990[20] = 0;
   out_7195252624375717990[21] = 0;
   out_7195252624375717990[22] = 1;
   out_7195252624375717990[23] = 0;
   out_7195252624375717990[24] = 0;
   out_7195252624375717990[25] = 0;
   out_7195252624375717990[26] = 0;
   out_7195252624375717990[27] = 0;
   out_7195252624375717990[28] = 0;
   out_7195252624375717990[29] = 0;
   out_7195252624375717990[30] = 0;
   out_7195252624375717990[31] = 0;
   out_7195252624375717990[32] = 0;
   out_7195252624375717990[33] = 0;
   out_7195252624375717990[34] = 0;
   out_7195252624375717990[35] = 0;
   out_7195252624375717990[36] = 0;
   out_7195252624375717990[37] = 0;
   out_7195252624375717990[38] = 0;
   out_7195252624375717990[39] = 0;
   out_7195252624375717990[40] = 0;
   out_7195252624375717990[41] = 1;
   out_7195252624375717990[42] = 0;
   out_7195252624375717990[43] = 0;
   out_7195252624375717990[44] = 0;
   out_7195252624375717990[45] = 0;
   out_7195252624375717990[46] = 0;
   out_7195252624375717990[47] = 0;
   out_7195252624375717990[48] = 0;
   out_7195252624375717990[49] = 0;
   out_7195252624375717990[50] = 0;
   out_7195252624375717990[51] = 0;
   out_7195252624375717990[52] = 0;
   out_7195252624375717990[53] = 0;
}
void h_14(double *state, double *unused, double *out_3628435478100133571) {
   out_3628435478100133571[0] = state[6];
   out_3628435478100133571[1] = state[7];
   out_3628435478100133571[2] = state[8];
}
void H_14(double *state, double *unused, double *out_6444285593368566262) {
   out_6444285593368566262[0] = 0;
   out_6444285593368566262[1] = 0;
   out_6444285593368566262[2] = 0;
   out_6444285593368566262[3] = 0;
   out_6444285593368566262[4] = 0;
   out_6444285593368566262[5] = 0;
   out_6444285593368566262[6] = 1;
   out_6444285593368566262[7] = 0;
   out_6444285593368566262[8] = 0;
   out_6444285593368566262[9] = 0;
   out_6444285593368566262[10] = 0;
   out_6444285593368566262[11] = 0;
   out_6444285593368566262[12] = 0;
   out_6444285593368566262[13] = 0;
   out_6444285593368566262[14] = 0;
   out_6444285593368566262[15] = 0;
   out_6444285593368566262[16] = 0;
   out_6444285593368566262[17] = 0;
   out_6444285593368566262[18] = 0;
   out_6444285593368566262[19] = 0;
   out_6444285593368566262[20] = 0;
   out_6444285593368566262[21] = 0;
   out_6444285593368566262[22] = 0;
   out_6444285593368566262[23] = 0;
   out_6444285593368566262[24] = 0;
   out_6444285593368566262[25] = 1;
   out_6444285593368566262[26] = 0;
   out_6444285593368566262[27] = 0;
   out_6444285593368566262[28] = 0;
   out_6444285593368566262[29] = 0;
   out_6444285593368566262[30] = 0;
   out_6444285593368566262[31] = 0;
   out_6444285593368566262[32] = 0;
   out_6444285593368566262[33] = 0;
   out_6444285593368566262[34] = 0;
   out_6444285593368566262[35] = 0;
   out_6444285593368566262[36] = 0;
   out_6444285593368566262[37] = 0;
   out_6444285593368566262[38] = 0;
   out_6444285593368566262[39] = 0;
   out_6444285593368566262[40] = 0;
   out_6444285593368566262[41] = 0;
   out_6444285593368566262[42] = 0;
   out_6444285593368566262[43] = 0;
   out_6444285593368566262[44] = 1;
   out_6444285593368566262[45] = 0;
   out_6444285593368566262[46] = 0;
   out_6444285593368566262[47] = 0;
   out_6444285593368566262[48] = 0;
   out_6444285593368566262[49] = 0;
   out_6444285593368566262[50] = 0;
   out_6444285593368566262[51] = 0;
   out_6444285593368566262[52] = 0;
   out_6444285593368566262[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_4281085523000754016) {
  err_fun(nom_x, delta_x, out_4281085523000754016);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_7803285783767947319) {
  inv_err_fun(nom_x, true_x, out_7803285783767947319);
}
void pose_H_mod_fun(double *state, double *out_4559444642956092070) {
  H_mod_fun(state, out_4559444642956092070);
}
void pose_f_fun(double *state, double dt, double *out_819200123031164352) {
  f_fun(state,  dt, out_819200123031164352);
}
void pose_F_fun(double *state, double dt, double *out_1797944479821926981) {
  F_fun(state,  dt, out_1797944479821926981);
}
void pose_h_4(double *state, double *unused, double *out_1556670692920147002) {
  h_4(state, unused, out_1556670692920147002);
}
void pose_H_4(double *state, double *unused, double *out_8039217624001500825) {
  H_4(state, unused, out_8039217624001500825);
}
void pose_h_10(double *state, double *unused, double *out_38039999239194626) {
  h_10(state, unused, out_38039999239194626);
}
void pose_H_10(double *state, double *unused, double *out_6268676454903036611) {
  H_10(state, unused, out_6268676454903036611);
}
void pose_h_13(double *state, double *unused, double *out_6341535430578010249) {
  h_13(state, unused, out_6341535430578010249);
}
void pose_H_13(double *state, double *unused, double *out_7195252624375717990) {
  H_13(state, unused, out_7195252624375717990);
}
void pose_h_14(double *state, double *unused, double *out_3628435478100133571) {
  h_14(state, unused, out_3628435478100133571);
}
void pose_H_14(double *state, double *unused, double *out_6444285593368566262) {
  H_14(state, unused, out_6444285593368566262);
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
