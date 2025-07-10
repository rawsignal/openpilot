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
void err_fun(double *nom_x, double *delta_x, double *out_5394071911806177632) {
   out_5394071911806177632[0] = delta_x[0] + nom_x[0];
   out_5394071911806177632[1] = delta_x[1] + nom_x[1];
   out_5394071911806177632[2] = delta_x[2] + nom_x[2];
   out_5394071911806177632[3] = delta_x[3] + nom_x[3];
   out_5394071911806177632[4] = delta_x[4] + nom_x[4];
   out_5394071911806177632[5] = delta_x[5] + nom_x[5];
   out_5394071911806177632[6] = delta_x[6] + nom_x[6];
   out_5394071911806177632[7] = delta_x[7] + nom_x[7];
   out_5394071911806177632[8] = delta_x[8] + nom_x[8];
   out_5394071911806177632[9] = delta_x[9] + nom_x[9];
   out_5394071911806177632[10] = delta_x[10] + nom_x[10];
   out_5394071911806177632[11] = delta_x[11] + nom_x[11];
   out_5394071911806177632[12] = delta_x[12] + nom_x[12];
   out_5394071911806177632[13] = delta_x[13] + nom_x[13];
   out_5394071911806177632[14] = delta_x[14] + nom_x[14];
   out_5394071911806177632[15] = delta_x[15] + nom_x[15];
   out_5394071911806177632[16] = delta_x[16] + nom_x[16];
   out_5394071911806177632[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_390924532607337478) {
   out_390924532607337478[0] = -nom_x[0] + true_x[0];
   out_390924532607337478[1] = -nom_x[1] + true_x[1];
   out_390924532607337478[2] = -nom_x[2] + true_x[2];
   out_390924532607337478[3] = -nom_x[3] + true_x[3];
   out_390924532607337478[4] = -nom_x[4] + true_x[4];
   out_390924532607337478[5] = -nom_x[5] + true_x[5];
   out_390924532607337478[6] = -nom_x[6] + true_x[6];
   out_390924532607337478[7] = -nom_x[7] + true_x[7];
   out_390924532607337478[8] = -nom_x[8] + true_x[8];
   out_390924532607337478[9] = -nom_x[9] + true_x[9];
   out_390924532607337478[10] = -nom_x[10] + true_x[10];
   out_390924532607337478[11] = -nom_x[11] + true_x[11];
   out_390924532607337478[12] = -nom_x[12] + true_x[12];
   out_390924532607337478[13] = -nom_x[13] + true_x[13];
   out_390924532607337478[14] = -nom_x[14] + true_x[14];
   out_390924532607337478[15] = -nom_x[15] + true_x[15];
   out_390924532607337478[16] = -nom_x[16] + true_x[16];
   out_390924532607337478[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_9013755718701150594) {
   out_9013755718701150594[0] = 1.0;
   out_9013755718701150594[1] = 0.0;
   out_9013755718701150594[2] = 0.0;
   out_9013755718701150594[3] = 0.0;
   out_9013755718701150594[4] = 0.0;
   out_9013755718701150594[5] = 0.0;
   out_9013755718701150594[6] = 0.0;
   out_9013755718701150594[7] = 0.0;
   out_9013755718701150594[8] = 0.0;
   out_9013755718701150594[9] = 0.0;
   out_9013755718701150594[10] = 0.0;
   out_9013755718701150594[11] = 0.0;
   out_9013755718701150594[12] = 0.0;
   out_9013755718701150594[13] = 0.0;
   out_9013755718701150594[14] = 0.0;
   out_9013755718701150594[15] = 0.0;
   out_9013755718701150594[16] = 0.0;
   out_9013755718701150594[17] = 0.0;
   out_9013755718701150594[18] = 0.0;
   out_9013755718701150594[19] = 1.0;
   out_9013755718701150594[20] = 0.0;
   out_9013755718701150594[21] = 0.0;
   out_9013755718701150594[22] = 0.0;
   out_9013755718701150594[23] = 0.0;
   out_9013755718701150594[24] = 0.0;
   out_9013755718701150594[25] = 0.0;
   out_9013755718701150594[26] = 0.0;
   out_9013755718701150594[27] = 0.0;
   out_9013755718701150594[28] = 0.0;
   out_9013755718701150594[29] = 0.0;
   out_9013755718701150594[30] = 0.0;
   out_9013755718701150594[31] = 0.0;
   out_9013755718701150594[32] = 0.0;
   out_9013755718701150594[33] = 0.0;
   out_9013755718701150594[34] = 0.0;
   out_9013755718701150594[35] = 0.0;
   out_9013755718701150594[36] = 0.0;
   out_9013755718701150594[37] = 0.0;
   out_9013755718701150594[38] = 1.0;
   out_9013755718701150594[39] = 0.0;
   out_9013755718701150594[40] = 0.0;
   out_9013755718701150594[41] = 0.0;
   out_9013755718701150594[42] = 0.0;
   out_9013755718701150594[43] = 0.0;
   out_9013755718701150594[44] = 0.0;
   out_9013755718701150594[45] = 0.0;
   out_9013755718701150594[46] = 0.0;
   out_9013755718701150594[47] = 0.0;
   out_9013755718701150594[48] = 0.0;
   out_9013755718701150594[49] = 0.0;
   out_9013755718701150594[50] = 0.0;
   out_9013755718701150594[51] = 0.0;
   out_9013755718701150594[52] = 0.0;
   out_9013755718701150594[53] = 0.0;
   out_9013755718701150594[54] = 0.0;
   out_9013755718701150594[55] = 0.0;
   out_9013755718701150594[56] = 0.0;
   out_9013755718701150594[57] = 1.0;
   out_9013755718701150594[58] = 0.0;
   out_9013755718701150594[59] = 0.0;
   out_9013755718701150594[60] = 0.0;
   out_9013755718701150594[61] = 0.0;
   out_9013755718701150594[62] = 0.0;
   out_9013755718701150594[63] = 0.0;
   out_9013755718701150594[64] = 0.0;
   out_9013755718701150594[65] = 0.0;
   out_9013755718701150594[66] = 0.0;
   out_9013755718701150594[67] = 0.0;
   out_9013755718701150594[68] = 0.0;
   out_9013755718701150594[69] = 0.0;
   out_9013755718701150594[70] = 0.0;
   out_9013755718701150594[71] = 0.0;
   out_9013755718701150594[72] = 0.0;
   out_9013755718701150594[73] = 0.0;
   out_9013755718701150594[74] = 0.0;
   out_9013755718701150594[75] = 0.0;
   out_9013755718701150594[76] = 1.0;
   out_9013755718701150594[77] = 0.0;
   out_9013755718701150594[78] = 0.0;
   out_9013755718701150594[79] = 0.0;
   out_9013755718701150594[80] = 0.0;
   out_9013755718701150594[81] = 0.0;
   out_9013755718701150594[82] = 0.0;
   out_9013755718701150594[83] = 0.0;
   out_9013755718701150594[84] = 0.0;
   out_9013755718701150594[85] = 0.0;
   out_9013755718701150594[86] = 0.0;
   out_9013755718701150594[87] = 0.0;
   out_9013755718701150594[88] = 0.0;
   out_9013755718701150594[89] = 0.0;
   out_9013755718701150594[90] = 0.0;
   out_9013755718701150594[91] = 0.0;
   out_9013755718701150594[92] = 0.0;
   out_9013755718701150594[93] = 0.0;
   out_9013755718701150594[94] = 0.0;
   out_9013755718701150594[95] = 1.0;
   out_9013755718701150594[96] = 0.0;
   out_9013755718701150594[97] = 0.0;
   out_9013755718701150594[98] = 0.0;
   out_9013755718701150594[99] = 0.0;
   out_9013755718701150594[100] = 0.0;
   out_9013755718701150594[101] = 0.0;
   out_9013755718701150594[102] = 0.0;
   out_9013755718701150594[103] = 0.0;
   out_9013755718701150594[104] = 0.0;
   out_9013755718701150594[105] = 0.0;
   out_9013755718701150594[106] = 0.0;
   out_9013755718701150594[107] = 0.0;
   out_9013755718701150594[108] = 0.0;
   out_9013755718701150594[109] = 0.0;
   out_9013755718701150594[110] = 0.0;
   out_9013755718701150594[111] = 0.0;
   out_9013755718701150594[112] = 0.0;
   out_9013755718701150594[113] = 0.0;
   out_9013755718701150594[114] = 1.0;
   out_9013755718701150594[115] = 0.0;
   out_9013755718701150594[116] = 0.0;
   out_9013755718701150594[117] = 0.0;
   out_9013755718701150594[118] = 0.0;
   out_9013755718701150594[119] = 0.0;
   out_9013755718701150594[120] = 0.0;
   out_9013755718701150594[121] = 0.0;
   out_9013755718701150594[122] = 0.0;
   out_9013755718701150594[123] = 0.0;
   out_9013755718701150594[124] = 0.0;
   out_9013755718701150594[125] = 0.0;
   out_9013755718701150594[126] = 0.0;
   out_9013755718701150594[127] = 0.0;
   out_9013755718701150594[128] = 0.0;
   out_9013755718701150594[129] = 0.0;
   out_9013755718701150594[130] = 0.0;
   out_9013755718701150594[131] = 0.0;
   out_9013755718701150594[132] = 0.0;
   out_9013755718701150594[133] = 1.0;
   out_9013755718701150594[134] = 0.0;
   out_9013755718701150594[135] = 0.0;
   out_9013755718701150594[136] = 0.0;
   out_9013755718701150594[137] = 0.0;
   out_9013755718701150594[138] = 0.0;
   out_9013755718701150594[139] = 0.0;
   out_9013755718701150594[140] = 0.0;
   out_9013755718701150594[141] = 0.0;
   out_9013755718701150594[142] = 0.0;
   out_9013755718701150594[143] = 0.0;
   out_9013755718701150594[144] = 0.0;
   out_9013755718701150594[145] = 0.0;
   out_9013755718701150594[146] = 0.0;
   out_9013755718701150594[147] = 0.0;
   out_9013755718701150594[148] = 0.0;
   out_9013755718701150594[149] = 0.0;
   out_9013755718701150594[150] = 0.0;
   out_9013755718701150594[151] = 0.0;
   out_9013755718701150594[152] = 1.0;
   out_9013755718701150594[153] = 0.0;
   out_9013755718701150594[154] = 0.0;
   out_9013755718701150594[155] = 0.0;
   out_9013755718701150594[156] = 0.0;
   out_9013755718701150594[157] = 0.0;
   out_9013755718701150594[158] = 0.0;
   out_9013755718701150594[159] = 0.0;
   out_9013755718701150594[160] = 0.0;
   out_9013755718701150594[161] = 0.0;
   out_9013755718701150594[162] = 0.0;
   out_9013755718701150594[163] = 0.0;
   out_9013755718701150594[164] = 0.0;
   out_9013755718701150594[165] = 0.0;
   out_9013755718701150594[166] = 0.0;
   out_9013755718701150594[167] = 0.0;
   out_9013755718701150594[168] = 0.0;
   out_9013755718701150594[169] = 0.0;
   out_9013755718701150594[170] = 0.0;
   out_9013755718701150594[171] = 1.0;
   out_9013755718701150594[172] = 0.0;
   out_9013755718701150594[173] = 0.0;
   out_9013755718701150594[174] = 0.0;
   out_9013755718701150594[175] = 0.0;
   out_9013755718701150594[176] = 0.0;
   out_9013755718701150594[177] = 0.0;
   out_9013755718701150594[178] = 0.0;
   out_9013755718701150594[179] = 0.0;
   out_9013755718701150594[180] = 0.0;
   out_9013755718701150594[181] = 0.0;
   out_9013755718701150594[182] = 0.0;
   out_9013755718701150594[183] = 0.0;
   out_9013755718701150594[184] = 0.0;
   out_9013755718701150594[185] = 0.0;
   out_9013755718701150594[186] = 0.0;
   out_9013755718701150594[187] = 0.0;
   out_9013755718701150594[188] = 0.0;
   out_9013755718701150594[189] = 0.0;
   out_9013755718701150594[190] = 1.0;
   out_9013755718701150594[191] = 0.0;
   out_9013755718701150594[192] = 0.0;
   out_9013755718701150594[193] = 0.0;
   out_9013755718701150594[194] = 0.0;
   out_9013755718701150594[195] = 0.0;
   out_9013755718701150594[196] = 0.0;
   out_9013755718701150594[197] = 0.0;
   out_9013755718701150594[198] = 0.0;
   out_9013755718701150594[199] = 0.0;
   out_9013755718701150594[200] = 0.0;
   out_9013755718701150594[201] = 0.0;
   out_9013755718701150594[202] = 0.0;
   out_9013755718701150594[203] = 0.0;
   out_9013755718701150594[204] = 0.0;
   out_9013755718701150594[205] = 0.0;
   out_9013755718701150594[206] = 0.0;
   out_9013755718701150594[207] = 0.0;
   out_9013755718701150594[208] = 0.0;
   out_9013755718701150594[209] = 1.0;
   out_9013755718701150594[210] = 0.0;
   out_9013755718701150594[211] = 0.0;
   out_9013755718701150594[212] = 0.0;
   out_9013755718701150594[213] = 0.0;
   out_9013755718701150594[214] = 0.0;
   out_9013755718701150594[215] = 0.0;
   out_9013755718701150594[216] = 0.0;
   out_9013755718701150594[217] = 0.0;
   out_9013755718701150594[218] = 0.0;
   out_9013755718701150594[219] = 0.0;
   out_9013755718701150594[220] = 0.0;
   out_9013755718701150594[221] = 0.0;
   out_9013755718701150594[222] = 0.0;
   out_9013755718701150594[223] = 0.0;
   out_9013755718701150594[224] = 0.0;
   out_9013755718701150594[225] = 0.0;
   out_9013755718701150594[226] = 0.0;
   out_9013755718701150594[227] = 0.0;
   out_9013755718701150594[228] = 1.0;
   out_9013755718701150594[229] = 0.0;
   out_9013755718701150594[230] = 0.0;
   out_9013755718701150594[231] = 0.0;
   out_9013755718701150594[232] = 0.0;
   out_9013755718701150594[233] = 0.0;
   out_9013755718701150594[234] = 0.0;
   out_9013755718701150594[235] = 0.0;
   out_9013755718701150594[236] = 0.0;
   out_9013755718701150594[237] = 0.0;
   out_9013755718701150594[238] = 0.0;
   out_9013755718701150594[239] = 0.0;
   out_9013755718701150594[240] = 0.0;
   out_9013755718701150594[241] = 0.0;
   out_9013755718701150594[242] = 0.0;
   out_9013755718701150594[243] = 0.0;
   out_9013755718701150594[244] = 0.0;
   out_9013755718701150594[245] = 0.0;
   out_9013755718701150594[246] = 0.0;
   out_9013755718701150594[247] = 1.0;
   out_9013755718701150594[248] = 0.0;
   out_9013755718701150594[249] = 0.0;
   out_9013755718701150594[250] = 0.0;
   out_9013755718701150594[251] = 0.0;
   out_9013755718701150594[252] = 0.0;
   out_9013755718701150594[253] = 0.0;
   out_9013755718701150594[254] = 0.0;
   out_9013755718701150594[255] = 0.0;
   out_9013755718701150594[256] = 0.0;
   out_9013755718701150594[257] = 0.0;
   out_9013755718701150594[258] = 0.0;
   out_9013755718701150594[259] = 0.0;
   out_9013755718701150594[260] = 0.0;
   out_9013755718701150594[261] = 0.0;
   out_9013755718701150594[262] = 0.0;
   out_9013755718701150594[263] = 0.0;
   out_9013755718701150594[264] = 0.0;
   out_9013755718701150594[265] = 0.0;
   out_9013755718701150594[266] = 1.0;
   out_9013755718701150594[267] = 0.0;
   out_9013755718701150594[268] = 0.0;
   out_9013755718701150594[269] = 0.0;
   out_9013755718701150594[270] = 0.0;
   out_9013755718701150594[271] = 0.0;
   out_9013755718701150594[272] = 0.0;
   out_9013755718701150594[273] = 0.0;
   out_9013755718701150594[274] = 0.0;
   out_9013755718701150594[275] = 0.0;
   out_9013755718701150594[276] = 0.0;
   out_9013755718701150594[277] = 0.0;
   out_9013755718701150594[278] = 0.0;
   out_9013755718701150594[279] = 0.0;
   out_9013755718701150594[280] = 0.0;
   out_9013755718701150594[281] = 0.0;
   out_9013755718701150594[282] = 0.0;
   out_9013755718701150594[283] = 0.0;
   out_9013755718701150594[284] = 0.0;
   out_9013755718701150594[285] = 1.0;
   out_9013755718701150594[286] = 0.0;
   out_9013755718701150594[287] = 0.0;
   out_9013755718701150594[288] = 0.0;
   out_9013755718701150594[289] = 0.0;
   out_9013755718701150594[290] = 0.0;
   out_9013755718701150594[291] = 0.0;
   out_9013755718701150594[292] = 0.0;
   out_9013755718701150594[293] = 0.0;
   out_9013755718701150594[294] = 0.0;
   out_9013755718701150594[295] = 0.0;
   out_9013755718701150594[296] = 0.0;
   out_9013755718701150594[297] = 0.0;
   out_9013755718701150594[298] = 0.0;
   out_9013755718701150594[299] = 0.0;
   out_9013755718701150594[300] = 0.0;
   out_9013755718701150594[301] = 0.0;
   out_9013755718701150594[302] = 0.0;
   out_9013755718701150594[303] = 0.0;
   out_9013755718701150594[304] = 1.0;
   out_9013755718701150594[305] = 0.0;
   out_9013755718701150594[306] = 0.0;
   out_9013755718701150594[307] = 0.0;
   out_9013755718701150594[308] = 0.0;
   out_9013755718701150594[309] = 0.0;
   out_9013755718701150594[310] = 0.0;
   out_9013755718701150594[311] = 0.0;
   out_9013755718701150594[312] = 0.0;
   out_9013755718701150594[313] = 0.0;
   out_9013755718701150594[314] = 0.0;
   out_9013755718701150594[315] = 0.0;
   out_9013755718701150594[316] = 0.0;
   out_9013755718701150594[317] = 0.0;
   out_9013755718701150594[318] = 0.0;
   out_9013755718701150594[319] = 0.0;
   out_9013755718701150594[320] = 0.0;
   out_9013755718701150594[321] = 0.0;
   out_9013755718701150594[322] = 0.0;
   out_9013755718701150594[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_559878194513068250) {
   out_559878194513068250[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_559878194513068250[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_559878194513068250[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_559878194513068250[3] = dt*state[12] + state[3];
   out_559878194513068250[4] = dt*state[13] + state[4];
   out_559878194513068250[5] = dt*state[14] + state[5];
   out_559878194513068250[6] = state[6];
   out_559878194513068250[7] = state[7];
   out_559878194513068250[8] = state[8];
   out_559878194513068250[9] = state[9];
   out_559878194513068250[10] = state[10];
   out_559878194513068250[11] = state[11];
   out_559878194513068250[12] = state[12];
   out_559878194513068250[13] = state[13];
   out_559878194513068250[14] = state[14];
   out_559878194513068250[15] = state[15];
   out_559878194513068250[16] = state[16];
   out_559878194513068250[17] = state[17];
}
void F_fun(double *state, double dt, double *out_3177690484241037676) {
   out_3177690484241037676[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3177690484241037676[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3177690484241037676[2] = 0;
   out_3177690484241037676[3] = 0;
   out_3177690484241037676[4] = 0;
   out_3177690484241037676[5] = 0;
   out_3177690484241037676[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3177690484241037676[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3177690484241037676[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3177690484241037676[9] = 0;
   out_3177690484241037676[10] = 0;
   out_3177690484241037676[11] = 0;
   out_3177690484241037676[12] = 0;
   out_3177690484241037676[13] = 0;
   out_3177690484241037676[14] = 0;
   out_3177690484241037676[15] = 0;
   out_3177690484241037676[16] = 0;
   out_3177690484241037676[17] = 0;
   out_3177690484241037676[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3177690484241037676[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3177690484241037676[20] = 0;
   out_3177690484241037676[21] = 0;
   out_3177690484241037676[22] = 0;
   out_3177690484241037676[23] = 0;
   out_3177690484241037676[24] = 0;
   out_3177690484241037676[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3177690484241037676[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3177690484241037676[27] = 0;
   out_3177690484241037676[28] = 0;
   out_3177690484241037676[29] = 0;
   out_3177690484241037676[30] = 0;
   out_3177690484241037676[31] = 0;
   out_3177690484241037676[32] = 0;
   out_3177690484241037676[33] = 0;
   out_3177690484241037676[34] = 0;
   out_3177690484241037676[35] = 0;
   out_3177690484241037676[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3177690484241037676[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3177690484241037676[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3177690484241037676[39] = 0;
   out_3177690484241037676[40] = 0;
   out_3177690484241037676[41] = 0;
   out_3177690484241037676[42] = 0;
   out_3177690484241037676[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3177690484241037676[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3177690484241037676[45] = 0;
   out_3177690484241037676[46] = 0;
   out_3177690484241037676[47] = 0;
   out_3177690484241037676[48] = 0;
   out_3177690484241037676[49] = 0;
   out_3177690484241037676[50] = 0;
   out_3177690484241037676[51] = 0;
   out_3177690484241037676[52] = 0;
   out_3177690484241037676[53] = 0;
   out_3177690484241037676[54] = 0;
   out_3177690484241037676[55] = 0;
   out_3177690484241037676[56] = 0;
   out_3177690484241037676[57] = 1;
   out_3177690484241037676[58] = 0;
   out_3177690484241037676[59] = 0;
   out_3177690484241037676[60] = 0;
   out_3177690484241037676[61] = 0;
   out_3177690484241037676[62] = 0;
   out_3177690484241037676[63] = 0;
   out_3177690484241037676[64] = 0;
   out_3177690484241037676[65] = 0;
   out_3177690484241037676[66] = dt;
   out_3177690484241037676[67] = 0;
   out_3177690484241037676[68] = 0;
   out_3177690484241037676[69] = 0;
   out_3177690484241037676[70] = 0;
   out_3177690484241037676[71] = 0;
   out_3177690484241037676[72] = 0;
   out_3177690484241037676[73] = 0;
   out_3177690484241037676[74] = 0;
   out_3177690484241037676[75] = 0;
   out_3177690484241037676[76] = 1;
   out_3177690484241037676[77] = 0;
   out_3177690484241037676[78] = 0;
   out_3177690484241037676[79] = 0;
   out_3177690484241037676[80] = 0;
   out_3177690484241037676[81] = 0;
   out_3177690484241037676[82] = 0;
   out_3177690484241037676[83] = 0;
   out_3177690484241037676[84] = 0;
   out_3177690484241037676[85] = dt;
   out_3177690484241037676[86] = 0;
   out_3177690484241037676[87] = 0;
   out_3177690484241037676[88] = 0;
   out_3177690484241037676[89] = 0;
   out_3177690484241037676[90] = 0;
   out_3177690484241037676[91] = 0;
   out_3177690484241037676[92] = 0;
   out_3177690484241037676[93] = 0;
   out_3177690484241037676[94] = 0;
   out_3177690484241037676[95] = 1;
   out_3177690484241037676[96] = 0;
   out_3177690484241037676[97] = 0;
   out_3177690484241037676[98] = 0;
   out_3177690484241037676[99] = 0;
   out_3177690484241037676[100] = 0;
   out_3177690484241037676[101] = 0;
   out_3177690484241037676[102] = 0;
   out_3177690484241037676[103] = 0;
   out_3177690484241037676[104] = dt;
   out_3177690484241037676[105] = 0;
   out_3177690484241037676[106] = 0;
   out_3177690484241037676[107] = 0;
   out_3177690484241037676[108] = 0;
   out_3177690484241037676[109] = 0;
   out_3177690484241037676[110] = 0;
   out_3177690484241037676[111] = 0;
   out_3177690484241037676[112] = 0;
   out_3177690484241037676[113] = 0;
   out_3177690484241037676[114] = 1;
   out_3177690484241037676[115] = 0;
   out_3177690484241037676[116] = 0;
   out_3177690484241037676[117] = 0;
   out_3177690484241037676[118] = 0;
   out_3177690484241037676[119] = 0;
   out_3177690484241037676[120] = 0;
   out_3177690484241037676[121] = 0;
   out_3177690484241037676[122] = 0;
   out_3177690484241037676[123] = 0;
   out_3177690484241037676[124] = 0;
   out_3177690484241037676[125] = 0;
   out_3177690484241037676[126] = 0;
   out_3177690484241037676[127] = 0;
   out_3177690484241037676[128] = 0;
   out_3177690484241037676[129] = 0;
   out_3177690484241037676[130] = 0;
   out_3177690484241037676[131] = 0;
   out_3177690484241037676[132] = 0;
   out_3177690484241037676[133] = 1;
   out_3177690484241037676[134] = 0;
   out_3177690484241037676[135] = 0;
   out_3177690484241037676[136] = 0;
   out_3177690484241037676[137] = 0;
   out_3177690484241037676[138] = 0;
   out_3177690484241037676[139] = 0;
   out_3177690484241037676[140] = 0;
   out_3177690484241037676[141] = 0;
   out_3177690484241037676[142] = 0;
   out_3177690484241037676[143] = 0;
   out_3177690484241037676[144] = 0;
   out_3177690484241037676[145] = 0;
   out_3177690484241037676[146] = 0;
   out_3177690484241037676[147] = 0;
   out_3177690484241037676[148] = 0;
   out_3177690484241037676[149] = 0;
   out_3177690484241037676[150] = 0;
   out_3177690484241037676[151] = 0;
   out_3177690484241037676[152] = 1;
   out_3177690484241037676[153] = 0;
   out_3177690484241037676[154] = 0;
   out_3177690484241037676[155] = 0;
   out_3177690484241037676[156] = 0;
   out_3177690484241037676[157] = 0;
   out_3177690484241037676[158] = 0;
   out_3177690484241037676[159] = 0;
   out_3177690484241037676[160] = 0;
   out_3177690484241037676[161] = 0;
   out_3177690484241037676[162] = 0;
   out_3177690484241037676[163] = 0;
   out_3177690484241037676[164] = 0;
   out_3177690484241037676[165] = 0;
   out_3177690484241037676[166] = 0;
   out_3177690484241037676[167] = 0;
   out_3177690484241037676[168] = 0;
   out_3177690484241037676[169] = 0;
   out_3177690484241037676[170] = 0;
   out_3177690484241037676[171] = 1;
   out_3177690484241037676[172] = 0;
   out_3177690484241037676[173] = 0;
   out_3177690484241037676[174] = 0;
   out_3177690484241037676[175] = 0;
   out_3177690484241037676[176] = 0;
   out_3177690484241037676[177] = 0;
   out_3177690484241037676[178] = 0;
   out_3177690484241037676[179] = 0;
   out_3177690484241037676[180] = 0;
   out_3177690484241037676[181] = 0;
   out_3177690484241037676[182] = 0;
   out_3177690484241037676[183] = 0;
   out_3177690484241037676[184] = 0;
   out_3177690484241037676[185] = 0;
   out_3177690484241037676[186] = 0;
   out_3177690484241037676[187] = 0;
   out_3177690484241037676[188] = 0;
   out_3177690484241037676[189] = 0;
   out_3177690484241037676[190] = 1;
   out_3177690484241037676[191] = 0;
   out_3177690484241037676[192] = 0;
   out_3177690484241037676[193] = 0;
   out_3177690484241037676[194] = 0;
   out_3177690484241037676[195] = 0;
   out_3177690484241037676[196] = 0;
   out_3177690484241037676[197] = 0;
   out_3177690484241037676[198] = 0;
   out_3177690484241037676[199] = 0;
   out_3177690484241037676[200] = 0;
   out_3177690484241037676[201] = 0;
   out_3177690484241037676[202] = 0;
   out_3177690484241037676[203] = 0;
   out_3177690484241037676[204] = 0;
   out_3177690484241037676[205] = 0;
   out_3177690484241037676[206] = 0;
   out_3177690484241037676[207] = 0;
   out_3177690484241037676[208] = 0;
   out_3177690484241037676[209] = 1;
   out_3177690484241037676[210] = 0;
   out_3177690484241037676[211] = 0;
   out_3177690484241037676[212] = 0;
   out_3177690484241037676[213] = 0;
   out_3177690484241037676[214] = 0;
   out_3177690484241037676[215] = 0;
   out_3177690484241037676[216] = 0;
   out_3177690484241037676[217] = 0;
   out_3177690484241037676[218] = 0;
   out_3177690484241037676[219] = 0;
   out_3177690484241037676[220] = 0;
   out_3177690484241037676[221] = 0;
   out_3177690484241037676[222] = 0;
   out_3177690484241037676[223] = 0;
   out_3177690484241037676[224] = 0;
   out_3177690484241037676[225] = 0;
   out_3177690484241037676[226] = 0;
   out_3177690484241037676[227] = 0;
   out_3177690484241037676[228] = 1;
   out_3177690484241037676[229] = 0;
   out_3177690484241037676[230] = 0;
   out_3177690484241037676[231] = 0;
   out_3177690484241037676[232] = 0;
   out_3177690484241037676[233] = 0;
   out_3177690484241037676[234] = 0;
   out_3177690484241037676[235] = 0;
   out_3177690484241037676[236] = 0;
   out_3177690484241037676[237] = 0;
   out_3177690484241037676[238] = 0;
   out_3177690484241037676[239] = 0;
   out_3177690484241037676[240] = 0;
   out_3177690484241037676[241] = 0;
   out_3177690484241037676[242] = 0;
   out_3177690484241037676[243] = 0;
   out_3177690484241037676[244] = 0;
   out_3177690484241037676[245] = 0;
   out_3177690484241037676[246] = 0;
   out_3177690484241037676[247] = 1;
   out_3177690484241037676[248] = 0;
   out_3177690484241037676[249] = 0;
   out_3177690484241037676[250] = 0;
   out_3177690484241037676[251] = 0;
   out_3177690484241037676[252] = 0;
   out_3177690484241037676[253] = 0;
   out_3177690484241037676[254] = 0;
   out_3177690484241037676[255] = 0;
   out_3177690484241037676[256] = 0;
   out_3177690484241037676[257] = 0;
   out_3177690484241037676[258] = 0;
   out_3177690484241037676[259] = 0;
   out_3177690484241037676[260] = 0;
   out_3177690484241037676[261] = 0;
   out_3177690484241037676[262] = 0;
   out_3177690484241037676[263] = 0;
   out_3177690484241037676[264] = 0;
   out_3177690484241037676[265] = 0;
   out_3177690484241037676[266] = 1;
   out_3177690484241037676[267] = 0;
   out_3177690484241037676[268] = 0;
   out_3177690484241037676[269] = 0;
   out_3177690484241037676[270] = 0;
   out_3177690484241037676[271] = 0;
   out_3177690484241037676[272] = 0;
   out_3177690484241037676[273] = 0;
   out_3177690484241037676[274] = 0;
   out_3177690484241037676[275] = 0;
   out_3177690484241037676[276] = 0;
   out_3177690484241037676[277] = 0;
   out_3177690484241037676[278] = 0;
   out_3177690484241037676[279] = 0;
   out_3177690484241037676[280] = 0;
   out_3177690484241037676[281] = 0;
   out_3177690484241037676[282] = 0;
   out_3177690484241037676[283] = 0;
   out_3177690484241037676[284] = 0;
   out_3177690484241037676[285] = 1;
   out_3177690484241037676[286] = 0;
   out_3177690484241037676[287] = 0;
   out_3177690484241037676[288] = 0;
   out_3177690484241037676[289] = 0;
   out_3177690484241037676[290] = 0;
   out_3177690484241037676[291] = 0;
   out_3177690484241037676[292] = 0;
   out_3177690484241037676[293] = 0;
   out_3177690484241037676[294] = 0;
   out_3177690484241037676[295] = 0;
   out_3177690484241037676[296] = 0;
   out_3177690484241037676[297] = 0;
   out_3177690484241037676[298] = 0;
   out_3177690484241037676[299] = 0;
   out_3177690484241037676[300] = 0;
   out_3177690484241037676[301] = 0;
   out_3177690484241037676[302] = 0;
   out_3177690484241037676[303] = 0;
   out_3177690484241037676[304] = 1;
   out_3177690484241037676[305] = 0;
   out_3177690484241037676[306] = 0;
   out_3177690484241037676[307] = 0;
   out_3177690484241037676[308] = 0;
   out_3177690484241037676[309] = 0;
   out_3177690484241037676[310] = 0;
   out_3177690484241037676[311] = 0;
   out_3177690484241037676[312] = 0;
   out_3177690484241037676[313] = 0;
   out_3177690484241037676[314] = 0;
   out_3177690484241037676[315] = 0;
   out_3177690484241037676[316] = 0;
   out_3177690484241037676[317] = 0;
   out_3177690484241037676[318] = 0;
   out_3177690484241037676[319] = 0;
   out_3177690484241037676[320] = 0;
   out_3177690484241037676[321] = 0;
   out_3177690484241037676[322] = 0;
   out_3177690484241037676[323] = 1;
}
void h_4(double *state, double *unused, double *out_3640963725510673355) {
   out_3640963725510673355[0] = state[6] + state[9];
   out_3640963725510673355[1] = state[7] + state[10];
   out_3640963725510673355[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_3584906548256442301) {
   out_3584906548256442301[0] = 0;
   out_3584906548256442301[1] = 0;
   out_3584906548256442301[2] = 0;
   out_3584906548256442301[3] = 0;
   out_3584906548256442301[4] = 0;
   out_3584906548256442301[5] = 0;
   out_3584906548256442301[6] = 1;
   out_3584906548256442301[7] = 0;
   out_3584906548256442301[8] = 0;
   out_3584906548256442301[9] = 1;
   out_3584906548256442301[10] = 0;
   out_3584906548256442301[11] = 0;
   out_3584906548256442301[12] = 0;
   out_3584906548256442301[13] = 0;
   out_3584906548256442301[14] = 0;
   out_3584906548256442301[15] = 0;
   out_3584906548256442301[16] = 0;
   out_3584906548256442301[17] = 0;
   out_3584906548256442301[18] = 0;
   out_3584906548256442301[19] = 0;
   out_3584906548256442301[20] = 0;
   out_3584906548256442301[21] = 0;
   out_3584906548256442301[22] = 0;
   out_3584906548256442301[23] = 0;
   out_3584906548256442301[24] = 0;
   out_3584906548256442301[25] = 1;
   out_3584906548256442301[26] = 0;
   out_3584906548256442301[27] = 0;
   out_3584906548256442301[28] = 1;
   out_3584906548256442301[29] = 0;
   out_3584906548256442301[30] = 0;
   out_3584906548256442301[31] = 0;
   out_3584906548256442301[32] = 0;
   out_3584906548256442301[33] = 0;
   out_3584906548256442301[34] = 0;
   out_3584906548256442301[35] = 0;
   out_3584906548256442301[36] = 0;
   out_3584906548256442301[37] = 0;
   out_3584906548256442301[38] = 0;
   out_3584906548256442301[39] = 0;
   out_3584906548256442301[40] = 0;
   out_3584906548256442301[41] = 0;
   out_3584906548256442301[42] = 0;
   out_3584906548256442301[43] = 0;
   out_3584906548256442301[44] = 1;
   out_3584906548256442301[45] = 0;
   out_3584906548256442301[46] = 0;
   out_3584906548256442301[47] = 1;
   out_3584906548256442301[48] = 0;
   out_3584906548256442301[49] = 0;
   out_3584906548256442301[50] = 0;
   out_3584906548256442301[51] = 0;
   out_3584906548256442301[52] = 0;
   out_3584906548256442301[53] = 0;
}
void h_10(double *state, double *unused, double *out_2998629541313996374) {
   out_2998629541313996374[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_2998629541313996374[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_2998629541313996374[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_1479014506561746056) {
   out_1479014506561746056[0] = 0;
   out_1479014506561746056[1] = 9.8100000000000005*cos(state[1]);
   out_1479014506561746056[2] = 0;
   out_1479014506561746056[3] = 0;
   out_1479014506561746056[4] = -state[8];
   out_1479014506561746056[5] = state[7];
   out_1479014506561746056[6] = 0;
   out_1479014506561746056[7] = state[5];
   out_1479014506561746056[8] = -state[4];
   out_1479014506561746056[9] = 0;
   out_1479014506561746056[10] = 0;
   out_1479014506561746056[11] = 0;
   out_1479014506561746056[12] = 1;
   out_1479014506561746056[13] = 0;
   out_1479014506561746056[14] = 0;
   out_1479014506561746056[15] = 1;
   out_1479014506561746056[16] = 0;
   out_1479014506561746056[17] = 0;
   out_1479014506561746056[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_1479014506561746056[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_1479014506561746056[20] = 0;
   out_1479014506561746056[21] = state[8];
   out_1479014506561746056[22] = 0;
   out_1479014506561746056[23] = -state[6];
   out_1479014506561746056[24] = -state[5];
   out_1479014506561746056[25] = 0;
   out_1479014506561746056[26] = state[3];
   out_1479014506561746056[27] = 0;
   out_1479014506561746056[28] = 0;
   out_1479014506561746056[29] = 0;
   out_1479014506561746056[30] = 0;
   out_1479014506561746056[31] = 1;
   out_1479014506561746056[32] = 0;
   out_1479014506561746056[33] = 0;
   out_1479014506561746056[34] = 1;
   out_1479014506561746056[35] = 0;
   out_1479014506561746056[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_1479014506561746056[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_1479014506561746056[38] = 0;
   out_1479014506561746056[39] = -state[7];
   out_1479014506561746056[40] = state[6];
   out_1479014506561746056[41] = 0;
   out_1479014506561746056[42] = state[4];
   out_1479014506561746056[43] = -state[3];
   out_1479014506561746056[44] = 0;
   out_1479014506561746056[45] = 0;
   out_1479014506561746056[46] = 0;
   out_1479014506561746056[47] = 0;
   out_1479014506561746056[48] = 0;
   out_1479014506561746056[49] = 0;
   out_1479014506561746056[50] = 1;
   out_1479014506561746056[51] = 0;
   out_1479014506561746056[52] = 0;
   out_1479014506561746056[53] = 1;
}
void h_13(double *state, double *unused, double *out_7662457042295327166) {
   out_7662457042295327166[0] = state[3];
   out_7662457042295327166[1] = state[4];
   out_7662457042295327166[2] = state[5];
}
void H_13(double *state, double *unused, double *out_6797180373588775102) {
   out_6797180373588775102[0] = 0;
   out_6797180373588775102[1] = 0;
   out_6797180373588775102[2] = 0;
   out_6797180373588775102[3] = 1;
   out_6797180373588775102[4] = 0;
   out_6797180373588775102[5] = 0;
   out_6797180373588775102[6] = 0;
   out_6797180373588775102[7] = 0;
   out_6797180373588775102[8] = 0;
   out_6797180373588775102[9] = 0;
   out_6797180373588775102[10] = 0;
   out_6797180373588775102[11] = 0;
   out_6797180373588775102[12] = 0;
   out_6797180373588775102[13] = 0;
   out_6797180373588775102[14] = 0;
   out_6797180373588775102[15] = 0;
   out_6797180373588775102[16] = 0;
   out_6797180373588775102[17] = 0;
   out_6797180373588775102[18] = 0;
   out_6797180373588775102[19] = 0;
   out_6797180373588775102[20] = 0;
   out_6797180373588775102[21] = 0;
   out_6797180373588775102[22] = 1;
   out_6797180373588775102[23] = 0;
   out_6797180373588775102[24] = 0;
   out_6797180373588775102[25] = 0;
   out_6797180373588775102[26] = 0;
   out_6797180373588775102[27] = 0;
   out_6797180373588775102[28] = 0;
   out_6797180373588775102[29] = 0;
   out_6797180373588775102[30] = 0;
   out_6797180373588775102[31] = 0;
   out_6797180373588775102[32] = 0;
   out_6797180373588775102[33] = 0;
   out_6797180373588775102[34] = 0;
   out_6797180373588775102[35] = 0;
   out_6797180373588775102[36] = 0;
   out_6797180373588775102[37] = 0;
   out_6797180373588775102[38] = 0;
   out_6797180373588775102[39] = 0;
   out_6797180373588775102[40] = 0;
   out_6797180373588775102[41] = 1;
   out_6797180373588775102[42] = 0;
   out_6797180373588775102[43] = 0;
   out_6797180373588775102[44] = 0;
   out_6797180373588775102[45] = 0;
   out_6797180373588775102[46] = 0;
   out_6797180373588775102[47] = 0;
   out_6797180373588775102[48] = 0;
   out_6797180373588775102[49] = 0;
   out_6797180373588775102[50] = 0;
   out_6797180373588775102[51] = 0;
   out_6797180373588775102[52] = 0;
   out_6797180373588775102[53] = 0;
}
void h_14(double *state, double *unused, double *out_9220208786383067342) {
   out_9220208786383067342[0] = state[6];
   out_9220208786383067342[1] = state[7];
   out_9220208786383067342[2] = state[8];
}
void H_14(double *state, double *unused, double *out_7548147404595926830) {
   out_7548147404595926830[0] = 0;
   out_7548147404595926830[1] = 0;
   out_7548147404595926830[2] = 0;
   out_7548147404595926830[3] = 0;
   out_7548147404595926830[4] = 0;
   out_7548147404595926830[5] = 0;
   out_7548147404595926830[6] = 1;
   out_7548147404595926830[7] = 0;
   out_7548147404595926830[8] = 0;
   out_7548147404595926830[9] = 0;
   out_7548147404595926830[10] = 0;
   out_7548147404595926830[11] = 0;
   out_7548147404595926830[12] = 0;
   out_7548147404595926830[13] = 0;
   out_7548147404595926830[14] = 0;
   out_7548147404595926830[15] = 0;
   out_7548147404595926830[16] = 0;
   out_7548147404595926830[17] = 0;
   out_7548147404595926830[18] = 0;
   out_7548147404595926830[19] = 0;
   out_7548147404595926830[20] = 0;
   out_7548147404595926830[21] = 0;
   out_7548147404595926830[22] = 0;
   out_7548147404595926830[23] = 0;
   out_7548147404595926830[24] = 0;
   out_7548147404595926830[25] = 1;
   out_7548147404595926830[26] = 0;
   out_7548147404595926830[27] = 0;
   out_7548147404595926830[28] = 0;
   out_7548147404595926830[29] = 0;
   out_7548147404595926830[30] = 0;
   out_7548147404595926830[31] = 0;
   out_7548147404595926830[32] = 0;
   out_7548147404595926830[33] = 0;
   out_7548147404595926830[34] = 0;
   out_7548147404595926830[35] = 0;
   out_7548147404595926830[36] = 0;
   out_7548147404595926830[37] = 0;
   out_7548147404595926830[38] = 0;
   out_7548147404595926830[39] = 0;
   out_7548147404595926830[40] = 0;
   out_7548147404595926830[41] = 0;
   out_7548147404595926830[42] = 0;
   out_7548147404595926830[43] = 0;
   out_7548147404595926830[44] = 1;
   out_7548147404595926830[45] = 0;
   out_7548147404595926830[46] = 0;
   out_7548147404595926830[47] = 0;
   out_7548147404595926830[48] = 0;
   out_7548147404595926830[49] = 0;
   out_7548147404595926830[50] = 0;
   out_7548147404595926830[51] = 0;
   out_7548147404595926830[52] = 0;
   out_7548147404595926830[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_5394071911806177632) {
  err_fun(nom_x, delta_x, out_5394071911806177632);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_390924532607337478) {
  inv_err_fun(nom_x, true_x, out_390924532607337478);
}
void pose_H_mod_fun(double *state, double *out_9013755718701150594) {
  H_mod_fun(state, out_9013755718701150594);
}
void pose_f_fun(double *state, double dt, double *out_559878194513068250) {
  f_fun(state,  dt, out_559878194513068250);
}
void pose_F_fun(double *state, double dt, double *out_3177690484241037676) {
  F_fun(state,  dt, out_3177690484241037676);
}
void pose_h_4(double *state, double *unused, double *out_3640963725510673355) {
  h_4(state, unused, out_3640963725510673355);
}
void pose_H_4(double *state, double *unused, double *out_3584906548256442301) {
  H_4(state, unused, out_3584906548256442301);
}
void pose_h_10(double *state, double *unused, double *out_2998629541313996374) {
  h_10(state, unused, out_2998629541313996374);
}
void pose_H_10(double *state, double *unused, double *out_1479014506561746056) {
  H_10(state, unused, out_1479014506561746056);
}
void pose_h_13(double *state, double *unused, double *out_7662457042295327166) {
  h_13(state, unused, out_7662457042295327166);
}
void pose_H_13(double *state, double *unused, double *out_6797180373588775102) {
  H_13(state, unused, out_6797180373588775102);
}
void pose_h_14(double *state, double *unused, double *out_9220208786383067342) {
  h_14(state, unused, out_9220208786383067342);
}
void pose_H_14(double *state, double *unused, double *out_7548147404595926830) {
  H_14(state, unused, out_7548147404595926830);
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
