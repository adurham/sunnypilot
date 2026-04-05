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
void err_fun(double *nom_x, double *delta_x, double *out_7559082103702812710) {
   out_7559082103702812710[0] = delta_x[0] + nom_x[0];
   out_7559082103702812710[1] = delta_x[1] + nom_x[1];
   out_7559082103702812710[2] = delta_x[2] + nom_x[2];
   out_7559082103702812710[3] = delta_x[3] + nom_x[3];
   out_7559082103702812710[4] = delta_x[4] + nom_x[4];
   out_7559082103702812710[5] = delta_x[5] + nom_x[5];
   out_7559082103702812710[6] = delta_x[6] + nom_x[6];
   out_7559082103702812710[7] = delta_x[7] + nom_x[7];
   out_7559082103702812710[8] = delta_x[8] + nom_x[8];
   out_7559082103702812710[9] = delta_x[9] + nom_x[9];
   out_7559082103702812710[10] = delta_x[10] + nom_x[10];
   out_7559082103702812710[11] = delta_x[11] + nom_x[11];
   out_7559082103702812710[12] = delta_x[12] + nom_x[12];
   out_7559082103702812710[13] = delta_x[13] + nom_x[13];
   out_7559082103702812710[14] = delta_x[14] + nom_x[14];
   out_7559082103702812710[15] = delta_x[15] + nom_x[15];
   out_7559082103702812710[16] = delta_x[16] + nom_x[16];
   out_7559082103702812710[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_1140165562353213950) {
   out_1140165562353213950[0] = -nom_x[0] + true_x[0];
   out_1140165562353213950[1] = -nom_x[1] + true_x[1];
   out_1140165562353213950[2] = -nom_x[2] + true_x[2];
   out_1140165562353213950[3] = -nom_x[3] + true_x[3];
   out_1140165562353213950[4] = -nom_x[4] + true_x[4];
   out_1140165562353213950[5] = -nom_x[5] + true_x[5];
   out_1140165562353213950[6] = -nom_x[6] + true_x[6];
   out_1140165562353213950[7] = -nom_x[7] + true_x[7];
   out_1140165562353213950[8] = -nom_x[8] + true_x[8];
   out_1140165562353213950[9] = -nom_x[9] + true_x[9];
   out_1140165562353213950[10] = -nom_x[10] + true_x[10];
   out_1140165562353213950[11] = -nom_x[11] + true_x[11];
   out_1140165562353213950[12] = -nom_x[12] + true_x[12];
   out_1140165562353213950[13] = -nom_x[13] + true_x[13];
   out_1140165562353213950[14] = -nom_x[14] + true_x[14];
   out_1140165562353213950[15] = -nom_x[15] + true_x[15];
   out_1140165562353213950[16] = -nom_x[16] + true_x[16];
   out_1140165562353213950[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_7826065894907355794) {
   out_7826065894907355794[0] = 1.0;
   out_7826065894907355794[1] = 0.0;
   out_7826065894907355794[2] = 0.0;
   out_7826065894907355794[3] = 0.0;
   out_7826065894907355794[4] = 0.0;
   out_7826065894907355794[5] = 0.0;
   out_7826065894907355794[6] = 0.0;
   out_7826065894907355794[7] = 0.0;
   out_7826065894907355794[8] = 0.0;
   out_7826065894907355794[9] = 0.0;
   out_7826065894907355794[10] = 0.0;
   out_7826065894907355794[11] = 0.0;
   out_7826065894907355794[12] = 0.0;
   out_7826065894907355794[13] = 0.0;
   out_7826065894907355794[14] = 0.0;
   out_7826065894907355794[15] = 0.0;
   out_7826065894907355794[16] = 0.0;
   out_7826065894907355794[17] = 0.0;
   out_7826065894907355794[18] = 0.0;
   out_7826065894907355794[19] = 1.0;
   out_7826065894907355794[20] = 0.0;
   out_7826065894907355794[21] = 0.0;
   out_7826065894907355794[22] = 0.0;
   out_7826065894907355794[23] = 0.0;
   out_7826065894907355794[24] = 0.0;
   out_7826065894907355794[25] = 0.0;
   out_7826065894907355794[26] = 0.0;
   out_7826065894907355794[27] = 0.0;
   out_7826065894907355794[28] = 0.0;
   out_7826065894907355794[29] = 0.0;
   out_7826065894907355794[30] = 0.0;
   out_7826065894907355794[31] = 0.0;
   out_7826065894907355794[32] = 0.0;
   out_7826065894907355794[33] = 0.0;
   out_7826065894907355794[34] = 0.0;
   out_7826065894907355794[35] = 0.0;
   out_7826065894907355794[36] = 0.0;
   out_7826065894907355794[37] = 0.0;
   out_7826065894907355794[38] = 1.0;
   out_7826065894907355794[39] = 0.0;
   out_7826065894907355794[40] = 0.0;
   out_7826065894907355794[41] = 0.0;
   out_7826065894907355794[42] = 0.0;
   out_7826065894907355794[43] = 0.0;
   out_7826065894907355794[44] = 0.0;
   out_7826065894907355794[45] = 0.0;
   out_7826065894907355794[46] = 0.0;
   out_7826065894907355794[47] = 0.0;
   out_7826065894907355794[48] = 0.0;
   out_7826065894907355794[49] = 0.0;
   out_7826065894907355794[50] = 0.0;
   out_7826065894907355794[51] = 0.0;
   out_7826065894907355794[52] = 0.0;
   out_7826065894907355794[53] = 0.0;
   out_7826065894907355794[54] = 0.0;
   out_7826065894907355794[55] = 0.0;
   out_7826065894907355794[56] = 0.0;
   out_7826065894907355794[57] = 1.0;
   out_7826065894907355794[58] = 0.0;
   out_7826065894907355794[59] = 0.0;
   out_7826065894907355794[60] = 0.0;
   out_7826065894907355794[61] = 0.0;
   out_7826065894907355794[62] = 0.0;
   out_7826065894907355794[63] = 0.0;
   out_7826065894907355794[64] = 0.0;
   out_7826065894907355794[65] = 0.0;
   out_7826065894907355794[66] = 0.0;
   out_7826065894907355794[67] = 0.0;
   out_7826065894907355794[68] = 0.0;
   out_7826065894907355794[69] = 0.0;
   out_7826065894907355794[70] = 0.0;
   out_7826065894907355794[71] = 0.0;
   out_7826065894907355794[72] = 0.0;
   out_7826065894907355794[73] = 0.0;
   out_7826065894907355794[74] = 0.0;
   out_7826065894907355794[75] = 0.0;
   out_7826065894907355794[76] = 1.0;
   out_7826065894907355794[77] = 0.0;
   out_7826065894907355794[78] = 0.0;
   out_7826065894907355794[79] = 0.0;
   out_7826065894907355794[80] = 0.0;
   out_7826065894907355794[81] = 0.0;
   out_7826065894907355794[82] = 0.0;
   out_7826065894907355794[83] = 0.0;
   out_7826065894907355794[84] = 0.0;
   out_7826065894907355794[85] = 0.0;
   out_7826065894907355794[86] = 0.0;
   out_7826065894907355794[87] = 0.0;
   out_7826065894907355794[88] = 0.0;
   out_7826065894907355794[89] = 0.0;
   out_7826065894907355794[90] = 0.0;
   out_7826065894907355794[91] = 0.0;
   out_7826065894907355794[92] = 0.0;
   out_7826065894907355794[93] = 0.0;
   out_7826065894907355794[94] = 0.0;
   out_7826065894907355794[95] = 1.0;
   out_7826065894907355794[96] = 0.0;
   out_7826065894907355794[97] = 0.0;
   out_7826065894907355794[98] = 0.0;
   out_7826065894907355794[99] = 0.0;
   out_7826065894907355794[100] = 0.0;
   out_7826065894907355794[101] = 0.0;
   out_7826065894907355794[102] = 0.0;
   out_7826065894907355794[103] = 0.0;
   out_7826065894907355794[104] = 0.0;
   out_7826065894907355794[105] = 0.0;
   out_7826065894907355794[106] = 0.0;
   out_7826065894907355794[107] = 0.0;
   out_7826065894907355794[108] = 0.0;
   out_7826065894907355794[109] = 0.0;
   out_7826065894907355794[110] = 0.0;
   out_7826065894907355794[111] = 0.0;
   out_7826065894907355794[112] = 0.0;
   out_7826065894907355794[113] = 0.0;
   out_7826065894907355794[114] = 1.0;
   out_7826065894907355794[115] = 0.0;
   out_7826065894907355794[116] = 0.0;
   out_7826065894907355794[117] = 0.0;
   out_7826065894907355794[118] = 0.0;
   out_7826065894907355794[119] = 0.0;
   out_7826065894907355794[120] = 0.0;
   out_7826065894907355794[121] = 0.0;
   out_7826065894907355794[122] = 0.0;
   out_7826065894907355794[123] = 0.0;
   out_7826065894907355794[124] = 0.0;
   out_7826065894907355794[125] = 0.0;
   out_7826065894907355794[126] = 0.0;
   out_7826065894907355794[127] = 0.0;
   out_7826065894907355794[128] = 0.0;
   out_7826065894907355794[129] = 0.0;
   out_7826065894907355794[130] = 0.0;
   out_7826065894907355794[131] = 0.0;
   out_7826065894907355794[132] = 0.0;
   out_7826065894907355794[133] = 1.0;
   out_7826065894907355794[134] = 0.0;
   out_7826065894907355794[135] = 0.0;
   out_7826065894907355794[136] = 0.0;
   out_7826065894907355794[137] = 0.0;
   out_7826065894907355794[138] = 0.0;
   out_7826065894907355794[139] = 0.0;
   out_7826065894907355794[140] = 0.0;
   out_7826065894907355794[141] = 0.0;
   out_7826065894907355794[142] = 0.0;
   out_7826065894907355794[143] = 0.0;
   out_7826065894907355794[144] = 0.0;
   out_7826065894907355794[145] = 0.0;
   out_7826065894907355794[146] = 0.0;
   out_7826065894907355794[147] = 0.0;
   out_7826065894907355794[148] = 0.0;
   out_7826065894907355794[149] = 0.0;
   out_7826065894907355794[150] = 0.0;
   out_7826065894907355794[151] = 0.0;
   out_7826065894907355794[152] = 1.0;
   out_7826065894907355794[153] = 0.0;
   out_7826065894907355794[154] = 0.0;
   out_7826065894907355794[155] = 0.0;
   out_7826065894907355794[156] = 0.0;
   out_7826065894907355794[157] = 0.0;
   out_7826065894907355794[158] = 0.0;
   out_7826065894907355794[159] = 0.0;
   out_7826065894907355794[160] = 0.0;
   out_7826065894907355794[161] = 0.0;
   out_7826065894907355794[162] = 0.0;
   out_7826065894907355794[163] = 0.0;
   out_7826065894907355794[164] = 0.0;
   out_7826065894907355794[165] = 0.0;
   out_7826065894907355794[166] = 0.0;
   out_7826065894907355794[167] = 0.0;
   out_7826065894907355794[168] = 0.0;
   out_7826065894907355794[169] = 0.0;
   out_7826065894907355794[170] = 0.0;
   out_7826065894907355794[171] = 1.0;
   out_7826065894907355794[172] = 0.0;
   out_7826065894907355794[173] = 0.0;
   out_7826065894907355794[174] = 0.0;
   out_7826065894907355794[175] = 0.0;
   out_7826065894907355794[176] = 0.0;
   out_7826065894907355794[177] = 0.0;
   out_7826065894907355794[178] = 0.0;
   out_7826065894907355794[179] = 0.0;
   out_7826065894907355794[180] = 0.0;
   out_7826065894907355794[181] = 0.0;
   out_7826065894907355794[182] = 0.0;
   out_7826065894907355794[183] = 0.0;
   out_7826065894907355794[184] = 0.0;
   out_7826065894907355794[185] = 0.0;
   out_7826065894907355794[186] = 0.0;
   out_7826065894907355794[187] = 0.0;
   out_7826065894907355794[188] = 0.0;
   out_7826065894907355794[189] = 0.0;
   out_7826065894907355794[190] = 1.0;
   out_7826065894907355794[191] = 0.0;
   out_7826065894907355794[192] = 0.0;
   out_7826065894907355794[193] = 0.0;
   out_7826065894907355794[194] = 0.0;
   out_7826065894907355794[195] = 0.0;
   out_7826065894907355794[196] = 0.0;
   out_7826065894907355794[197] = 0.0;
   out_7826065894907355794[198] = 0.0;
   out_7826065894907355794[199] = 0.0;
   out_7826065894907355794[200] = 0.0;
   out_7826065894907355794[201] = 0.0;
   out_7826065894907355794[202] = 0.0;
   out_7826065894907355794[203] = 0.0;
   out_7826065894907355794[204] = 0.0;
   out_7826065894907355794[205] = 0.0;
   out_7826065894907355794[206] = 0.0;
   out_7826065894907355794[207] = 0.0;
   out_7826065894907355794[208] = 0.0;
   out_7826065894907355794[209] = 1.0;
   out_7826065894907355794[210] = 0.0;
   out_7826065894907355794[211] = 0.0;
   out_7826065894907355794[212] = 0.0;
   out_7826065894907355794[213] = 0.0;
   out_7826065894907355794[214] = 0.0;
   out_7826065894907355794[215] = 0.0;
   out_7826065894907355794[216] = 0.0;
   out_7826065894907355794[217] = 0.0;
   out_7826065894907355794[218] = 0.0;
   out_7826065894907355794[219] = 0.0;
   out_7826065894907355794[220] = 0.0;
   out_7826065894907355794[221] = 0.0;
   out_7826065894907355794[222] = 0.0;
   out_7826065894907355794[223] = 0.0;
   out_7826065894907355794[224] = 0.0;
   out_7826065894907355794[225] = 0.0;
   out_7826065894907355794[226] = 0.0;
   out_7826065894907355794[227] = 0.0;
   out_7826065894907355794[228] = 1.0;
   out_7826065894907355794[229] = 0.0;
   out_7826065894907355794[230] = 0.0;
   out_7826065894907355794[231] = 0.0;
   out_7826065894907355794[232] = 0.0;
   out_7826065894907355794[233] = 0.0;
   out_7826065894907355794[234] = 0.0;
   out_7826065894907355794[235] = 0.0;
   out_7826065894907355794[236] = 0.0;
   out_7826065894907355794[237] = 0.0;
   out_7826065894907355794[238] = 0.0;
   out_7826065894907355794[239] = 0.0;
   out_7826065894907355794[240] = 0.0;
   out_7826065894907355794[241] = 0.0;
   out_7826065894907355794[242] = 0.0;
   out_7826065894907355794[243] = 0.0;
   out_7826065894907355794[244] = 0.0;
   out_7826065894907355794[245] = 0.0;
   out_7826065894907355794[246] = 0.0;
   out_7826065894907355794[247] = 1.0;
   out_7826065894907355794[248] = 0.0;
   out_7826065894907355794[249] = 0.0;
   out_7826065894907355794[250] = 0.0;
   out_7826065894907355794[251] = 0.0;
   out_7826065894907355794[252] = 0.0;
   out_7826065894907355794[253] = 0.0;
   out_7826065894907355794[254] = 0.0;
   out_7826065894907355794[255] = 0.0;
   out_7826065894907355794[256] = 0.0;
   out_7826065894907355794[257] = 0.0;
   out_7826065894907355794[258] = 0.0;
   out_7826065894907355794[259] = 0.0;
   out_7826065894907355794[260] = 0.0;
   out_7826065894907355794[261] = 0.0;
   out_7826065894907355794[262] = 0.0;
   out_7826065894907355794[263] = 0.0;
   out_7826065894907355794[264] = 0.0;
   out_7826065894907355794[265] = 0.0;
   out_7826065894907355794[266] = 1.0;
   out_7826065894907355794[267] = 0.0;
   out_7826065894907355794[268] = 0.0;
   out_7826065894907355794[269] = 0.0;
   out_7826065894907355794[270] = 0.0;
   out_7826065894907355794[271] = 0.0;
   out_7826065894907355794[272] = 0.0;
   out_7826065894907355794[273] = 0.0;
   out_7826065894907355794[274] = 0.0;
   out_7826065894907355794[275] = 0.0;
   out_7826065894907355794[276] = 0.0;
   out_7826065894907355794[277] = 0.0;
   out_7826065894907355794[278] = 0.0;
   out_7826065894907355794[279] = 0.0;
   out_7826065894907355794[280] = 0.0;
   out_7826065894907355794[281] = 0.0;
   out_7826065894907355794[282] = 0.0;
   out_7826065894907355794[283] = 0.0;
   out_7826065894907355794[284] = 0.0;
   out_7826065894907355794[285] = 1.0;
   out_7826065894907355794[286] = 0.0;
   out_7826065894907355794[287] = 0.0;
   out_7826065894907355794[288] = 0.0;
   out_7826065894907355794[289] = 0.0;
   out_7826065894907355794[290] = 0.0;
   out_7826065894907355794[291] = 0.0;
   out_7826065894907355794[292] = 0.0;
   out_7826065894907355794[293] = 0.0;
   out_7826065894907355794[294] = 0.0;
   out_7826065894907355794[295] = 0.0;
   out_7826065894907355794[296] = 0.0;
   out_7826065894907355794[297] = 0.0;
   out_7826065894907355794[298] = 0.0;
   out_7826065894907355794[299] = 0.0;
   out_7826065894907355794[300] = 0.0;
   out_7826065894907355794[301] = 0.0;
   out_7826065894907355794[302] = 0.0;
   out_7826065894907355794[303] = 0.0;
   out_7826065894907355794[304] = 1.0;
   out_7826065894907355794[305] = 0.0;
   out_7826065894907355794[306] = 0.0;
   out_7826065894907355794[307] = 0.0;
   out_7826065894907355794[308] = 0.0;
   out_7826065894907355794[309] = 0.0;
   out_7826065894907355794[310] = 0.0;
   out_7826065894907355794[311] = 0.0;
   out_7826065894907355794[312] = 0.0;
   out_7826065894907355794[313] = 0.0;
   out_7826065894907355794[314] = 0.0;
   out_7826065894907355794[315] = 0.0;
   out_7826065894907355794[316] = 0.0;
   out_7826065894907355794[317] = 0.0;
   out_7826065894907355794[318] = 0.0;
   out_7826065894907355794[319] = 0.0;
   out_7826065894907355794[320] = 0.0;
   out_7826065894907355794[321] = 0.0;
   out_7826065894907355794[322] = 0.0;
   out_7826065894907355794[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_8956214327200202745) {
   out_8956214327200202745[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_8956214327200202745[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_8956214327200202745[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_8956214327200202745[3] = dt*state[12] + state[3];
   out_8956214327200202745[4] = dt*state[13] + state[4];
   out_8956214327200202745[5] = dt*state[14] + state[5];
   out_8956214327200202745[6] = state[6];
   out_8956214327200202745[7] = state[7];
   out_8956214327200202745[8] = state[8];
   out_8956214327200202745[9] = state[9];
   out_8956214327200202745[10] = state[10];
   out_8956214327200202745[11] = state[11];
   out_8956214327200202745[12] = state[12];
   out_8956214327200202745[13] = state[13];
   out_8956214327200202745[14] = state[14];
   out_8956214327200202745[15] = state[15];
   out_8956214327200202745[16] = state[16];
   out_8956214327200202745[17] = state[17];
}
void F_fun(double *state, double dt, double *out_475163589378229554) {
   out_475163589378229554[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_475163589378229554[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_475163589378229554[2] = 0;
   out_475163589378229554[3] = 0;
   out_475163589378229554[4] = 0;
   out_475163589378229554[5] = 0;
   out_475163589378229554[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_475163589378229554[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_475163589378229554[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_475163589378229554[9] = 0;
   out_475163589378229554[10] = 0;
   out_475163589378229554[11] = 0;
   out_475163589378229554[12] = 0;
   out_475163589378229554[13] = 0;
   out_475163589378229554[14] = 0;
   out_475163589378229554[15] = 0;
   out_475163589378229554[16] = 0;
   out_475163589378229554[17] = 0;
   out_475163589378229554[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_475163589378229554[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_475163589378229554[20] = 0;
   out_475163589378229554[21] = 0;
   out_475163589378229554[22] = 0;
   out_475163589378229554[23] = 0;
   out_475163589378229554[24] = 0;
   out_475163589378229554[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_475163589378229554[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_475163589378229554[27] = 0;
   out_475163589378229554[28] = 0;
   out_475163589378229554[29] = 0;
   out_475163589378229554[30] = 0;
   out_475163589378229554[31] = 0;
   out_475163589378229554[32] = 0;
   out_475163589378229554[33] = 0;
   out_475163589378229554[34] = 0;
   out_475163589378229554[35] = 0;
   out_475163589378229554[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_475163589378229554[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_475163589378229554[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_475163589378229554[39] = 0;
   out_475163589378229554[40] = 0;
   out_475163589378229554[41] = 0;
   out_475163589378229554[42] = 0;
   out_475163589378229554[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_475163589378229554[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_475163589378229554[45] = 0;
   out_475163589378229554[46] = 0;
   out_475163589378229554[47] = 0;
   out_475163589378229554[48] = 0;
   out_475163589378229554[49] = 0;
   out_475163589378229554[50] = 0;
   out_475163589378229554[51] = 0;
   out_475163589378229554[52] = 0;
   out_475163589378229554[53] = 0;
   out_475163589378229554[54] = 0;
   out_475163589378229554[55] = 0;
   out_475163589378229554[56] = 0;
   out_475163589378229554[57] = 1;
   out_475163589378229554[58] = 0;
   out_475163589378229554[59] = 0;
   out_475163589378229554[60] = 0;
   out_475163589378229554[61] = 0;
   out_475163589378229554[62] = 0;
   out_475163589378229554[63] = 0;
   out_475163589378229554[64] = 0;
   out_475163589378229554[65] = 0;
   out_475163589378229554[66] = dt;
   out_475163589378229554[67] = 0;
   out_475163589378229554[68] = 0;
   out_475163589378229554[69] = 0;
   out_475163589378229554[70] = 0;
   out_475163589378229554[71] = 0;
   out_475163589378229554[72] = 0;
   out_475163589378229554[73] = 0;
   out_475163589378229554[74] = 0;
   out_475163589378229554[75] = 0;
   out_475163589378229554[76] = 1;
   out_475163589378229554[77] = 0;
   out_475163589378229554[78] = 0;
   out_475163589378229554[79] = 0;
   out_475163589378229554[80] = 0;
   out_475163589378229554[81] = 0;
   out_475163589378229554[82] = 0;
   out_475163589378229554[83] = 0;
   out_475163589378229554[84] = 0;
   out_475163589378229554[85] = dt;
   out_475163589378229554[86] = 0;
   out_475163589378229554[87] = 0;
   out_475163589378229554[88] = 0;
   out_475163589378229554[89] = 0;
   out_475163589378229554[90] = 0;
   out_475163589378229554[91] = 0;
   out_475163589378229554[92] = 0;
   out_475163589378229554[93] = 0;
   out_475163589378229554[94] = 0;
   out_475163589378229554[95] = 1;
   out_475163589378229554[96] = 0;
   out_475163589378229554[97] = 0;
   out_475163589378229554[98] = 0;
   out_475163589378229554[99] = 0;
   out_475163589378229554[100] = 0;
   out_475163589378229554[101] = 0;
   out_475163589378229554[102] = 0;
   out_475163589378229554[103] = 0;
   out_475163589378229554[104] = dt;
   out_475163589378229554[105] = 0;
   out_475163589378229554[106] = 0;
   out_475163589378229554[107] = 0;
   out_475163589378229554[108] = 0;
   out_475163589378229554[109] = 0;
   out_475163589378229554[110] = 0;
   out_475163589378229554[111] = 0;
   out_475163589378229554[112] = 0;
   out_475163589378229554[113] = 0;
   out_475163589378229554[114] = 1;
   out_475163589378229554[115] = 0;
   out_475163589378229554[116] = 0;
   out_475163589378229554[117] = 0;
   out_475163589378229554[118] = 0;
   out_475163589378229554[119] = 0;
   out_475163589378229554[120] = 0;
   out_475163589378229554[121] = 0;
   out_475163589378229554[122] = 0;
   out_475163589378229554[123] = 0;
   out_475163589378229554[124] = 0;
   out_475163589378229554[125] = 0;
   out_475163589378229554[126] = 0;
   out_475163589378229554[127] = 0;
   out_475163589378229554[128] = 0;
   out_475163589378229554[129] = 0;
   out_475163589378229554[130] = 0;
   out_475163589378229554[131] = 0;
   out_475163589378229554[132] = 0;
   out_475163589378229554[133] = 1;
   out_475163589378229554[134] = 0;
   out_475163589378229554[135] = 0;
   out_475163589378229554[136] = 0;
   out_475163589378229554[137] = 0;
   out_475163589378229554[138] = 0;
   out_475163589378229554[139] = 0;
   out_475163589378229554[140] = 0;
   out_475163589378229554[141] = 0;
   out_475163589378229554[142] = 0;
   out_475163589378229554[143] = 0;
   out_475163589378229554[144] = 0;
   out_475163589378229554[145] = 0;
   out_475163589378229554[146] = 0;
   out_475163589378229554[147] = 0;
   out_475163589378229554[148] = 0;
   out_475163589378229554[149] = 0;
   out_475163589378229554[150] = 0;
   out_475163589378229554[151] = 0;
   out_475163589378229554[152] = 1;
   out_475163589378229554[153] = 0;
   out_475163589378229554[154] = 0;
   out_475163589378229554[155] = 0;
   out_475163589378229554[156] = 0;
   out_475163589378229554[157] = 0;
   out_475163589378229554[158] = 0;
   out_475163589378229554[159] = 0;
   out_475163589378229554[160] = 0;
   out_475163589378229554[161] = 0;
   out_475163589378229554[162] = 0;
   out_475163589378229554[163] = 0;
   out_475163589378229554[164] = 0;
   out_475163589378229554[165] = 0;
   out_475163589378229554[166] = 0;
   out_475163589378229554[167] = 0;
   out_475163589378229554[168] = 0;
   out_475163589378229554[169] = 0;
   out_475163589378229554[170] = 0;
   out_475163589378229554[171] = 1;
   out_475163589378229554[172] = 0;
   out_475163589378229554[173] = 0;
   out_475163589378229554[174] = 0;
   out_475163589378229554[175] = 0;
   out_475163589378229554[176] = 0;
   out_475163589378229554[177] = 0;
   out_475163589378229554[178] = 0;
   out_475163589378229554[179] = 0;
   out_475163589378229554[180] = 0;
   out_475163589378229554[181] = 0;
   out_475163589378229554[182] = 0;
   out_475163589378229554[183] = 0;
   out_475163589378229554[184] = 0;
   out_475163589378229554[185] = 0;
   out_475163589378229554[186] = 0;
   out_475163589378229554[187] = 0;
   out_475163589378229554[188] = 0;
   out_475163589378229554[189] = 0;
   out_475163589378229554[190] = 1;
   out_475163589378229554[191] = 0;
   out_475163589378229554[192] = 0;
   out_475163589378229554[193] = 0;
   out_475163589378229554[194] = 0;
   out_475163589378229554[195] = 0;
   out_475163589378229554[196] = 0;
   out_475163589378229554[197] = 0;
   out_475163589378229554[198] = 0;
   out_475163589378229554[199] = 0;
   out_475163589378229554[200] = 0;
   out_475163589378229554[201] = 0;
   out_475163589378229554[202] = 0;
   out_475163589378229554[203] = 0;
   out_475163589378229554[204] = 0;
   out_475163589378229554[205] = 0;
   out_475163589378229554[206] = 0;
   out_475163589378229554[207] = 0;
   out_475163589378229554[208] = 0;
   out_475163589378229554[209] = 1;
   out_475163589378229554[210] = 0;
   out_475163589378229554[211] = 0;
   out_475163589378229554[212] = 0;
   out_475163589378229554[213] = 0;
   out_475163589378229554[214] = 0;
   out_475163589378229554[215] = 0;
   out_475163589378229554[216] = 0;
   out_475163589378229554[217] = 0;
   out_475163589378229554[218] = 0;
   out_475163589378229554[219] = 0;
   out_475163589378229554[220] = 0;
   out_475163589378229554[221] = 0;
   out_475163589378229554[222] = 0;
   out_475163589378229554[223] = 0;
   out_475163589378229554[224] = 0;
   out_475163589378229554[225] = 0;
   out_475163589378229554[226] = 0;
   out_475163589378229554[227] = 0;
   out_475163589378229554[228] = 1;
   out_475163589378229554[229] = 0;
   out_475163589378229554[230] = 0;
   out_475163589378229554[231] = 0;
   out_475163589378229554[232] = 0;
   out_475163589378229554[233] = 0;
   out_475163589378229554[234] = 0;
   out_475163589378229554[235] = 0;
   out_475163589378229554[236] = 0;
   out_475163589378229554[237] = 0;
   out_475163589378229554[238] = 0;
   out_475163589378229554[239] = 0;
   out_475163589378229554[240] = 0;
   out_475163589378229554[241] = 0;
   out_475163589378229554[242] = 0;
   out_475163589378229554[243] = 0;
   out_475163589378229554[244] = 0;
   out_475163589378229554[245] = 0;
   out_475163589378229554[246] = 0;
   out_475163589378229554[247] = 1;
   out_475163589378229554[248] = 0;
   out_475163589378229554[249] = 0;
   out_475163589378229554[250] = 0;
   out_475163589378229554[251] = 0;
   out_475163589378229554[252] = 0;
   out_475163589378229554[253] = 0;
   out_475163589378229554[254] = 0;
   out_475163589378229554[255] = 0;
   out_475163589378229554[256] = 0;
   out_475163589378229554[257] = 0;
   out_475163589378229554[258] = 0;
   out_475163589378229554[259] = 0;
   out_475163589378229554[260] = 0;
   out_475163589378229554[261] = 0;
   out_475163589378229554[262] = 0;
   out_475163589378229554[263] = 0;
   out_475163589378229554[264] = 0;
   out_475163589378229554[265] = 0;
   out_475163589378229554[266] = 1;
   out_475163589378229554[267] = 0;
   out_475163589378229554[268] = 0;
   out_475163589378229554[269] = 0;
   out_475163589378229554[270] = 0;
   out_475163589378229554[271] = 0;
   out_475163589378229554[272] = 0;
   out_475163589378229554[273] = 0;
   out_475163589378229554[274] = 0;
   out_475163589378229554[275] = 0;
   out_475163589378229554[276] = 0;
   out_475163589378229554[277] = 0;
   out_475163589378229554[278] = 0;
   out_475163589378229554[279] = 0;
   out_475163589378229554[280] = 0;
   out_475163589378229554[281] = 0;
   out_475163589378229554[282] = 0;
   out_475163589378229554[283] = 0;
   out_475163589378229554[284] = 0;
   out_475163589378229554[285] = 1;
   out_475163589378229554[286] = 0;
   out_475163589378229554[287] = 0;
   out_475163589378229554[288] = 0;
   out_475163589378229554[289] = 0;
   out_475163589378229554[290] = 0;
   out_475163589378229554[291] = 0;
   out_475163589378229554[292] = 0;
   out_475163589378229554[293] = 0;
   out_475163589378229554[294] = 0;
   out_475163589378229554[295] = 0;
   out_475163589378229554[296] = 0;
   out_475163589378229554[297] = 0;
   out_475163589378229554[298] = 0;
   out_475163589378229554[299] = 0;
   out_475163589378229554[300] = 0;
   out_475163589378229554[301] = 0;
   out_475163589378229554[302] = 0;
   out_475163589378229554[303] = 0;
   out_475163589378229554[304] = 1;
   out_475163589378229554[305] = 0;
   out_475163589378229554[306] = 0;
   out_475163589378229554[307] = 0;
   out_475163589378229554[308] = 0;
   out_475163589378229554[309] = 0;
   out_475163589378229554[310] = 0;
   out_475163589378229554[311] = 0;
   out_475163589378229554[312] = 0;
   out_475163589378229554[313] = 0;
   out_475163589378229554[314] = 0;
   out_475163589378229554[315] = 0;
   out_475163589378229554[316] = 0;
   out_475163589378229554[317] = 0;
   out_475163589378229554[318] = 0;
   out_475163589378229554[319] = 0;
   out_475163589378229554[320] = 0;
   out_475163589378229554[321] = 0;
   out_475163589378229554[322] = 0;
   out_475163589378229554[323] = 1;
}
void h_4(double *state, double *unused, double *out_1919401175009708031) {
   out_1919401175009708031[0] = state[6] + state[9];
   out_1919401175009708031[1] = state[7] + state[10];
   out_1919401175009708031[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_897543215216634988) {
   out_897543215216634988[0] = 0;
   out_897543215216634988[1] = 0;
   out_897543215216634988[2] = 0;
   out_897543215216634988[3] = 0;
   out_897543215216634988[4] = 0;
   out_897543215216634988[5] = 0;
   out_897543215216634988[6] = 1;
   out_897543215216634988[7] = 0;
   out_897543215216634988[8] = 0;
   out_897543215216634988[9] = 1;
   out_897543215216634988[10] = 0;
   out_897543215216634988[11] = 0;
   out_897543215216634988[12] = 0;
   out_897543215216634988[13] = 0;
   out_897543215216634988[14] = 0;
   out_897543215216634988[15] = 0;
   out_897543215216634988[16] = 0;
   out_897543215216634988[17] = 0;
   out_897543215216634988[18] = 0;
   out_897543215216634988[19] = 0;
   out_897543215216634988[20] = 0;
   out_897543215216634988[21] = 0;
   out_897543215216634988[22] = 0;
   out_897543215216634988[23] = 0;
   out_897543215216634988[24] = 0;
   out_897543215216634988[25] = 1;
   out_897543215216634988[26] = 0;
   out_897543215216634988[27] = 0;
   out_897543215216634988[28] = 1;
   out_897543215216634988[29] = 0;
   out_897543215216634988[30] = 0;
   out_897543215216634988[31] = 0;
   out_897543215216634988[32] = 0;
   out_897543215216634988[33] = 0;
   out_897543215216634988[34] = 0;
   out_897543215216634988[35] = 0;
   out_897543215216634988[36] = 0;
   out_897543215216634988[37] = 0;
   out_897543215216634988[38] = 0;
   out_897543215216634988[39] = 0;
   out_897543215216634988[40] = 0;
   out_897543215216634988[41] = 0;
   out_897543215216634988[42] = 0;
   out_897543215216634988[43] = 0;
   out_897543215216634988[44] = 1;
   out_897543215216634988[45] = 0;
   out_897543215216634988[46] = 0;
   out_897543215216634988[47] = 1;
   out_897543215216634988[48] = 0;
   out_897543215216634988[49] = 0;
   out_897543215216634988[50] = 0;
   out_897543215216634988[51] = 0;
   out_897543215216634988[52] = 0;
   out_897543215216634988[53] = 0;
}
void h_10(double *state, double *unused, double *out_4006136634733641093) {
   out_4006136634733641093[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_4006136634733641093[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_4006136634733641093[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_8577831172590958455) {
   out_8577831172590958455[0] = 0;
   out_8577831172590958455[1] = 9.8100000000000005*cos(state[1]);
   out_8577831172590958455[2] = 0;
   out_8577831172590958455[3] = 0;
   out_8577831172590958455[4] = -state[8];
   out_8577831172590958455[5] = state[7];
   out_8577831172590958455[6] = 0;
   out_8577831172590958455[7] = state[5];
   out_8577831172590958455[8] = -state[4];
   out_8577831172590958455[9] = 0;
   out_8577831172590958455[10] = 0;
   out_8577831172590958455[11] = 0;
   out_8577831172590958455[12] = 1;
   out_8577831172590958455[13] = 0;
   out_8577831172590958455[14] = 0;
   out_8577831172590958455[15] = 1;
   out_8577831172590958455[16] = 0;
   out_8577831172590958455[17] = 0;
   out_8577831172590958455[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_8577831172590958455[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_8577831172590958455[20] = 0;
   out_8577831172590958455[21] = state[8];
   out_8577831172590958455[22] = 0;
   out_8577831172590958455[23] = -state[6];
   out_8577831172590958455[24] = -state[5];
   out_8577831172590958455[25] = 0;
   out_8577831172590958455[26] = state[3];
   out_8577831172590958455[27] = 0;
   out_8577831172590958455[28] = 0;
   out_8577831172590958455[29] = 0;
   out_8577831172590958455[30] = 0;
   out_8577831172590958455[31] = 1;
   out_8577831172590958455[32] = 0;
   out_8577831172590958455[33] = 0;
   out_8577831172590958455[34] = 1;
   out_8577831172590958455[35] = 0;
   out_8577831172590958455[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_8577831172590958455[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_8577831172590958455[38] = 0;
   out_8577831172590958455[39] = -state[7];
   out_8577831172590958455[40] = state[6];
   out_8577831172590958455[41] = 0;
   out_8577831172590958455[42] = state[4];
   out_8577831172590958455[43] = -state[3];
   out_8577831172590958455[44] = 0;
   out_8577831172590958455[45] = 0;
   out_8577831172590958455[46] = 0;
   out_8577831172590958455[47] = 0;
   out_8577831172590958455[48] = 0;
   out_8577831172590958455[49] = 0;
   out_8577831172590958455[50] = 1;
   out_8577831172590958455[51] = 0;
   out_8577831172590958455[52] = 0;
   out_8577831172590958455[53] = 1;
}
void h_13(double *state, double *unused, double *out_655481990136332419) {
   out_655481990136332419[0] = state[3];
   out_655481990136332419[1] = state[4];
   out_655481990136332419[2] = state[5];
}
void H_13(double *state, double *unused, double *out_2314730610115697813) {
   out_2314730610115697813[0] = 0;
   out_2314730610115697813[1] = 0;
   out_2314730610115697813[2] = 0;
   out_2314730610115697813[3] = 1;
   out_2314730610115697813[4] = 0;
   out_2314730610115697813[5] = 0;
   out_2314730610115697813[6] = 0;
   out_2314730610115697813[7] = 0;
   out_2314730610115697813[8] = 0;
   out_2314730610115697813[9] = 0;
   out_2314730610115697813[10] = 0;
   out_2314730610115697813[11] = 0;
   out_2314730610115697813[12] = 0;
   out_2314730610115697813[13] = 0;
   out_2314730610115697813[14] = 0;
   out_2314730610115697813[15] = 0;
   out_2314730610115697813[16] = 0;
   out_2314730610115697813[17] = 0;
   out_2314730610115697813[18] = 0;
   out_2314730610115697813[19] = 0;
   out_2314730610115697813[20] = 0;
   out_2314730610115697813[21] = 0;
   out_2314730610115697813[22] = 1;
   out_2314730610115697813[23] = 0;
   out_2314730610115697813[24] = 0;
   out_2314730610115697813[25] = 0;
   out_2314730610115697813[26] = 0;
   out_2314730610115697813[27] = 0;
   out_2314730610115697813[28] = 0;
   out_2314730610115697813[29] = 0;
   out_2314730610115697813[30] = 0;
   out_2314730610115697813[31] = 0;
   out_2314730610115697813[32] = 0;
   out_2314730610115697813[33] = 0;
   out_2314730610115697813[34] = 0;
   out_2314730610115697813[35] = 0;
   out_2314730610115697813[36] = 0;
   out_2314730610115697813[37] = 0;
   out_2314730610115697813[38] = 0;
   out_2314730610115697813[39] = 0;
   out_2314730610115697813[40] = 0;
   out_2314730610115697813[41] = 1;
   out_2314730610115697813[42] = 0;
   out_2314730610115697813[43] = 0;
   out_2314730610115697813[44] = 0;
   out_2314730610115697813[45] = 0;
   out_2314730610115697813[46] = 0;
   out_2314730610115697813[47] = 0;
   out_2314730610115697813[48] = 0;
   out_2314730610115697813[49] = 0;
   out_2314730610115697813[50] = 0;
   out_2314730610115697813[51] = 0;
   out_2314730610115697813[52] = 0;
   out_2314730610115697813[53] = 0;
}
void h_14(double *state, double *unused, double *out_3170072544189949915) {
   out_3170072544189949915[0] = state[6];
   out_3170072544189949915[1] = state[7];
   out_3170072544189949915[2] = state[8];
}
void H_14(double *state, double *unused, double *out_3980331647512007284) {
   out_3980331647512007284[0] = 0;
   out_3980331647512007284[1] = 0;
   out_3980331647512007284[2] = 0;
   out_3980331647512007284[3] = 0;
   out_3980331647512007284[4] = 0;
   out_3980331647512007284[5] = 0;
   out_3980331647512007284[6] = 1;
   out_3980331647512007284[7] = 0;
   out_3980331647512007284[8] = 0;
   out_3980331647512007284[9] = 0;
   out_3980331647512007284[10] = 0;
   out_3980331647512007284[11] = 0;
   out_3980331647512007284[12] = 0;
   out_3980331647512007284[13] = 0;
   out_3980331647512007284[14] = 0;
   out_3980331647512007284[15] = 0;
   out_3980331647512007284[16] = 0;
   out_3980331647512007284[17] = 0;
   out_3980331647512007284[18] = 0;
   out_3980331647512007284[19] = 0;
   out_3980331647512007284[20] = 0;
   out_3980331647512007284[21] = 0;
   out_3980331647512007284[22] = 0;
   out_3980331647512007284[23] = 0;
   out_3980331647512007284[24] = 0;
   out_3980331647512007284[25] = 1;
   out_3980331647512007284[26] = 0;
   out_3980331647512007284[27] = 0;
   out_3980331647512007284[28] = 0;
   out_3980331647512007284[29] = 0;
   out_3980331647512007284[30] = 0;
   out_3980331647512007284[31] = 0;
   out_3980331647512007284[32] = 0;
   out_3980331647512007284[33] = 0;
   out_3980331647512007284[34] = 0;
   out_3980331647512007284[35] = 0;
   out_3980331647512007284[36] = 0;
   out_3980331647512007284[37] = 0;
   out_3980331647512007284[38] = 0;
   out_3980331647512007284[39] = 0;
   out_3980331647512007284[40] = 0;
   out_3980331647512007284[41] = 0;
   out_3980331647512007284[42] = 0;
   out_3980331647512007284[43] = 0;
   out_3980331647512007284[44] = 1;
   out_3980331647512007284[45] = 0;
   out_3980331647512007284[46] = 0;
   out_3980331647512007284[47] = 0;
   out_3980331647512007284[48] = 0;
   out_3980331647512007284[49] = 0;
   out_3980331647512007284[50] = 0;
   out_3980331647512007284[51] = 0;
   out_3980331647512007284[52] = 0;
   out_3980331647512007284[53] = 0;
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
void pose_err_fun(double *nom_x, double *delta_x, double *out_7559082103702812710) {
  err_fun(nom_x, delta_x, out_7559082103702812710);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_1140165562353213950) {
  inv_err_fun(nom_x, true_x, out_1140165562353213950);
}
void pose_H_mod_fun(double *state, double *out_7826065894907355794) {
  H_mod_fun(state, out_7826065894907355794);
}
void pose_f_fun(double *state, double dt, double *out_8956214327200202745) {
  f_fun(state,  dt, out_8956214327200202745);
}
void pose_F_fun(double *state, double dt, double *out_475163589378229554) {
  F_fun(state,  dt, out_475163589378229554);
}
void pose_h_4(double *state, double *unused, double *out_1919401175009708031) {
  h_4(state, unused, out_1919401175009708031);
}
void pose_H_4(double *state, double *unused, double *out_897543215216634988) {
  H_4(state, unused, out_897543215216634988);
}
void pose_h_10(double *state, double *unused, double *out_4006136634733641093) {
  h_10(state, unused, out_4006136634733641093);
}
void pose_H_10(double *state, double *unused, double *out_8577831172590958455) {
  H_10(state, unused, out_8577831172590958455);
}
void pose_h_13(double *state, double *unused, double *out_655481990136332419) {
  h_13(state, unused, out_655481990136332419);
}
void pose_H_13(double *state, double *unused, double *out_2314730610115697813) {
  H_13(state, unused, out_2314730610115697813);
}
void pose_h_14(double *state, double *unused, double *out_3170072544189949915) {
  h_14(state, unused, out_3170072544189949915);
}
void pose_H_14(double *state, double *unused, double *out_3980331647512007284) {
  H_14(state, unused, out_3980331647512007284);
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
