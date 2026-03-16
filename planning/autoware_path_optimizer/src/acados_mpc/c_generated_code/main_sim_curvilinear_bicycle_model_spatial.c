/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/math.h"
#include "acados/utils/print.h"
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_curvilinear_bicycle_model_spatial.h"

#define NX CURVILINEAR_BICYCLE_MODEL_SPATIAL_NX
#define NZ CURVILINEAR_BICYCLE_MODEL_SPATIAL_NZ
#define NU CURVILINEAR_BICYCLE_MODEL_SPATIAL_NU
#define NP CURVILINEAR_BICYCLE_MODEL_SPATIAL_NP

int main()
{
  int status = 0;
  curvilinear_bicycle_model_spatial_sim_solver_capsule * capsule =
    curvilinear_bicycle_model_spatial_acados_sim_solver_create_capsule();
  status = curvilinear_bicycle_model_spatial_acados_sim_create(capsule);

  if (status) {
    printf("acados_create() returned status %d. Exiting.\n", status);
    exit(1);
  }

  sim_config * acados_sim_config = curvilinear_bicycle_model_spatial_acados_get_sim_config(capsule);
  sim_in * acados_sim_in = curvilinear_bicycle_model_spatial_acados_get_sim_in(capsule);
  sim_out * acados_sim_out = curvilinear_bicycle_model_spatial_acados_get_sim_out(capsule);
  void * acados_sim_dims = curvilinear_bicycle_model_spatial_acados_get_sim_dims(capsule);

  // initial condition
  double x_current[NX];
  x_current[0] = 0.0;
  x_current[1] = 0.0;

  x_current[0] = 0;
  x_current[1] = 0;

  // initial value for control input
  double u0[NU];
  u0[0] = 0.0;
  // set parameters
  double p[NP];
  p[0] = 0;
  p[1] = 0;
  p[2] = 0;
  p[3] = 0;
  p[4] = 0;
  p[5] = 0;
  p[6] = 0;
  p[7] = 0;
  p[8] = 0;
  p[9] = 0;
  p[10] = 0;
  p[11] = 0;
  p[12] = 0;
  p[13] = 0;
  p[14] = 0;
  p[15] = 0;
  p[16] = 0;
  p[17] = 0;
  p[18] = 0;
  p[19] = 0;
  p[20] = 0;
  p[21] = 0;
  p[22] = 0;
  p[23] = 0;
  p[24] = 0;
  p[25] = 0;
  p[26] = 0;
  p[27] = 0;
  p[28] = 0;
  p[29] = 0;
  p[30] = 0;
  p[31] = 0;
  p[32] = 0;
  p[33] = 0;
  p[34] = 0;
  p[35] = 0;
  p[36] = 0;
  p[37] = 0;
  p[38] = 0;
  p[39] = 0;
  p[40] = 0;
  p[41] = 0;
  p[42] = 0;
  p[43] = 0;
  p[44] = 0;
  p[45] = 0;
  p[46] = 0;
  p[47] = 0;
  p[48] = 0;
  p[49] = 0;
  p[50] = 0;
  p[51] = 0;
  p[52] = 0;
  p[53] = 0;
  p[54] = 0;
  p[55] = 0;
  p[56] = 0;
  p[57] = 0;
  p[58] = 0;
  p[59] = 0;
  p[60] = 0;
  p[61] = 0;
  p[62] = 0;
  p[63] = 0;
  p[64] = 0;
  p[65] = 0;
  p[66] = 0;
  p[67] = 0;
  p[68] = 0;
  p[69] = 0;
  p[70] = 0;
  p[71] = 0;
  p[72] = 0;
  p[73] = 0;
  p[74] = 0;
  p[75] = 0;
  p[76] = 0;
  p[77] = 0;
  p[78] = 0;
  p[79] = 0;
  p[80] = 0;
  p[81] = 0;
  p[82] = 0;
  p[83] = 0;
  p[84] = 0;
  p[85] = 0;
  p[86] = 0;
  p[87] = 0;
  p[88] = 0;
  p[89] = 0;
  p[90] = 0;
  p[91] = 0;
  p[92] = 0;
  p[93] = 0;
  p[94] = 0;
  p[95] = 0;
  p[96] = 0;
  p[97] = 0;
  p[98] = 0;
  p[99] = 0;
  p[100] = 0;
  p[101] = 0;
  p[102] = 0;
  p[103] = 0;
  p[104] = 0;
  p[105] = 0;
  p[106] = 0;
  p[107] = 0;
  p[108] = 0;
  p[109] = 0;
  p[110] = 0;
  p[111] = 0;
  p[112] = 0;
  p[113] = 0;
  p[114] = 0;
  p[115] = 0;
  p[116] = 0;
  p[117] = 0;
  p[118] = 0;
  p[119] = 0;
  p[120] = 0;
  p[121] = 0;
  p[122] = 0;
  p[123] = 0;
  p[124] = 0;
  p[125] = 0;
  p[126] = 0;
  p[127] = 0;
  p[128] = 0;
  p[129] = 0;
  p[130] = 0;
  p[131] = 0;
  p[132] = 0;
  p[133] = 0;
  p[134] = 0;
  p[135] = 0;
  p[136] = 0;
  p[137] = 0;
  p[138] = 0;
  p[139] = 0;
  p[140] = 0;
  p[141] = 0;
  p[142] = 0;
  p[143] = 0;
  p[144] = 0;
  p[145] = 0;
  p[146] = 0;
  p[147] = 0;
  p[148] = 0;
  p[149] = 0;
  p[150] = 0;
  p[151] = 0;
  p[152] = 0;
  p[153] = 0;
  p[154] = 0;
  p[155] = 0;
  p[156] = 0;
  p[157] = 0;
  p[158] = 0;
  p[159] = 0;
  p[160] = 0;
  p[161] = 0;
  p[162] = 0;
  p[163] = 0;
  p[164] = 0;
  p[165] = 0;
  p[166] = 0;
  p[167] = 0;
  p[168] = 0;
  p[169] = 0;
  p[170] = 0;
  p[171] = 0;
  p[172] = 0;
  p[173] = 0;
  p[174] = 0;
  p[175] = 0;
  p[176] = 0;
  p[177] = 0;
  p[178] = 0;
  p[179] = 0;
  p[180] = 0;
  p[181] = 0;
  p[182] = 0;
  p[183] = 0;
  p[184] = 0;
  p[185] = 0;
  p[186] = 0;
  p[187] = 0;
  p[188] = 0;
  p[189] = 0;
  p[190] = 0;
  p[191] = 0;
  p[192] = 0;
  p[193] = 0;
  p[194] = 0;
  p[195] = 0;
  p[196] = 0;
  p[197] = 0;
  p[198] = 0;
  p[199] = 0;
  p[200] = 0;
  p[201] = 0;
  p[202] = 0;
  p[203] = 0;
  p[204] = 0;
  p[205] = 0;
  p[206] = 0;
  p[207] = 0;
  p[208] = 0;
  p[209] = 0;
  p[210] = 0;
  p[211] = 0;
  p[212] = 0;
  p[213] = 0;
  p[214] = 0;
  p[215] = 0;
  p[216] = 0;
  p[217] = 0;
  p[218] = 0;
  p[219] = 0;
  p[220] = 0;
  p[221] = 0;
  p[222] = 0;
  p[223] = 0;
  p[224] = 0;
  p[225] = 0;
  p[226] = 0;
  p[227] = 0;
  p[228] = 0;
  p[229] = 0;
  p[230] = 0;
  p[231] = 0;
  p[232] = 0;
  p[233] = 0;
  p[234] = 0;
  p[235] = 0;
  p[236] = 0;
  p[237] = 0;
  p[238] = 0;
  p[239] = 0;
  p[240] = 0;
  p[241] = 0;
  p[242] = 0;
  p[243] = 0;
  p[244] = 0;
  p[245] = 0;
  p[246] = 0;
  p[247] = 0;
  p[248] = 0;
  p[249] = 0;
  p[250] = 0;
  p[251] = 0;
  p[252] = 0;
  p[253] = 0;
  p[254] = 0;
  p[255] = 0;
  p[256] = 0;
  p[257] = 0;
  p[258] = 0;
  p[259] = 0;
  p[260] = 0;
  p[261] = 0;
  p[262] = 0;
  p[263] = 0;
  p[264] = 0;
  p[265] = 0;
  p[266] = 0;
  p[267] = 0;
  p[268] = 0;
  p[269] = 0;
  p[270] = 0;
  p[271] = 0;
  p[272] = 0;
  p[273] = 0;
  p[274] = 0;
  p[275] = 0;
  p[276] = 0;
  p[277] = 0;
  p[278] = 0;
  p[279] = 0;
  p[280] = 0;
  p[281] = 0;
  p[282] = 0;
  p[283] = 0;
  p[284] = 0;
  p[285] = 0;
  p[286] = 0;
  p[287] = 0;
  p[288] = 0;
  p[289] = 0;
  p[290] = 0;
  p[291] = 0;
  p[292] = 0;
  p[293] = 0;
  p[294] = 0;
  p[295] = 0;
  p[296] = 0;
  p[297] = 0;
  p[298] = 0;
  p[299] = 0;
  p[300] = 0;
  p[301] = 0;
  p[302] = 0;
  p[303] = 0;
  p[304] = 0;
  p[305] = 0;
  p[306] = 0;
  p[307] = 0;
  p[308] = 0;
  p[309] = 0;
  p[310] = 0;
  p[311] = 0;
  p[312] = 0;
  p[313] = 0;
  p[314] = 0;
  p[315] = 0;
  p[316] = 0;
  p[317] = 0;
  p[318] = 0;
  p[319] = 0;
  p[320] = 0;
  p[321] = 0;
  p[322] = 0;
  p[323] = 0;
  p[324] = 0;
  p[325] = 0;
  p[326] = 0;
  p[327] = 0;
  p[328] = 0;
  p[329] = 0;
  p[330] = 0;
  p[331] = 0;
  p[332] = 0;
  p[333] = 0;
  p[334] = 0;
  p[335] = 0;
  p[336] = 0;
  p[337] = 0;
  p[338] = 0;
  p[339] = 0;
  p[340] = 0;
  p[341] = 0;
  p[342] = 0;
  p[343] = 0;
  p[344] = 0;
  p[345] = 0;
  p[346] = 0;
  p[347] = 0;
  p[348] = 0;
  p[349] = 0;
  p[350] = 0;
  p[351] = 0;
  p[352] = 0;
  p[353] = 0;
  p[354] = 0;
  p[355] = 0;
  p[356] = 0;
  p[357] = 0;
  p[358] = 0;
  p[359] = 0;
  p[360] = 0;
  p[361] = 0;
  p[362] = 0;
  p[363] = 0;
  p[364] = 0;
  p[365] = 0;
  p[366] = 0;
  p[367] = 0;
  p[368] = 0;
  p[369] = 0;
  p[370] = 0;
  p[371] = 0;
  p[372] = 0;
  p[373] = 0;
  p[374] = 0;
  p[375] = 0;
  p[376] = 0;
  p[377] = 0;
  p[378] = 0;
  p[379] = 0;
  p[380] = 0;
  p[381] = 0;
  p[382] = 0;
  p[383] = 0;
  p[384] = 0;
  p[385] = 0;
  p[386] = 0;
  p[387] = 0;
  p[388] = 0;
  p[389] = 0;
  p[390] = 0;
  p[391] = 0;
  p[392] = 0;
  p[393] = 0;
  p[394] = 0;
  p[395] = 0;
  p[396] = 0;
  p[397] = 0;
  p[398] = 0;
  p[399] = 0;
  p[400] = 0;
  p[401] = 0;
  p[402] = 0;
  p[403] = 0;
  p[404] = 0;
  p[405] = 0;
  p[406] = 0;
  p[407] = 0;
  p[408] = 0;
  p[409] = 0;
  p[410] = 0;
  p[411] = 0;
  p[412] = 0;
  p[413] = 0;
  p[414] = 0;
  p[415] = 0;
  p[416] = 0;
  p[417] = 0;
  p[418] = 0;
  p[419] = 0;
  p[420] = 0;
  p[421] = 0;
  p[422] = 0;
  p[423] = 0;
  p[424] = 0;
  p[425] = 0;
  p[426] = 0;
  p[427] = 0;
  p[428] = 0;
  p[429] = 0;
  p[430] = 0;
  p[431] = 0;
  p[432] = 0;
  p[433] = 0;
  p[434] = 0;
  p[435] = 0;
  p[436] = 0;
  p[437] = 0;
  p[438] = 0;
  p[439] = 0;
  p[440] = 0;
  p[441] = 0;
  p[442] = 0;
  p[443] = 0;
  p[444] = 0;
  p[445] = 0;
  p[446] = 0;
  p[447] = 0;
  p[448] = 0;
  p[449] = 0;
  p[450] = 0;
  p[451] = 0;
  p[452] = 0;
  p[453] = 0;
  p[454] = 0;
  p[455] = 0;
  p[456] = 0;
  p[457] = 0;
  p[458] = 0;
  p[459] = 0;
  p[460] = 0;
  p[461] = 0;
  p[462] = 0;
  p[463] = 0;
  p[464] = 0;
  p[465] = 0;
  p[466] = 0;
  p[467] = 0;
  p[468] = 0;
  p[469] = 0;
  p[470] = 0;
  p[471] = 0;
  p[472] = 0;
  p[473] = 0;
  p[474] = 0;
  p[475] = 0;
  p[476] = 0;
  p[477] = 0;
  p[478] = 0;
  p[479] = 0;
  p[480] = 0;
  p[481] = 0;
  p[482] = 0;
  p[483] = 0;
  p[484] = 0;
  p[485] = 0;
  p[486] = 0;
  p[487] = 0;
  p[488] = 0;
  p[489] = 0;
  p[490] = 0;
  p[491] = 0;
  p[492] = 0;
  p[493] = 0;
  p[494] = 0;
  p[495] = 0;
  p[496] = 0;
  p[497] = 0;
  p[498] = 0;
  p[499] = 0;
  p[500] = 0;
  p[501] = 0;
  p[502] = 0;
  p[503] = 0;
  p[504] = 0;
  p[505] = 0;
  p[506] = 0;
  p[507] = 0;
  p[508] = 0;
  p[509] = 0;
  p[510] = 0;
  p[511] = 0;
  p[512] = 0;
  p[513] = 0;
  p[514] = 0;
  p[515] = 0;
  p[516] = 0;

  curvilinear_bicycle_model_spatial_acados_sim_update_params(capsule, p, NP);

  int n_sim_steps = 3;
  // solve ocp in loop
  for (int ii = 0; ii < n_sim_steps; ii++) {
    // set inputs
    sim_in_set(acados_sim_config, acados_sim_dims, acados_sim_in, "x", x_current);
    sim_in_set(acados_sim_config, acados_sim_dims, acados_sim_in, "u", u0);

    // solve
    status = curvilinear_bicycle_model_spatial_acados_sim_solve(capsule);
    if (status != ACADOS_SUCCESS) {
      printf("acados_solve() failed with status %d.\n", status);
    }

    // get outputs
    sim_out_get(acados_sim_config, acados_sim_dims, acados_sim_out, "x", x_current);

    // print solution
    printf("\nx_current, %d\n", ii);
    for (int jj = 0; jj < NX; jj++) {
      printf("%e\n", x_current[jj]);
    }
  }

  printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

  // free solver
  status = curvilinear_bicycle_model_spatial_acados_sim_free(capsule);
  if (status) {
    printf("curvilinear_bicycle_model_spatial_acados_sim_free() returned status %d. \n", status);
  }

  curvilinear_bicycle_model_spatial_acados_sim_solver_free_capsule(capsule);

  return status;
}
