
/*****************************************************************************
* Microchip Graphics Library
* Graphics Display Designer (GDD) Template
*****************************************************************************
 
* Dependencies:    See INCLUDES section below
* Processor:       PIC24F, PIC24H, dsPIC, PIC32
* Compiler:        MPLAB C30 V3.22, MPLAB C32 V1.05b
* Linker:          MPLAB LINK30, MPLAB LINK32
* Company:         Microchip Technology Incorporated
*
* Software License Agreement
*
* Copyright (c) 2010 Microchip Technology Inc.  All rights reserved.
* Microchip licenses to you the right to use, modify, copy and distribute
* Software only when embedded on a Microchip microcontroller or digital
* signal controller, which is integrated into your product or third party
* product (pursuant to the sublicense terms in the accompanying license
* agreement).  
*
* You should refer to the license agreement accompanying this Software
* for additional information regarding your rights and obligations.
*
* SOFTWARE AND DOCUMENTATION ARE PROVIDED AS IS WITHOUT WARRANTY OF ANY
* KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY
* OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR
* PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR
* OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION,
 
* BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT
* DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL,
* INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA,
* COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY
* CLAIMS BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF),
* OR OTHER SIMILAR COSTS.
*
* Author               Date        Comment
*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
*
*****************************************************************************/


#ifndef    _GDD_SCREENS_H_
#define    _GDD_SCREENS_H_


/***************************************************
*FUNCTION DECLARATION
***************************************************/

void GDDDemoCreateFirstScreen(void);
void GDDDemoGOLDrawCallback(void);
void GDDDemoNextScreen(void);
void GDDDemoGoToScreen(int screenIndex);
void GDDDemoGOLMsgCallback(WORD objMsg, OBJ_HEADER *pObj, GOL_MSG *pMsg);


/***************************************************
*IMAGE DECLARATION
***************************************************/
extern const IMAGE_FLASH logo_small2_db2;
extern const IMAGE_FLASH No_Train;
extern const IMAGE_FLASH IR_Logo_small_4_DB2;
extern const IMAGE_FLASH log_bmp_blue_t_dark2;
extern const IMAGE_FLASH logo_small2_db2_inv;


/***************************************************
*FONT DECLARATION
***************************************************/
extern const FONT_FLASH Times_New_Roman_12;
extern const FONT_FLASH Handel_Gothic_22;
extern const FONT_FLASH Book_Antiqua_18;
extern const FONT_FLASH Monospaced_bold_Bold_14_1;
extern const FONT_FLASH Calibri_Light_18;
extern const FONT_FLASH Calibri_Bold_14;
extern const FONT_FLASH Gentium_16;


/***************************************************
*SCREEN DECLARATION
***************************************************/
void CreateHome(void);
void CreatePrimitivesForHome(void);
void CreateCPU_3D2S(void);
void CreateCPU_2DP(void);
void CreatePrimitivesForCPU_2DP(void);
void CreateCPU_3DP_AS(void);
void CreatePrimitivesForCPU_3DP_AS(void);
void CreateCPU_LCWS(void);
void CreateSMCPU(void);
void CreateRESET(void);
void CreateWAIT_FOR_RESET(void);
void CreateCPU_DE(void);
void CreatePrimitivesForCPU_DE(void);
void CreateCPU_4D1S(void);



/***************************************************
*UNIQUE WIDGET ID'S
***************************************************/
#define PCB_253 1
#define STE_611 2
#define STE_89 3
#define STE_90 4
#define STE_91 5
#define STE_92 6
#define STE_93 7
#define STE_94 8
#define STE_95 9
#define PCB_214 10
#define PCB_82 11
#define BTN_276 12
#define BTN_277 13
#define BTN_278 14
#define BTN_279 15
#define STE_281 16
#define STE_283 17
#define STE_284 18
#define STE_285 19
#define STE_286 20
#define STE_287 21
#define STE_289 22
#define STE_290 23
#define STE_291 24
#define STE_292 25
#define STE_293 26
#define STE_294 27
#define STE_312 28
#define STE_313 29
#define STE_314 30
#define STE_315 31
#define STE_316 32
#define STE_317 33
#define STE_325 34
#define STE_326 35
#define STE_327 36
#define STE_329 37
#define STE_330 38
#define STE_331 39
#define STE_334 40
#define STE_335 41
#define PCB_336 42
#define PCB_337 43
#define RDB_366 44
#define RDB_367 45
#define RDB_368 46
#define STE_503 47
#define STE_504 48
#define STE_505 49
#define STE_506 50
#define STE_507 51
#define STE_508 52
#define STE_509 53
#define STE_510 54
#define BTN_608 55
#define BTN_149 56
#define BTN_151 57
#define BTN_152 58
#define BTN_153 59
#define STE_157 60
#define STE_158 61
#define STE_159 62
#define STE_163 63
#define STE_164 64
#define STE_165 65
#define STE_166 66
#define STE_175 67
#define STE_176 68
#define STE_177 69
#define STE_179 70
#define PCB_204 71
#define PCB_205 72
#define PCB_215 73
#define STE_196 74
#define STE_302 75
#define STE_303 76
#define STE_304 77
#define STE_305 78
#define STE_306 79
#define STE_307 80
#define STE_308 81
#define STE_309 82
#define STE_266 83
#define STE_267 84
#define STE_269 85
#define STE_270 86
#define STE_272 87
#define STE_273 88
#define STE_274 89
#define RDB_439 90
#define RDB_484 91
#define STE_203 92
#define STE_204 93
#define STE_205 94
#define STE_206 95
#define STE_207 96
#define STE_208 97
#define STE_209 98
#define PCB_206 99
#define PCB_216 100
#define STE_338 101
#define STE_339 102
#define STE_340 103
#define STE_341 104
#define STE_350 105
#define STE_351 106
#define STE_352 107
#define STE_356 108
#define STE_357 109
#define STE_372 110
#define STE_373 111
#define STE_374 112
#define STE_376 113
#define STE_377 114
#define STE_378 115
#define BTN_379 116
#define BTN_380 117
#define BTN_381 118
#define BTN_382 119
#define STE_467 120
#define STE_468 121
#define STE_469 122
#define STE_470 123
#define STE_471 124
#define STE_472 125
#define STE_473 126
#define STE_474 127
#define STE_369 128
#define STE_370 129
#define STE_371 130
#define STE_375 131
#define PCB_377 132
#define PCB_378 133
#define PCB_379 134
#define RDB_380 135
#define RDB_381 136
#define RDB_382 137
#define BTN_610 138
#define STE_222 139
#define STE_223 140
#define STE_224 141
#define STE_225 142
#define STE_226 143
#define STE_227 144
#define STE_228 145
#define PCB_209 146
#define PCB_217 147
#define STE_384 148
#define STE_385 149
#define STE_386 150
#define STE_387 151
#define STE_388 152
#define STE_390 153
#define STE_391 154
#define STE_392 155
#define STE_393 156
#define STE_394 157
#define STE_395 158
#define STE_396 159
#define STE_397 160
#define STE_398 161
#define STE_399 162
#define STE_400 163
#define STE_401 164
#define STE_402 165
#define STE_403 166
#define STE_404 167
#define STE_405 168
#define STE_406 169
#define STE_407 170
#define STE_408 171
#define STE_410 172
#define STE_416 173
#define BTN_419 174
#define BTN_420 175
#define BTN_421 176
#define BTN_422 177
#define PCB_424 178
#define PCB_434 179
#define RDB_389 180
#define RDB_390 181
#define RDB_391 182
#define RDB_392 183
#define BTN_161 184
#define BTN_165 185
#define STE_167 186
#define STE_168 187
#define STE_170 188
#define STE_171 189
#define STE_172 190
#define STE_173 191
#define GRB_193 192
#define STE_243 193
#define STE_244 194
#define STE_245 195
#define STE_246 196
#define STE_247 197
#define STE_248 198
#define PCB_213 199
#define PCB_218 200
#define STE_169 201
#define STE_609 202
#define STE_610 203
#define STE_612 204
#define STE_613 205
#define BTN_201 206
#define STE_211 207
#define STE_249 208
#define STE_250 209
#define STE_252 210
#define STE_253 211
#define STE_254 212
#define STE_256 213
#define PCB_268 214
#define PCB_269 215
#define STE_419 216
#define STE_420 217
#define STE_421 218
#define STE_422 219
#define STE_423 220
#define STE_424 221
#define STE_425 222
#define STE_426 223
#define STE_427 224
#define STE_428 225
#define STE_429 226
#define STE_430 227
#define STE_431 228
#define STE_432 229
#define STE_433 230
#define STE_434 231
#define STE_437 232
#define STE_438 233
#define STE_439 234
#define STE_440 235
#define STE_441 236
#define STE_442 237
#define STE_443 238
#define STE_444 239
#define STE_445 240
#define STE_446 241
#define STE_447 242
#define STE_448 243
#define STE_449 244
#define STE_450 245
#define STE_451 246
#define STE_452 247
#define STE_453 248
#define STE_454 249
#define STE_455 250
#define STE_456 251
#define STE_457 252
#define STE_458 253
#define STE_459 254
#define STE_460 255
#define STE_461 256
#define STE_462 257
#define STE_463 258
#define STE_464 259
#define STE_465 260
#define STE_466 261
#define STE_475 262
#define STE_476 263
#define STE_477 264
#define STE_478 265
#define STE_479 266
#define STE_480 267
#define STE_481 268
#define STE_482 269
#define STE_483 270
#define BTN_484 271
#define BTN_485 272
#define BTN_486 273
#define BTN_487 274
#define STE_488 275
#define STE_489 276
#define STE_490 277
#define STE_491 278
#define STE_492 279
#define STE_493 280
#define STE_494 281
#define STE_495 282
#define STE_496 283
#define PCB_499 284
#define PCB_501 285
#define STE_511 286
#define STE_497 287
#define STE_501 288
#define STE_502 289
#define STE_417 290
#define STE_418 291
#define BTN_423 292
#define BTN_424 293
#define BTN_425 294
#define BTN_426 295
#define STE_484 296
#define STE_485 297
#define STE_486 298
#define STE_487 299
#define STE_498 300
#define STE_499 301
#define STE_500 302
#define STE_512 303
#define STE_513 304
#define STE_515 305
#define PCB_516 306
#define PCB_517 307
#define PCB_518 308
#define STE_519 309
#define STE_520 310
#define STE_521 311
#define STE_529 312
#define STE_530 313
#define STE_531 314
#define RDB_535 315
#define STE_528 316
#define STE_532 317
#define STE_533 318
#define STE_534 319
#define STE_535 320
#define STE_536 321
#define STE_537 322
#define PCB_538 323
#define PCB_539 324
#define STE_544 325
#define STE_545 326
#define STE_546 327
#define STE_549 328
#define BTN_555 329
#define BTN_556 330
#define BTN_557 331
#define BTN_558 332
#define STE_564 333
#define STE_567 334
#define STE_568 335
#define STE_583 336
#define STE_584 337
#define STE_585 338
#define STE_586 339
#define STE_587 340
#define STE_588 341
#define STE_589 342
#define STE_590 343
#define STE_591 344
#define STE_592 345
#define STE_593 346
#define STE_594 347
#define STE_595 348
#define STE_596 349
#define STE_597 350
#define STE_598 351
#define STE_599 352
#define STE_600 353
#define STE_601 354
#define STE_602 355
#define STE_603 356
#define STE_604 357
#define STE_605 358
#define RDB_608 359
#define RDB_610 360
#define PCB_611 361
#define RDB_612 362
#define PCB_613 363
#define RDB_614 364
#define STE_615 365
#define STE_616 366
#define BTN_609 367

#define	NUM_GDD_SCREENS 10
#endif
