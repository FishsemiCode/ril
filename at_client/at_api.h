/****************************************************************************
 * external/services/ril/at_client/at_api.h
 *
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Pinecone <pinecone@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef AT_API_H
#define AT_API_H

#include "at_client.h"
#include "at_tok.h"

#define IMEI_LENGTH (15)
#define IMSI_LENGTH (15)
#define MCC_LENGTH (3)
#define MNC_LENGTH (3)
#define ICCID_LENGTH (20)
#define MODEL_LENGTH (16)
#define VERSION_LENGTH (20)

typedef struct
{
  char imei[IMEI_LENGTH + 1];
} at_spi_imei;


typedef struct
{
  char imsi[IMSI_LENGTH + 1];
} at_api_imsi;

typedef struct
{
  char iccid[ICCID_LENGTH + 1];
} at_spi_iccid;

typedef struct
{
  char model[MODEL_LENGTH + 1];
} at_spi_model;

typedef struct
{
  char version[VERSION_LENGTH + 1];
} at_api_version;


typedef enum
{
  DUPLEX_MODE_FDD,
  DUPLEX_MODE_TDD,
  DUPLEX_MODE_UNKNOWN,
} DUPLEX_MODE;

typedef enum
{
  CURR_MODE_GSM,
  CURR_MODE_WCDMA,
  CURR_MODE_LTE,
  CURR_MODE_TDS,
  CURR_MODE_CDMA,
  CURR_MODE_1xDO,
  CURR_MODE_1xLTE,
  CURR_MODE_NOne,
  CURR_MODE_UNKNOWN,
} CURR_MODE;


typedef struct
{
  char mcc[MCC_LENGTH + 1];
  char mnc[MNC_LENGTH + 1];
  CURR_MODE curr_mod;
  DUPLEX_MODE duplex_mode;
  int ue_category;
  unsigned short cellId;
  unsigned int lacId;
  int rsrp;
  short rsrq;
  short snr;
  unsigned char band;
  unsigned int arfcn;
  unsigned short pci;
} at_api_cellinfo;


int set_radiopower(int clientfd, bool on);
int get_imei(int clientfd, at_spi_imei *pimei);
int get_imsi(int clientfd, at_api_imsi *pimsi);
int set_ceregindicationstatus(int clientfd, int status);
int get_cellinfo(int clientfd, at_api_cellinfo *pcellinfo);
int start_gps(int clientfd);
int stop_gps(int clientfd);
int get_iccid(int clientfd, at_spi_iccid *piccid);
int get_model(int clientfd, at_spi_model *pmodel);
int get_version(int clientfd, at_api_version *pversion);
#endif
