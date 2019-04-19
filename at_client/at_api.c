/****************************************************************************
 * external/services/ril/at_client/at_api.c
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
#include <nuttx/config.h>

#include <nuttx/serial/pty.h>
#include <sys/select.h>
#include <sys/time.h>
#include <sys/un.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>
#include <fcntl.h>
#include <poll.h>
#include <sched.h>
#include <string.h>
#include <strings.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include "at_api.h"
#include "at_tok.h"

int set_radiopower(int clientfd, bool on)
{
  ATResponse *response = NULL;
  int ret;
  if (on)
    {
      ret = sendATRequest(clientfd, "AT+CFUN=1", &response);
    }
  else
    {
      ret = sendATRequest(clientfd, "AT+CFUN=0", &response);
    }
  if (ret < 0 || response->error != NONE_ERROR)
    {
      ret = -1;
      goto clean;
    }
  ret = 0;
clean:
  at_c_response_free(response);
  return ret;
}

int get_imei(int clientfd, at_spi_imei *pimei)
{
  ATResponse *response = NULL;
  int ret;
  char *line, *p;
  memset(pimei, 0x0, sizeof(at_spi_imei));
  ret = sendATRequest(clientfd, "AT+CGSN", &response);
  if (ret < 0 || response->error != NONE_ERROR)
    {
      ret = -1;
      goto clean;
    }
  line = response->lines[0];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }
  ret = at_tok_nextstr(&line, &p);
  if (ret < 0)
    {
      goto clean;
    }
  strcpy(pimei->imei, p);
  ret = 0;
clean:
  at_c_response_free(response);
  return ret;
}

int get_imsi(int clientfd, at_api_imsi *pimsi)
{
  ATResponse *response = NULL;
  int ret;
  char *line;
  memset(pimsi, 0x0, sizeof(at_api_imsi));
  ret = sendATRequest(clientfd, "AT+CIMI", &response);
  if (ret < 0 || response->error != NONE_ERROR)
    {
      ret = -1;
      goto clean;
    }
  line = response->lines[0];
  strcpy(pimsi->imsi, line);
  ret = 0;
clean:
  at_c_response_free(response);
  return ret;
}

int set_ceregindicationstatus(int clientfd, int status)
{
  ATResponse *response = NULL;
  int ret;
  char buf[11] = {0};
  snprintf(buf, sizeof(buf), "AT+CEREG=%d", status);
  ret = sendATRequest(clientfd, buf, &response);
  if (ret < 0 || response->error != NONE_ERROR)
    {
      ret = -1;
      goto clean;
    }
  ret = 0;
clean:
  at_c_response_free(response);
  return ret;
}

int get_cellinfo(int clientfd, at_api_cellinfo *pcellinfo)
{
  ATResponse *response = NULL;
  int ret;
  char *line;
  int value;
  memset(pcellinfo, 0x0, sizeof(at_api_cellinfo));
  ret = sendATRequest(clientfd, "AT+NCCELLINFO", &response);
  if (ret < 0 || response->error != NONE_ERROR)
    {
      ret = -1;
      goto clean;
    }
  line = response->lines[1];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }
  strncpy(pcellinfo->mcc, line, MCC_LENGTH);
  line = response->lines[2];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }
  strncpy(pcellinfo->mnc, line, MNC_LENGTH);
  line = response->lines[3];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }
  ret = at_tok_nextint(&line, &value);
  if (ret < 0)
    {
      goto clean;
    }
  pcellinfo->curr_mod = value;

  line = response->lines[4];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }

  pcellinfo->duplex_mode = DUPLEX_MODE_FDD;
  line = response->lines[5];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }
  ret = at_tok_nextint(&line, &(pcellinfo->ue_category));
  if (ret < 0)
    {
      goto clean;
    }
  line = response->lines[6];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }
  ret = at_tok_nexthexint(&line, &value);
  if (ret < 0)
    {
      goto clean;
    }
  pcellinfo->cellId = value;
  line = response->lines[7];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }
  ret = at_tok_nexthexint(&line, (int *)&(pcellinfo->lacId));
  if (ret < 0)
    {
      goto clean;
    }
  line = response->lines[8];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }
  ret = at_tok_nextint(&line, &(pcellinfo->rsrp));
  if (ret < 0)
    {
      goto clean;
    }

  line = response->lines[9];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }
  ret = at_tok_nextint(&line, &value);
  if (ret < 0)
    {
      goto clean;
    }
  pcellinfo->rsrq = value;
  line = response->lines[10];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }
  ret = at_tok_nextint(&line, &value);
  if (ret < 0)
    {
      goto clean;
    }
  pcellinfo->snr = value;
  line = response->lines[11];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }
  ret = at_tok_nexthexint(&line, &value);
  if (ret < 0)
    {
      goto clean;
    }
  pcellinfo->band = value;
  line = response->lines[12];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }
  ret = at_tok_nextint(&line, &value);
  if (ret < 0)
    {
      goto clean;
    }
  pcellinfo->arfcn = (unsigned int)value;
  line = response->lines[13];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }
  ret = at_tok_nextint(&line, &value);
  if (ret < 0)
    {
      goto clean;
    }
  pcellinfo->pci = value;
  ret = 0;

clean:
  at_c_response_free(response);
  return ret;
}

int start_gps(int clientfd)
{
  ATResponse *response = NULL;
  int ret;
  ret = sendATRequest(clientfd, "AT+PGNSS+$START", &response);
  if (ret < 0 || response->error != NONE_ERROR)
    {
      ret = -1;
      goto clean;
    }
  ret = 0;
clean:
  at_c_response_free(response);
  return ret;
}

int stop_gps(int clientfd)
{
  ATResponse *response = NULL;
  int ret;
  ret = sendATRequest(clientfd, "AT+PGNSS+$STOP", &response);
  if (ret < 0 || response->error != NONE_ERROR)
    {
      ret = -1;
      goto clean;
    }
  ret = 0;
clean:
  at_c_response_free(response);
  return ret;
}

int get_iccid(int clientfd, at_spi_iccid *piccid)
{
  ATResponse *response = NULL;
  int ret;
  char *line, *p;
  memset(piccid, 0x0, sizeof(at_spi_iccid));
  ret = sendATRequest(clientfd, "AT+NCCID", &response);
  if (ret < 0 || response->error != NONE_ERROR)
    {
      ret = -1;
      goto clean;
    }
  line = response->lines[0];
  ret = at_tok_start(&line);
  if (ret < 0)
    {
      goto clean;
    }
  ret = at_tok_nextstr(&line, &p);
  if (ret < 0)
    {
      goto clean;
    }
  strcpy(piccid->iccid, p);
  ret = 0;
clean:
  at_c_response_free(response);
  return ret;
}

int get_model(int clientfd, at_spi_model *pmodel)
{
  ATResponse *response = NULL;
  int ret;
  char *line;
  memset(pmodel, 0x0, sizeof(at_spi_model));
  ret = sendATRequest(clientfd, "AT+CGMM", &response);
  if (ret < 0 || response->error != NONE_ERROR)
    {
      ret = -1;
      goto clean;
    }
  line = response->lines[0];
  strncpy(pmodel->model, line, sizeof(pmodel->model) - 1);
  ret = 0;
clean:
  at_c_response_free(response);
  return ret;
}
