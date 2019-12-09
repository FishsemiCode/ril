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

int get_version(int clientfd, at_api_version *pversion)
{
  ATResponse *response = NULL;
  int ret;
  char *line;
  memset(pversion, 0x0, sizeof(at_api_version));
  ret = sendATRequest(clientfd, "AT+CGMR", &response);
  if (ret < 0 || response->error != NONE_ERROR)
    {
      ret = -1;
      goto clean;
    }
  line = response->lines[0];
  strncpy(pversion->version, line, sizeof(pversion->version) - 1);
  ret = 0;
clean:
  at_c_response_free(response);
  return ret;
}

int set_singalstrengthindicationstatus(int clientfd, int status)
{
  ATResponse *response = NULL;
  int ret;
  char buf[11] = {0};
  snprintf(buf, sizeof(buf), "AT*CSQ=%d", status);
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

int get_simstatus(int clientfd, SIM_STATUS *pstatus)
{
  ATResponse *response = NULL;
  int ret;
  char *line;
  int value;
  *pstatus = SIM_STATUS_UNKNOWN;
  ret = sendATRequest(clientfd, "AT^SIMST?", &response);
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
  ret = at_tok_nextint(&line, &value);
  if (ret < 0)
    {
      goto clean;
    }

  if (value == 1)
    {
      *pstatus = SIM_STATUS_EXIST;
    }
clean:
  at_c_response_free(response);
  return ret;
}

int get_currentoper(int clientfd, at_api_curroper *pcurroper)
{
  ATResponse *response = NULL;
  int ret;
  char *line;
  int ignore;
  int format;
  char *p;
  char buf[12] = {0};

  memset(pcurroper, 0x0, sizeof(at_api_curroper));
  for (format = 0; format < 3; format++)
    {
      snprintf(buf, sizeof(buf), "AT+COPS=3,%d", format);
      ret = sendATRequest(clientfd, buf, &response);
      if (ret < 0 || response->error != NONE_ERROR)
      {
        ret = -1;
        goto clean;
      }
      at_c_response_free(response);

      ret = sendATRequest(clientfd, "AT+COPS?", &response);
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
      ret = at_tok_nextint(&line, &ignore);
      if (ret < 0)
      {
        goto clean;
      }
      ret = at_tok_nextint(&line, &ignore);
      if (ret < 0)
      {
        goto clean;
      }
      ret = at_tok_nextstr(&line, &p);
      if (ret < 0)
      {
        goto clean;
      }
      if (format == 0)
        {
          strncpy(pcurroper->fullName, p, sizeof(pcurroper->fullName) - 1);
        }
      else if (format == 1)
        {
          strncpy(pcurroper->shortName, p, sizeof(pcurroper->shortName) - 1);
        }
      else
        {
          strncpy(pcurroper->numeric, p, sizeof(pcurroper->numeric) - 1);
        }
      at_c_response_free(response);
      response = NULL;
    }
clean:
  at_c_response_free(response);
  return ret;
}

static int parse_pdpcontexstlist(char *line, int *pCid, char **pType, char **pApn, char **pAddress, char **pDns)
{
  int err;
  char *out, *type, *apn, *address;

  type = apn = address = NULL;

  err = at_tok_start(&line);
  if (err < 0)
    {
      goto error;
    }

  err = at_tok_nextint(&line, pCid);
  if (err < 0)
    {
      goto error;
    }

  err = at_tok_nextstr(&line, &out);
  if (err < 0)
    {
      goto error;
    }
  if (pType != NULL && out[0])
    {
      type = strdup(out);
      *pType = type;
    }

  err = at_tok_nextstr(&line, &out);
  if (err < 0)
    {
      goto error;
    }
  if (pApn != NULL && out[0])
    {
      apn = strdup(out);
      *pApn = apn;
    }

  err = at_tok_nextstr(&line, &out);
  if (err < 0)
    {
      goto error;
    }
  if (pAddress != NULL && out[0])
    {
      address = strdup(out);
      *pAddress = address;
    }
  return 0;

error:
  if (type)
    {
      free(type);
    }
  if (apn)
    {
      free(apn);
    }
  if (address)
    {
      free(address);
    }
  if (pType)
    {
      *pType = NULL;
    }
  if (pApn)
    {
      *pApn = NULL;
    }
  if (pAddress)
    {
      *pAddress = NULL;
    }
  return err;
}

static at_api_pdpcontexinfo *new_pdpcontextinfo(int cid, char *pdp_type, char *apn, char *address)
{
  at_api_pdpcontexinfo *p = (at_api_pdpcontexinfo *)malloc(sizeof(at_api_pdpcontexinfo));
  p->cid = cid;
  p->pdp_type = pdp_type;
  p->apn = apn;
  p->address = address;
  return p;
}


int get_pdpcontextinfolist(int clientfd, at_api_pdpcontexinfo ***pppdpcontextinfoarray,
    int *parraysize)
{
  int ret;
  ATResponse *response = NULL;
  char **p_cur;
  int index = 0;

  *parraysize = 0;

  ret = sendATRequest(clientfd, "AT+CGDCONT?", &response);
  if (ret < 0 || response->error != NONE_ERROR)
    {
      ret = -1;
      goto clean;
    }

  p_cur = response->lines;
  for (; index < response->lineNumber; p_cur++)
    {
      int cid;
      char *pdp_type = NULL, *apn = NULL, *address = NULL;
      char *line = *p_cur;

      ret = parse_pdpcontexstlist(line, &cid, &pdp_type, &apn, &address, NULL);

      *pppdpcontextinfoarray = (at_api_pdpcontexinfo **)malloc(MAX_DATA_CALLS * sizeof(at_api_pdpcontexinfo *));
      memset(*pppdpcontextinfoarray, 0x0, MAX_DATA_CALLS * sizeof(at_api_pdpcontexinfo *));
      if (ret == 0)
        {
          (*pppdpcontextinfoarray)[index] = new_pdpcontextinfo(cid, pdp_type, apn, address);
        }
      *parraysize = (*parraysize + 1);
      index++;
    }
clean:
  at_c_response_free(response);
  return ret;
}

void free_pdpcontextinfolist(at_api_pdpcontexinfo **ppdpcontextinfoarray,
    int arraysize)
{
  int index = 0;
  for (; index < arraysize; index++)
    {
      if (ppdpcontextinfoarray[index])
        {
          if (ppdpcontextinfoarray[index]->pdp_type)
            {
              free(ppdpcontextinfoarray[index]->pdp_type);
            }
          if (ppdpcontextinfoarray[index]->apn)
            {
              free(ppdpcontextinfoarray[index]->apn);
            }
          if (ppdpcontextinfoarray[index]->address)
            {
              free(ppdpcontextinfoarray[index]->address);
            }
        }
    }
  if (arraysize > 0)
    {
      free(ppdpcontextinfoarray);
    }
}

static int definePDPContext(int clientfd, int cid, const char *protocol, const char *apn)
{
  int ret;
  ATResponse *response = NULL;
  char cmdString[50];

  snprintf(cmdString, sizeof(cmdString), "AT+CGDCONT=%d,\"%s\",\"%s\"", cid, protocol, apn);
  ret = sendATRequest(clientfd, cmdString, &response);
  if (ret < 0 || response->error != NONE_ERROR)
  {
    ret = -1;
    goto clean;
  }
clean:
  at_c_response_free(response);
  return ret;
}

static int activatePDPContext(int clientfd, int cid)
{
  int ret;
  ATResponse *response = NULL;
  char cmdString[15];

  snprintf(cmdString, sizeof(cmdString), "AT+CGACT=1,%d", cid);
  ret = sendATRequest(clientfd, cmdString, &response);
  if (ret < 0 || response->error != NONE_ERROR)
  {
    ret = -1;
    goto clean;
  }
clean:
  at_c_response_free(response);
  return ret;
}

int setup_datacall(int clientfd, at_api_setupdatacallreq *psetupdatacallreq, int *pCid)
{
  char *pTypeStr = NULL;
  int ret;
  int cid = 1;
  switch(psetupdatacallreq->pdpType)
    {
    case PDP_TYPE_IP:
      pTypeStr = "IP";
      break;
    case PDP_TYPE_IPV6:
      pTypeStr = "IPV6";
      break;
    case PDP_TYPE_IPV4V6:
      pTypeStr = "IPV4V6";
      break;
    default:
      pTypeStr = "UNKNOWN";
      syslog(LOG_ERR, "%s, invalid pdp type:%d\n", __func__, psetupdatacallreq->pdpType);
      return -1;
    }
  if (psetupdatacallreq->profile_type == DATA_PROFILE_CAT)
    {
      cid = 2;
    }
  ret = definePDPContext(clientfd, cid, pTypeStr, psetupdatacallreq->apn);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s, define pdp context error\n", __func__);
      return -1;
    }

  ret = activatePDPContext(clientfd, cid);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s, active pdp context error\n", __func__);
      return -1;
    }
  *pCid = cid;
  return ret;
}

int deactivate_pdpcontext(int clientfd, int cid)
{
  int ret;
  ATResponse *response = NULL;
  char cmdString[15];

  snprintf(cmdString, sizeof(cmdString), "AT+CGACT=0,%d", cid);
  ret = sendATRequest(clientfd, cmdString, &response);
  if (ret < 0 || response->error != NONE_ERROR)
  {
    ret = -1;
    goto clean;
  }
  at_c_response_free(response);
  snprintf(cmdString, sizeof(cmdString), "AT+CGDCONT=%d", cid);
  ret = sendATRequest(clientfd, cmdString, &response);
  if (ret < 0 || response->error != NONE_ERROR)
  {
    ret = -1;
    goto clean;
  }
clean:
  at_c_response_free(response);
  return ret;
}

int send_usat(int clientfd, char *data)
{
  ATResponse *response = NULL;
  int ret;
  char *buf;
  int len;
  if (data == NULL)
    {
      syslog(LOG_ERR, "%s: invalid param\n",  __func__);
      return -1;
    }
  len = strlen(data);
  buf = malloc(15 + len);
  memset(buf, 0x0, 15 + len);
  sprintf(buf, "AT+CSIM=%d,%s", len / 2, data);
  ret = sendATRequest(clientfd, buf, &response);

  if (ret < 0 || response->error != NONE_ERROR)
    {
      ret = -1;
      goto clean;
    }
clean:
  at_c_response_free(response);
  free(buf);
  return ret;
}


int release_signalconnection(int clientfd)
{
  int ret;
  at_api_pdpcontexinfo **ppdpcontextinfoarray = NULL;
  int size = 0;
  int index;
  ATResponse *response = NULL;
  char buf[20];
  ret = get_pdpcontextinfolist(clientfd, &ppdpcontextinfoarray, &size);
  if (ret < 0)
    {
      syslog(LOG_ERR, "%s: get pdp context info list error\n", __func__);
      free_pdpcontextinfolist(ppdpcontextinfoarray, size);
      return ret;
    }
  for (index = 0; index < size; index++)
    {
      if (ppdpcontextinfoarray[index] &&
        ppdpcontextinfoarray[index]->address != NULL &&
        ppdpcontextinfoarray[index]->address[0] != '\0')
        {
          sprintf(buf, "AT+CSODCP=%d,0,\"\",1", ppdpcontextinfoarray[index]->cid);
          ret = sendATRequest(clientfd, buf, &response);
          if (ret < 0 || response->error != NONE_ERROR)
            {
              ret = -1;
              goto clean;
            }
          at_c_response_free(response);
          response = NULL;
        }
    }
clean:
  free_pdpcontextinfolist(ppdpcontextinfoarray, size);
  at_c_response_free(response);
  return ret;
}