/****************************************************************************
 * external/services/ril/at_client/at_client.h
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

#ifndef AT_CLIENT_H
#define AT_CLIENT_H

#include <pthread.h>

#define AT_CLIENT_TIMEOUT  (40 * 1000)

typedef void (*atindication_handler)(const char *s);

typedef enum
{
  NO_RESULT,
  NUMERIC,
  SINGLELINE,
  MULTILINE,
}ATCommandType;

typedef enum
{
  AT_RESPONSE_TYPE_SOLICITED,
  AT_RESPONSE_TYPE_UNSOLICITED,
  AT_RESPONSE_TYPE_NUM
}ATResponseType;

typedef enum
{
  NONE_ERROR,
  GENERIC_ERROR,
  CONNECTION_ERROR,
  NOSUPPORT_ERROR,
  TIMEOUT_ERROR,
}ATResponseError;

typedef struct
{
  int error;
  int finalResponse;
  int lineNumber;
  char **lines;
}ATResponse;

typedef struct ATRequest
{
  struct ATRequest *next;
  int serial;
  int clientid;
  ATResponse *response;
  pthread_cond_t s_commandcond;
  pthread_mutex_t s_commandmutex;
}ATRequest;

typedef struct ATIndicationReg
{
  sq_entry_t sqNode;
  int clientid;
  atindication_handler handler;
  char *prefix;
}ATIndicationReg;


int sendATRequest(int clientid, const char *ATLine, ATResponse **pp_outResponse);
int register_indication(int clientid, const char *s, atindication_handler handler);
int deregister_indication(int clientid, const char *s);


int at_client_open(void);
int at_client_close(int clientid);

void at_c_response_free(ATResponse *p_response);

#endif
