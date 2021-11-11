/****************************************************************************
 * external/services/ril/at_client/at_client.c
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

#include <nuttx/list.h>
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
#include "at_client.h"
#include "at_log.h"

static const char *SOCK_NAME = "/dev/atil";

#define MAX_MSG_LENGTH 600
#define MAX_RETRY_NUM 5
#define LOG_TAG "at_client"

static sq_queue_t gATIndicationRegs;

static pthread_mutex_t ATIndicationRegListMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t gATRequestMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t gATClientFdMutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t gATPairListMutex = PTHREAD_MUTEX_INITIALIZER;

static int gClientId;

static struct list_node g_clientfd_id_pair_list = LIST_INITIAL_VALUE(g_clientfd_id_pair_list);

struct clientfd_id_pair_s
{
  int clientid;
  int clientfd;
  struct list_node list;
};

ATRequest *gAtRequest;

static void setTimespecRelative(struct timespec *p_ts, long long msec)
{
  struct timeval tv;

  gettimeofday(&tv, (struct timezone *) NULL);

  /* what's really funny about this is that I know
     pthread_cond_timedwait just turns around and makes this
     a relative time again */
  p_ts->tv_sec = tv.tv_sec + (msec / 1000);
  p_ts->tv_nsec = (tv.tv_usec + (msec % 1000) * 1000L ) * 1000L;
}

int pthreadCondWait(pthread_cond_t *waitCond, pthread_mutex_t *waitMutex, long long timeoutMsec)
{
  int err = 0;

  struct timespec ts;

  if (timeoutMsec != 0)
    {
      setTimespecRelative(&ts, timeoutMsec);
    }

  if (timeoutMsec != 0)
    {
      err = pthread_cond_timedwait(waitCond, waitMutex, &ts);
    }
  else
    {
      err = pthread_cond_wait(waitCond, waitMutex);
    }
  return err;
}

void at_c_response_free(ATResponse *p_response)
{
  int i;
  if (p_response == NULL)
    {
      return;
    }
  for (i = 0; i < p_response->lineNumber; i++)
    {
      if(p_response->lines[i])
        {
          free(p_response->lines[i]);
        }
    }
  if(p_response->lines)
    {
      free(p_response->lines);
    }
  free(p_response);
}

ATResponse *at_c_response_copy(ATResponse *p_response)
{
  int i;
  ATResponse *p_newResponse;

  if (p_response == NULL)
    {
      return NULL;
    }
  p_newResponse = (ATResponse *)malloc(sizeof(ATResponse));
  p_newResponse->error = p_response->error;
  p_newResponse->lineNumber = p_response->lineNumber;
  p_newResponse->finalResponse = p_response->finalResponse;
  p_newResponse->lines = (char **)malloc(p_newResponse->lineNumber * sizeof(char *));

  for (i = 0; i < p_response->lineNumber; i++)
    {
      if(p_response->lines[i])
        {
          p_newResponse->lines[i] = strdup(p_response->lines[i]);
        }
    }
  return p_newResponse;
}

void at_c_request_free(ATRequest *p_atRequest)
{
  if (p_atRequest == NULL)
    {
      return;
    }

  if (p_atRequest->response)
    {
      at_c_response_free(p_atRequest->response);
    }
  free(p_atRequest);
}


static int strStartsWith(const char *line, const char *prefix)
{
  for (; *line != '\0' && *prefix != '\0'; line++, prefix++)
    {
      if (*line != *prefix)
        {
          return 0;
        }
    }
  return *prefix == '\0';
}

static int sendRawRequest(int fd, const void *data, char size)
{
  int ret;
  do
    {
      ret = write(fd, (const uint8_t *)&size, sizeof(size));
    } while (ret < 0 && errno == EINTR);

  if (ret != sizeof(size))
    {
      return -1;
    }

  do
    {
      ret = write(fd, (const uint8_t *)data, size);
    } while (ret < 0 && errno == EINTR);

  if (ret != (int)size)
    {
      return -1;
    }
  return ret;
}

static int getClientFdFromList(int clientid)
{
  struct clientfd_id_pair_s *pair;

  pthread_mutex_lock(&gATPairListMutex);
  list_for_every_entry(&g_clientfd_id_pair_list, pair, struct clientfd_id_pair_s, list)
  {
    if (pair->clientid == clientid)
    {
      pthread_mutex_unlock(&gATPairListMutex);
      return pair->clientfd;
    }
  }
  pthread_mutex_unlock(&gATPairListMutex);

  return -1;
}

static struct clientfd_id_pair_s * getPairFromList(int clientid)
{
  struct clientfd_id_pair_s *pair;

  pthread_mutex_lock(&gATPairListMutex);
  list_for_every_entry(&g_clientfd_id_pair_list, pair, struct clientfd_id_pair_s, list)
  {
    if (pair->clientid == clientid)
    {
      pthread_mutex_unlock(&gATPairListMutex);
      return pair;
    }
  }
  pthread_mutex_unlock(&gATPairListMutex);

  return NULL;
}


int sendATRequest(int clientid, const char *ATLine, ATResponse **pp_outResponse)
{
  int ret;
  static int mserial = 0;
  ATRequest *atReq = NULL;
  int clientfd = getClientFdFromList(clientid);

  if (clientfd == -1)
    {
      rillog(LOG_ERR, "%s: Failed to get clientfd from clientid %d\n", __func__, clientid);
      return -1;
    }

  atReq = (ATRequest *)calloc(1, sizeof(ATRequest));
  if (atReq == NULL)
    {
      return -1;
    }

  atReq->serial = mserial++;
  atReq->clientid = clientid;

  atReq->response = (ATResponse *)calloc(1, sizeof(ATResponse));
  if (atReq->response == NULL)
    {
      free(atReq);
      return -1;
    }

  pthread_mutex_init(&atReq->s_commandmutex, NULL);
  pthread_cond_init(&atReq->s_commandcond, NULL);

  pthread_mutex_lock(&gATRequestMutex);
  gAtRequest = atReq;

  rillog(LOG_INFO, "%s %s: clientfd(%d): send command: %s\n", LOG_TAG, __func__, clientfd, ATLine);
  ret = sendRawRequest(clientfd, ATLine, strlen(ATLine));
  if (ret == -1)
    {
      at_c_request_free(gAtRequest);
      gAtRequest = NULL;
      pthread_mutex_unlock(&gATRequestMutex);
      return -1;
    }

  pthread_mutex_lock(&atReq->s_commandmutex);
  while (atReq->response->finalResponse == 0)
    {
      ret = pthreadCondWait(&atReq->s_commandcond, &atReq->s_commandmutex, 30000);
      if (ret == ETIMEDOUT)
        {
          pthread_mutex_unlock(&atReq->s_commandmutex);
          at_c_request_free(gAtRequest);
          gAtRequest = NULL;
          pthread_mutex_unlock(&gATRequestMutex);
          return -1;
        }
    }
  pthread_mutex_unlock(&atReq->s_commandmutex);
  ret = 0;

  if (pp_outResponse == NULL)
    {
      at_c_response_free(atReq->response);
    }
  else
    {
      *pp_outResponse = at_c_response_copy(atReq->response);
      at_c_response_free(atReq->response);
    }
  free(gAtRequest);
  gAtRequest = NULL;
  pthread_mutex_unlock(&gATRequestMutex);
  return ret;
}

int register_indication(int clientid, const char *s, atindication_handler handler)
{
  if (s == NULL)
    {
      rillog(LOG_ERR, "%s %s: register indication can't be NULL!", LOG_TAG, __func__);
      return -1;
    }
  ATIndicationReg *atIndicationReg = (ATIndicationReg *)calloc(1, sizeof(ATIndicationReg));
  if (atIndicationReg == NULL)
    {
      return -1;
    }

  atIndicationReg->clientid = clientid;
  atIndicationReg->prefix = strdup(s);
  if(atIndicationReg->prefix == NULL)
    {
      rillog(LOG_ERR, "%s %s: Not enough memory for indication register", LOG_TAG, __func__);
      free(atIndicationReg);
      return -1;
    }
  atIndicationReg->handler = handler;

  pthread_mutex_lock(&ATIndicationRegListMutex);
  sq_addlast(&(atIndicationReg->sqNode), &gATIndicationRegs);
  pthread_mutex_unlock(&ATIndicationRegListMutex);

  return 0;
}

int deregister_indication(int clientid, const char *s)
{
  int ret = -1;

  ATIndicationReg *atIndicationReg, *atIndicationReg_next;

  pthread_mutex_lock(&ATIndicationRegListMutex);
  atIndicationReg = (ATIndicationReg*)(gATIndicationRegs.head);
  while (atIndicationReg)
    {
      if (atIndicationReg->clientid == clientid &&
          strcmp(atIndicationReg->prefix, s) == 0)
        {
          sq_rem(&(atIndicationReg->sqNode), &gATIndicationRegs);
          break;
        }
      atIndicationReg_next = (ATIndicationReg*)sq_next(&(atIndicationReg->sqNode));
      atIndicationReg = atIndicationReg_next;
    }
  pthread_mutex_unlock(&ATIndicationRegListMutex);
  return ret;
}

static int readATResponse(int fd, char *buffer)
{
  int count;
  size_t msgLen;
  char *p = buffer;
  int remain = 2;
  do
    {
      do
        {
          count = read(fd, p, remain);
        } while (count < 0 && errno == EINTR);
      if(count <= 0)
        {
          return -1;
        }
      p += count;
      remain -= count;
    } while (remain > 0);

  msgLen = (buffer[1] << 8) + buffer[0];
  if (msgLen > MAX_MSG_LENGTH)
    {
      rillog(LOG_ERR, "%s %s: message is too long, truncate it!", LOG_TAG, __func__);
      msgLen = MAX_MSG_LENGTH ;
    }

  p = buffer;
  remain = msgLen;
  do
    {
      do
        {
          count = read(fd, p, remain);
        } while (count < 0 && errno == EINTR);
      if (count <= 0)
        {
          return -1;
        }
      p += count;
      remain -= count;
    } while (remain > 0);
  return msgLen;
}

static void notifyIndication(int clientid, char *str)
{
  ATIndicationReg *atIndicationReg;
  pthread_mutex_lock(&ATIndicationRegListMutex);

  for (atIndicationReg = (ATIndicationReg*)(gATIndicationRegs.head);
            atIndicationReg;
            atIndicationReg = (ATIndicationReg*)sq_next(&(atIndicationReg->sqNode)))
    {
      if (atIndicationReg->clientid == clientid &&
          strStartsWith(str, atIndicationReg->prefix))
        {
          if(atIndicationReg->handler)
            {
              atIndicationReg->handler(str);
            }
          break;
        }
    }
  pthread_mutex_unlock(&ATIndicationRegListMutex);
}

static char * findNextEOL(char *cur)
{
  while (*cur != '\0' && *cur != '\r' && *cur != '\n')
    {
      cur++;
    }

  return *cur == '\0' ? NULL : cur;
}

static const char *getLine(char **line)
{
  char *p_eol = NULL;
  char *ret = NULL;

  if (**line == '\0')
    {
      return NULL;
    }

  while (**line == '\r' || **line == '\n')
    {
      (*line)++;
    }
  p_eol = findNextEOL(*line);

  if (p_eol)
    {
      ret = (*line);
      *p_eol = '\0';
      *line = p_eol + 1;
      while (**line == '\r' || **line == '\n')
        {
          (*line)++;
        }
    }
  return ret;
}
static void *readerLoop(void *arg)
{
  int clientid = (int)(long)arg;
  char buffer[MAX_MSG_LENGTH];
  int buflen;
  ATRequest *atReq;
  int clientfd = getClientFdFromList(clientid);

  if (clientfd == -1)
    {
      rillog(LOG_ERR, "%s: Failed to get clientfd from clientid %d\n", __func__, clientid);
      return NULL;
    }

  snprintf(buffer, sizeof(buffer), "atClientReadLoop-%d", clientid);
  pthread_setname_np(pthread_self(), buffer);
  for (;;)
    {
      memset(buffer, 0x0, sizeof(buffer));
      buflen = readATResponse(clientfd, buffer);
      if (buflen < 0)
        {
          rillog(LOG_ERR, "%s End of Stream\n", LOG_TAG);
          break;
        }
      rillog(LOG_INFO, "%s readerLoop read %d,%d\n", LOG_TAG, buflen, buffer[0]);
      if (buffer[0] == AT_RESPONSE_TYPE_SOLICITED)
        {
          if (gAtRequest != NULL)
            {
              int i;
              char *line;
              atReq = gAtRequest;
              atReq->response->error = buffer[1];

              if (atReq->response->error == 0 && buflen > 2 && buffer[2] != 0)
                {
                  atReq->response->lineNumber = buffer[2];
                  atReq->response->lines = (char **)malloc(atReq->response->lineNumber * sizeof(char *));
                  line = buffer + 3;
                  rillog(LOG_INFO, "%s readerLoop read %s\n", LOG_TAG, line);
                  for(i = 0; i < atReq->response->lineNumber; i++)
                    {
                      atReq->response->lines[i] = strdup(getLine(&line));
                      rillog(LOG_INFO, "%s readerLoop read line %d,%s\n", LOG_TAG, i, atReq->response->lines[i]);
                    }
                }
              pthread_mutex_lock(&atReq->s_commandmutex);
              atReq->response->finalResponse = 1;
              pthread_cond_signal(&atReq->s_commandcond);
              pthread_mutex_unlock(&atReq->s_commandmutex);
            }
          else
            {
              rillog(LOG_ERR, "%s %s: There's no request corresponding to this response", LOG_TAG, __func__);
            }
        }
      else
        {
          char *s;
          s = (char *)malloc(buflen);
          memset(s, 0x0, buflen);
          memcpy(s, buffer + 1, buflen - 1);
          rillog(LOG_INFO, "%s clientfd(%d): receive indication: %s", LOG_TAG, clientfd, s);
          notifyIndication(clientid, s);
          free(s);
        }
    }
  return NULL;
}

int at_client_open(void)
{
  int ret;
  pthread_t atclient;
  int clientid;
  int sockfd;
  int len;
  int retryNum = 0;
  struct sockaddr_un address;
  struct clientfd_id_pair_s *pair;

  sockfd = socket(AF_UNIX, SOCK_STREAM, 0);
  address.sun_family = AF_UNIX;
  strcpy(address.sun_path, SOCK_NAME);
  len = sizeof(address);

  do {
    ret = connect(sockfd, (struct sockaddr *)&address, len);
    if (ret != 0)
      {
        retryNum++;
        usleep(100000);
      }
  } while (ret != 0 && retryNum < MAX_RETRY_NUM);
  if (ret != 0)
    {
      rillog(LOG_ERR, "%s %s: connect fail:%d,%s\n", LOG_TAG, __func__, errno, strerror(errno));
      return -1;
    }

  pthread_mutex_lock(&gATClientFdMutex);
  clientid = gClientId++;
  pthread_mutex_unlock(&gATClientFdMutex);

  pair = (struct clientfd_id_pair_s *)malloc(sizeof(*pair));
  if (pair == NULL)
    {
      rillog(LOG_ERR, "%s: Failed to alloc sock client pair!\n");
      close(sockfd);
      return -1;
    }

  pair->clientfd = sockfd;
  pair->clientid = clientid;
  pthread_mutex_lock(&gATPairListMutex);
  list_add_tail(&g_clientfd_id_pair_list, &pair->list);
  pthread_mutex_unlock(&gATPairListMutex);

  ret = pthread_create(&atclient, NULL, readerLoop, (void *)clientid);
  if (ret < 0)
    {
      rillog(LOG_ERR, "%s %s: thread create fail\n", LOG_TAG, __func__);
      close(sockfd);
      free(pair);
      return -1;
    }

  return pair->clientid;
}

int at_client_close(int clientid)
{
  ATIndicationReg *atIndicationReg, *atIndicationReg_next;
  struct clientfd_id_pair_s *pair = getPairFromList(clientid);
  int clientfd;

  if (!pair)
    {
      rillog(LOG_ERR, "%s: Failed to get pair from clientid %d\n", __func__, clientid);
      return clientfd;
    }

  clientfd = pair->clientfd;
  close(clientfd);
  pthread_mutex_lock(&gATPairListMutex);
  list_delete(&pair->list);
  pthread_mutex_unlock(&gATPairListMutex);
  free(pair);

  pthread_mutex_lock(&gATRequestMutex);
  if (gAtRequest != NULL && clientid == gAtRequest->clientid)
    {
      at_c_request_free(gAtRequest);
    }
  gAtRequest = NULL;
  pthread_mutex_unlock(&gATRequestMutex);
  pthread_mutex_lock(&ATIndicationRegListMutex);
  atIndicationReg = (ATIndicationReg*)(gATIndicationRegs.head);

  while (atIndicationReg)
    {
      atIndicationReg_next = (ATIndicationReg*)sq_next(&(atIndicationReg->sqNode));
      if (atIndicationReg->clientid == clientid)
        {
          sq_rem(&(atIndicationReg->sqNode), &gATIndicationRegs);
          free(atIndicationReg->prefix);
          free(atIndicationReg);
        }
      atIndicationReg = atIndicationReg_next;
    }
  pthread_mutex_unlock(&ATIndicationRegListMutex);

  return 0;
}
