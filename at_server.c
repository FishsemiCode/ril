/****************************************************************************
 * external/services/ril/at_server.c
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
#include <sys/ioctl.h>
#include <nuttx/misc/misc_rpmsg.h>
#include <nuttx/board.h>

#include "at_client/at_log.h"
#include "at_client/at_tok.h"


#define MAX_AT_SIZE   600
#define TIMEOUT_DEFALUT (20 * 1000)
#define AT_COMMAND_TIMEOUT (20 * 1000)

#define AT_RESPONSE_ERROR_NONE 0
#define AT_RESPONSE_ERROR_GENERIC 1
#define AT_RESPONSE_ERROR_CONNECTION 2
#define AT_RESPONSE_ERROR_NOSUPPORT 3

#define AT_RESPONSE_TYPE_SOLICITED 0
#define AT_RESPONSE_TYPE_UNSOLICITED 1

#define AT_ERROR_GENERIC -1
#define AT_ERROR_COMMAND_PENDING -2
#define AT_ERROR_CHANNEL_CLOSED -3
#define AT_ERROR_TIMEOUT -4

#define ATSERVER_UART_MODEM         0
#define ATSERVER_UART_GPS           1

#define ATSERVER_NUARTS 2

#define NUM_ELEMS(x) (sizeof(x) / sizeof(x[0]))
#define LOG_TAG "at_server"

#define GPS_MAX_DURATION_TIME (10392)  //2.92h minus 2min

typedef struct
{
  const char *mSocketName;
  int mSock;
  int mClientToken;
  pthread_t mThread;
  sq_queue_t mClients;
  pthread_mutex_t mClientsLock;
}ATServer;

typedef struct
{
  sq_entry_t mClientLcNode;
  unsigned int mToken;
  int mfd;
}ATClient;

typedef struct
{
  sq_entry_t mNode;
  ATClient *mAtCLient;
}ATPendingClient;


typedef enum
{
  NO_RESULT,
  NUMERIC,
  SINGLELINE,
  MULTILINE
}ATCommandType;

typedef struct ATLine
{
  struct ATLine *p_next;
  char *line;
}ATLine;

typedef struct
{
  int success;
  char *finalResponse;
  ATLine  *p_intermediates;
}ATResponse;

typedef struct
{
  char *p_atcmd;
  char *p_responsePrefix;
  ATCommandType eATcmdType;
  int uartIndex;
}atcmd_table_s;

typedef struct
{
  int mfd;
  char mATBuffer[MAX_AT_SIZE + 1];
  char *mATBufferCur;
  ATResponse *mSpResponse;
  pthread_mutex_t mCommandmutex;
  pthread_cond_t mCommandcond;
  ATCommandType mCommandType;
  const char *mResponsePrefix;
}ATUartS;

static const char *gATServerUartNames[ATSERVER_NUARTS] =
{
  "/dev/ttyAT1",
  "/dev/ttyGPS1",
};

static const atcmd_table_s g_atcmd[] =
{
  {"AT+CFUN=", "",   NO_RESULT, ATSERVER_UART_MODEM},
  {"AT+CEREG", "",   NO_RESULT, ATSERVER_UART_MODEM},
  {"AT+NCCELLINFO", "+NCCELLINFO",   MULTILINE, ATSERVER_UART_MODEM},
  {"AT+CGSN", "+CGSN",   SINGLELINE, ATSERVER_UART_MODEM},
  {"AT+CIMI", NULL,   NUMERIC, ATSERVER_UART_MODEM},
  {"AT+PGNSS", NULL,   NO_RESULT, ATSERVER_UART_GPS},
  {"AT+NCCID", "+NCCID",   SINGLELINE, ATSERVER_UART_MODEM},
  {"AT+CGMM", NULL,   NUMERIC, ATSERVER_UART_MODEM},
  {"AT+CGMR", NULL,   NUMERIC, ATSERVER_UART_MODEM},
  {"AT*CSQ", NULL,   NO_RESULT, ATSERVER_UART_MODEM},
  {"AT^SIMST", "^SIMST",  SINGLELINE, ATSERVER_UART_MODEM},
  {"AT+COPS=", NULL,   NO_RESULT, ATSERVER_UART_MODEM},
  {"AT+COPS?", "+COPS",   SINGLELINE, ATSERVER_UART_MODEM},
  {"AT+CGDCONT?", "+CGDCONT",   MULTILINE, ATSERVER_UART_MODEM},
  {"AT+CGDCONT=", NULL,   NO_RESULT, ATSERVER_UART_MODEM},
  {"AT+CGDATA=", NULL,   NO_RESULT, ATSERVER_UART_MODEM},
  {"AT+CGACT=", NULL,   NO_RESULT, ATSERVER_UART_MODEM},
  {"AT+CSIM=", NULL,   NO_RESULT, ATSERVER_UART_MODEM},
  {"AT+CSODCP=", NULL,   NO_RESULT, ATSERVER_UART_MODEM},
  {"AT+NUSATRSH=", NULL,   NO_RESULT, ATSERVER_UART_MODEM},
  {"AT+CFUN?", "",   SINGLELINE, ATSERVER_UART_MODEM},
  {"AT+WRQRY?", "+WRQRY",   SINGLELINE, ATSERVER_UART_MODEM},
  {"AT+SETPWRMAX=", "",   SINGLELINE, ATSERVER_UART_MODEM},
  {"AT+SETPWRMAX?", "",   SINGLELINE, ATSERVER_UART_MODEM},
  {"AT+ESMDATABUF?", "",   SINGLELINE, ATSERVER_UART_MODEM},
};

ATUartS gATUarts[ATSERVER_NUARTS];
ATServer *gServer;

pthread_mutex_t *gModemReadymutex = NULL;
pthread_cond_t *gModemReadycond = NULL;
static bool gModemReady = false;

static timer_t gGpsTimerid = NULL;
static int gGpsTimeoutPipe[2];
static int gIndicationDisable = false;
static int gGpsStart = false;
static struct file gGpsPipeFileWrite;


#ifdef CONFIG_CAN_PASS_STRUCTS
static void gpsDurationTimeout(union sigval value)
#else
static void gpsDurationTimeout(FAR void *sival_ptr)
#endif
{
#ifdef CONFIG_CAN_PASS_STRUCTS
  int sival_int = value.sival_int;
#else
  int sival_int = (int)((intptr_t)sival_ptr);
#endif

  int ret = file_write(&gGpsPipeFileWrite, " ", 1);
  rillog(LOG_INFO, "%s %s write:%d,%d,%d\n", LOG_TAG, __func__, ret, errno, sival_int);
}


static void stopGpsDurationTime(void)
{
  if (gGpsTimerid)
    {
      timer_delete(gGpsTimerid);
      gGpsTimerid = NULL;
    }
  rillog(LOG_INFO, "%s %s\n", LOG_TAG, __func__);
}


static int startGpsDurationTime(void)
{
  int ret = OK;
  if (!gGpsTimerid)
    {
      struct sigevent notify;
      struct itimerspec timer;
      notify.sigev_notify            = SIGEV_THREAD;
      notify.sigev_signo             = 0;
      notify.sigev_value.sival_int   = 0;
      notify.sigev_notify_function   = (void *)gpsDurationTimeout;
      notify.sigev_notify_attributes = NULL;

      ret = timer_create(CLOCK_REALTIME, &notify, &gGpsTimerid);
      if (ret != OK)
        {
          rillog(LOG_ERR, "%s %s: timer_create failed, errno=%d\n", LOG_TAG, __func__, errno);
          return ret;
        }
      timer.it_value.tv_sec     = GPS_MAX_DURATION_TIME;
      timer.it_value.tv_nsec    = 0;
      timer.it_interval.tv_sec  = GPS_MAX_DURATION_TIME;
      timer.it_interval.tv_nsec = 0;

      ret = timer_settime(gGpsTimerid, 0, &timer, NULL);
      if (ret != OK)
        {
          rillog(LOG_ERR, "%s %s: timer_settime failed, errno=%d\n", LOG_TAG, __func__, errno);
          stopGpsDurationTime();
        }
      rillog(LOG_INFO, "%s %s\n", LOG_TAG, __func__);
    }
  return ret;
}



static char * findNextEOL(char *cur)
{
  if (cur[0] == '>' && cur[1] == ' ' && cur[2] == '\0')
    {
      return cur + 2;
    }

  while (*cur != '\0' && *cur != '\r' && *cur != '\n')
    {
      cur++;
    }

  return *cur == '\0' ? NULL : cur;
}

static int findInt(char *cur)
{
  int num = 0;
  while (*cur != '\0' && *cur != '\r' && *cur != '\n')
    {
      if ((*cur >= '0') && (*cur <= '9'))
        {
          num = num * 10 + *cur - '0';
        }
      cur++;
    }

  return num;
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

static int strCaseStartsWith(const char *line, const char *prefix)
{
  for (; *line != '\0' && *prefix != '\0'; line++, prefix++)
    {
      if (toupper(*line) != toupper(*prefix))
        {
          return 0;
        }
    }
  return *prefix == '\0';
}

static int isFinalResponseSuccess(const char *line)
{
  static const char * s_finalResponsesSuccess[] =
    {
      "OK",
      "$OK"
    };
  size_t i;

  for (i = 0; i < NUM_ELEMS(s_finalResponsesSuccess); i++)
    {
      if (strStartsWith(line, s_finalResponsesSuccess[i]))
        {
          return 1;
        }
    }

  return 0;
}

static int isFinalResponseError(const char *line)
{
  static const char * s_finalResponsesError[] =
    {
      "ERROR",
      "+CMS ERROR:",
      "+CME ERROR:",
      "$ERROR"
    };
  size_t i;

  for (i = 0; i < NUM_ELEMS(s_finalResponsesError); i++)
    {
      if (strCaseStartsWith(line, s_finalResponsesError[i]))
        {
          return 1;
        }
    }
  return 0;
}

static ATResponse * at_response_new(void)
{
  return (ATResponse *)calloc(1, sizeof(ATResponse));
}

static void at_response_free(ATResponse *p_response)
{
  ATLine *p_line;

  if (p_response == NULL)
    {
      return;
    }

  p_line = p_response->p_intermediates;

  while (p_line != NULL)
    {
      ATLine *p_toFree;
      p_toFree = p_line;
      p_line = (ATLine *)(p_line->p_next);
      free(p_toFree->line);
      free(p_toFree);
    }
  free(p_response->finalResponse);
  free(p_response);
}

static void reverseIntermediates(ATResponse *p_response)
{
  ATLine *pcur, *pnext;

  pcur = p_response->p_intermediates;
  p_response->p_intermediates = NULL;

  while (pcur != NULL)
    {
      pnext = (ATLine *)(pcur->p_next);
      pcur->p_next = (ATLine *)(p_response->p_intermediates);
      p_response->p_intermediates = pcur;
      pcur = pnext;
    }
}

static void addIntermediate(ATResponse *p_response, const char *line)
{
  ATLine *p_new;

  p_new = (ATLine *)malloc(sizeof(ATLine));
  p_new->line = strdup(line);

  /* note: this adds to the head of the list, so the list
     will be in reverse order of lines received. the order is flipped
     again before passing on to the command issuer */
  p_new->p_next = (ATLine *)(p_response->p_intermediates);
  p_response->p_intermediates = p_new;
}

static void setTimespecRelative(struct timespec *p_ts, long long msec)
{
  struct timeval tv;

  gettimeofday(&tv, (struct timezone *) NULL);

  p_ts->tv_sec = tv.tv_sec + (msec / 1000);
  p_ts->tv_nsec = (tv.tv_usec + (msec % 1000) * 1000L ) * 1000L;
}

static int pthreadCondWait(pthread_cond_t *waitCond, pthread_mutex_t *waitMutex, long long timeoutMsec)
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

static int writeline(const char *s, int fd)
{
  size_t cur = 0;
  size_t len = strlen(s);
  ssize_t written;
  int s_fd = fd;

  if (s_fd < 0)
    {
      return AT_ERROR_CHANNEL_CLOSED;
    }
  char *t = (char *)malloc(len + 1);
  memcpy(t, s, len);
  t[len++] = '\r';
  while (cur < len)
    {
      do
        {
          written = write(s_fd, t + cur, len - cur);
        }while (written < 0 && errno == EINTR);
      if (written < 0)
        {
          free(t);
          return AT_ERROR_GENERIC;
        }
      cur += written;
    }
  free(t);
  return 0;
}

static void clearPendingCommand(ATUartS *pAtUarts)
{
  if (pAtUarts->mSpResponse != NULL)
    {
      at_response_free(pAtUarts->mSpResponse);
    }

  pAtUarts->mSpResponse = NULL;
  pAtUarts->mResponsePrefix = NULL;
}


static int send_command_full_nolock(const char *command, ATCommandType type,
			  const char *responsePrefix, const char *smspdu, long long timeoutMsec, ATResponse **pp_outResponse,
			  ATUartS *pAtUarts)
{
  int err = 0;

  if (pAtUarts->mSpResponse != NULL)
    {
      err = AT_ERROR_COMMAND_PENDING;
      rillog(LOG_ERR, "%s %s: at error command pending\n", LOG_TAG, __func__);
      goto error;
    }
  err = writeline(command, pAtUarts->mfd);
  if (err < 0)
    {
      goto error;
    }

  pAtUarts->mCommandType = type;
  pAtUarts->mResponsePrefix = responsePrefix;
  pAtUarts->mSpResponse = at_response_new();
  while (pAtUarts->mSpResponse->finalResponse == NULL && pAtUarts->mfd >= 0)
    {
      err = pthreadCondWait(&(pAtUarts->mCommandcond), &(pAtUarts->mCommandmutex), timeoutMsec);
      if (err == ETIMEDOUT)
        {
          err = AT_ERROR_TIMEOUT;
          goto error;
        }
    }
  if (pp_outResponse == NULL)
    {
      at_response_free(pAtUarts->mSpResponse);
    }
  else
    {
      reverseIntermediates(pAtUarts->mSpResponse);
      *pp_outResponse = pAtUarts->mSpResponse;
    }
  pAtUarts->mSpResponse = NULL;

  pAtUarts->mResponsePrefix = NULL;
  return err;

error:
  clearPendingCommand(pAtUarts);
  return err;
}


int at_send_command_full(const char *command, ATCommandType type,const char *responsePrefix, const char *smspdu, long long timeoutMsec, ATResponse **pp_outResponse, ATUartS *pAtUarts)
{
  if (timeoutMsec == 0)
    {
      timeoutMsec = TIMEOUT_DEFALUT;
    }

  pthread_mutex_lock(&(pAtUarts->mCommandmutex));
  int err = send_command_full_nolock(command, type, responsePrefix, smspdu, timeoutMsec, pp_outResponse, pAtUarts);
  pthread_mutex_unlock(&(pAtUarts->mCommandmutex));

  if (err == AT_ERROR_TIMEOUT)
    {
      rillog(LOG_ERR, "%s %s: %s timeout\n", LOG_TAG, __func__, command);
    }
  return err;
}


static int sendATcommand(const char *atcmd, ATCommandType type, const char *prefix, ATResponse **pp_response, ATUartS *pAtUarts)
{
  int err;
  err = at_send_command_full(atcmd, type, prefix, NULL, AT_COMMAND_TIMEOUT, pp_response, pAtUarts);
  rillog(LOG_INFO, "%s sendATcommand: %s, err:%d\n", LOG_TAG, atcmd, err);
  return err;
}

int findATcmdTypeAndPrefix(char *pCmd, char **ppPrefix, ATCommandType *pAtCmdType, int *pUartIndex)
{
  size_t i;

  for (i = 0; i < NUM_ELEMS(g_atcmd); i++)
  {
    if (strStartsWith(pCmd, g_atcmd[i].p_atcmd))
      {
        *ppPrefix = g_atcmd[i].p_responsePrefix;
        *pAtCmdType = g_atcmd[i].eATcmdType;
        *pUartIndex = g_atcmd[i].uartIndex;
        return 1;
      }
  }
  return 0;
}

static bool findVbatCmd(char *pCmd, char **ppPrefix, ATCommandType *pAtCmdType, int *pUartIndex)
{
  if (strStartsWith(pCmd, "AT+VBAT"))
    {
      *pAtCmdType = SINGLELINE;
      if (gGpsStart)
        {
          int port = findInt(pCmd);
          sprintf(pCmd, "AT+PGNSS+$VBAT,%d", port);
          *ppPrefix = "$VBAT";
          *pUartIndex = ATSERVER_UART_GPS;
        }
      else
        {
          *ppPrefix = "+VBAT";
          *pUartIndex = ATSERVER_UART_MODEM;
        }

      return TRUE;
    }

  return FALSE;
}

int writeUntil(const void *buffer, size_t len, int fd)
{
  size_t offset = 0;
  const uint8_t * tmpWrite;
  tmpWrite = (const uint8_t *)buffer;

  if (fd < 0)
    {
      return -1;
    }
  while (offset < len)
    {
      ssize_t written;
      do
        {
          written = write(fd, tmpWrite + offset, len - offset);
        } while (written < 0 && errno == EINTR);
      if (written >= 0)
        {
          offset += written;
        }
      else
        {
          rillog(LOG_ERR, "%s writeUntil: unexpected error on write %d errno:%s", LOG_TAG, fd, strerror(errno));
          return -1;
        }
    }
  return 0;
}


int sendResponse(int fd, bool success, const void *data, uint16_t size, char type, char lineNumber)
{
  int ret = writeUntil((void *)&size, sizeof(size), fd);
  if(ret < 0)
    {
      return ret;
    }
  ret = writeUntil(&type, 1, fd);
  if(ret < 0)
    {
      return ret;
    }
  if (type == AT_RESPONSE_TYPE_SOLICITED)
  {
    char error;
    if (success)
      {
        error = 0;
      }
    else
      {
        error = 1;
      }
    ret = writeUntil(&error, 1, fd);
    if(ret < 0)
      {
        return ret;
      }
  }
  if (data)
    {
      if (type == AT_RESPONSE_TYPE_SOLICITED)
        {
          ret = writeUntil(&lineNumber, 1, fd);
          if(ret < 0)
            {
              return ret;
            }
          return writeUntil(data,size - 3, fd);
        }
      else
        {
          return writeUntil(data,size - 1, fd);
        }
    }
  else
    {
      return 0;
    }
}


static void wakeup_cp(void)
{
  int fd;
  fd = open("/dev/miscsp", 0);
  if (fd >= 0)
    {
      struct misc_remote_boot_s remote =
        {
          .name = "cp",
        };
      ioctl(fd, MISC_REMOTE_BOOT, (unsigned long)&remote);
      close(fd);
    }
}

void processCommandBuffer(ATClient *pATClient, void *buffer, size_t buflen)
{
  int error;
  ATResponse *p_response = NULL;
  ATCommandType atCmdType;
  char* prefix;
  char *pStrResponse = NULL;
  char responseLen = 0;
  int uartIndex;
  char* cmd;

  cmd = (char*)buffer;
  if (!findVbatCmd(cmd, &prefix, &atCmdType, &uartIndex))
    {
      if (findATcmdTypeAndPrefix(cmd, &prefix, &atCmdType, &uartIndex) == 0)
      {
        rillog(LOG_ERR, "%s Command : %s not supported\n", LOG_TAG, cmd);
        return;
      }
    }

  if (uartIndex == ATSERVER_UART_GPS)
    {
      if (strstr(cmd, "START"))
        {
          gGpsStart = true;
        }
      else if (strstr(cmd, "STOP"))
        {
          gGpsStart = false;
        }
      startGpsDurationTime();
    }
  else if (uartIndex != ATSERVER_UART_GPS)
    {
      stopGpsDurationTime();
    }
  if (!gModemReady)
    {
      wakeup_cp();
      pthread_mutex_lock(gModemReadymutex);
      while (!gModemReady)
        {
          pthreadCondWait(gModemReadycond, gModemReadymutex, 0);
        }
      pthread_mutex_unlock(gModemReadymutex);
      if (gModemReadymutex)
        {
          pthread_mutex_destroy(gModemReadymutex);
          free(gModemReadymutex);
        }
      if (gModemReadycond)
        {
          pthread_cond_destroy(gModemReadycond);
          free(gModemReadycond);
        }
    }
  rillog(LOG_INFO, "%s Client %d processCommandBuffer: process cmd=%s, prefix=%s, index=%d\n",
    LOG_TAG, pATClient->mToken, cmd, prefix, uartIndex);
  error = sendATcommand(cmd, atCmdType, prefix, &p_response, gATUarts + uartIndex);
  pthread_mutex_lock(&(gServer->mClientsLock));
  if( error < 0 || p_response->success == 0)
    {
      sendResponse(pATClient->mfd, false, NULL, 2, AT_RESPONSE_TYPE_SOLICITED, 0);
    }
  else
    {
      int lineNumber = 0;
      ATLine *p = p_response->p_intermediates;
      while(p)
        {
          responseLen += strlen(p->line) + 1;
          p = (ATLine *)(p->p_next);
          lineNumber++;
        }
      if (responseLen > 0)
        {
          pStrResponse= (char *)malloc(responseLen + 1);
          memset(pStrResponse, 0x0, responseLen + 1);
          p = p_response->p_intermediates;
          while(p)
            {
              strcat(pStrResponse, p->line);
              strcat(pStrResponse, "\n");
              p = p->p_next;
            }
          sendResponse(pATClient->mfd, true, pStrResponse, strlen(pStrResponse) + 3, AT_RESPONSE_TYPE_SOLICITED, lineNumber);
          free(pStrResponse);
        }
      else
        {
          sendResponse(pATClient->mfd, true, NULL, 2, AT_RESPONSE_TYPE_SOLICITED, lineNumber);
        }
    }
  pthread_mutex_unlock(&(gServer->mClientsLock));
  at_response_free(p_response);
}

static void processGpsDurationTimeout(void)
{
  int ret;
  ATResponse *p_response = NULL;
  ATCommandType atCmdType;
  char* prefix;
  int uartIndex;

  rillog(LOG_INFO, "%s %s\n", LOG_TAG, __func__);

  gIndicationDisable = true;

  char* cmd = (char*)"AT+PGNSS+$STOP";
  if (findATcmdTypeAndPrefix(cmd, &prefix, &atCmdType, &uartIndex) == 0)
    {
      rillog(LOG_ERR, "%s %s Command : %s not supported\n", LOG_TAG, __func__, cmd);
      goto error;
    }
  ret = sendATcommand(cmd, atCmdType, prefix, &p_response, gATUarts + uartIndex);
  at_response_free(p_response);

  if (ret)
    {
      goto error;
    }

  cmd = (char*)"AT+CFUN=1";
  if (findATcmdTypeAndPrefix(cmd, &prefix, &atCmdType, &uartIndex) == 0)
    {
      rillog(LOG_ERR, "%s %s Command : %s not supported\n", LOG_TAG, __func__, cmd);
      goto error;
    }
  ret = sendATcommand(cmd, atCmdType, prefix, &p_response, gATUarts + uartIndex);
  at_response_free(p_response);

  if (ret)
    {
      goto error;
    }

  cmd = (char*)"AT+CFUN=0";
  if (findATcmdTypeAndPrefix(cmd, &prefix, &atCmdType, &uartIndex) == 0)
    {
      rillog(LOG_ERR, "%s %s Command : %s not supported\n", LOG_TAG, __func__, cmd);
      goto error;
    }
  ret = sendATcommand(cmd, atCmdType, prefix, &p_response, gATUarts + uartIndex);
  at_response_free(p_response);

  if (ret)
    {
      goto error;
    }

  gIndicationDisable = false;

  if (gGpsStart)
    {
      cmd = (char*)"AT+PGNSS+$START";
      if (findATcmdTypeAndPrefix(cmd, &prefix, &atCmdType, &uartIndex) == 0)
        {
          rillog(LOG_ERR, "%s %s Command : %s not supported\n", LOG_TAG, __func__, cmd);
          goto error;
        }
      ret = sendATcommand(cmd, atCmdType, prefix, &p_response, gATUarts + uartIndex);
      at_response_free(p_response);
      if (ret)
        {
          goto error;
        }
    }
  return;
error:
  rillog(LOG_ERR, "%s %s : error happen!!!!!!!\n", LOG_TAG, __func__);
  board_reset(0);
  return;
}


bool onDataAvailable(ATClient *pATClient)
{
  int ret;
  char *buf = NULL;
  char bufLen;
  char readn;

  buf = (char *)malloc(MAX_AT_SIZE + 1);
  memset(buf, 0x0, MAX_AT_SIZE + 1);
  ret = read(pATClient->mfd, &bufLen, 1);
  rillog(LOG_INFO, "%s onDataAvailable: read:%d,%d\n", LOG_TAG, ret, bufLen);
  if (ret != 1)
    {
      free(buf);
      return false;
    }
  readn = 0;
  for (;;)
    {
      ret = read(pATClient->mfd, buf + readn, bufLen - readn);
      if (ret <= 0)
        {
          break;
        }
      else
        {
          if (readn + ret >= bufLen)
            {
              processCommandBuffer(pATClient, buf, bufLen);
              break;
            }
        }
    }

  if (ret <= 0)
    {
      free(buf);
      return false;
    }
  free(buf);
  return true;
}


void run(ATServer *pATServer)
{
  rillog(LOG_INFO, "%s ATServer is started\n", LOG_TAG);
  sq_queue_t pendingList;
  sq_init(&pendingList);
  while(1)
    {
      fd_set read_fds;
      int rc = 0;
      int max = -1;
      ATClient *atClient;
      ATPendingClient *atPendingClient;
      FD_ZERO(&read_fds);
      max = pATServer->mSock;
      FD_SET(pATServer->mSock, &read_fds);
      FD_SET(gGpsTimeoutPipe[0], &read_fds);
      if (gGpsTimeoutPipe[0] > max)
        {
          max = gGpsTimeoutPipe[0];
        }
      pthread_mutex_lock(&(pATServer->mClientsLock));
      for (atClient = (ATClient*)(pATServer->mClients.head);
            atClient;
            atClient = (ATClient*)sq_next(&(atClient->mClientLcNode)))
        {
          int fd = atClient->mfd;
          rillog(LOG_INFO, "%s ATServer found client:%d\n", LOG_TAG, fd);
          FD_SET(fd, &read_fds);
          if (fd > max)
            {
              max = fd;
            }
        }
      pthread_mutex_unlock(&(pATServer->mClientsLock));
      if ((rc = select(max + 1, &read_fds, NULL, NULL, NULL)) < 0)
        {
          if (errno == EINTR)
            {
              continue;
            }
          rillog(LOG_ERR, "%s select failed (%s) max=%d\n", LOG_TAG, strerror(errno), max);
          sleep(1);
          continue;
        }
      else if (!rc)
        {
          continue;
        }
      if (FD_ISSET(pATServer->mSock, &read_fds))
        {
          struct sockaddr addr;
          socklen_t alen;
          int c;
          do
            {
              alen = sizeof(addr);
              c = accept(pATServer->mSock, &addr, &alen);
              rillog(LOG_INFO, "%s ATServer got %d from accept\n", LOG_TAG, c);
            } while (c < 0 && errno == EINTR);
          if (c < 0)
            {
              rillog(LOG_ERR, "%s accept failed (%s)\n", LOG_TAG, strerror(errno));
              sleep(1);
              continue;
            }
          if (fcntl(c, F_SETFL, O_NONBLOCK) < 0)
            {
              rillog(LOG_ERR, "%s Error setting O_NONBLOCK errno:%d\n", LOG_TAG, errno);
            }
          atClient = (ATClient*)malloc(sizeof(ATClient));
          atClient->mfd = c;
          atClient->mToken = pATServer->mClientToken++;
          pthread_mutex_lock(&(pATServer->mClientsLock));
          sq_addlast(&(atClient->mClientLcNode), &(pATServer->mClients));
          pthread_mutex_unlock(&(pATServer->mClientsLock));
        }
      if (FD_ISSET(gGpsTimeoutPipe[0], &read_fds))
        {
          char c;
          rillog(LOG_INFO, "%s gps duraion time out\n", LOG_TAG);
          read(gGpsTimeoutPipe[0], &c, 1);
          processGpsDurationTimeout();
        }
      pthread_mutex_lock(&(pATServer->mClientsLock));
      atClient = (ATClient*)(pATServer->mClients.head);
      while (atClient)
        {
          int fd = atClient->mfd;
          if (FD_ISSET(fd, &read_fds))
            {
              atPendingClient = (ATPendingClient*)malloc(sizeof(ATPendingClient));
              atPendingClient->mAtCLient = atClient;
              sq_addlast(&(atPendingClient->mNode), &pendingList);
            }
            atClient = (ATClient*)sq_next(&(atClient->mClientLcNode));
        }
      pthread_mutex_unlock(&(pATServer->mClientsLock));
      while (sq_count(&pendingList))
        {
          atPendingClient = (ATPendingClient*)sq_remfirst(&pendingList);
          atClient = atPendingClient->mAtCLient;
          if (!onDataAvailable(atClient))
            {
              rillog(LOG_INFO, "%s %s: remove client:%d\n", __func__, LOG_TAG, atClient->mfd);
              pthread_mutex_lock(&(pATServer->mClientsLock));
              sq_rem(&(atClient->mClientLcNode), &(pATServer->mClients));
              pthread_mutex_unlock(&(pATServer->mClientsLock));
              close(atClient->mfd);
              free(atClient);
            }
          free(atPendingClient);
        }
    }
  rillog(LOG_ERR, "%s ATServer is closed\n", LOG_TAG);
}

int startServer(ATServer *pATServer)
{
  struct sockaddr_un server_address;
  int server_len;
  int ret;
  unlink(pATServer->mSocketName);
  pATServer->mSock = socket(AF_UNIX, SOCK_STREAM, 0);
  if (pATServer->mSock < 0)
    {
      rillog(LOG_ERR, "%s %s: Failed to get socket %s\n", LOG_TAG, __func__, pATServer->mSocketName);
      return -1;
    }
  server_address.sun_family = AF_UNIX;
  strcpy(server_address.sun_path, pATServer->mSocketName);
  server_len = sizeof(server_address);

  ret = bind(pATServer->mSock, (struct sockaddr *)&server_address, server_len);
  if (ret < 0)
    {
      rillog(LOG_ERR, "%s %s: Failed to bind socket '%d': %s\n",
        LOG_TAG, __func__, ret, strerror(errno));
      close(pATServer->mSock);
      return -1;
    }

  ret = listen(pATServer->mSock, 4);
  if (ret < 0)
    {
      rillog(LOG_ERR, "%s %s: Failed to listen on control socket '%d': %s\n",
        LOG_TAG, __func__, ret, strerror(errno));
      close(pATServer->mSock);
      return -1;
    }
  run(pATServer);
  close(pATServer->mSock);
  return 0;
}

void ATServer_initialize(ATServer *pATServer, const char *socketName)
{
  pATServer->mSocketName = socketName;
  pATServer->mSock = -1;
  pATServer->mClientToken = 1;
  pATServer->mThread = pthread_self();
  pthread_mutex_init(&(pATServer->mClientsLock), NULL);
  sq_init(&(pATServer->mClients));
}


void ATServerRun(void)
{
  if (startServer(gServer))
    {
      rillog(LOG_ERR, "%s Unable to start at server\n", LOG_TAG);
    }
  free(gServer);
  return;
}

static int readChannel(ATUartS *pAtUarts)
{
  ssize_t count;
  char *p_read = NULL;

  if (*(pAtUarts->mATBufferCur) == '\0')
    {
      pAtUarts->mATBufferCur = pAtUarts->mATBuffer;
      *(pAtUarts->mATBufferCur) = '\0';
      p_read = pAtUarts->mATBuffer;
    }
  else
    {
      int CurIdx = pAtUarts->mATBufferCur - pAtUarts->mATBuffer;
      rillog(LOG_INFO, "%s %s ATBufferCur not empty, start at %d: %s\n", LOG_TAG, __func__, CurIdx, pAtUarts->mATBufferCur);
      size_t len = strlen(pAtUarts->mATBufferCur);
      if (CurIdx > 0)
        {
          memmove(pAtUarts->mATBuffer, pAtUarts->mATBufferCur, len + 1);
          pAtUarts->mATBufferCur = pAtUarts->mATBuffer;
        }
      p_read = pAtUarts->mATBufferCur + len;
    }

  if (0 == MAX_AT_SIZE - (p_read - pAtUarts->mATBuffer))
    {
      rillog(LOG_ERR, "%s ERROR: Input line exceeded buffer\n", LOG_TAG);
      pAtUarts->mATBufferCur = pAtUarts->mATBuffer;
      *(pAtUarts->mATBufferCur) = '\0';
      p_read = pAtUarts->mATBuffer;
    }
  do
    {
      rillog(LOG_INFO, "%s begin read bufsize=%d\n", LOG_TAG, (int)(MAX_AT_SIZE - (p_read - pAtUarts->mATBuffer)));
      count = read(pAtUarts->mfd, p_read, MAX_AT_SIZE - (p_read - pAtUarts->mATBuffer));
      rillog(LOG_INFO, "%s end read count=%d\n", LOG_TAG, count);
    } while (count < 0 && errno == EINTR);

  if (count > 0)
    {
      p_read[count] = '\0';
    }
  else
    {
      if (count == 0)
        {
          rillog(LOG_ERR, "%s atchannel: EOF reached\n", LOG_TAG);
        }
      else
        {
          rillog(LOG_ERR, "%s atchannel: read error %s\n", LOG_TAG, strerror(errno));
        }
    }
  return count;
}

static const char *getLine(ATUartS *pAtUarts)
{
  char *p_eol = NULL;
  char *ret = NULL;

  if (*(pAtUarts->mATBufferCur) == '\0')
    {
      return NULL;
    }

  while (*(pAtUarts->mATBufferCur) == '\r' || *(pAtUarts->mATBufferCur) == '\n')
    {
      (pAtUarts->mATBufferCur)++;
    }
  p_eol = findNextEOL(pAtUarts->mATBufferCur);

  if (p_eol)
    {
      ret = pAtUarts->mATBufferCur;
      *p_eol = '\0';
      pAtUarts->mATBufferCur = p_eol + 1;
      while (*(pAtUarts->mATBufferCur) == '\r' || *(pAtUarts->mATBufferCur) == '\n')
        {
          pAtUarts->mATBufferCur++;
        }
    }
  return ret;
}

static void handleFinalResponse(const char *line, ATUartS *pAtUarts)
{
  pAtUarts->mSpResponse->finalResponse = strdup(line);
  pthread_cond_signal(&(pAtUarts->mCommandcond));
}

void setSystemTime(const char *s)
{
  if(s == NULL)
    {
      rillog(LOG_ERR, "%s s is null, return\n", LOG_TAG);
      return;
    }
  if(strncmp(s, "+CTZE:", 6) != 0)
    {
      return;
    }
  char* strctze = strdup(s);
  char* strctze_temp = strctze;
  if(strctze == NULL)
    {
      return;
    }

  //  +CTZE: +32,0,"2020/11/20,07:01:57"
  char* p;
  char calsign[2];
  int timezone;

  int ret = at_tok_start(&strctze_temp);
  if (ret < 0)
    {
      return;
    }
  ret = at_tok_nextstr(&strctze_temp, &p);
  if (ret < 0)
    {
      return;
    }
  strncpy(calsign, p, 1);
  calsign[1] = '\0';

  ret = at_tok_nextint(&p, &timezone);
  if (ret < 0)
    {
      return;
    }
  ret = at_tok_nextstr(&strctze_temp, &p);
  if (ret < 0)
    {
      return;
    }
  ret = at_tok_nextstr(&strctze_temp, &p);
  if (ret < 0)
    {
      return;
    }

  struct tm tm;
  struct timeval tv;
  time_t timep;
  sscanf(p, "%d/%d/%d,%d:%d:%d",&tm.tm_year, &tm.tm_mon, &tm.tm_mday,&tm.tm_hour, &tm.tm_min, &tm.tm_sec);
  tm.tm_mon = tm.tm_mon-1;
  tm.tm_year = tm.tm_year-1900;

  timep = mktime(&tm);
  if(strncmp(calsign, "+", 1) == 0)
    {
      tv.tv_sec = timep + timezone * 15 * 60;
    }
  else
    {
      tv.tv_sec = timep - timezone * 15 * 60;
    }
  tv.tv_usec = 0;
  /* if(settimeofday(&tv, NULL) < 0)
    {
      rillog(LOG_ERR, "%s settimeofday error\n", LOG_TAG);
    } */

  if(strctze != NULL)
    {
      free(strctze);
      strctze = NULL;
    }
}

void handleUnsolicited(const char *s)
{
  ATClient *atClient;

  if (!gModemReady)
    {
      pthread_mutex_lock(gModemReadymutex);
      gModemReady = true;
      pthread_cond_signal(gModemReadycond);
      pthread_mutex_unlock(gModemReadymutex);
    }

  //by NAS time, set system time
  setSystemTime(s);

  if (!gIndicationDisable)
    {
      rillog(LOG_INFO, "%s ATServer client count:%d\n", LOG_TAG, sq_count(&(gServer->mClients)));
      pthread_mutex_lock(&(gServer->mClientsLock));
      for (atClient = (ATClient*)(gServer->mClients.head);
            atClient;
            atClient = (ATClient*)sq_next(&(atClient->mClientLcNode)))
        {
          rillog(LOG_INFO, "%s %s send %d %s\n", LOG_TAG, __func__, atClient->mfd, s);
          sendResponse(atClient->mfd, true, s, strlen(s)+ 1, AT_RESPONSE_TYPE_UNSOLICITED, 1);
        }
      pthread_mutex_unlock(&(gServer->mClientsLock));
    }
}

static void processLine(const char *line, ATUartS *pAtUarts)
{
  static bool bCellInfoProcess = false;
  rillog(LOG_INFO, "%s %s entry: process line: %s\n", LOG_TAG, __func__, line);
  pthread_mutex_lock(&(pAtUarts->mCommandmutex));
  if (pAtUarts->mSpResponse == NULL)
    {
      handleUnsolicited(line);
    }
  else if (isFinalResponseSuccess(line))
    {
      pAtUarts->mSpResponse->success = 1;
      if (bCellInfoProcess)
        {
          bCellInfoProcess = false;
        }
      handleFinalResponse(line, pAtUarts);
    }
  else if (isFinalResponseError(line))
    {
      pAtUarts->mSpResponse->success = 0;
      if (bCellInfoProcess)
        {
          bCellInfoProcess = false;
        }
      handleFinalResponse(line, pAtUarts);
    }
  else
    {
      switch (pAtUarts->mCommandType)
        {
          case NO_RESULT:
            {
              handleUnsolicited(line);
              break;
            }
          case NUMERIC:
            {
              if (pAtUarts->mSpResponse->p_intermediates == NULL)
                {
                  addIntermediate(pAtUarts->mSpResponse, line);
                }
              else
                {
                  handleUnsolicited(line);
                }
              }
              break;
          case SINGLELINE:
            {
              if (pAtUarts->mSpResponse->p_intermediates == NULL && pAtUarts->mResponsePrefix && strStartsWith(line, pAtUarts->mResponsePrefix))
                {
                  addIntermediate(pAtUarts->mSpResponse, line);
                }
              else
                {
                  handleUnsolicited(line);
                }
              break;
            }
          case MULTILINE:
            {
              if (strStartsWith(line, pAtUarts->mResponsePrefix) || bCellInfoProcess)
                {
                  if (!bCellInfoProcess && strcmp(pAtUarts->mResponsePrefix, "+NCCELLINFO") == 0)
                    {
                      bCellInfoProcess = true;
                    }
                  addIntermediate(pAtUarts->mSpResponse, line);
                }
              else
                {
                  handleUnsolicited(line);
                }
              break;
            }
          default:
            {
              rillog(LOG_ERR, "%s Unsupported AT command type %d\n", LOG_TAG, pAtUarts->mCommandType);
              handleUnsolicited(line);
            }
          }
    }
  rillog(LOG_INFO, "%s %s exit\n", LOG_TAG, __func__);
  pthread_mutex_unlock(&(pAtUarts->mCommandmutex));
}

static int channelReader(ATUartS *pAtUarts)
{
  const char *line;
  if (readChannel(pAtUarts) <= 0)
    {
      return -1;
    }
  while (1)
    {
      line = getLine(pAtUarts);
      if (line == NULL)
        {
          return 0;
        }
      else
        {
          rillog(LOG_INFO, "%s %s line:%s\n", LOG_TAG, __func__, line);
        }
      processLine(line, pAtUarts);
    }
  return 0;
}

void *reader_loop(void *obj)
{
  int i;
  pthread_setname_np(pthread_self(), "atServerReadLoop");
  for (i = 0; i < ATSERVER_NUARTS; i++)
    {
      memset(gATUarts[i].mATBuffer, 0x0, sizeof(gATUarts[i].mATBuffer));
      gATUarts[i].mATBufferCur = gATUarts[i].mATBuffer;
    }
  for (;;)
    {
      fd_set readset;
      int maxfd = 0;
      FD_ZERO(&readset);
      for (i = 0; i < ATSERVER_NUARTS; i++)
        {
          FD_SET(gATUarts[i].mfd, &readset);
          if (gATUarts[i].mfd > maxfd)
            {
              maxfd = gATUarts[i].mfd;
            }
        }
      int ret = select(maxfd + 1, &readset, NULL, NULL, NULL);
      if (ret < 0)
        {
          break;
        }
      bool readSuccess = true;
      for (i = 0; i < ATSERVER_NUARTS; i++)
        {
          if (FD_ISSET(gATUarts[i].mfd, &readset))
            {
              rillog(LOG_INFO, "%s %s index:%d\n", LOG_TAG, __func__, i);
              if (channelReader(gATUarts + i) < 0)
                {
                  readSuccess = false;
                }
            }
        }
    if (!readSuccess)
      {
        break;
      }
  }
  return NULL;
}

static int ril_daemon(int argc, char *argv[])
{
  int i;

  rillog(LOG_INFO, "%s %s: ril_daemon running\n", LOG_TAG, __func__);
  for (i = 0; i < ATSERVER_NUARTS; i++)
    {
      if ((gATUarts[i].mfd = open(gATServerUartNames[i], O_RDWR)) < 0)
        {
          rillog(LOG_ERR, "%s %s: open device %s error\n", LOG_TAG, __func__, gATServerUartNames[i]);
          goto clean;
        }
      pthread_mutex_init(&(gATUarts[i].mCommandmutex), NULL);
      pthread_cond_init(&(gATUarts[i].mCommandcond), NULL);
    }
  gModemReadymutex = (pthread_mutex_t *)malloc(sizeof(pthread_mutex_t));
  gModemReadycond  = (pthread_cond_t *)malloc(sizeof(pthread_cond_t));
  pthread_mutex_init(gModemReadymutex, NULL);
  pthread_cond_init(gModemReadycond, NULL);
  gServer = (ATServer *)malloc(sizeof(ATServer));
  ATServer_initialize(gServer, "/dev/atil");

  if (pipe(gGpsTimeoutPipe))
    {
      rillog(LOG_ERR, "%s %s: pipe (%s)\n", LOG_TAG, __func__, strerror(errno));
      goto clean;
    }
  file_detach(gGpsTimeoutPipe[1], &gGpsPipeFileWrite);
  gGpsTimeoutPipe[1] = -1;

  if (pthread_create(NULL, NULL, reader_loop, NULL))
    {
      rillog(LOG_ERR, "%s %s: pthread_create (%s)\n", LOG_TAG, __func__, strerror(errno));
      goto clean;
    }

  ATServerRun();

clean:
  if (gGpsTimeoutPipe[0] > 0)
    {
      close(gGpsTimeoutPipe[0]);
    }
  if (gGpsTimeoutPipe[1] > 0)
    {
      close(gGpsTimeoutPipe[1]);
    }
  file_close(&gGpsPipeFileWrite);
  for (i = 0; i < ATSERVER_NUARTS; i++)
    {
      if (gATUarts[i].mfd > 0)
        {
          close(gATUarts[i].mfd);
        }
      pthread_mutex_destroy(&(gATUarts[i].mCommandmutex));
      pthread_cond_destroy(&(gATUarts[i].mCommandcond));
    }
  if (gModemReadymutex)
    {
      pthread_mutex_destroy(gModemReadymutex);
      free(gModemReadymutex);
    }
  if (gModemReadycond)
    {
      pthread_cond_destroy(gModemReadycond);
      free(gModemReadycond);
    }
  if (gServer)
    {
      free(gServer);
    }
  return 0;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, char *argv[])
#else
int ril_main(int argc, char *argv[])
#endif
{
  int ret;
  ret = task_create(argv[0],
          CONFIG_SERVICES_RIL_PRIORITY,
          CONFIG_SERVICES_RIL_STACKSIZE,
          ril_daemon,
          argv + 1);
  return ret > 0 ? 0 : ret;
}
