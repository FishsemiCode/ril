/****************************************************************************
 * external/services/ril/at_client/at_tok.c
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

#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "at_tok.h"

/**
 * Starts tokenizing an AT response string
 * returns -1 if this is not a valid response string, 0 on success.
 * updates *p_cur with current position
 */
int at_tok_start(char **p_cur)
{
  if (*p_cur == NULL)
    {
      return -1;
    }

  *p_cur = strchr(*p_cur, ':');

  if (*p_cur == NULL)
    {
      return -1;
    }

  (*p_cur)++;

  return 0;
}

void skipWhiteSpace(char **p_cur)
{
  if (*p_cur == NULL)
    {
      return;
    }
  while (**p_cur != '\0' && isspace(**p_cur))
    {
      (*p_cur)++;
    }
}

static void skipNextComma(char **p_cur)
{
  if (*p_cur == NULL)
    {
      return;
    }
  while (**p_cur != '\0' && **p_cur != ',')
    {
      (*p_cur)++;
    }

  if (**p_cur == ',')
    {
      (*p_cur)++;
    }
}

static char *nextTok(char **p_cur)
{
  char *ret = NULL;

  skipWhiteSpace(p_cur);

  if (*p_cur == NULL)
    {
      ret = NULL;
    }
  else if (**p_cur == '"')
    {
      (*p_cur)++;
      ret = strsep(p_cur, "\"");
      skipNextComma(p_cur);
    }
  else
    {
      ret = strsep(p_cur, ",");
    }

  return ret;
}


/**
 * Parses the next integer in the AT response line and places it in *p_out
 * returns 0 on success and -1 on fail
 * updates *p_cur
 * "base" is the same as the base param in strtol
 */

static int at_tok_nextint_base(char **p_cur, int *p_out, int base, int uns)
{
  char *ret;

  if (*p_cur == NULL)
    {
      return -1;
    }

  ret = nextTok(p_cur);

  if (ret == NULL)
    {
      return -1;
    }
  else
    {
      long l;
      char *end;
      if (uns)
        {
          l = strtoul(ret, &end, base);
        }
      else
        {
          l = strtol(ret, &end, base);
        }
      *p_out = (int)l;
      if (end == ret)
        {
          return -1;
        }
    }
  return 0;
}

/**
 * Parses the next base 10 integer in the AT response line
 * and places it in *p_out
 * returns 0 on success and -1 on fail
 * updates *p_cur
 */
int at_tok_nextint(char **p_cur, int *p_out)
{
  return at_tok_nextint_base(p_cur, p_out, 10, 0);
}

/**
 * Parses the next base 16 integer in the AT response line
 * and places it in *p_out
 * returns 0 on success and -1 on fail
 * updates *p_cur
 */
int at_tok_nexthexint(char **p_cur, int *p_out)
{
  return at_tok_nextint_base(p_cur, p_out, 16, 1);
}

int at_tok_nextbool(char **p_cur, char *p_out)
{
  int ret;
  int result;

  ret = at_tok_nextint(p_cur, &result);

  if (ret < 0)
    {
      return -1;
    }

  // booleans should be 0 or 1
  if (!(result == 0 || result == 1))
    {
      return -1;
    }

  if (p_out != NULL)
    {
      *p_out = (char)result;
    }

  return ret;
}

int at_tok_nextstr(char **p_cur, char **p_out)
{
  if (*p_cur == NULL)
    {
      return -1;
    }
  *p_out = nextTok(p_cur);
  return 0;
}

/** returns 1 on "has more tokens" and 0 if no */
int at_tok_hasmore(char **p_cur)
{
  return !(*p_cur == NULL || **p_cur == '\0');
}
