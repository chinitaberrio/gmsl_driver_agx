/* Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  * Neither the name of NVIDIA CORPORATION nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
 * OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef NV_SVC_CONF_H
#define NV_SVC_CONF_H

#include "ace/Svc_Conf.h"
#include "ace/Svc_Conf_Param.h"

// Define Tokens
#define ACE_DYNAMIC 258
#define ACE_STATIC 259
#define ACE_SUSPEND 260
#define ACE_RESUME 261
#define ACE_REMOVE 262
#define ACE_USTREAM 263
#define ACE_MODULE_T 264
#define ACE_STREAM_T 265
#define ACE_SVC_OBJ_T 266
#define ACE_ACTIVE 267
#define ACE_INACTIVE 268
#define ACE_PATHNAME 269
#define ACE_IDENT 270
#define ACE_STRING 271

/*
 * nv_parse_directive
 *
 * Parses directives passed in svc_conf_param source
 * Directive must be in the following form:
 * dynamic <identifier> Service_Object * <pathname>:<function>()
 * After parsing, creates a new dynamic node and applies the
 * parsed config to the same
 *
 * param : ACE_Svc_Conf_Param object, containing a non-NULL config
 *         and svc_conf source
 *
 * Returns 0 on clean exit, -1 on error
 *
*/
int nv_parse_directive(ACE_Svc_Conf_Param *param);

/* Helper functions for string parsing */

char *
TrimStringFromLeft(char *pchRawString);

char *
TrimStringFromRight(char *pchRawString);

char *
Tokenize(char *pString, const char *pDelim, char **pSavePtr);

#endif /* NV_SVC_CONF_H */
