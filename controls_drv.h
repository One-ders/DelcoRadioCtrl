/* $datalink: , v1.1 2023/01/06 21:44:00 anders Exp $ */

/*
 * Copyright (c) 2021, Anders Franzen.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * @(#)datalink.h
 */
#define CTRLS0	"ctrls0"

#define EVENT_P1_PUSH		0x0001
#define EVENT_P2_PUSH		0x0002
#define EVENT_P3_PUSH		0x0004
#define EVENT_P4_PUSH		0x0008

#define EVENT_P1_REL		0x0010
#define EVENT_P2_REL		0x0020
#define EVENT_P3_REL		0x0040
#define EVENT_P4_REL		0x0080

#define EVENT_SEEK_DOWN		0x0100
#define EVENT_SEEK_UP		0x0200

#define EVENT_SET		0x0400

#define EVENT_TUNER_PUSH	0x0800
#define EVENT_VOL_PUSH		0x1000

#define EVENT_TUNE_DOWN		0x2000
#define EVENT_TUNE_UP		0x4000

#if 0
#define EVENT_STAT_ON		0x1

struct ctrls_event {
	int event_id;
	int event_stat;
};
#endif
