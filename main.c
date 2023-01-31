/* $RadioController: main.c, v1.0 2023/01/06 21:44:00 anders Exp $ */

/*
 * Copyright (c) 2023, Anders Franzen.
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
 * @(#)main.c
 */

#include "sys.h"
#include "sys_env.h"
#include "io.h"
#include "gpio_drv.h"

#include <string.h>
#include <ctype.h>

#include "dl_drv.h"
#include "tuner_wheel.h"

static int bub(int argc, char **argv, struct Env *env);

static struct cmd cmds[] = {
	{"help", generic_help_fnc},
	{"bub", bub},
	{0,0}
};

static struct cmd_node cmn = {
	"bum",
	cmds,
};

static int udelay(unsigned int usec) {
	volatile unsigned int count=0;
	volatile unsigned int utime=(120*usec/7);
	do {
		if (++count>utime) return 0;
	} while (1);

	return 0;
}

static char *toIhex(char *buf, int address, int len, unsigned char *data) {
	unsigned int ck=0;
	int offs=0;
	int i;
	ck=0x10+(address&0xff)+(address>>8);
	sprintf(buf,":%02x%04x%02x",len,address,0);
	offs=9;
	for(i=0;i<len;i++) {
		ck+=data[i];
		sprintf(&buf[offs+(i*2)],"%02x",data[i]);
	}
	ck=ck&0xff;
	ck=(~ck)+1;
	ck&=0xff;
	sprintf(&buf[offs+(2*len)],"%02x", ck);

	return buf;
}

static unsigned int char2num(unsigned char c) {
	switch(c) {
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			return c-'0';
			break;
		case 'a':
		case 'b':
		case 'c':
		case 'd':
		case 'e':
		case 'f':
			return c-'a'+10;
			break;
		case 'A':
		case 'B':
		case 'C':
		case 'D':
		case 'E':
		case 'F':
			return c-'A'+10;
			break;
	}
	return 0;
}

static unsigned int decode_dbyte(char *p) {
	unsigned int bval=0;
	int i;

	for(i=0;i<2;i++) {
		bval=(bval<<4)|char2num(p[i]);
	}
	return bval;
}

//unsigned char bin_buf[256];

#if 0
static int parse_ihex(char *line, int size, int offset, int io_fd) {
	int i=0;
	int j;
	int datalen=0;
	unsigned int address=0;
	int rtype=0;
	int cksum=0;
	int checksum;
	int val;

	while(i<size) {
		if (line[i++]==':') {
			break;
		}
	}
//	printf("new record, ");

//========== Read len
	datalen=val=decode_dbyte(&line[i]);
	i+=2;
//	printf("len=%d, ", datalen);
	cksum=val;

//========== Read address
	address=val=decode_dbyte(&line[i]);
	i+=2;
	cksum+=val;

	val=decode_dbyte(&line[i]);
	i+=2;
	address=(address<<8)|val;
	cksum+=val;
//	printf("address=%04x, data:", address);

//========== Read record type
	rtype=val=decode_dbyte(&line[i]);
	i+=2;
	cksum+=val;
	if (rtype!=0) {
		fprintf(io_fd, "unhandled record type %x\n", rtype);
	}
//	printf("rt=%02x, ", rtype);

//========== Read data
	for(j=0;j<datalen;j++) {
		bin_buf[j]=val=decode_dbyte(&line[i]);
		i+=2;
		cksum+=val;
//		printf("%02x,", bin_buf[j]);
	}

//========== Read check sum
	checksum=decode_dbyte(&line[i]);
	cksum+=checksum;
	cksum&=0xff;

	if (cksum) {
		fprintf(io_fd, "checksum error for line with address %04x\n", address);
		return -1;
	}

	burn_line(bin_buf, datalen, address+offset,io_fd);

	return 0;
}

#endif

static unsigned char buf[8];
static int myfd=-1;

static int bub(int argc, char **argv, struct Env *env) {
	int i;
	int bits;

	if (myfd<0) {
		printf("could not open SPI_DRV\n");
		return 0;
	}

	if (argc<2) {
		fprintf(env->io_fd, "need bit num as first arg\n");
		return -1;
	}

	bits=strtoul(argv[1],0,16);
	fprintf(env->io_fd, "will send %d bits\n", (bits&0x7f));

	if ((argc-2)<(((bits&0x7f)+7)/8)) {
		fprintf(env->io_fd, "to few arguments for %d bits\n", bits&0x7f);
		return -1;
	}

	buf[0]=bits;

	for(i=2;i<argc;i++) {
		buf[i-1]=strtoul(argv[i],0,16);
		fprintf(env->io_fd, "got byte 0x%02x\n",buf[i-1]);
	}

	io_write(myfd, buf, argc-1);
	return 0;
}

extern int init_pin_test();

static int dump_hex(unsigned char *buf, int len) {
	int i;
        printf("\t %02x",buf[0]);
        for(i=1;i<len;i++) {
                printf(":%02x",buf[i]);
        }
        printf("\n");
        return 0;

}

static unsigned int encode_digit(int val) {
	switch (val) {
		case 0:
			return 0x3f;
		case 1:
			return 0x06;
		case 2:
			return 0x5b;
		case 3:
			return 0x4f;
		case 4:
			return 0x66;
		case 5:
			return 0x6d;
		case 6:
			return 0x7d;
		case 7:
			return 0x07;
		case 8:
			return 0x7f;
		case 9:
			return 0x6f;
		default:
			printf("bad value at encode %x\n",val);
	}
	return 0;
}

static int update_vfd_fm_freq(int mhz, int hkhz) {
	unsigned char buf[6];
	int fc;
	int prescaler;
	int m;
	int test;
	int val;
	int rc;

	memset(buf,0,sizeof(buf));

	buf[0]=0x80|0x22;

	buf[1]=0x21;

	if (mhz>99) {
		buf[1]|=0x02;
	}

	buf[2]=encode_digit((mhz/10)%10);
	buf[3]=encode_digit(mhz%10);
	buf[4]=encode_digit(hkhz);

	io_write(myfd, buf, sizeof(buf));

	fc=(mhz*10)+hkhz;
	prescaler=(fc+4195)/8;
	test=(fc+4195)-(prescaler*8);
	switch(test) {
		case 0:
			m=0x0;
			break;
		case 2:
			prescaler++;
			m=0xC;
			break;
		case 4:
			prescaler++;
			m=0x8;
			break;
		case 6:
			prescaler++;
			m=0x4;
			break;
	}

	val=(prescaler<<4)|m;

	val=(val<<2);

	buf[0]=0x12;
	buf[1]=val&0xff;
	buf[2]=(val>>8)&0xff;
	buf[3]=0x03;

	rc=io_write(myfd, buf, 4);
	printf("rc %d syn str %02x, %02x, %02x, %02x\n", rc, buf[0], buf[1], buf[2], buf[3]);

	return 0;
}

static int update_freq(int FM_Mhz, int FM_Hkhz) {
	update_vfd_fm_freq(FM_Mhz, FM_Hkhz);
	return 0;
}


static void tuner(void *dum) {
	int fd_tuner_wheel=io_open(TW_DRV0);
	int FM_Mhz = 87;
	int FM_Hkhz = 7;

	if (fd_tuner_wheel<0) {
		printf("could not open tuner wheel\n");
		return;
	}

	while(1) {
		signed char b;
		int rc;
		rc=io_read(fd_tuner_wheel, &b, 1);
		if (rc<0) {
			printf("%t: got error\n");
		} else {
			if (b==1) {
				if ((FM_Mhz>=107) &&
					(FM_Hkhz>=9)) {
					continue;
				}
				FM_Hkhz+=2;
				if (FM_Hkhz>9) {
					FM_Hkhz=1;
					FM_Mhz++;
				}
				update_freq(FM_Mhz, FM_Hkhz);
				printf("wheel ++, f=%d.%d\n", FM_Mhz, FM_Hkhz);
			} else if (b==-1) {
				if ((FM_Mhz<=87) &&
					(FM_Hkhz<=7)) {
					continue;
				}
				FM_Hkhz-=2;
				if (FM_Hkhz<0) {
					FM_Hkhz=9;
					FM_Mhz--;
				}
				update_freq(FM_Mhz, FM_Hkhz);
				printf("wheel --, f=%d.%d\n", FM_Mhz, FM_Hkhz);
			} else {
				printf("wheel shit val is %d\n", b);
			}
		}
	}
}

//int main(void) {
int init_pkg(void) {
//	struct Env e;
//	e.io_fd=0;
//	init_pins(0,0,&e);
	myfd=io_open(SPI_DRV0);
	install_cmd_node(&cmn, root_cmd_node);
	init_pin_test();
	thread_create(tuner,0,0,1,"tuner");
	printf("back from thread create\n");
	return 0;
}
