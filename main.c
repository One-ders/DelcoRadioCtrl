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
#include "pwr_mgr_drv.h"

#include <string.h>
#include <ctype.h>

#include "dl_drv.h"
#include "controls_drv.h"
#include "tstats_drv.h"
#include "asynchio.h"
#include "timer.h"
#include "clock_drv.h"

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

#if 0
static int udelay(unsigned int usec) {
	volatile unsigned int count=0;
	volatile unsigned int utime=(120*usec/7);
	do {
		if (++count>utime) return 0;
	} while (1);

	return 0;
}
#endif

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

#if 0
static unsigned int decode_dbyte(char *p) {
	unsigned int bval=0;
	int i;

	for(i=0;i<2;i++) {
		bval=(bval<<4)|char2num(p[i]);
	}
	return bval;
}
#endif

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
static int conf_fd;

#define MAIN_STATE_IGN_OFF	0
#define MAIN_STATE_IGN_ON	1
#define MAIN_STATE_RADIO_ON	2
static unsigned int main_state;

#define SUB_STATE_RADIO		0
#define SUB_STATE_BLUETOOTH	1
static unsigned int sub_state;
static int init_clk;

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

#if 0
static int dump_hex(unsigned char *buf, int len) {
	int i;
        printf("\t %02x",buf[0]);
        for(i=1;i<len;i++) {
                printf(":%02x",buf[i]);
        }
        printf("\n");
        return 0;

}
#endif

struct FM_Freq {
	unsigned char Mhz;
	unsigned char Hkhz;
};

// Radio operations data
static int tstats_fd=-1;
static int clock_fd=-1;
static int ctrls_fd=-1;
static int set		= 0;
static int stereo	= 0;
#define SEEK_DIR_DOWN 0
#define SEEK_DIR_UP   1
static int seek_dir;
static int seek_running;
static int pkey;
static int show_pkey;
static int show_clock;
static int only_clock;
static int clock_colon;

static unsigned char hours;
static unsigned char minutes;
static struct FM_Freq active = {87,7};
static struct FM_Freq presets[11] = {
		{87,7},
		{87,7},
		{87,7},
		{87,7},
		{87,7},
		{87,7},
		{87,7},
		{87,7},
		{87,7},
		{87,7},
		{87,7},
};

static void *f_view_timer;



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

static int update_vfd() {
	unsigned char buf[6];
	int rc;

	memset(buf,0,sizeof(buf));

	if (only_clock || show_clock) {
		buf[0]=0x80|0x22;
		if (clock_colon) {
			buf[1]=0x8;
		} else {
			buf[1]=0;
		}
		if (hours>9) {
			buf[1]|=0x2;
		}
		buf[2]=encode_digit(hours%10);
		buf[3]=encode_digit(minutes/10);
		buf[4]=encode_digit(minutes%10);

		if (set) {
			buf[1]|=0x80;
		}

		if (stereo) {
			buf[1]|=0x04;
		}

	} else if (show_pkey) {
		buf[0]=0x80|0x22;
		buf[2]= 0x73;    // P
		if ((pkey+1)>9) {
			buf[3]=encode_digit((pkey+1)/10);
			buf[4]=encode_digit((pkey+1)%10);
		} else {
			buf[3]= encode_digit(pkey+1);
			buf[4]=0;
		}
	} else {
		if (sub_state==SUB_STATE_RADIO) {
			buf[0]=0x80|0x22;

			buf[1]=0;

			if (set) {
				buf[1]|=0x80;
			}

			if (stereo) {
				buf[1]|=0x04;
			}

			buf[1]|=0x21;   // FM and '.'

			if (active.Mhz>99) {
				buf[1]|=0x02;  // top 1 in 100.3
			}

			buf[2]=encode_digit((active.Mhz/10)%10);
			buf[3]=encode_digit(active.Mhz%10);
			buf[4]=encode_digit(active.Hkhz);
		} else if (sub_state==SUB_STATE_BLUETOOTH) {
			buf[0]=0x80|0x22;
			buf[1]=0;
			buf[2]=0x7c;
			buf[3]=0x38;
			buf[4]=0x1c;
		}
	}

	rc=io_write(myfd, buf, sizeof(buf));
	if (rc<0) {
		printf("error from update vfd: %d\n",rc);
	}
//	printf("update vfd return %d\n",rc);

	return 0;
}

static int update_syn() {
	unsigned char buf[6];
	int fc;
	int prescaler;
	int m=0;
	int test;
	int val;
	int rc;

	fc=(active.Mhz*10)+active.Hkhz;
	prescaler=(fc+4195)/8;
	test=(fc+4195)-(prescaler*8);
	switch(test) {
		case 0:
			m=0x0;
			break;
		case 1:
			prescaler++;
			m=0xE;
			break;
		case 2:
			prescaler++;
			m=0xC;
			break;
		case 3:
			prescaler++;
			m=0xA;
			break;
		case 4:
			prescaler++;
			m=0x8;
			break;
		case 5:
			prescaler++;
			m=0x6;
			break;
		case 6:
			prescaler++;
			m=0x4;
			break;
		case 7:
			prescaler++;
			m=0x2;
	}

	val=(prescaler<<4)|m;

	val=(val<<2);

	buf[0]=0x12;
	buf[1]=val&0xff;
	buf[2]=(val>>8)&0xff;
	buf[3]=0x03;

	rc=io_write(myfd, buf, 4);
	if (rc<0) {
		printf("error from write to syn: %d\n", rc);
	}
#if 0
	printf("rc %d syn str %02x, %02x, %02x, %02x\n", rc, buf[0], buf[1], buf[2], buf[3]);
#endif

	return 0;
}

static int mute_radio() {
	int mute=1;
	int rc;
	rc=io_control(tstats_fd, SET_RADIO_MUTE, &mute, sizeof(mute));
	if (rc<0) {
		printf("error from radio mute\n");
	}
	return 0;
}

static int unmute_radio() {
	int mute=0;
	int rc;
	rc=io_control(tstats_fd, SET_RADIO_MUTE, &mute, sizeof(mute));
	if (rc<0) {
		printf("error from radio unmute\n");
	}
	return 0;
}

static int audio_radio() {
	int off=0;
	int on=1;
	int rc;

	rc=io_control(tstats_fd, SET_BT_MUTE, &on, sizeof(on));
	if (rc<0) {
		printf("error from bt mute\n");
	}
	rc=io_control(tstats_fd, SET_RADIO_MUTE, &off, sizeof(off));
	if (rc<0) {
		printf("error from radio run\n");
	}
	return 0;
}

static int audio_bt() {
	int off=0;
	int on=1;
	int rc;

	rc=io_control(tstats_fd, SET_RADIO_MUTE, &on, sizeof(on));
	if (rc<0) {
		printf("error from radio run\n");
	}
	rc=io_control(tstats_fd, SET_BT_MUTE, &off, sizeof(off));
	if (rc<0) {
		printf("error from bt mute\n");
	}
	return 0;
}



struct time_buf tbuf;

static int pulse_event(int fd, int event, void *dum) {
	// toggle colon
	io_control(fd, CLOCK_GET_TIME, &tbuf, sizeof(tbuf));
//	show_clock=1;
	if (only_clock||show_clock) {
		hours=tbuf.hours;
		minutes=tbuf.minutes;
		clock_colon=(clock_colon?0:1);
		update_vfd();
	}
	return 0;
}

static int update_freq() {
	update_vfd();
	update_syn();
	return 0;
}

static void *set_timer;
static int set_timeout(void *udat) { // timeout on set button
//	printf("set_timeout\n");
	set_timer=0;
	set=0;
	update_vfd();
	return 0;
}

static int first;
static int seek_timeout(void *udat) {
	if (first) {
		first=0;
	} else {
		int stopflag;
		io_control(tstats_fd, GET_STOP_FLAG, &stopflag, sizeof(stopflag));
//		printf("stopflag is %x\n", stopflag);
		if (stopflag!=0) {
			if (seek_running) {
				seek_running=0;
				timer_set_seek_cb(0,0);
				unmute_radio();
				return 0;
			}
		}
	}
	if (seek_dir==SEEK_DIR_DOWN) {
		if ((active.Mhz<=87) &&
			(active.Hkhz<=7)) {
			active.Mhz=107;
			active.Hkhz=9;
		} else {
			active.Hkhz-=1;
			if (active.Hkhz>9) {
				active.Hkhz=9;
				active.Mhz--;
			}
		}
	} else {
		if ((active.Mhz>=107) &&
			(active.Hkhz>=9)) {
			active.Mhz=87;
			active.Hkhz=7;
		} else {
			active.Hkhz+=1;
			if (active.Hkhz>9) {
				active.Hkhz=0;
				active.Mhz++;
			}
		}
	}
	update_freq();
	return 0;
}

static int push_preset(int button) {
	if (set) {
		int rc;
		presets[button]=active;
		if (set_timer) {
			timer_delete(set_timer);
		}
		set_timer=0;
		set=0;
		show_pkey=1;
		pkey=button;
		update_vfd();
		rc=io_write(conf_fd,presets,sizeof(presets));
		if (rc!=sizeof(presets)) {
			printf("failed to store presets\n");
		}
	} else {
		active=presets[button];
		show_pkey=1;
		pkey=button;
		update_vfd();
		mute_radio();
		update_syn();
		sleep(100);
		unmute_radio();
	}
	return 0;
}

static int f_view_timeout(void *dum) {
	f_view_timer=0;
	show_clock=1;
	return 0;
}

static int release_preset(int button) {
	show_pkey=0;

	if (!only_clock) {
		if (f_view_timer) {
			timer_delete(f_view_timer);
		}
		f_view_timer=timer_create(4,f_view_timeout,0);
		show_clock=0;
	}
	update_vfd();
	return 0;
}

static int check_rsense() {
	int rc;
	int rsense;

	rc=io_control(tstats_fd, GET_RSENSE_STAT, &rsense, sizeof(rsense));
	if (rc<0) {
		return 0;
	}

	printf("check_rsense: got %d\n", rsense);

	if (rsense) {
		if (main_state==MAIN_STATE_IGN_ON) {
			main_state=MAIN_STATE_RADIO_ON;
			only_clock=0;
			if (f_view_timer) {
				timer_delete(f_view_timer);
			}
			f_view_timer=timer_create(4,f_view_timeout,0);
			show_clock=0;
			if (sub_state==SUB_STATE_RADIO) {
				update_freq();
				audio_radio();
			} else {
				update_vfd();
				audio_bt();
			}
		}
	} else {
		if (main_state==MAIN_STATE_RADIO_ON) {
			only_clock=1;
			mute_radio();
			update_vfd();
			main_state=MAIN_STATE_IGN_ON;
		}
	}
	return 0;
}

static int controls_event(int fd, int event, void *dum);
static int ignition_on() {
	if (main_state==MAIN_STATE_IGN_OFF) {
		int pwr_fd=io_open("pwr_mgr");
		unsigned int mode=3;
		io_control(pwr_fd, SET_POWER_MODE, &mode,sizeof(mode));
		main_state=MAIN_STATE_IGN_ON;
		io_close(pwr_fd);

		mute_radio();

		clock_fd	=io_open(CLOCK_DRV);
		if (clock_fd<0) {
			printf("could not open tod clock\n");
			return 0;
		}

		register_event(clock_fd, EV_STATE, pulse_event, 0);
		only_clock=1;


		ctrls_fd=io_open(CTRLS0);
		if (ctrls_fd<0) {
			printf("could not open controls driver\n");
			return 0;
		}
		io_control(ctrls_fd,F_SETFL,(void *)O_NONBLOCK,0);

		register_event(ctrls_fd, EV_READ, controls_event, 0);

		check_rsense();
	}
	return 0;
}

static int ignition_off() {
	if (main_state!=MAIN_STATE_IGN_OFF) {
		int pwr_fd;
		unsigned int mode;

		mute_radio();
		if (ctrls_fd>=0) {
			register_event(ctrls_fd, EV_READ, 0, 0);
			io_close(ctrls_fd);
			ctrls_fd=-1;
		}

		if (clock_fd>=0) {
			register_event(clock_fd, EV_STATE, 0, 0);
			io_close(clock_fd);
			clock_fd=-1;
		}

		pwr_fd=io_open("pwr_mgr");
		if (pwr_fd<0) return 0;
		if (init_clk>=84000000) {
			mode=3; // fast clock + wait for interrupt
		} else {
			mode=2; // slow clock + wait for interrupt
		}
		io_control(pwr_fd, SET_POWER_MODE, &mode,sizeof(mode));
		main_state=MAIN_STATE_IGN_OFF;
		io_close(pwr_fd);
		only_clock=0;
	}
	return 0;
}


static int check_ignition(int state) {
	int rc;
	int ignition;

	rc=io_control(tstats_fd, GET_IGNITION_STAT, &ignition, sizeof(ignition));
	if (rc<0) {
		return 0;
	}

	ignition=(ignition!=0)?1:0;
	if (ignition==state) {
		return 1;
	}

	return 0;
}


static int tstats_event(int fd, int event, void *dum) {
	int b;
	int rc;

	rc=io_read(fd, &b, 4);
	if (rc<0) {
		printf("%t: reading tstats got error\n");
	} else {
//		printf("got tstats event %d\n",b);
		if (b==EVENT_STEREO_ON) {
			stereo=1;
			update_vfd();
		} else if (b==EVENT_STEREO_OFF) {
			stereo=0;
			update_vfd();
		} else if (b==EVENT_RSENSE_ON) {
			printf("rsense on\n");
			sleep(100);
			check_rsense();
		} else if (b==EVENT_RSENSE_OFF) {
			printf("rsense off\n");
			sleep(100);
			check_rsense();
		} else if (b==EVENT_IGNITION_ON) {
			printf("ignition on\n");
			ignition_on();
		} else if (b==EVENT_IGNITION_OFF) {
			printf("ignition off\n");
			sleep(200);
			if (check_ignition(0)) {
				ignition_off();
			} else {
				printf("false alarm\n");
			}
		}
	}
	return 0;
}

static int toggle_radio_display(int theclock)  {

	printf("toggle_radio_display\n");
	if (theclock) {
		show_clock=0;
		if (f_view_timer) {
			timer_delete(f_view_timer);
		}
		f_view_timer=timer_create(4,f_view_timeout,0);
	} else {
		show_clock=1;
	}
	update_vfd();
	return 0;
}

static int controls_event(int fd, int event, void *dum) {
	int b;
	int rc;
	int pshow_clock=0;

	rc=io_read(fd, &b, 4);
	if (rc<0) {
		printf("%t: reading controls got error\n");
	} else {
		if (!only_clock) {
			if (f_view_timer) {
				timer_delete(f_view_timer);
			}
			f_view_timer=timer_create(4,f_view_timeout,0);
			pshow_clock=show_clock;
			show_clock=0;
		}
		if (b==EVENT_TUNE_UP) {
			if ((active.Mhz>=107) &&
				(active.Hkhz>=9)) {
				active.Mhz=87;
				active.Hkhz=7;
			} else {
				active.Hkhz+=1;
				if (active.Hkhz>9) {
					active.Hkhz=0;
					active.Mhz++;
				}
			}
			update_freq();
			printf("wheel ++, f=%d.%d\n", active.Mhz, active.Hkhz);
		} else if (b==EVENT_TUNE_DOWN) {
			if ((active.Mhz<=87) &&
				(active.Hkhz<=7)) {
				active.Mhz=107;
				active.Hkhz=9;
			} else {
				active.Hkhz-=1;
				if (active.Hkhz>9) {
					active.Hkhz=9;
					active.Mhz--;
				}
			}
			update_freq();
			printf("wheel --, f=%d.%d\n", active.Mhz, active.Hkhz);
		} else if (b==EVENT_SET) {
			show_clock=pshow_clock;
			if (!set) {
				set=1;
				if (set_timer) {
					timer_delete(set_timer);
				}
				set_timer=timer_create(4,set_timeout,0);
			} else {
				set=0;
				if (set_timer) {
					timer_delete(set_timer);
					set_timer=0;
				}
			}
			update_vfd();
		} else if (b==EVENT_P1_PUSH) {
			push_preset(0);
		} else if (b==EVENT_P2_PUSH) {
			push_preset(1);
		} else if (b==EVENT_P3_PUSH) {
			push_preset(2);
		} else if (b==EVENT_P4_PUSH) {
			push_preset(3);
		} else if (b==EVENT_P5_PUSH) {
			push_preset(4);
		} else if (b==EVENT_P6_PUSH) {
			push_preset(5);
		} else if (b==EVENT_P7_PUSH) {
			push_preset(6);
		} else if (b==EVENT_P8_PUSH) {
			push_preset(7);
		} else if (b==EVENT_P9_PUSH) {
			push_preset(8);
		} else if (b==EVENT_P10_PUSH) {
			push_preset(9);
		} else if (b==EVENT_P11_PUSH) {
			push_preset(10);
		} else if (b==EVENT_P1_REL) {
			release_preset(0);
		} else if (b==EVENT_P2_REL) {
			release_preset(1);
		} else if (b==EVENT_P3_REL) {
			release_preset(2);
		} else if (b==EVENT_P4_REL) {
			release_preset(3);
		} else if (b==EVENT_P5_REL) {
			release_preset(4);
		} else if (b==EVENT_P6_REL) {
			release_preset(5);
		} else if (b==EVENT_P7_REL) {
			release_preset(6);
		} else if (b==EVENT_P8_REL) {
			release_preset(7);
		} else if (b==EVENT_P9_REL) {
			release_preset(8);
		} else if (b==EVENT_P10_REL) {
			release_preset(9);
		} else if (b==EVENT_P11_REL) {
			release_preset(10);
		} else if (b==EVENT_SEEK_UP) {
//			if (only_clock && set) {
			if (set) {
				show_clock=1;
				if (minutes>=59) {
					minutes=0;
				} else {
					minutes++;
				}
				update_vfd();
				if (set_timer) {
					timer_delete(set_timer);
				}
				set_timer=timer_create(4,set_timeout,0);
				tbuf.minutes=minutes;
				io_control(clock_fd, CLOCK_SET_TIME, &tbuf, sizeof(tbuf));
			} else {
				seek_dir=SEEK_DIR_UP;
				mute_radio();
				seek_running=1;
				first=1;
				timer_set_seek_cb(seek_timeout,0);
			}
		} else if (b==EVENT_SEEK_DOWN) {
//			if (only_clock && set) {
			if (set) {
				show_clock=1;
				if (hours>=12) {
					hours=1;
				} else {
					hours++;
				}
				update_vfd();
				if (set_timer) {
					timer_delete(set_timer);
				}
				set_timer=timer_create(4,set_timeout,0);
				tbuf.hours=hours;
				io_control(clock_fd, CLOCK_SET_TIME, &tbuf, sizeof(tbuf));
			} else {
				seek_dir=SEEK_DIR_DOWN;
				mute_radio();
				seek_running=1;
				first=1;
				timer_set_seek_cb(seek_timeout,0);
			}
		} else if (b==EVENT_VOL_PUSH) {
			if (main_state==MAIN_STATE_RADIO_ON) {
				toggle_radio_display(pshow_clock);
			}
		} else if (b==EVENT_TUNER_PUSH) {
			if (sub_state==SUB_STATE_RADIO) {
				sub_state=SUB_STATE_BLUETOOTH;
				audio_bt();
			} else {
				sub_state=SUB_STATE_RADIO;
				audio_radio();
			}
			update_vfd();
		} else {
			printf("bad event val is %d\n", b);
		}
	}
	return 0;
}

static void tuner(void *dum) {

	tstats_fd	=io_open(TSTATS0);

	if (tstats_fd<0) {
//		printf("could not open tstat driver\n");
		goto skip1;
	}
	io_control(tstats_fd,F_SETFL,(void *)O_NONBLOCK,0);

	register_event(tstats_fd, EV_READ, tstats_event, 0);

	if (check_ignition(1)) ignition_on();
	check_rsense();

skip1:
	while(1) {
		do_event();
	}
}

//int main(void) {
int init_pkg(void) {
	int pwr_fd=io_open("pwr_mgr");
	if (pwr_fd<0) return 0;
	io_control(pwr_fd, GET_SYS_CLOCK, &init_clk,sizeof(init_clk));
	io_close(pwr_fd);

	myfd=io_open(SPI_DRV0);
	conf_fd=io_open("conf0");
	if (conf_fd>=0) {
		io_read(conf_fd, presets, sizeof(presets));
	}
//	install_cmd_node(&cmn, root_cmd_node);
//	init_pin_test();
	thread_create(tuner,0,0,1,"tuner");
	return 0;
}
