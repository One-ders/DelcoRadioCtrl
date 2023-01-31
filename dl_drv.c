/* $datalink drv: , v1.1 2023/01/06 21:44:00 anders Exp $ */

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
 * @(#)dl_drv.c
 */
#include <sys.h>
#include <io.h>
#include <string.h>
#include <led_drv.h>
#include <hr_timer.h>
#include <gpio_drv.h>

#include "dl_drv.h"

#define MIN(a,b) (a<b)?a:b
#define MAX_USERS 4

static struct driver *leddrv;
static struct driver *pindrv;
static struct driver *timerdrv;

//static int zero=0;
//static int one=1;

#ifdef LED_GREEN
unsigned int green=LED_GREEN;
#endif
#ifdef LED_RED
unsigned int red=LED_RED;
#endif
#ifdef LED_BLUE
unsigned int blue=LED_BLUE;
#endif

struct dl_data {
	struct device_handle *led_dh;
	struct device_handle *clk_pin_dh;
	struct device_handle *vf_strobe_pin_dh;
	struct device_handle *syn_strobe_pin_dh;
	struct device_handle *data_pin_dh;
	struct device_handle *timer_dh;
	struct blocker_list wblocker_list;
};

struct dl_user {
	struct device_handle dh;
	DRV_CBH callback;
	void	*userdata;
	int	events;
	int	in_use;
	struct	dl_data *dl_data;
};



static struct dl_data dl_data_0 = {
};

static struct dl_user dl_user[MAX_USERS];

static struct dl_user *get_dl_user(void) {
	int i;
	for(i=0;i<MAX_USERS;i++) {
		if (!dl_user[i].in_use) {
			dl_user[i].in_use=1;
			return &dl_user[i];
		}
	}
	return 0;
}

#if 0
static int wakeup_dl_users(struct dl_data *dld,int ev) {
	int i;
	for(i=0;i<MAX_USERS;i++) {
		if ((dl_user[i].in_use) &&
			(dl_user[i].dl_data==dld) &&
			(dl_user[i].events&ev) &&
			(dl_user[i].callback)) {
			dl_user[i].callback(&dl_user[i].dh,
				ev&dl_user[i].events,
				dl_user[i].userdata);
		}
	}
	return 0;
}
#endif

static void led_on(struct device_handle *led_dh) {
#ifdef LED_RED
	leddrv->ops->control(led_dh,LED_CTRL_ACTIVATE,&red,sizeof(red));
#elif defined(LED_BLUE)
	leddrv->ops->control(led_dh,LED_CTRL_ACTIVATE,&blue,sizeof(blue));
#endif
}

static void led_off(struct device_handle *led_dh) {
#ifdef LED_RED
	leddrv->ops->control(led_dh,LED_CTRL_DEACTIVATE,&red,sizeof(red));
#elif defined(LED_BLUE)
	leddrv->ops->control(led_dh,LED_CTRL_DEACTIVATE,&blue,sizeof(blue));
#endif
}

static int clkHalfPeriod=100;
#if 0
static int vf_strobe=0;
static int syn_strobe=0;
static int buf_in=0;
static int buf_out=0;
static const int numbufs=32;

struct bit_array {
	unsigned char bits[6];
};

struct bit_array live_line;
struct bit_array saved_data[32];

static int bits=0;

static void shift_in_bit(struct bit_array *ba, int bit) {

	int b7 =(ba->bits[0]&0x80)?1:0;
	int b15=(ba->bits[1]&0x80)?1:0;
	int b23=(ba->bits[2]&0x80)?1:0;
	int b31=(ba->bits[3]&0x80)?1:0;

	ba->bits[4]=(ba->bits[4]<<1)|b31;
	ba->bits[3]=(ba->bits[3]<<1)|b23;
	ba->bits[2]=(ba->bits[2]<<1)|b15;
	ba->bits[1]=(ba->bits[1]<<1)|b7;
	ba->bits[0]=(ba->bits[0]<<1)|bit;
}


static void save_line(struct bit_array *ba) {
	int i;
	int bytes=(ba->bits[5]+7)/8;
//	memcpy(&saved_data[buf_in&(numbufs-1)],ba, sizeof(struct bit_array));
	for (i=0;i<bytes;i++) {
		saved_data[buf_in&(numbufs-1)].bits[i]=ba->bits[bytes-i-1];
	}
	saved_data[buf_in&(numbufs-1)].bits[5]=ba->bits[5];
	buf_in++;
	if ((buf_in-buf_out)>numbufs) {
		buf_out=buf_in-numbufs;
//		sys_printf("buffer overrun\n");
	}
}
#endif

static volatile int busy=0;
static int current_bit_val=0;
static int current_bit_no=-1;
static int up=0;
static unsigned char barr[8];    // 8*8 -> 64 bits


static int clk_up(struct dl_data *dld) {
	int clk_hi=1;
	pindrv->ops->control(dld->clk_pin_dh,GPIO_SET_PIN,&clk_hi,sizeof(clk_hi));
//	sys_printf("clk_up\n");
	return 0;
}

static int clk_down(struct dl_data *dld) {
	int clk_lo=0;
	pindrv->ops->control(dld->clk_pin_dh,GPIO_SET_PIN,&clk_lo,sizeof(clk_lo));
//	sys_printf("clk_down\n");
	return 0;
}

static int data_out(struct dl_data *dld, int val) {
	pindrv->ops->control(dld->data_pin_dh,GPIO_SET_PIN,&val,sizeof(val));
//	sys_printf("data %d\n", val);
	return 0;
}

int fnc_vf;
static int strobe_on(struct dl_data *dld) {
	int on=1;
	if (fnc_vf) {
		pindrv->ops->control(dld->vf_strobe_pin_dh,GPIO_SET_PIN,&on,sizeof(on));
	} else {
		pindrv->ops->control(dld->syn_strobe_pin_dh,GPIO_SET_PIN,&on,sizeof(on));
	}
//	sys_printf("strobe on\n");
	return 0;
}

static int strobe_off(struct dl_data *dld) {
	int off=0;
	if (fnc_vf) {
		pindrv->ops->control(dld->vf_strobe_pin_dh,GPIO_SET_PIN,&off,sizeof(off));
	} else {
		pindrv->ops->control(dld->syn_strobe_pin_dh,GPIO_SET_PIN,&off,sizeof(off));
	}
//	sys_printf("strobe off\n");
	return 0;
}




//	pindrv->ops->control(dld->syn_strobe_pin_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));

static int strobe=0;

static int dl_timeout(struct device_handle *dh, int ev, void *dum) {
	struct dl_data *dld=(struct dl_data *)dum;

	if (up) {
		up=0;
		clk_up(dld);
		if (current_bit_no<0) {
			strobe=1;
			strobe_on(dld);
		}
	} else {
		up=1;
		clk_down(dld);
		if (current_bit_no<0) {
			if (strobe) {
				strobe=0;
				strobe_off(dld);
				led_off(dld->led_dh);
				busy=0;
				return 0;
#if 0
			} else {
				strobe=1;
				strobe_on(dld);
				timerdrv->ops->control(dld->timer_dh,HR_TIMER_SET,&clkHalfPeriod,sizeof(clkHalfPeriod));
				return 0;
#endif
			}
			return 0;
		}
		data_out(dld, current_bit_val?1:0);
		current_bit_no--;
		if (current_bit_no>=0) {
			current_bit_val=barr[current_bit_no/8]&(1<<(current_bit_no%8));
		}
	}
	timerdrv->ops->control(dld->timer_dh,HR_TIMER_SET,&clkHalfPeriod,sizeof(clkHalfPeriod));

	return 0;
}

static void put_out_bit(struct dl_data *dld) {
	clk_down(dld);
	up=1;
	data_out(dld, current_bit_val?1:0);
	current_bit_no--;
	current_bit_val=barr[current_bit_no/8]&(1<<(current_bit_no%8));
	timerdrv->ops->control(dld->timer_dh,HR_TIMER_SET,&clkHalfPeriod,sizeof(clkHalfPeriod));
}



static int write_bits(struct dl_data *dld, unsigned char *buf, int size) {
	int bits;

	busy=1;
	bits=buf[0]&0x7F;
	fnc_vf=buf[0]&0x80;
	memcpy(barr,&buf[1],size-1);
	led_on(dld->led_dh);
	current_bit_no=bits-1;
	current_bit_val=barr[current_bit_no/8]&(1<<(current_bit_no%8));
	put_out_bit(dld);
	while(busy);
	return size;
}

/*****  Driver API *****/

static struct device_handle *dl_drv_open(void *inst, DRV_CBH cb, void *udata) {
	struct dl_user *u=get_dl_user();

	if (!u) return 0;
	u->dl_data=(struct dl_data *)inst;
	u->callback=cb;
	u->userdata=udata;
	u->events=0;
	return &u->dh;
}

static int dl_drv_close(struct device_handle *dh) {
	struct dl_user *u=(struct dl_user *)dh;
	if (!u) {
		return 0;
	}
	if (u) {
		u->in_use=0;
		u->dl_data=0;
	}
	return 0;
}

static int dl_drv_control(struct device_handle *dh, int cmd, void *arg, int size) {
	struct dl_user *u=(struct dl_user *)dh;
	struct dl_data *dld;

	if (!u) {
		return -1;
	}
	dld=u->dl_data;

	switch(cmd) {
		case RD_CHAR: {
#if 0
			int len=0;
			int *buf=(int *)arg;
			if ((buf_in-buf_out)<=0) {
				u->events|=EV_READ;
				return -DRV_AGAIN;
			}

			u->events&=~EV_READ;
			len=MIN(size,sizeof(struct bit_array));
			memcpy(buf,&saved_data[buf_out&(numbufs-1)],len);
			buf_out++;
			return len;
#endif
			return -1;
			break;
		}
		case WR_CHAR: {
			unsigned char *buf=(unsigned char *)arg;

			if (busy) return -1;
			return write_bits(dld,buf,size);

			break;
		}
		case IO_POLL: {
//			unsigned int events=(unsigned int)arg;
			unsigned int revents=0;
#if 0
			if (EV_READ&events) {
				if (buf_in-buf_out) {
					revents|=EV_READ;
				} else {
					u->events|=EV_READ;
				}
			}
#endif
			return revents;
			break;
		}
	}
	return -1;
}

static int dl_drv_init(void *inst) {
	return 0;
}

static int dl_drv_start(void *inst) {
	struct dl_data *dld=(struct dl_data *)inst;
	int flags;
	int rc;
	int clk_pin;
	int vf_strobe_pin;
	int syn_strobe_pin;
	int data_pin;

	/* Open Led driver so we can flash the leds a bit */
	if (!leddrv) leddrv=driver_lookup(LED_DRV);
	if (!leddrv) return 0;
	dld->led_dh=leddrv->ops->open(leddrv->instance,0,0);
	if (!dld->led_dh) return 0;

	if (!pindrv) pindrv=driver_lookup(GPIO_DRV);
	if (!pindrv) {
		sys_printf("DL: missing GPIO_DRV\n");
		goto out1;
	}

	dld->clk_pin_dh=pindrv->ops->open(pindrv->instance,0,(void *)dld);
	if (!dld->clk_pin_dh) {
		sys_printf("DL: could not open GPIO_DRV\n");
		goto out1;
	}

	/* bind vf_strobe pin */
	dld->vf_strobe_pin_dh=pindrv->ops->open(pindrv->instance,0,(void *)dld);
	if (!dld->vf_strobe_pin_dh) {
		sys_printf("DL: could not open second inst. GPIO_DRV\n");
		goto out2;
	}

	/* open syn_strobe pin */
	dld->syn_strobe_pin_dh=pindrv->ops->open(pindrv->instance,0,(void *)dld);
	if (!dld->syn_strobe_pin_dh) {
		sys_printf("DL: could not open third inst. GPIO_DRV\n");
		goto out2;
	}

	/* open data pin */
	dld->data_pin_dh=pindrv->ops->open(pindrv->instance,0,(void *)dld);
	if (!dld->data_pin_dh) {
		sys_printf("DL: could not open third inst. GPIO_DRV\n");
		goto out2;
	}

	/* Open High Resolution timer for pulse meassurements */
	if (!timerdrv) timerdrv=driver_lookup(HR_TIMER);
	if (!timerdrv) {
		sys_printf("DL: missing HR_TIMER\n");
		goto out3;
	}
	dld->timer_dh=timerdrv->ops->open(timerdrv->instance,dl_timeout,(void *)dld);
	if (!dld->timer_dh) {
		sys_printf("DL: could not open HR_TIMER\n");
		goto out3;
	}

	if (dld==&dl_data_0) {
		clk_pin=CLK_PIN;
		vf_strobe_pin=VF_STROBE_PIN;
		syn_strobe_pin=SYN_STROBE_PIN;
		data_pin=DATA_PIN;
	} else {
		sys_printf("DL protocol driver: error no pin assigned for driver\n");
		goto out4;
	}

	/* Program clk pin to output */
	rc=pindrv->ops->control(dld->clk_pin_dh,GPIO_BIND_PIN,&clk_pin,sizeof(clk_pin));
	if (rc<0) {
		sys_printf("DL protocol driver: failed to bind clk pin\n");
		goto out4;
	}

	flags=GPIO_DIR(0,GPIO_OUTPUT);
	flags=GPIO_DRIVE(flags,GPIO_PUSHPULL);
	flags=GPIO_SPEED(flags,GPIO_SPEED_HIGH);
//	flags=GPIO_IRQ_ENABLE(flags);
	rc=pindrv->ops->control(dld->clk_pin_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("DL reader driver: pin_flags update failed\n");
		goto out4;
	}

	/* Program vf strobe pin to be output push pull*/
	rc=pindrv->ops->control(dld->vf_strobe_pin_dh,GPIO_BIND_PIN,&vf_strobe_pin,sizeof(vf_strobe_pin));
	if (rc<0) {
		sys_printf("DL reader driver: vf_strobe bind failed\n");
		goto out4;
	}

	flags=GPIO_DIR(0,GPIO_OUTPUT);
	flags=GPIO_DRIVE(flags,GPIO_PUSHPULL);
	flags=GPIO_SPEED(flags,GPIO_SPEED_HIGH);
//	flags=GPIO_IRQ_ENABLE(flags);
	rc=pindrv->ops->control(dld->vf_strobe_pin_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("DL reader driver: vf_strobe_flags update failed\n");
		goto out4;
	}

	/* Program syn strobe pin to be output*/
	rc=pindrv->ops->control(dld->syn_strobe_pin_dh,GPIO_BIND_PIN,&syn_strobe_pin,sizeof(syn_strobe_pin));
	if (rc<0) {
		sys_printf("DL reader driver: syn_strobe bind failed\n");
		goto out4;
	}

	flags=GPIO_DIR(0,GPIO_OUTPUT);
	flags=GPIO_DRIVE(flags,GPIO_PUSHPULL);
	flags=GPIO_SPEED(flags,GPIO_SPEED_HIGH);
//	flags=GPIO_IRQ_ENABLE(flags);
	rc=pindrv->ops->control(dld->syn_strobe_pin_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("DL reader driver: syn_strobe_flags update failed\n");
		goto out4;
	}

	/* Program data pin to be output */
	rc=pindrv->ops->control(dld->data_pin_dh,GPIO_BIND_PIN,&data_pin,sizeof(data_pin));
	if (rc<0) {
		sys_printf("DL reader driver: data pin bind failed\n");
		goto out4;
	}

	flags=GPIO_DIR(0,GPIO_OUTPUT);
	flags=GPIO_DRIVE(flags,GPIO_PUSHPULL);
	flags=GPIO_SPEED(flags,GPIO_SPEED_HIGH);
//	flags=GPIO_IRQ_ENABLE(flags);
	rc=pindrv->ops->control(dld->data_pin_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("DL reader driver: strobe_flags update failed\n");
		goto out4;
	}

	led_off(dld->led_dh);
	clk_down(dld);
	fnc_vf=1;
	strobe_off(dld);
	fnc_vf=0;
	strobe_off(dld);
	data_out(dld, 0);

	sys_printf("DL protocol driver: Started\n");

	return 0;

out4:
	sys_printf("DL: failed to bind pin to GPIO\n");
	timerdrv->ops->close(dld->timer_dh);

out3:
	pindrv->ops->close(dld->syn_strobe_pin_dh);
	pindrv->ops->close(dld->vf_strobe_pin_dh);

out2:
	pindrv->ops->close(dld->clk_pin_dh);

out1:
	leddrv->ops->close(dld->led_dh);
	return 0;
}



static struct driver_ops dl_drv_ops = {
	dl_drv_open,
	dl_drv_close,
	dl_drv_control,
	dl_drv_init,
	dl_drv_start,
};

static struct driver dl_drv = {
	SPI_DRV0,
	&dl_data_0,
	&dl_drv_ops,
};


void init_dl(void) {
	driver_publish(&dl_drv);
}

INIT_FUNC(init_dl);
