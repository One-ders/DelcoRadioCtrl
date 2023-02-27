/* $ctrls drv: , v1.1 2023/02/10 21:44:00 anders Exp $ */

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
 * @(#)controls.c
 */
#include <sys.h>
#include <io.h>
#include <string.h>
#include <led_drv.h>
#include <hr_timer.h>
#include <gpio_drv.h>

#include "controls_drv.h"

#define MIN(a,b) (a<b)?a:b
#define MAX_USERS 4

static struct driver *pindrv;
static struct driver *timerdrv;

static int zero=0;
static int one=1;

static int sweep_step_time=50000;

struct ctrls_data {
	struct device_handle *timer_dh;

	/* for tuning knob */
	struct device_handle *tw_pin1_dh;
	struct device_handle *tw_pin2_dh;
	struct device_handle *tw_pin4_dh;

	/* for Key matrix pins */
	struct device_handle *p4_seek_dh;
	struct device_handle *p2_dh;
	struct device_handle *p1p3_dh;
	struct device_handle *set_scan_dh;
	struct device_handle *cp2p3p4set_dh;
	struct device_handle *cp1scan_seek_dh;

	/* other pins */
	struct device_handle *recal_dh;
	struct device_handle *am_fm_dh;



	struct blocker_list wblocker_list;
};

#define EVENT_BUF_SIZE	32
static unsigned int event[EVENT_BUF_SIZE];
static unsigned int ev_i=0;
static unsigned int ev_o=0;

#define to_ix(a)		(a&(EVENT_BUF_SIZE-1))

struct drv_user {
	struct device_handle dh;
	DRV_CBH callback;
	void	*userdata;
	int	events;
	int	in_use;
	struct	ctrls_data *ctrls_data;
};



static struct ctrls_data ctrls_data_0 = {
};

static struct drv_user drv_user[MAX_USERS];

static struct drv_user *get_drv_user(void) {
	int i;
	for(i=0;i<MAX_USERS;i++) {
		if (!drv_user[i].in_use) {
			drv_user[i].in_use=1;
			return &drv_user[i];
		}
	}
	return 0;
}

static int wakeup_drv_users(struct ctrls_data *ctd,int ev) {
	int i;
	for(i=0;i<MAX_USERS;i++) {
		if ((drv_user[i].in_use) &&
			(drv_user[i].ctrls_data==ctd) &&
			(drv_user[i].events&ev) &&
			(drv_user[i].callback)) {
			drv_user[i].callback(&drv_user[i].dh,
				ev&drv_user[i].events,
				drv_user[i].userdata);
		}
	}
	return 0;
}


/////////////////// Tune knob turned clockwise or counter clockwise ///////////////
static int prev_pin;

static int tune_event(struct ctrls_data *ctd, int up) {
//	t_ev=up;
	if (up>0) {
		sys_printf("tune up\n");
		event[to_ix(ev_i)]=EVENT_TUNE_UP;
	} else {
		sys_printf("tune do\n");
		event[to_ix(ev_i)]=EVENT_TUNE_DOWN;
	}
	ev_i++;
	wakeup_drv_users(ctd, EV_READ);
	return 0;
}

static int tw_pin1_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct ctrls_data *ctd=&ctrls_data_0;

	pindrv->ops->control(ctd->tw_pin1_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));

        if (pin_stat==0) {
		sys_printf("enter pin 1 low\n");
		switch (prev_pin) {
			case 1:
				break;
			case 2:
				tune_event(ctd, -1);
				break;
			case 4:
				tune_event(ctd, 1);
				break;
			default:
				sys_printf("pin 1 unhandled prev pin val\n");
		}
		prev_pin=1;
                return 0;
	} else {
		prev_pin=1;
		sys_printf("leaving pin %d\n", prev_pin);
	}
	return 0;
}

static int tw_pin2_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct ctrls_data *ctd=&ctrls_data_0;

	pindrv->ops->control(ctd->tw_pin2_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));

        if (pin_stat==0) {
		sys_printf("enter pin 2 low\n");
		switch (prev_pin) {
			case 1:
				tune_event(ctd, 1);
				break;
			case 2:
				break;
			case 4:
				tune_event(ctd, -1);
				break;
			default:
				sys_printf("pin 2 unhandled prev pin val\n");
		}
		prev_pin=2;
                return 0;
        } else {
		prev_pin=2;
		sys_printf("leaving pin %d\n", prev_pin);
	}

	return 0;
}

static int tw_pin4_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct ctrls_data *ctd=&ctrls_data_0;

	pindrv->ops->control(ctd->tw_pin4_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));

        if (pin_stat==0) {
		sys_printf("enter pin 4 low\n");
		switch (prev_pin) {
			case 1:
				tune_event(ctd, -1);
				break;
			case 2:
				tune_event(ctd, 1);
				break;
			case 4:
				break;
			default:
				sys_printf("pin 4 unhandled prev pin val\n");
		}
		prev_pin=4;
                return 0;
        } else {
		prev_pin=4;
		sys_printf("leaving pin %d\n", prev_pin);
	}
	return 0;
}

////////////////////////////////////////////////////////////////////////////////////
////////////////////// Key Matrix //////////////////////////////////////////////////

#define PB_STATE_OFF 0
#define PB_STATE_ON  1
struct phys_button {
	int state;
	int lb;
	int time;
};

static int cp1scan_seek=0;
static int cp2p3p4set=0;

static int seek=0;
static int scan=0;
static int set=0;

static struct phys_button phys_button[4];

static int sweep_cnt=0;

int (*pb_tfunc)(struct phys_button *, int);
struct phys_button *pb_tdata;

#if 0

				event[to_ix(ev_i)]=EVENT_P4_PUSH;
				ev_i++;
				wakeup_drv_users(ctd, EV_READ);
//				sys_printf("p4 pushed\n");
				p4=1;
#endif

static int key2ev(int key, int state) {
	int ev=0;
	switch(key) {
		case 1:
			ev=(state==1)?EVENT_P1_PUSH:EVENT_P1_REL;
			break;
		case 2:
			ev=(state==1)?EVENT_P2_PUSH:EVENT_P2_REL;
			break;
		case 3:
			ev=(state==1)?EVENT_P3_PUSH:EVENT_P3_REL;
			break;
		case 4:
			ev=(state==1)?EVENT_P4_PUSH:EVENT_P4_REL;
			break;
		case 5:
			ev=(state==1)?EVENT_P5_PUSH:EVENT_P5_REL;
			break;
		case 6:
			ev=(state==1)?EVENT_P6_PUSH:EVENT_P6_REL;
			break;
		case 7:
			ev=(state==1)?EVENT_P7_PUSH:EVENT_P7_REL;
			break;
		case 8:
			ev=(state==1)?EVENT_P8_PUSH:EVENT_P8_REL;
			break;
		case 9:
			ev=(state==1)?EVENT_P9_PUSH:EVENT_P9_REL;
			break;
		case 10:
			ev=(state==1)?EVENT_P10_PUSH:EVENT_P10_REL;
			break;
		case 11:
			ev=(state==1)?EVENT_P11_PUSH:EVENT_P11_REL;
			break;
	}
	return ev;
}


static int lb_on(int key) {
	struct ctrls_data *ctd=&ctrls_data_0;
	sys_printf("key %d active\n", key);

	event[to_ix(ev_i)]=key2ev(key,1);
	ev_i++;
	wakeup_drv_users(ctd, EV_READ);
	return 0;
}

static int lb_off(int key) {
	struct ctrls_data *ctd=&ctrls_data_0;
	sys_printf("key %d deactive\n", key);
	event[to_ix(ev_i)]=key2ev(key,0);
	ev_i++;
	wakeup_drv_users(ctd, EV_READ);
	return 0;
}


static int pb_timeout(struct phys_button *b, int time) {
//	sys_printf("pb timeout for %x at %d\n",
//				b-phys_button,time);
	if ((!b->lb)&&time&&(time-b->time)>=5) {
		b->time=time;
		b->lb=(b-phys_button)+1;
//		sys_printf("pb_tout prim function\n");
		lb_on(b->lb);
		if (b->state==PB_STATE_ON) {
			pb_tfunc=0;
		}
	} else if ((b->lb) && ((time-b->time)>4)) {
		pb_tfunc=0;
		b->time=0;
//		sys_printf("pb_tout function off\n");
		lb_off(b->lb);
		b->lb=0;
	}

	return 0;
}

static int wait;
static int pb_event(struct phys_button *b, int state, int time) {
	b->state=state;
//	sys_printf("pb_event: b=%x, state=%d, time=%d\n",
//			b-phys_button, state, time);

	if (state==PB_STATE_ON) {
		switch(b-phys_button) {
			case 0: // first button
				if (phys_button[1].state==PB_STATE_ON) {
					b->time=0;
					phys_button[1].time=0;
					pb_tfunc=0;
					b->lb=9;
					phys_button[1].lb=9;
//					sys_printf("lb=9\n");
					lb_on(9);
					goto out;
				}
				break;
			case 1:
				if (phys_button[0].state==PB_STATE_ON) {
					b->time=0;
					phys_button[0].time=0;
					pb_tfunc=0;
					b->lb=9;
					phys_button[0].lb=9;
//					sys_printf("lb=9\n");
					lb_on(9);
					goto out;
				} else if (phys_button[2].state==PB_STATE_ON) {
					b->time=0;
					phys_button[2].time=0;
					pb_tfunc=0;
					b->lb=10;
					phys_button[2].lb=10;
//					sys_printf("lb=10\n");
					lb_on(10);
					goto out;
				}
				break;
			case 2:
				if (phys_button[1].state==PB_STATE_ON) {
					b->time=0;
					phys_button[1].time=0;
					pb_tfunc=0;
					b->lb=10;
					phys_button[1].lb=10;
//					sys_printf("lb=10\n");
					lb_on(10);
					goto out;
				} else if (phys_button[3].state==PB_STATE_ON) {
					b->time=0;
					phys_button[3].time=0;
					pb_tfunc=0;
					b->lb=11;
					phys_button[3].lb=11;
//					sys_printf("lb=11\n");
					lb_on(11);
					goto out;
				}
				break;
			case 3:
				if (phys_button[2].state==PB_STATE_ON) {
					b->time=0;
					phys_button[2].time=0;
					pb_tfunc=0;
					b->lb=11;
					phys_button[2].lb=11;
//					sys_printf("lb=11\n");
					lb_on(11);
					goto out;
				}
				break;
		}


		if (!b->time) {
			b->time=time;
		} else if (b->time&&((time-b->time)<=4)) {
			b->time=0;
			pb_tfunc=0;
			b->lb=(b-phys_button)+5;
//			sys_printf("pb_event: activate second\n");
			lb_on(b->lb);
			goto out;
		}
	} else {
		if (b->lb) {
			b->time=0;
			pb_tfunc=0;
//			sys_printf("pb_event: deactivate %d\n", b->lb);
			lb_off(b->lb);
			if (b->lb>8) { // double keys
				int hi_b=b->lb-8;
				int pb=b-phys_button;
				if (pb==hi_b) {
					phys_button[hi_b-1].lb=0;
				} else {
					phys_button[hi_b].lb=0;
				}
				wait=1;
			} else {
				wait=0;
			}
			b->lb=0;
			goto out;
		}
	}

	if (wait) {
		b->time=0;
		pb_tfunc=0;
		wait=0;
	} else {
		pb_tdata=b;
		pb_tfunc=pb_timeout;
	}
out:
	return 0;
}


static int p4_seek_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct ctrls_data *ctd=&ctrls_data_0;

	pindrv->ops->control(ctd->p4_seek_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
	if (!pin_stat) {
		if (cp1scan_seek) {
			if (!seek) {
				event[to_ix(ev_i)]=EVENT_SEEK_UP;
				ev_i++;
				wakeup_drv_users(ctd, EV_READ);
//				sys_printf("seek pushed\n");
				seek=1;
			}
		} else if (cp2p3p4set) {
			if (phys_button[3].state==PB_STATE_OFF) {
				pb_event(&phys_button[3],PB_STATE_ON,sweep_cnt);
			}
		} else {
			sys_printf("p4_seek spurious irq\n");
		}
	}

	return 0;
}

static int p2_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct ctrls_data *ctd=&ctrls_data_0;

	pindrv->ops->control(ctd->p2_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
	if (!pin_stat) {
		if (cp1scan_seek) {
			sys_printf("pb2 spurious\n");
		} else if (cp2p3p4set) {
			if (phys_button[1].state==PB_STATE_OFF) {
				pb_event(&phys_button[1],PB_STATE_ON,sweep_cnt);
			}
		} else {
			sys_printf("p2 spurious2 irq\n");
		}
	}
	return 0;
}

static int p1p3_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct ctrls_data *ctd=&ctrls_data_0;

	pindrv->ops->control(ctd->p1p3_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
	if (!pin_stat) {
		if (cp1scan_seek) {
			if (phys_button[0].state==PB_STATE_OFF) {
				pb_event(&phys_button[0],PB_STATE_ON,sweep_cnt);
			}
		} else if (cp2p3p4set) {
			if (phys_button[2].state==PB_STATE_OFF) {
				pb_event(&phys_button[2],PB_STATE_ON,sweep_cnt);
			}
		} else {
			sys_printf("p1p3 spurious irq\n");
		}
	}
	return 0;
}

static int set_scan_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct ctrls_data *ctd=&ctrls_data_0;

	pindrv->ops->control(ctd->set_scan_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
	if (!pin_stat) {
		if (cp1scan_seek) {
			if (!scan) {
				event[to_ix(ev_i)]=EVENT_SEEK_DOWN;
				ev_i++;
				wakeup_drv_users(ctd, EV_READ);
//				sys_printf("scan pushed\n");
				scan=1;
			}
		} else if (cp2p3p4set) {
			if (!set) {
				event[to_ix(ev_i)]=EVENT_SET;
				ev_i++;
				wakeup_drv_users(ctd, EV_READ);
//				sys_printf("set pushed\n");
				set=1;
			}
		} else {
			sys_printf("set_scan spurious irq\n");
		}
	} else {
//		sys_printf("set_scan went hi\n");
	}
	return 0;
}

static int ctrls_timeout(struct device_handle *dh, int ev, void *dum) {
	struct ctrls_data *ctd=(struct ctrls_data *)dum;

	sweep_cnt++;

	if (!cp1scan_seek) {
		int pin_stat=0;
		cp1scan_seek=1;
		cp2p3p4set=0;
		if (phys_button[1].state==PB_STATE_ON) {
			pindrv->ops->control(ctd->p2_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
			if (pin_stat) {
				pb_event(&phys_button[1],PB_STATE_OFF,sweep_cnt);
			}
		}
		if (phys_button[2].state==PB_STATE_ON) {
			pindrv->ops->control(ctd->p1p3_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
			if (pin_stat) {
				pb_event(&phys_button[2],PB_STATE_OFF,sweep_cnt);
			}
		}
		if (phys_button[3].state==PB_STATE_ON) {
			pindrv->ops->control(ctd->p4_seek_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
			if (pin_stat) {
				pb_event(&phys_button[3],PB_STATE_OFF,sweep_cnt);
			}
		}
		if (set) {
			pindrv->ops->control(ctd->set_scan_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
			if (pin_stat) {
				set=0;
				sys_printf("set released\n");
			}
		}

		pindrv->ops->control(ctd->cp2p3p4set_dh, GPIO_SET_PIN,&one,sizeof(one));
		pindrv->ops->control(ctd->cp1scan_seek_dh, GPIO_SET_PIN,&zero,sizeof(zero));
	} else {
		int pin_stat=0;
		cp1scan_seek=0;
		cp2p3p4set=1;
			pindrv->ops->control(ctd->p2_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
		if (phys_button[0].state==PB_STATE_ON) {
			pindrv->ops->control(ctd->p1p3_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
			if (pin_stat) {
				pb_event(&phys_button[0],PB_STATE_OFF,sweep_cnt);
			}
		}
		if (scan) {
			pindrv->ops->control(ctd->set_scan_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
			if (pin_stat) {
				scan=0;
			}
		}
		if (seek) {
			pindrv->ops->control(ctd->p4_seek_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
			if (pin_stat) {
				seek=0;
			}
		}

		pindrv->ops->control(ctd->cp1scan_seek_dh, GPIO_SET_PIN,&one,sizeof(one));
		pindrv->ops->control(ctd->cp2p3p4set_dh, GPIO_SET_PIN,&zero,sizeof(zero));
	}
	if (pb_tfunc) {
		pb_tfunc(pb_tdata,sweep_cnt);
	}
	timerdrv->ops->control(ctd->timer_dh,HR_TIMER_SET,&sweep_step_time,sizeof(sweep_step_time));
	return 0;
}

//
static int recal_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct ctrls_data *ctd=&ctrls_data_0;

	pindrv->ops->control(ctd->recal_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));

	if (pin_stat==0) {
		event[to_ix(ev_i)]=EVENT_VOL_PUSH;
		ev_i++;
		wakeup_drv_users(ctd, EV_READ);
	}
	return 0;
}

static int am_fm_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct ctrls_data *ctd=&ctrls_data_0;

	pindrv->ops->control(ctd->am_fm_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
	if (pin_stat==0) {
		event[to_ix(ev_i)]=EVENT_TUNER_PUSH;
		ev_i++;
		wakeup_drv_users(ctd, EV_READ);
	}
	return 0;
}

/*****  Driver API *****/

static struct device_handle *ctrls_open(void *inst, DRV_CBH cb, void *udata) {
	struct drv_user *u=get_drv_user();
	struct ctrls_data *ctd;

	if (!u) return 0;
	ctd=u->ctrls_data=(struct ctrls_data *)inst;
	u->callback=cb;
	u->userdata=udata;
	u->events=0;
	timerdrv->ops->control(ctd->timer_dh,HR_TIMER_SET,&sweep_step_time,sizeof(sweep_step_time));
	return &u->dh;
}

static int ctrls_close(struct device_handle *dh) {
	struct drv_user *u=(struct drv_user *)dh;
	if (!u) {
		return 0;
	}
	if (u) {
		u->in_use=0;
		u->ctrls_data=0;
	}
	return 0;
}

static int ctrls_control(struct device_handle *dh, int cmd, void *arg, int size) {
	struct drv_user *u=(struct drv_user *)dh;
//	struct tw_data *twd;

	if (!u) {
		return -1;
	}
//	twd=u->tw_data;

	switch(cmd) {
		case RD_CHAR: {
			int i;
			for(i=ev_o;i<ev_i;i++) {
				*((unsigned int *)arg)=event[to_ix(i)];
				ev_o=++i;
				return sizeof(arg);
			}
			return -DRV_AGAIN;
			break;
		}
		case WR_CHAR: {
			return -1;

			break;
		}
		case IO_POLL: {
			unsigned int events=(unsigned int)arg;
			unsigned int revents=0;
			if (EV_READ&events) {
				if (ev_i>ev_o) {
					revents|=EV_READ;
				} else {
					u->events|=EV_READ;
				}
			}
			return revents;
			break;
		}
	}
	return -1;
}

static int ctrls_init(void *inst) {
	return 0;
}

static int ctrls_start(void *inst) {
	struct ctrls_data *ctd=(struct ctrls_data *)inst;
	int flags;
	int rc;
	int tw_pin1;
	int tw_pin2;
	int tw_pin4;

	int p4_seek_pin=P4Seek;
	int p2_pin=P2;
	int p1p3_pin=P1P3;
	int set_scan_pin=SetScan;

	int cp2p3p4set_pin=CP2P3P4Set;
	int cp1scan_seek_pin=CP1ScanSeek;

	int recal_pin=RECAL;
	int am_fm_pin=AM_FM;


	if (!pindrv) pindrv=driver_lookup(GPIO_DRV);
	if (!pindrv) {
		sys_printf("DL: missing GPIO_DRV\n");
		goto out1;
	}

	if (!timerdrv) timerdrv=driver_lookup(HR_TIMER);
	if (!timerdrv) {
		goto out1;
	}

	ctd->timer_dh=timerdrv->ops->open(timerdrv->instance,ctrls_timeout,(void *)ctd);
	if (!ctd->timer_dh) {
		goto out1;
	}

	ctd->tw_pin1_dh=pindrv->ops->open(pindrv->instance,tw_pin1_irq,(void *)ctd);
	if (!ctd->tw_pin1_dh) {
		sys_printf("CTRLS: could not open GPIO_DRV\n");
		goto out1;
	}

	/* open tw pin 2 */
	ctd->tw_pin2_dh=pindrv->ops->open(pindrv->instance,tw_pin2_irq,(void *)ctd);
	if (!ctd->tw_pin2_dh) {
		sys_printf("CTRLS: could not open second inst. GPIO_DRV\n");
		goto out2;
	}

	/* open tw pin 4 */
	ctd->tw_pin4_dh=pindrv->ops->open(pindrv->instance,tw_pin4_irq,(void *)ctd);
	if (!ctd->tw_pin4_dh) {
		sys_printf("CTRLS: could not open third inst. GPIO_DRV\n");
		goto out2;
	}

	if (ctd==&ctrls_data_0) {
		tw_pin1=TUNE_WHEEL_1;
		tw_pin2=TUNE_WHEEL_2;
		tw_pin4=TUNE_WHEEL_4;
	} else {
		sys_printf("CTRLS protocol driver: error no pin assigned for driver\n");
		goto out4;
	}

	/* Program tune wheel pins to input and pull up */
	rc=pindrv->ops->control(ctd->tw_pin1_dh,GPIO_BIND_PIN,&tw_pin1,sizeof(tw_pin1));
	if (rc<0) {
		sys_printf("CTRLS protocol driver: failed to bind TW pin\n");
		goto out4;
	}

	flags=GPIO_DIR(0,GPIO_INPUT);
	flags=GPIO_DRIVE(flags,GPIO_PULLUP);
	flags=GPIO_SPEED(flags,GPIO_SPEED_MEDIUM);
	flags=GPIO_IRQ_ENABLE(flags);
	rc=pindrv->ops->control(ctd->tw_pin1_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("TW reader driver: pin_flags update failed\n");
		goto out4;
	}

	rc=pindrv->ops->control(ctd->tw_pin2_dh,GPIO_BIND_PIN,&tw_pin2,sizeof(tw_pin2));
	if (rc<0) {
		sys_printf("TW reader driver: tw pin 2 bind failed\n");
		goto out4;
	}

#if 0
	flags=GPIO_DIR(0,GPIO_INPUT);
	flags=GPIO_DRIVE(flags,GPIO_PULLUP);
	flags=GPIO_SPEED(flags,GPIO_SPEED_MEDIUM);
	flags=GPIO_IRQ_ENABLE(flags);
#endif
	rc=pindrv->ops->control(ctd->tw_pin2_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("TW reader driver: pin flags update failed\n");
		goto out4;
	}

	rc=pindrv->ops->control(ctd->tw_pin4_dh,GPIO_BIND_PIN,&tw_pin4,sizeof(tw_pin4));
	if (rc<0) {
		sys_printf("TW reader driver: tw pin 3 bind failed\n");
		goto out4;
	}

#if 0
	flags=GPIO_DIR(0,GPIO_INPUT);
	flags=GPIO_DRIVE(flags,GPIO_PULLUP);
	flags=GPIO_SPEED(flags,GPIO_SPEED_MEDIUM);
	flags=GPIO_IRQ_ENABLE(flags);
#endif
	rc=pindrv->ops->control(ctd->tw_pin4_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("TW reader driver: pin flags update failed\n");
		goto out4;
	}

	sys_printf("TW protocol driver: Started\n");


//	take care of Key matrix pins

	ctd->p4_seek_dh=pindrv->ops->open(pindrv->instance,p4_seek_irq,(void *)ctd);
	if (!ctd->p4_seek_dh) {
		sys_printf("CTRLS: could not open GPIO_DRV\n");
		goto out4;
	}

	ctd->p2_dh=pindrv->ops->open(pindrv->instance,p2_irq,(void *)ctd);
	if (!ctd->p2_dh) {
		sys_printf("CTRLS: could not open GPIO_DRV\n");
		goto out4;
	}

	ctd->p1p3_dh=pindrv->ops->open(pindrv->instance,p1p3_irq,(void *)ctd);
	if (!ctd->p1p3_dh) {
		sys_printf("CTRLS: could not open GPIO_DRV\n");
		goto out4;
	}

	ctd->set_scan_dh=pindrv->ops->open(pindrv->instance,set_scan_irq,(void *)ctd);
	if (!ctd->set_scan_dh) {
		sys_printf("CTRLS: could not open GPIO_DRV\n");
		goto out4;
	}

	ctd->cp2p3p4set_dh=pindrv->ops->open(pindrv->instance,NULL,(void *)ctd);
	if (!ctd->cp2p3p4set_dh) {
		sys_printf("CTRLS: could not open GPIO_DRV\n");
		goto out4;
	}

	ctd->cp1scan_seek_dh=pindrv->ops->open(pindrv->instance,NULL,(void *)ctd);
	if (!ctd->cp1scan_seek_dh) {
		sys_printf("CTRLS: could not open GPIO_DRV\n");
		goto out4;
	}


	/* Program key matrix irq pins to input and pull up */
	rc=pindrv->ops->control(ctd->p4_seek_dh,GPIO_BIND_PIN,&p4_seek_pin,sizeof(p4_seek_pin));
	if (rc<0) {
		sys_printf("CTRLS protocol driver: failed to bind KM pin\n");
		goto out4;
	}

	rc=pindrv->ops->control(ctd->p2_dh,GPIO_BIND_PIN,&p2_pin,sizeof(p2_pin));
	if (rc<0) {
		sys_printf("CTRLS protocol driver: failed to bind KM pin\n");
		goto out4;
	}

	rc=pindrv->ops->control(ctd->p1p3_dh,GPIO_BIND_PIN,&p1p3_pin,sizeof(p1p3_pin));
	if (rc<0) {
		sys_printf("CTRLS protocol driver: failed to bind KM pin\n");
		goto out4;
	}

	rc=pindrv->ops->control(ctd->set_scan_dh,GPIO_BIND_PIN,&set_scan_pin,sizeof(set_scan_pin));
	if (rc<0) {
		sys_printf("CTRLS protocol driver: failed to bind KM pin\n");
		goto out4;
	}


#if 0
	flags=GPIO_DIR(0,GPIO_INPUT);
	flags=GPIO_DRIVE(flags,GPIO_PULLUP);
	flags=GPIO_SPEED(flags,GPIO_SPEED_MEDIUM);
	flags=GPIO_IRQ_ENABLE(flags);
#endif

	rc=pindrv->ops->control(ctd->p4_seek_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("TW reader driver: pin_flags update failed\n");
		goto out4;
	}

	rc=pindrv->ops->control(ctd->p2_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("TW reader driver: pin_flags update failed\n");
		goto out4;
	}

	rc=pindrv->ops->control(ctd->p1p3_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("TW reader driver: pin_flags update failed\n");
		goto out4;
	}

	rc=pindrv->ops->control(ctd->set_scan_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("TW reader driver: pin_flags update failed\n");
		goto out4;
	}



	rc=pindrv->ops->control(ctd->cp2p3p4set_dh,GPIO_BIND_PIN,&cp2p3p4set_pin,sizeof(cp2p3p4set_pin));
	if (rc<0) {
		sys_printf("CTRLS protocol driver: failed to bind KM pin\n");
		goto out4;
	}

	rc=pindrv->ops->control(ctd->cp1scan_seek_dh,GPIO_BIND_PIN,&cp1scan_seek_pin,sizeof(cp1scan_seek_pin));
	if (rc<0) {
		sys_printf("CTRLS protocol driver: failed to bind KM pin\n");
		goto out4;
	}

        flags=GPIO_DIR(0,GPIO_OUTPUT);
        flags=GPIO_DRIVE(flags,GPIO_PUSHPULL);
        flags=GPIO_SPEED(flags,GPIO_SPEED_MEDIUM);
        rc=pindrv->ops->control(ctd->cp2p3p4set_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
        if (rc<0) {
                sys_printf("CTRLS driver: cp2p3p4set flags update failed\n");
                goto out4;
        }

        rc=pindrv->ops->control(ctd->cp1scan_seek_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
        if (rc<0) {
                sys_printf("CTRLS driver: cp1scan_seek flags update failed\n");
                goto out4;
        }

	// the push of the wheels
	ctd->recal_dh=pindrv->ops->open(pindrv->instance,recal_irq,(void *)ctd);
	if (!ctd->recal_dh) {
		sys_printf("CTRLS: could not open GPIO_DRV\n");
		goto out4;
	}

	ctd->am_fm_dh=pindrv->ops->open(pindrv->instance,am_fm_irq,(void *)ctd);
	if (!ctd->am_fm_dh) {
		sys_printf("CTRLS: could not open GPIO_DRV\n");
		goto out4;
	}

	rc=pindrv->ops->control(ctd->recal_dh,GPIO_BIND_PIN,&recal_pin,sizeof(recal_pin));
	if (rc<0) {
		sys_printf("CTRLS protocol driver: failed to bind push button pin\n");
		goto out4;
	}

	rc=pindrv->ops->control(ctd->am_fm_dh,GPIO_BIND_PIN,&am_fm_pin,sizeof(am_fm_pin));
	if (rc<0) {
		sys_printf("CTRLS protocol driver: failed to bind push button pin\n");
		goto out4;
	}


	flags=GPIO_DIR(0,GPIO_INPUT);
	flags=GPIO_DRIVE(flags,GPIO_PULLUP);
	flags=GPIO_SPEED(flags,GPIO_SPEED_MEDIUM);
	flags=GPIO_IRQ_ENABLE(flags);

	rc=pindrv->ops->control(ctd->recal_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("PUSH WHEEL reader driver: pin_flags update failed\n");
		goto out4;
	}

	rc=pindrv->ops->control(ctd->am_fm_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("PUSH WHEEL reader driver: pin_flags update failed\n");
		goto out4;
	}

	return 0;

out4:
	sys_printf("TW: failed to bind pin to GPIO\n");

	pindrv->ops->close(ctd->tw_pin1_dh);
	pindrv->ops->close(ctd->tw_pin2_dh);
	pindrv->ops->close(ctd->tw_pin4_dh);

out2:

out1:
	return 0;
}



static struct driver_ops ctrls_ops = {
	ctrls_open,
	ctrls_close,
	ctrls_control,
	ctrls_init,
	ctrls_start,
};

static struct driver ctrls_drv = {
	CTRLS0,
	&ctrls_data_0,
	&ctrls_ops,
};


void init_ctrls(void) {
	driver_publish(&ctrls_drv);
}

INIT_FUNC(init_ctrls);
