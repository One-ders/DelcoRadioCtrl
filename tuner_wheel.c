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

#include "tuner_wheel.h"

#define MIN(a,b) (a<b)?a:b
#define MAX_USERS 4

static struct driver *pindrv;

//static int zero=0;
//static int one=1;

struct tw_data {
	struct device_handle *tw_pin1_dh;
	struct device_handle *tw_pin2_dh;
	struct device_handle *tw_pin3_dh;
	struct blocker_list wblocker_list;
};

struct drv_user {
	struct device_handle dh;
	DRV_CBH callback;
	void	*userdata;
	int	events;
	int	in_use;
	struct	tw_data *tw_data;
};



static struct tw_data tw_data_0 = {
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

static int wakeup_drv_users(struct tw_data *twd,int ev) {
	int i;
	for(i=0;i<MAX_USERS;i++) {
		if ((drv_user[i].in_use) &&
			(drv_user[i].tw_data==twd) &&
			(drv_user[i].events&ev) &&
			(drv_user[i].callback)) {
			drv_user[i].callback(&drv_user[i].dh,
				ev&drv_user[i].events,
				drv_user[i].userdata);
		}
	}
	return 0;
}

static int prev_pin;
static int t_ev;

static int tune_event(struct tw_data *twd, int up) {
	t_ev=up;
	wakeup_drv_users(twd, EV_READ);
	return 0;
}

static int tw_pin1_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct tw_data *twd=&tw_data_0;

	pindrv->ops->control(twd->tw_pin1_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));

        if (pin_stat==0) {
		switch (prev_pin) {
			case 1:
				break;
			case 2:
				tune_event(twd, -1);
				break;
			case 3:
				tune_event(twd, 1);
				break;
			default:
				sys_printf("pin 1 unhandled prev pin val\n");
		}
                return 0;
	} else {
		prev_pin = 1;
	}
	return 0;
}

static int tw_pin2_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct tw_data *twd=&tw_data_0;

	pindrv->ops->control(twd->tw_pin2_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));

        if (pin_stat==0) {
		switch (prev_pin) {
			case 1:
				tune_event(twd, 1);
				break;
			case 2:
				break;
			case 3:
				tune_event(twd, -1);
				break;
			default:
				sys_printf("pin 2 unhandled prev pin val\n");
		}
                return 0;
        } else {
		prev_pin = 2;
	}

	return 0;
}

static int tw_pin3_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct tw_data *twd=&tw_data_0;

	pindrv->ops->control(twd->tw_pin3_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));

        if (pin_stat==0) {
		switch (prev_pin) {
			case 1:
				tune_event(twd, -1);
				break;
			case 2:
				tune_event(twd, 1);
				break;
			case 3:
				break;
			default:
				sys_printf("pin 3 unhandled prev pin val\n");
		}
                return 0;
        } else {
		prev_pin = 3;
	}
	return 0;
}



/*****  Driver API *****/

static struct device_handle *tw_drv_open(void *inst, DRV_CBH cb, void *udata) {
	struct drv_user *u=get_drv_user();

	if (!u) return 0;
	u->tw_data=(struct tw_data *)inst;
	u->callback=cb;
	u->userdata=udata;
	u->events=0;
	return &u->dh;
}

static int tw_drv_close(struct device_handle *dh) {
	struct drv_user *u=(struct drv_user *)dh;
	if (!u) {
		return 0;
	}
	if (u) {
		u->in_use=0;
		u->tw_data=0;
	}
	return 0;
}

static int tw_drv_control(struct device_handle *dh, int cmd, void *arg, int size) {
	struct drv_user *u=(struct drv_user *)dh;
//	struct tw_data *twd;

	if (!u) {
		return -1;
	}
//	twd=u->tw_data;

	switch(cmd) {
		case RD_CHAR: {
			if (t_ev) {
				if (size>0) {
					*(char *)arg=t_ev;
					t_ev=0;
					return 1;
				}
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
				if (t_ev!=0) {
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

static int tw_drv_init(void *inst) {
	return 0;
}

static int tw_drv_start(void *inst) {
	struct tw_data *twd=(struct tw_data *)inst;
	int flags;
	int rc;
	int tw_pin1;
	int tw_pin2;
	int tw_pin3;

#if 0
	/* Open Led driver so we can flash the leds a bit */
	if (!leddrv) leddrv=driver_lookup(LED_DRV);
	if (!leddrv) return 0;
	dld->led_dh=leddrv->ops->open(leddrv->instance,0,0);
	if (!dld->led_dh) return 0;
#endif

	if (!pindrv) pindrv=driver_lookup(GPIO_DRV);
	if (!pindrv) {
		sys_printf("DL: missing GPIO_DRV\n");
		goto out1;
	}

	twd->tw_pin1_dh=pindrv->ops->open(pindrv->instance,tw_pin1_irq,(void *)twd);
	if (!twd->tw_pin1_dh) {
		sys_printf("TW: could not open GPIO_DRV\n");
		goto out1;
	}

	/* open tw pin 2 */
	twd->tw_pin2_dh=pindrv->ops->open(pindrv->instance,tw_pin2_irq,(void *)twd);
	if (!twd->tw_pin2_dh) {
		sys_printf("TW: could not open second inst. GPIO_DRV\n");
		goto out2;
	}

	/* open tw pin 3 */
	twd->tw_pin3_dh=pindrv->ops->open(pindrv->instance,tw_pin3_irq,(void *)twd);
	if (!twd->tw_pin3_dh) {
		sys_printf("TW: could not open third inst. GPIO_DRV\n");
		goto out2;
	}

	if (twd==&tw_data_0) {
		tw_pin1=TUNE_WHEEL_1;
		tw_pin2=TUNE_WHEEL_2;
		tw_pin3=TUNE_WHEEL_3;
	} else {
		sys_printf("TW protocol driver: error no pin assigned for driver\n");
		goto out4;
	}

	/* Program tune wheel pins to input and pull up */
	rc=pindrv->ops->control(twd->tw_pin1_dh,GPIO_BIND_PIN,&tw_pin1,sizeof(tw_pin1));
	if (rc<0) {
		sys_printf("TW protocol driver: failed to bind TW pin\n");
		goto out4;
	}

	flags=GPIO_DIR(0,GPIO_INPUT);
	flags=GPIO_DRIVE(flags,GPIO_PULLUP);
	flags=GPIO_SPEED(flags,GPIO_SPEED_SLOW);
	flags=GPIO_IRQ_ENABLE(flags);
	rc=pindrv->ops->control(twd->tw_pin1_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("TW reader driver: pin_flags update failed\n");
		goto out4;
	}

	rc=pindrv->ops->control(twd->tw_pin2_dh,GPIO_BIND_PIN,&tw_pin2,sizeof(tw_pin2));
	if (rc<0) {
		sys_printf("TW reader driver: tw pin 2 bind failed\n");
		goto out4;
	}

	flags=GPIO_DIR(0,GPIO_INPUT);
	flags=GPIO_DRIVE(flags,GPIO_PULLUP);
	flags=GPIO_SPEED(flags,GPIO_SPEED_SLOW);
	flags=GPIO_IRQ_ENABLE(flags);
	rc=pindrv->ops->control(twd->tw_pin2_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("TW reader driver: pin flags update failed\n");
		goto out4;
	}

	rc=pindrv->ops->control(twd->tw_pin3_dh,GPIO_BIND_PIN,&tw_pin3,sizeof(tw_pin3));
	if (rc<0) {
		sys_printf("TW reader driver: tw pin 3 bind failed\n");
		goto out4;
	}

	flags=GPIO_DIR(0,GPIO_INPUT);
	flags=GPIO_DRIVE(flags,GPIO_PULLUP);
	flags=GPIO_SPEED(flags,GPIO_SPEED_SLOW);
	flags=GPIO_IRQ_ENABLE(flags);
	rc=pindrv->ops->control(twd->tw_pin3_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("TW reader driver: pin flags update failed\n");
		goto out4;
	}

	sys_printf("TW protocol driver: Started\n");

	return 0;

out4:
	sys_printf("TW: failed to bind pin to GPIO\n");

out3:
	pindrv->ops->close(twd->tw_pin1_dh);
	pindrv->ops->close(twd->tw_pin2_dh);
	pindrv->ops->close(twd->tw_pin3_dh);

out2:

out1:
	return 0;
}



static struct driver_ops tw_drv_ops = {
	tw_drv_open,
	tw_drv_close,
	tw_drv_control,
	tw_drv_init,
	tw_drv_start,
};

static struct driver tw_drv = {
	TW_DRV0,
	&tw_data_0,
	&tw_drv_ops,
};


void init_tw(void) {
	driver_publish(&tw_drv);
}

INIT_FUNC(init_tw);
