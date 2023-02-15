/* $tstat drv: , v1.1 2023/02/11 21:44:00 anders Exp $ */

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
 * @(#)tstat_drv.c
 */
#include <sys.h>
#include <io.h>
#include <string.h>
#include <gpio_drv.h>

#include "tstats_drv.h"

#define MIN(a,b) (a<b)?a:b
#define MAX_USERS 4



static struct driver *pindrv;

struct tstats_data {
	struct device_handle *stereo_pin_dh;
	struct device_handle *stop_pin_dh;
	struct device_handle *mute_pin_dh;
	struct device_handle *rsense_pin_dh;
};


struct drv_user {
        struct device_handle dh;
        DRV_CBH callback;
        void    *userdata;
        int     events;
        int     in_use;
        struct  tstats_data *tstats_data;
};

#define EVENT_BUF_SIZE 16
static unsigned int event[EVENT_BUF_SIZE];
static unsigned int ev_i=0;
static unsigned int ev_o=0;

#define to_ix(a)                (a&(EVENT_BUF_SIZE-1))

static struct tstats_data tstats_data_0= {};

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

static int wakeup_drv_users(struct tstats_data *tsd,int ev) {
	int i;
	for(i=0;i<MAX_USERS;i++) {
		if ((drv_user[i].in_use) &&
				(drv_user[i].tstats_data==tsd) &&
				(drv_user[i].events&ev) &&
				(drv_user[i].callback)) {
			drv_user[i].callback(&drv_user[i].dh,
						ev&drv_user[i].events,
						drv_user[i].userdata);
                }
        }
        return 0;
}

//////////////////////////////////////////////////////////////////////////
///
static int stereo_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct tstats_data *tsd=&tstats_data_0;
	pindrv->ops->control(tsd->stereo_pin_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
//	sys_printf("stereo irq pin=%d\n",pin_stat);

	if (pin_stat==0) { // active low
		event[to_ix(ev_i)]=EVENT_STEREO_ON;
	} else {
		event[to_ix(ev_i)]=EVENT_STEREO_OFF;
	}
	ev_i++;
	wakeup_drv_users(tsd, EV_READ);
	return 0;
}

#if 0
static int stop_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct tstats_data *tsd=&tstats_data_0;
	pindrv->ops->control(tsd->stop_pin_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
	sys_printf("stop irq pin=%d\n", pin_stat);
	if (pin_stat!=0) { // active hi
		event[to_ix(ev_i)]=EVENT_STOP;
		ev_i++;
		wakeup_drv_users(tsd, EV_READ);
	}
	return 0;
}
#endif

static int rsense_irq(struct device_handle *dh, int ev, void *dum) {
	int pin_stat;
	struct tstats_data *tsd=&tstats_data_0;
	pindrv->ops->control(tsd->rsense_pin_dh,GPIO_SENSE_PIN,&pin_stat,sizeof(pin_stat));
	sys_printf("rsense irq pin=%d\n", pin_stat);
	return 0;
}


//////////////////////////////////////////////////////////////////////////
///

static struct device_handle *tstats_open(void *inst, DRV_CBH cb, void *udata) {
        struct drv_user *u=get_drv_user();
//        struct tstats_data *tsd;

        if (!u) return 0;
        u->tstats_data=(struct tstats_data *)inst;
        u->callback=cb;
        u->userdata=udata;
        u->events=0;
        return &u->dh;
}

static int tstats_close(struct device_handle *dh) {
        struct drv_user *u=(struct drv_user *)dh;
        if (!u) {
                return 0;
        }
        if (u) {
                u->in_use=0;
                u->tstats_data=0;
        }
        return 0;
}

static int tstats_control(struct device_handle *dh, int cmd, void *arg, int size) {
        struct drv_user *u=(struct drv_user *)dh;
	struct tstats_data *tsd=(struct tstats_data *)u->tstats_data;

        if (!u) {
                return -1;
        }

	switch(cmd) {
		case RD_CHAR: {
			if (ev_i>ev_o) {
				*((unsigned int *)arg)=event[to_ix(ev_o)];
				ev_o++;
				return sizeof(arg);
			}
			return -DRV_AGAIN;
			break;
		}
		case WR_CHAR: {
			return -1;
			break;
		}
		case GET_STOP_FLAG: {
			int pin_stat;
			unsigned long int req=(unsigned long int)req;
			pindrv->ops->control(
				tsd->stop_pin_dh,
				GPIO_SENSE_PIN,
				&pin_stat,sizeof(pin_stat));

			*((unsigned int *)arg)=pin_stat;
			return 0;
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


static int tstats_init(void *inst) {
	return 0;
}

static int tstats_start(void *inst) {
	struct tstats_data *tsd=(struct tstats_data *)inst;
	int stereo_pin	= STEREO;
	int stop_pin	= STOP;
	int rsense_pin	= RSENSE;
	int mute_pin	= RMUTE;
	int rc;
	int flags;

	if (!pindrv) pindrv=driver_lookup(GPIO_DRV);
	if (!pindrv) {
		sys_printf("TST: missing GPIO_DRV\n");
		goto out1;
	}

	tsd->stereo_pin_dh=
		pindrv->ops->open(pindrv->instance,stereo_irq,(void *)tsd);
	if (!tsd->stereo_pin_dh) {
		goto out1;
	}

	tsd->stop_pin_dh=
		pindrv->ops->open(pindrv->instance,NULL,(void *)tsd);
//		pindrv->ops->open(pindrv->instance,stop_irq,(void *)tsd);
	if (!tsd->stop_pin_dh) {
		goto out1;
	}

	tsd->rsense_pin_dh=
		pindrv->ops->open(pindrv->instance,rsense_irq,(void *)tsd);
	if (!tsd->rsense_pin_dh) {
		goto out1;
	}

	tsd->mute_pin_dh=
		pindrv->ops->open(pindrv->instance,NULL,(void *)tsd);
	if (!tsd->mute_pin_dh) {
		goto out1;
	}

	       /* bind pins to driver instances */
        rc=pindrv->ops->control(tsd->stereo_pin_dh,GPIO_BIND_PIN,&stereo_pin,sizeof(stereo_pin));
        if (rc<0) {
                goto out1;
        }

        rc=pindrv->ops->control(tsd->stop_pin_dh,GPIO_BIND_PIN,&stop_pin,sizeof(stop_pin));
        if (rc<0) {
                goto out1;
        }

#if 0
        rc=pindrv->ops->control(tsd->rsense_pin_dh,GPIO_BIND_PIN,&rsense_pin,sizeof(rsense_pin));
        if (rc<0) {
                goto out1;
        }

        rc=pindrv->ops->control(tsd->mute_pin_dh,GPIO_BIND_PIN,&mute_pin,sizeof(mute_pin));
        if (rc<0) {
                goto out1;
        }

#endif
	// Config pins functions //

	// flags for input pins
	flags=GPIO_DIR(0,GPIO_INPUT);
	flags=GPIO_DRIVE(flags,GPIO_FLOAT);
	flags=GPIO_SPEED(flags,GPIO_SPEED_MEDIUM);
	flags=GPIO_IRQ_ENABLE(flags);

	rc=pindrv->ops->control(tsd->stereo_pin_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("TSTAT driver: pin_flags update failed\n");
		goto out1;
	}

#if 0
	rc=pindrv->ops->control(tsd->rsense_pin_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("TSTAT reader driver: pin_flags update failed\n");
		goto out1;
	}
#endif

	flags=GPIO_DIR(0,GPIO_INPUT);
	flags=GPIO_DRIVE(flags,GPIO_PULLDOWN);
	flags=GPIO_SPEED(flags,GPIO_SPEED_MEDIUM);
//	flags=GPIO_IRQ_ENABLE(flags);

	rc=pindrv->ops->control(tsd->stop_pin_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
	if (rc<0) {
		sys_printf("TSTAT reader driver: pin_flags update failed\n");
		goto out1;
	}


#if 0
	flags=GPIO_DIR(0,GPIO_OUTPUT);
	flags=GPIO_DRIVE(flags,GPIO_PUSHPULL);
	flags=GPIO_SPEED(flags,GPIO_SPEED_SLOW);
	rc=pindrv->ops->control(tsd->mute_pin_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
        if (rc<0) {
                sys_printf("TSTAT driver: flags update failed\n");
                goto out1;
        }
#endif

out1:
	return 0;
}

static struct driver_ops tstats_ops = {
	tstats_open,
	tstats_close,
	tstats_control,
	tstats_init,
	tstats_start,
};

static struct driver tstats_drv = {
	TSTATS0,
	&tstats_data_0,
	&tstats_ops,
};

void init_tstats(void) {
	driver_publish(&tstats_drv);
}

INIT_FUNC(init_tstats);
