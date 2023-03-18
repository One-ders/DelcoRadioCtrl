#include <sys.h>
#include <io.h>
#include <string.h>
#include <led_drv.h>
#include <hr_timer.h>
#include <gpio_drv.h>
#include <devices.h>

#include "clock_drv.h"

#include "drv_macros.h"

struct clock_data {
	struct device_handle *tod_pin_dh;
};

static struct clock_data clock_data_0 = {
};

static struct driver *pindrv;

Driver_helper(clock, 4);


static unsigned char hours;
static unsigned char minutes;
static int curr_sec;
static int irqs;

static int tod_pin_irq(struct device_handle *dh, int ev, void *dum) {
	struct clock_data *cd=(struct clock_data *)dum;
//	unsigned int *SCR=(unsigned int *)0xe000ed10;
	irqs++;
	if (irqs>=100) { // irq both on up and down flank
		irqs=0;
		wakeup_drv_users(cd, EV_STATE);
		if (curr_sec>=59) {
			curr_sec=0;
			if (minutes>=59) {
				minutes=0;
				if (hours>=12) {
					hours=1;
				} else {
					hours++;
				}
			} else {
				minutes++;
			}
		} else {
			curr_sec++;
		}
//		sys_printf("1 sec irq\n");
	}

	return 0;
}

/******************** Driver API ****************/

static struct device_handle *clock_open(void *inst, DRV_CBH cb, void *udata) {
	struct drv_user *u=get_drv_user();
	struct clock_data *clock_data;
	if (!u) {
		sys_printf("clock_open: failed to open tod clock\n");
		return 0;
	}
	clock_data=u->clock_data=(struct clock_data *)inst;
	u->callback=cb;
	u->userdata=udata;
//	u->events=0;
	u->events=EV_STATE;

	sys_printf("clock_open: tod clock open rc  %x\n", &u->dh);
	return &u->dh;
}

static int clock_close(struct device_handle *dh) {
	struct drv_user *u=(struct drv_user *)dh;
	if (!u) {
		sys_printf("clock_close: tod clock close bad u %x\n", dh);
		return 0;
	}
	if (u) {
		sys_printf("clock_close: tod clock close %x\n", dh);
		u->in_use=0;
		u->clock_data=0;
	}
	return 0;
}

static int clock_control(struct device_handle *dh, int cmd, void *arg, int size) {
	struct drv_user *u=(struct drv_user *)dh;

	switch(cmd) {
		case RD_CHAR: {
			return -1;
			break;
		}
		case WR_CHAR: {
			return -1;
			break;
		}
		case IO_POLL: {
			return 0;
			break;
		}
		case CLOCK_GET_TIME: {
			struct time_buf *tbuf=(struct time_buf *)arg;
			if (size<sizeof(struct time_buf)) return -1;
			tbuf->hours=hours;
			tbuf->minutes=minutes;
			tbuf->seconds=curr_sec;
			return sizeof(tbuf);
		}
		case CLOCK_SET_TIME: {
			struct time_buf *tbuf=(struct time_buf *)arg;
			if (size<sizeof(struct time_buf)) return -1;
			hours=tbuf->hours;
			minutes=tbuf->minutes;
			curr_sec=0;
			return sizeof(struct time_buf);
		}
	}

	return 0;
}

static int clock_start(void *inst) {
	struct clock_data *cd=(struct clock_data *)inst;
	int flags;
	int rc;
	int tod_pin;

	if (!pindrv) pindrv=driver_lookup(GPIO_DRV);
        if (!pindrv) {
                sys_printf("CLOCK: missing GPIO_DRV\n");
                goto out1;
        }

	cd->tod_pin_dh=pindrv->ops->open(pindrv->instance,tod_pin_irq,(void *)cd);
        if (!cd->tod_pin_dh) {
                sys_printf("CLOCK: could not open GPIO_DRV\n");
                goto out1;
        }

	       /* Program 50Hz pin */
	tod_pin=TOD;
        rc=pindrv->ops->control(cd->tod_pin_dh,GPIO_BIND_PIN,&tod_pin,sizeof(tod_pin));
        if (rc<0) {
                sys_printf("CTRLS protocol driver: failed to bind TW pin\n");
                goto out1;
        }

        flags=GPIO_DIR(0,GPIO_INPUT);
        flags=GPIO_DRIVE(flags,GPIO_PULLUP);
        flags=GPIO_SPEED(flags,GPIO_SPEED_SLOW);
        flags=GPIO_IRQ_ENABLE(flags);
        rc=pindrv->ops->control(cd->tod_pin_dh,GPIO_SET_FLAGS,&flags,sizeof(flags));
        if (rc<0) {
                sys_printf("CLOCK driver: pin_flags update failed\n");
                goto out1;
        }

out1:
	return 0;
}


static int clock_init(void *inst) {
	struct clock_data *cd=(struct clock_data *)inst;
	hours=1;
	minutes=0;
	return 0;
}

Driver(clock);
