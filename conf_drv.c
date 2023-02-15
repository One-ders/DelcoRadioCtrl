
#include <sys.h>
#include <io.h>
#include <string.h>
#include <led_drv.h>
#include <hr_timer.h>
#include <gpio_drv.h>
#include <devices.h>



#include "drv_macros.h"


struct conf_data {
	unsigned char	*conf;
	int		 size;
	unsigned char	*free;
};

static struct conf_data conf_data_0 = {
};

Driver_helper(conf, 4);




///////////////////////////
///

#define KEY1 0x45670123
#define KEY2 0xCDEF89AB

#define SEGMENT_7_ADDR	(unsigned char *)0x08060000
#define CFG_AREA_ADDR SEGMENT_7_ADDR

static int unlock_flash_control_reg() {

// verify that flash is locked
	if (!(FLASH->CR&FLASH_CR_LOCK)) {
		sys_printf("Flash is not locked\n");
		return 0;
	}

// write unlock keys
	FLASH->KEYR=KEY1;
	FLASH->KEYR=KEY2;

// verify that flash is unlocked
	if (!(FLASH->CR&FLASH_CR_LOCK)) {
		sys_printf("unlock_flash: Flash is now unlocked\n");
	}
	return 0;
}

static int lock_flash_control_reg() {
	FLASH->CR|=FLASH_CR_LOCK;

	if (!(FLASH->CR&FLASH_CR_LOCK)) {
		sys_printf("Flash is not locked\n");
		return 0;
	}
	sys_printf("lock_flash: Flash is locked\n");
	return 0;
}

static int enable_programming() {
//	FLASH->CR=(0x2<<FLASH_CR_PSIZE_SHIFT) |     // 32 bits write
	FLASH->CR=  // 8 bits write
			(0x7<<FLASH_CR_SNB_SHIFT) | // Segment 7
			1;    			    // enable program
	return 0;
}


static int disable_programming() {
	FLASH->CR=0;
	return 0;
}


static int erase_config_sector() {

	unlock_flash_control_reg();
	while(FLASH->SR&FLASH_SR_BSY) {
		sys_printf("flash is busy\n");
	};

	FLASH->CR=FLASH_CR_SER|(7<<FLASH_CR_SNB_SHIFT)|(2<<FLASH_CR_PSIZE_SHIFT);
	FLASH->CR|=FLASH_CR_STRT;

	while(FLASH->SR&FLASH_SR_BSY) {
		sys_printf("flash is busy\n");
	};
	lock_flash_control_reg();
	return 0;
}


/************ Driver API *************/
//BEGIN_OPEN_FUNC(conf)
static struct  device_handle *conf_open(void *inst, DRV_CBH cb, void *udata) {

	struct drv_user *u=get_drv_user();
        struct conf_data *conf_data;
        if (!u) return 0;
        conf_data=u->conf_data=(struct conf_data *)inst;
        u->callback=cb;
        u->userdata=udata;
        u->events=0;

	{ unsigned char *cfg_addr=CFG_AREA_ADDR;
		int i;
		unsigned char *first_free=0;
		for (i=0;i<(128*1024);i++) {
			if((cfg_addr[i]!=0xff)&&
				(cfg_addr[i]!=0)) {
				conf_data->conf=(&cfg_addr[i])+1;
				conf_data->size=cfg_addr[i];
				sys_printf("found cfg area of %d bytes\n",cfg_addr[i]);
				goto ok;
			}
			if (cfg_addr[i]==0xff) {
				if (!first_free) {
					first_free=&cfg_addr[i];
					conf_data->free=first_free;
				}
			}
		}
		sys_printf("did not find any cfg area\n");
ok:
		if (conf_data->free&&(conf_data->conf>conf_data->free)) {
			erase_config_sector();
			conf_data->free=cfg_addr;conf_data->conf=0;conf_data->size=0;

		} else if (conf_data->conf[conf_data->size]==0xff) {
			conf_data->free=&conf_data->conf[conf_data->size];
		} else {
			erase_config_sector();
			conf_data->free=cfg_addr;conf_data->conf=0;conf_data->size=0;
		}
		sys_printf("hejhej\n");
	}

	return &u->dh;
}
//END_OPEN_FUNC

static int conf_close(struct device_handle *dh) {
//	DRIVER_CLOSE(conf);
	return 0;
}

static int conf_control(struct device_handle *dh, int cmd, void *arg, int size) {
	struct drv_user *u=(struct drv_user *)dh;
	struct conf_data *conf_data=u->conf_data;
	unsigned char *cfg_addr=CFG_AREA_ADDR;
	int i;

	if (!u) {
		return -1;
	}

	switch(cmd) {
		case RD_CHAR: {
			if(size==conf_data->size) {
				memcpy(arg,conf_data->conf,size);
				return size;
			}
			return -1;
			break;
		}
		case WR_CHAR: {
			unsigned char *ptr=(unsigned char *)arg;
			while(FLASH->SR&FLASH_SR_BSY);
			unlock_flash_control_reg();
			while(FLASH->SR&FLASH_SR_BSY);
			enable_programming();
			if (conf_data->size) {
				for(i=0;i<(conf_data->size+1);i++) {
					conf_data->conf[i-1]=0;  // len is before data
					while(FLASH->SR&FLASH_SR_BSY);
				}
//				memset(conf_data->conf-1,0,conf_data->size);
			}
//			memcpy(conf_data->free,&size,1);
			conf_data->free[0]=size;
			conf_data->free++;
			conf_data->conf=conf_data->free;
			conf_data->size=size;
//			memcpy(conf_data->free,arg,size);
			for (i=0;i<size;i++) {
				conf_data->free[i]=*ptr++;
				while(FLASH->SR&FLASH_SR_BSY);
			}
			conf_data->free+=size;
			while(FLASH->SR&FLASH_SR_BSY);
			disable_programming();
			lock_flash_control_reg();
			return size;
		}
	}
	return 0;
}

static int conf_init(void *inst) {
	return 0;
}

static int conf_start(void *inst) {
	return 0;
}



Driver(conf);
