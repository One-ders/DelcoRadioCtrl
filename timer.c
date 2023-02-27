#include "sys.h"
#include "sys_env.h"
#include "io.h"

#include <string.h>

#include <assert.h>

#include "asynchio.h"
#include "timer.h"

#define NULL 0


struct Timer_obj {
	unsigned int     tfire;
	TimeoutHandler   th;
	void             *udata;
	struct Timer_obj *next;
	struct Timer_obj **prev_nextp;
};



static struct Timer_obj *hsecs[120];
static int hsec_now;

static struct Timer_obj *mins[60];
static int min_now;

static struct Timer_obj *hours[1024];
static int hour_now;

#define NR_TIMERS	16
static struct Timer_obj timers[NR_TIMERS];
static int tmask=0xffff;

struct Timer_obj *get_timer_obj() {
	int i=ffs(tmask);
	if (i) {
//		printf("get timer obj %d\n", i-1);
		tmask&=~(1<<(i-1));
		return &timers[i-1];
	}
	return 0;
}

void put_timer_obj(struct Timer_obj *tobj) {
	int i=tobj-timers;

	if (!tobj) {
		printf("release of null timer\n");
		return;
	}
//	printf("put timer obj %d\n", i);
	if ((i<0)||(i>(NR_TIMERS-1))) {
		printf("timer return error\n");
		return;
	}
	if (tmask&(1<<i)) {
		printf("release of idle timer\n");
		return;
	}
//	timer[i]=tobj;
	tmask|=1<<i;
}

static int (*seek_cb)(void*);
static void *seek_cb_data;

void timer_set_seek_cb(TimeoutHandler th, void *udata) {
	seek_cb=th;
	seek_cb_data=udata;
}

static int tictic=0;
static int handle_tic(int dum, int dum2, void *bum) {
	struct Timer_obj *tmp, *tmp1;
	tictic++;

	if (seek_cb) seek_cb(seek_cb_data);
	if (tictic%10) return 0;
	hsec_now++;
//	printf("handle_tic\n");
	if (hsec_now>=120) {
		min_now++;
		hsec_now-=120;
	}
	if (min_now>=60) {
		hour_now++;
		min_now-=60;
	}

	if (hour_now>=1024) {
		hour_now-=1024;
	}
	tmp=hours[hour_now];
	hours[hour_now]=NULL;
	while(tmp) {
		tmp1=tmp->next;
		tmp->tfire-=(60*120);
		if (tmp->tfire>=120) {
			int tmins=tmp->tfire/120;
			int m_ix=(tmins+min_now)%60;
			assert(tmp->tfire<60*120);
			tmp->next=mins[m_ix];
			tmp->prev_nextp=&mins[m_ix];
			if (mins[m_ix]) {
				mins[m_ix]->prev_nextp=&tmp->next;
			}
			mins[m_ix]=tmp;
		} else {
			int hs_ix=(tmp->tfire+hsec_now)%120;
			assert(tmp->tfire<120);
			tmp->next=hsecs[hs_ix];
			tmp->prev_nextp=&hsecs[hs_ix];
			if (hsecs[hs_ix]) {
				hsecs[hs_ix]->prev_nextp=&tmp->next;
			}
			hsecs[hs_ix]=tmp;
		}
		tmp=tmp1;
	}

	tmp=mins[min_now];
	mins[min_now]=NULL;
	while(tmp) {
		int hs_ix;
		tmp1=tmp->next;
		tmp->tfire-=120;
		assert(tmp->tfire<120);
		hs_ix=(tmp->tfire+hsec_now)%120;
		tmp->next=hsecs[hs_ix];
		tmp->prev_nextp=&hsecs[hs_ix];
		if (hsecs[hs_ix]) {
			hsecs[hs_ix]->prev_nextp=&tmp->next;
		}
		hsecs[hs_ix]=tmp;
		tmp=tmp1;
	}
	tmp=hsecs[hsec_now];
	hsecs[hsec_now]=NULL;
	while(tmp) {
		tmp1=tmp->next;
		if (tmp->th) {
			tmp->th(tmp->udata);
		}
		put_timer_obj(tmp);
		tmp=tmp1;
	}
	return 0;
}

static int timerRunning=0;
static int timer_count=0;

static int init_timer(void) {
	hsec_now=min_now=hour_now=0;
	timerRunning=1;
//	printf("starting timer\n");
	register_timer(50, handle_tic, NULL);
	return 0;
}

static int stop_timer(void) {
//	printf("stopping timer\n");
	timerRunning=0;
	register_timer(0,NULL,NULL);
	return 0;
}

void *timer_create(unsigned int seconds, TimeoutHandler th, void *udata) {
	struct Timer_obj *timer=get_timer_obj();
	if (!timer) {
		return 0;
	}

	memset(timer,0,sizeof(*timer));
	timer->tfire=seconds;
	timer->th=th;
	timer->udata=udata;

	if (!timerRunning) {
		init_timer();
	}

	if (seconds>(120*60)) {
		int h_ix=(hour_now+(seconds/(120*60)))%1024;
		timer->next=hours[h_ix];
		timer->prev_nextp=&hours[h_ix];
		if (timer->next) {
			timer->next->prev_nextp=&timer->next;
		}
		hours[h_ix]=timer;
	} else if (seconds>120) {
		int m_ix=(min_now+(seconds/120))%60;
		timer->next=mins[m_ix];
		timer->prev_nextp=&mins[m_ix];
		if (timer->next) {
			timer->next->prev_nextp=&timer->next;
		}
		mins[m_ix]=timer;
	} else {
		int hs_ix=(hsec_now+(seconds*2))%120;
		timer->next=hsecs[hs_ix];
		timer->prev_nextp=&hsecs[hs_ix];
		if (timer->next) {
			timer->next->prev_nextp=&timer->next;
		}
		hsecs[hs_ix]=timer;
//		printf("timer created at %x, next is %x, prev_nextp is %x\n",
//			timer, timer->next, timer->prev_nextp);
	}
	return timer;
}

void timer_delete(void *th_v) {
	struct Timer_obj *timer=(struct Timer_obj *)th_v;
	timer->th=NULL;
	if (timer->next) {
		timer->next->prev_nextp=timer->prev_nextp;
	}
	*timer->prev_nextp=timer->next;

//	printf("timer delete at %x\n");
	timer->next=NULL;
	timer->prev_nextp=NULL;
	put_timer_obj(timer);
}
