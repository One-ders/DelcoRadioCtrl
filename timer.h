
typedef int (*TimeoutHandler)(void *udata);

void *timer_create(unsigned int seconds, TimeoutHandler th, void *udata);
void timer_delete(void *th_v);

void timer_set_seek_cb(TimeoutHandler th, void *udata);
