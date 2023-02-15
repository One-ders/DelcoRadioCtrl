// static struct drv_user *get_drv_user(void);

#define xstr(a) str(a)
#define str(a) #a

#define Driver(name) static struct driver_ops name##_ops = {    \
				name##_open,			\
				name##_close,			\
				name##_control,			\
				name##_init,			\
				name##_start,			\
};								\
								\
			static struct driver name##_drv = {	\
				xstr(name##0),			\
				&name##_data_0,			\
				&name##_ops,			\
};								\
								\
		void init_##name(void) {			\
			driver_publish(&name##_drv);		\
		}						\
								\
		INIT_FUNC(init_##name)

#define Driver_helper(name,users)				\
 								\
struct drv_user {						\
	struct device_handle dh;				\
	DRV_CBH callback;					\
	void	*userdata;					\
	int	events;						\
	int	in_use;						\
	struct name##_data *name##_data;			\
};								\
								\
static struct drv_user drv_user[users];				\
								\
static struct drv_user *get_drv_user(void) {			\
	int i;							\
	for(i=0;i<users;i++) {					\
		if (!drv_user[i].in_use) {			\
			drv_user[i].in_use=1;			\
			return &drv_user[i];			\
		}						\
	}							\
	return 0;						\
}								\
								\
static int wakeup_drv_users(struct name##_data *name##_data, int ev) { \
	int i;							\
	for(i=0;i<users;i++) {					\
		if ((drv_user[i].name##_data==name##_data) &&	\
			(drv_user[i].events&ev) &&		\
			(drv_user[i].callback)) {		\
			drv_user[i].callback(&drv_user[i].dh,	\
				ev&drv_user[i].events,		\
				drv_user[i].userdata);		\
		}						\
	}							\
	return 0;						\
}

#define BEGIN_OPEN_FUNC(name) static struct device_handle *name##_open( \
			void *inst, DRV_CBH cb, void *udata) {	\
			struct drv_user *u=get_drv_user(); 	\
			struct name##_data *name##_data;		\
			if (!u) return 0;			\
			name##_data=u->name##_data=(struct name##_data *)inst;\
			u->callback=cb;				\
			u->userdata=udata;			\
			u->events=0;

#define END_OPEN_FUNC	return &u->dh; }
