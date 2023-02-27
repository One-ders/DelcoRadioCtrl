
#define CLOCK_DRV "clock0"


#define CLOCK_GET_TIME 0x1001
#define CLOCK_SET_TIME 0x1002

struct time_buf {
	unsigned char hours;
	unsigned char minutes;
	unsigned char seconds;
};
