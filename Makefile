
#KREL=../../krel
KREL=../RTScheduler-Discovery/boards/BLACKPILL



## application brings own driver, specify the make target in the
## macro below. The make file is expected to create a object file
## with the name of the target+".o" and put it under 'obj/drv_obj'
APPLICATION_DRIVERS=my_drivers

OBJ:=$(KREL)/obj
LOBJ:=obj

include $(KREL)/Makefile

DRIVERS=stddrv.o
DRIVERS+=gpio_drv.o
#DRIVERS+=usart_drv.o
DRIVERS+=led_drv.o
DRIVERS+=usb_core_drv.o
DRIVERS+=usb_serial_drv.o
DRIVERS+=hr_timer.o

#DTARGETS=$(DRIVERS:.o=)
#DOBJS=$(patsubst %, $(OBJ)/drv_obj/%, $(DRIVERS))

usr.bin.o: $(OBJ)/usr/sys_cmd/sys_cmd.o $(OBJ)/usr/pin_test/pin_test.o $(LOBJ)/usr/radioctrl.o
	$(LD) -o $(LOBJ)/usr/usr.bin.o $(LDFLAGS_USR) $^
	$(OBJCOPY) --prefix-symbols=__usr_ $(LOBJ)/usr/usr.bin.o

my_drivers: $(LOBJ)/usr $(LOBJ)/usr/radio_drivers.o
	$(LD) -r -o $(LOBJ)/usr/usr.drivers.o $(LOBJ)/usr/radio_drivers.o
	mkdir -p $(LOBJ)/drv_obj
	cp $(LOBJ)/usr/usr.drivers.o  $(LOBJ)/drv_obj/$@.o

$(LOBJ)/usr:
	mkdir -p $(LOBJ)/usr

$(LOBJ)/usr/radio_drivers.o: dl_drv.o controls_drv.o tstats_drv.o conf_drv.o clock_drv.o
	$(CC) -r -nostdlib -o $@ $^


$(LOBJ)/usr/radioctrl.o: $(LOBJ)/usr main.o asynchio.o timer.o
	$(CC) -r -nostdlib -o $@ main.o asynchio.o timer.o

%_drv.o: %_drv.c
	$(CC) $(CFLAGS) -c -o $@ $<

main.o: main.c
	$(CC) $(CFLAGS_USR) -c -o $@ $<

asynchio.o: asynchio.c
	$(CC) $(CFLAGS_USR) -c -o $@ $<

timer.o: timer.c
	$(CC) $(CFLAGS_USR) -c -o $@ $<

clean:
	rm -rf *.o obj myCore myCore.bin
