
#KREL=../../krel
KREL=../RTScheduler-Discovery/boards/BLACKPILL



## application brings own driver, specify the make target in the
## macro below. The make file is expected to create a object file
## with the name of the target+".o" and put it under 'obj/drv_obj'
APPLICATION_DRIVERS=my_drivers

OBJ:=$(KREL)/obj
LOBJ:=obj

include $(KREL)/Makefile

usr.bin.o: $(OBJ)/usr/sys_cmd/sys_cmd.o $(OBJ)/usr/pin_test/pin_test.o $(LOBJ)/usr/radioctrl.o
	$(LD) -o $(LOBJ)/usr/usr.bin.o $(LDFLAGS_USR) $^
	$(OBJCOPY) --prefix-symbols=__usr_ $(LOBJ)/usr/usr.bin.o

my_drivers: $(LOBJ)/usr $(LOBJ)/usr/radio_drivers.o
	$(LD) -r -o $(LOBJ)/usr/usr.drivers.o $(LOBJ)/usr/radio_drivers.o
	mkdir -p $(LOBJ)/drv_obj
	cp $(LOBJ)/usr/usr.drivers.o  $(LOBJ)/drv_obj/$@.o

$(LOBJ)/usr:
	mkdir -p $(LOBJ)/usr

$(LOBJ)/usr/radio_drivers.o: dl_drv.o tuner_wheel.o
	$(CC) -r -nostdlib -o $@ $^


$(LOBJ)/usr/radioctrl.o: $(LOBJ)/usr main.o
	$(CC) -r -nostdlib -o $@ main.o

%_drv.o: %_drv.c
	$(CC) $(CFLAGS) -c -o $@ $<

main.o: main.c
	$(CC) $(CFLAGS_USR) -c -o $@ $<

#asynchio.o: asynchio.c asynchio.h
#	$(CC) $(CFLAGS_USR) -c -o $@ $<

#menu.o: menu.c
#	$(CC) $(CFLAGS_USR) -I./panellib -c -o $@ $<

#menu.c: menues.tpl
#	pcomp -r -o $(basename $@) $<

#panellib/panellib.a: panellib/panel.o

#panellib/panel.o: panellib/panel.c
#	$(CC) $(CFLAGS_USR) -I./panellib -c -o $@ $<


clean:
	rm -rf *.o obj myCore myCore.bin
