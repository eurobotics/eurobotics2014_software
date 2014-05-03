TARGET = main

# repertoire des modules
AVERSIVE_DIR = ../libs/aversive4dspic
#AVERSIVE_DIR = ../libs/microb_2010

SRC  = $(TARGET).c cmdline.c commands_gen.c
SRC += commands_cs.c commands_mainboard.c commands_traj.c commands.c
SRC += i2c_mem.c i2c_protocol.c beacon.c sensor.c actuator.c cs.c
SRC += strat_utils.c strat_base.c strat_avoid.c strat.c wt11.c
#SRC += bt_protocol.c
SRC += strat_main.c strat_begin.c strat_fruits.c
# strat_fire.c
ifeq ($(H),1)
SRC += robotsim.c
endif

ASRC = 

CFLAGS += -Wall
#-Werror -Wno-unused-but-set-variable -Wno-unused-value -Wno-unused-result

########################################

-include .aversive_conf
include $(AVERSIVE_DIR)/mk/aversive_project.mk

AVRDUDE_DELAY=50

program_noerase: $(TARGET).$(FORMAT_EXTENSION) $(TARGET).eep
	echo $(AVRDUDE) -D -V $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM) ;\
	$(AVRDUDE) -D -V $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH) $(AVRDUDE_WRITE_EEPROM) ;\

