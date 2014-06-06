/*
 *  Copyright Droids Corporation (2009)
 * 
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 *  Revision : $Id$
 *
 */

/*  
 *  Copyright Robotics Association of Coslada, Eurobotics Engineering (2011)
 *  Javier Baliñas Santos <javier@arc-robots.org>
 *
 *  Code ported to family of microcontrollers dsPIC from
 *  i2c_protocol.c,v 1.7 2009/05/27 20:04:07 zer0 Exp.
 */

#include <stdio.h>
#include <string.h>

#include <aversive/pgmspace.h>
#include <aversive/wait.h>
#include <aversive/error.h>

#include <uart.h>
#include <dac_mc.h>
#include <i2c_mem.h>
#include <clock_time.h>
#include <pwm_servo.h>

#include <parse.h>
#include <rdline.h>

#include <pid.h>
#include <quadramp.h>
#include <control_system_manager.h>
#include <trajectory_manager.h>
#include <trajectory_manager_utils.h>
//#include <trajectory_manager_core.h>
#include <vect_base.h>
#include <lines.h>
#include <polygon.h>
#include <obstacle_avoidance.h>
#include <blocking_detection_manager.h>
#include <robot_system.h>
#include <position_manager.h>

#include "../common/i2c_commands.h"
#include "main.h"
#include "sensor.h"
#include "i2c_protocol.h"
#ifdef HOST_VERSION
#include "robotsim.h"
#endif


#define I2C_STATE_MAX 			3
#define I2C_TIMEOUT 				100 /* ms */
#define I2C_WATCH_DOG_TIMEOUT	10

/* local headers */
static int8_t i2c_read_gpios_01_values(void);
static int8_t i2c_read_gpios_23_values(void);
static int8_t i2c_req_slavedspic_status(void);

/* limited error logs */
#define I2C_MAX_ERRORS	5
#define I2C_ERROR(args...) do {						\
		if (error_log < I2C_MAX_LOG) {				\
			ERROR(E_USER_I2C_PROTO, args);			\
			error_log ++;									\
			if (error_log == I2C_MAX_LOG) {			\
				ERROR(E_USER_I2C_PROTO,					\
				      "i2c logs are now warnings");	\
			}													\
		}														\
		else													\
			WARNING(E_USER_I2C_PROTO, args);			\
	} while(0)

/* i2c pulling state */
static volatile uint8_t i2c_state = 0;
#define I2C_READ_GPIOS_01_VALUES 	0
#define I2C_READ_GPIOS_23_VALUES 	1
#define I2C_REQ_SLAVEDSPIC				2

/* pulling counter */
static volatile uint8_t i2c_poll_num = 0;

/* comunication errors */
static volatile uint16_t i2c_errors = 0;

/* type of running operation */
#define OP_READY 0 		/* no i2c op running */
#define OP_POLL  1 		/* a polling (req / ans) is running */
#define OP_CMD   2 		/* a user command is running */

/* actual running operation */
static volatile uint8_t running_op = OP_READY;

/* error log counter */
static uint8_t error_log = 0;
#define I2C_MAX_LOG	1

/* gpios */
volatile uint8_t gpio_addr = 0;

/* commands */
volatile uint16_t command_dest=-1;
volatile uint16_t command_size=0;
uint8_t command_buf[I2C_SEND_BUFFER_SIZE];

/* debug */
uint8_t dummy = 0;

/* init i2c stuff */
void i2c_protocol_init(void)
{
	/* add initialization setup here */
}

/* debug protocol */
void i2c_protocol_debug(void)
{
#ifdef HOST_VERSION
	return;
#else
	printf_P(PSTR("I2C protocol debug infos:\r\n"));
	printf_P(PSTR("  i2c_state=%d\r\n"), i2c_state);
	printf_P(PSTR("  i2c_errors=%d\r\n"), i2c_errors);
	printf_P(PSTR("  running_op=%d\r\n"), running_op);
	printf_P(PSTR("  command_size=%d\r\n"), command_size);
	printf_P(PSTR("  command_dest=%d\r\n"), command_dest);
	printf_P(PSTR("  i2c_status=%x\r\n"), i2c_status());
#endif
}

#ifndef HOST_VERSION
/* update to next state */
static void i2cproto_next_state(uint8_t inc)
{
	i2c_state += inc;
	if (i2c_state >= I2C_STATE_MAX) {
		i2c_state = 0;
		i2c_poll_num ++;
	}
}

#define I2C_WAIT_COND_OR_TIMEOUT(cond, timeout)                   \
({                                                            \
        microseconds __us = time_get_us2();                   \
        uint8_t __ret = 1;                                    \
        while(! (cond)) {                                     \
                if (time_get_us2() - __us > (timeout)*1000L) {\
                        __ret = 0;                            \
                        break;                                \
                }                                             \
        }                                                     \
	if (!__ret)					      					      \
		DEBUG(E_USER_I2C_PROTO, "%s timeout",     \
		      __FUNCTION__);			      \
							      \
        __ret;                                                \
})

/* wait one cycle of pulling or timeout ,
 * useful to synchronize processes        */
void i2cproto_wait_update(void)
{
	uint8_t poll_num;
	poll_num = i2c_poll_num;
	I2C_WAIT_COND_OR_TIMEOUT((i2c_poll_num-poll_num) > 1, 200);
}



/* called periodically : the goal of this 'thread' is to send requests
 * and read answers on i2c slaves in the correct order. 						*/
void i2c_poll_slaves(void *dummy)
{
	uint8_t flags;
	int8_t err;
	static uint8_t a = 0;
	static uint8_t watchdog_cnt = 0;

	/* goto error implementation */
	void * p_error_pull;
	p_error_pull = &&error_pull;

	/* watchdog */
	watchdog_cnt++;	
	if(watchdog_cnt == I2C_WATCH_DOG_TIMEOUT){
		
		if(running_op == OP_CMD)
			I2C_ERROR("I2C wathdog timeout wating COMMAND");
		else{
			I2C_ERROR("I2C wathdog timeout wating POLLING %d", i2c_state);
			i2cproto_next_state(1);
		}	
		running_op = OP_READY;
		i2c_errors = 0;

		/* XXX reset slavedspic */
//		set_uart_mux(SLAVEDSPIC_CHANNEL);
//		uart_send(MUX_UART,'\n');
//		uart_send(MUX_UART,'\r');
//		uart_send(MUX_UART,'r');
//		uart_send(MUX_UART,'e');
//		uart_send(MUX_UART,'s');
//		uart_send(MUX_UART,'e');
//		uart_send(MUX_UART,'t');
//		uart_send(MUX_UART,'\n');
//		uart_send(MUX_UART,'\r');

		/* XXX wait send reset */
		//wait_ms(10);

//		set_uart_mux(BEACON_CHANNEL);

		/* reset local i2c */
		i2c_reset();
		return;
	}

	/* start of critical section */		
	IRQ_LOCK(flags);

	/* return if last operation not finished */
	if (running_op != OP_READY) {
		
		/* end of critical section */
		IRQ_UNLOCK(flags);
		return;
	}

	/* reset watchdog */
	watchdog_cnt = 0;
	
	/* led debug */
	a++;
	if (a & 0x4)
		LED2_TOGGLE();

	/* if a command is ready to be sent, so send it */
	if (command_size) {
		running_op = OP_CMD;
		err = i2c_write(command_dest, I2C_CMD_GENERIC, command_buf, command_size);		

		/* if i2c write error -> end with error */
		if (err)
			goto *p_error_pull;
		IRQ_UNLOCK(flags);

		/* return and wait the send ends */
		return;
	}

	/* at this point: no command, so do the polling */
	running_op = OP_POLL;
	
	/* poll status of gpios and microcontrollers */
	//i2c_state = I2C_REQ_SLAVEDSPIC;
	switch(i2c_state) {

		case I2C_READ_GPIOS_01_VALUES:
			if ((err = i2c_read_gpios_01_values()))
				goto *p_error_pull;
			break;
	
		case I2C_READ_GPIOS_23_VALUES:
			if ((err = i2c_read_gpios_23_values()))
				goto *p_error_pull;
			break;

		case I2C_REQ_SLAVEDSPIC:
			if ((err = i2c_req_slavedspic_status()))
				goto *p_error_pull;
			break;

		/* nothing, go to the first request */
		default:
			i2c_state = 0;
			running_op = OP_READY;
	}

	/* end critical section */
	IRQ_UNLOCK(flags);
	return;

/* error during pull operation */
error_pull:

	/* reset op */
	running_op = OP_READY;
	
	/* end critical section */
	IRQ_UNLOCK(flags);
	
	/* manage error */
	i2c_errors++;
	if (i2c_errors > I2C_MAX_ERRORS) {
		I2C_ERROR("I2C send is_cmd=%d proto_state=%d " 
		      "err=%d i2c_status=%x", !!command_size, i2c_state, err, i2c_status());
		
		/* reset slavedspic */
//		set_uart_mux(SLAVEDSPIC_CHANNEL);
//		uart_send(MUX_UART,'\n');
//		uart_send(MUX_UART,'\r');
//		uart_send(MUX_UART,'r');
//		uart_send(MUX_UART,'e');
//		uart_send(MUX_UART,'s');
//		uart_send(MUX_UART,'e');
//		uart_send(MUX_UART,'t');
//		uart_send(MUX_UART,'\n');
//		uart_send(MUX_UART,'\r');

		/* XXX wait send reset */
		//wait_ms(10);

		/* reset local i2c */
		i2c_reset();
		i2c_errors = 0;
	}
}

/* called when the xmit is finished */
void i2c_write_event(uint16_t size)
{
	/* pull or cmd OK sended */
	if (size > 0) {
		if (running_op == OP_POLL) {
			i2cproto_next_state(1);
		}
		else
			command_size = 0;
	}
	/* error */
	else {
		i2c_errors++;
		NOTICE(E_USER_I2C_PROTO, "send error state=%d size=%d "
			"op=%d", i2c_state, size, running_op);
				
		if (i2c_errors > I2C_MAX_ERRORS) {
			I2C_ERROR("I2C error, slave not ready");

			i2c_reset();
			i2c_errors = 0;
		}
		
		if (running_op == OP_POLL) {
			/* skip associated answer */
			i2cproto_next_state(2);
		}
	}

	/* ready for next op */
	running_op = OP_READY;
}

/* called read event */
void i2c_read_event(uint8_t * buf, uint16_t size)
{
	volatile uint8_t i2c_state_save = i2c_state;
	void * p_error_recv;
	p_error_recv = &&error_recv;
	
	/* if actual op is pulling, go next pulling state */
	if (running_op == OP_POLL)
		i2cproto_next_state(1);

	/* recv is only trigged after a poll */
	running_op = OP_READY;
	
	/* error if null size */
	if (size == 0) {
		goto *p_error_recv;
	}

	/* parse GPIOS answers */
	if(i2c_state_save == I2C_READ_GPIOS_01_VALUES ||
		i2c_state_save == I2C_READ_GPIOS_23_VALUES)
	{
		struct i2c_gpios_status * ans = 
			(struct i2c_gpios_status *)buf;
		
		/* error */
		/* XXX read even is called on every data received,
       * will zero size be managed as error  ??         */
		if (size != sizeof (*ans))
			goto *p_error_recv;

		/* GPIO_01 */
		if(gpio_addr == I2C_GPIOS_01_ADDR){
			gen.i2c_gpio0 = ans->gpio0;
			gen.i2c_gpio1 = ans->gpio1;
			
		}
		/* GPIO_23 */
		else if(gpio_addr == I2C_GPIOS_23_ADDR){
			gen.i2c_gpio2 = ans->gpio0;
			gen.i2c_gpio3 = ans->gpio1;
		
		}
		
	}

	/* parse microcontrolers answers */
	switch (buf[0]) 
	{
		/* slavedspic */	
		case I2C_ANS_SLAVEDSPIC_STATUS: {
			struct i2c_slavedspic_status * ans = 
				(struct i2c_slavedspic_status *)buf;
			
			if (size != sizeof (*ans))
				goto *p_error_recv;
	      
            /* XXX syncronized with slavedspic */

         	/* infos */
         	slavedspic.status = ans->status;
            slavedspic.nb_stored_fires = ans->nb_stored_fires;
			break;
		}
	

		default:
			break;
	}

	return;

 /* received error */	
error_recv:

	/* manage error */
	i2c_errors++;
	NOTICE(E_USER_I2C_PROTO, "recv error state=%d op=%d", 
	       i2c_state, running_op);

	if (i2c_errors > I2C_MAX_ERRORS) {
		I2C_ERROR("I2C error, slave not ready");

		i2c_reset();
		i2c_errors = 0;
	}
}
#endif /* !HOST_VERSION */

/* send generic command */
static int8_t i2c_send_command(uint8_t addr, uint8_t *buf, uint8_t size) 
{
#ifdef HOST_VERSION
	return robotsim_i2c(addr, buf, size);
#else
	uint8_t flags;
  	microseconds us = time_get_us2();
	uint8_t i;

	/* HACK: send 3 times the cmd */
	for(i = 0; i < 3; i++) {
		/* time out prevent */
		while ((time_get_us2() - us) < (I2C_TIMEOUT)*1000L) 
		{
			IRQ_LOCK(flags);
	
			/* copy data and fill params of cmd */
			if (command_size == 0) {
				memcpy(command_buf, buf, size);
				command_dest = addr;
				command_size = size;
				
				IRQ_UNLOCK(flags);
				return 0;;
			}
			IRQ_UNLOCK(flags);
		}
	}

	/* XXX: this should not happen... except if we are called from an
	 * interrupt context, but it's forbidden */

	/* simulate write_event */
	i2c_write_event(2);
	I2C_ERROR("I2C command send failed");
	return -EBUSY;
#endif
}

/***************************************************************************
 * PULLING COMMANDS (read from slaves)
 **************************************************************************/

#ifndef HOST_VERSION
/* read i2c gpio ports 0 and 1 */
static int8_t i2c_read_gpios_01_values(void)
{
	struct i2c_gpios_status buf;
	int8_t err;

	gpio_addr = I2C_GPIOS_01_ADDR;
	err = i2c_read(I2C_GPIOS_01_ADDR, I2C_REQ_GPIOS_STATUS,	sizeof(buf));
	
	//printf("gp0 = %d gp1 = %d gp2 = %d gp3 = %d\r\n",
	//gen.i2c_gpio0, gen.i2c_gpio1, gen.i2c_gpio2, gen.i2c_gpio3);
	
	return err;
}

/* read i2c gpio ports 0 and 1 */
static int8_t i2c_read_gpios_23_values(void)
{
	struct i2c_gpios_status buf;
	int8_t err;

	gpio_addr = I2C_GPIOS_23_ADDR;
	err = i2c_read(I2C_GPIOS_23_ADDR, I2C_REQ_GPIOS_STATUS,	sizeof(buf));
	return err;
}

/* read slavedspic status */
static int8_t i2c_req_slavedspic_status(void)
{
	struct i2c_slavedspic_status buf;
	int8_t err;

	err = i2c_read(I2C_SLAVEDSPIC_ADDR, I2C_REQ_SLAVEDSPIC_STATUS, sizeof(buf));
	return err;
}
#endif /* !HOST_VERSION */

/****************************************************************************
 * CONTROL COMMANDS (write to slaves) 
 ****************************************************************************/

/* dummy slavedspic led control */
int8_t i2c_led_control(uint8_t addr, uint8_t led, uint8_t state)
{
	struct i2c_cmd_led_control buf;
	buf.hdr.cmd = I2C_CMD_LED_CONTROL;
	buf.led_num = led;
	buf.state = state;
	return i2c_send_command(addr, (uint8_t*)&buf, sizeof(buf));
}

/*******************************************************************************
 * 2014 functions
 ******************************************************************************/

/****** GENERIC FUNCTIONS */

int8_t i2c_slavedspic_mode_init(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_INIT;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_power_off(void)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_POWER_OFF;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

/* wait for slavedspic is ready */
void i2c_slavedspic_wait_ready(void)
{ 
#ifndef HOST_VERSION                   
   //microseconds __us = time_get_us2();                 
   //uint8_t __ret = 1;             
    
	/* DANGEROUS */
   do{
      i2cproto_wait_update();
   } while(slavedspic.status == I2C_SLAVEDSPIC_STATUS_BUSY);
#endif
}

/****** SIMPLE ACTUATORS */

/* set stick mode */
int8_t i2c_slavedspic_mode_stick(uint8_t type, uint8_t mode, int8_t offset)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_STICK;
	buf.stick.mode = mode;
	buf.stick.type = type;
	buf.stick.offset = offset;
	
	if(mode==I2C_STICK_MODE_CLEAN_FLOOR) 
	{
		if(type==I2C_STICK_TYPE_RIGHT)
			buf.stick.offset = 30;
		else
			buf.stick.offset = 40;
	}
		

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}


/* set comb mode */
int8_t i2c_slavedspic_mode_combs(uint8_t mode, int8_t offset)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_COMBS;
	buf.combs.mode = mode;
	buf.combs.offset = offset;
	
	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

/* set tree tray */
int8_t i2c_slavedspic_mode_tree_tray (uint8_t mode, int8_t offset)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_TREE_TRAY;
	buf.tree_tray.mode = mode;
	buf.tree_tray.offset = offset;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

/* set boot tray mode */
int8_t i2c_slavedspic_mode_boot_tray(uint8_t mode)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_BOOT_TRAY;
	buf.boot_tray.mode = mode;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

/* set boot door mode*/
int8_t i2c_slavedspic_mode_boot_door(uint8_t mode)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_BOOT_DOOR;
	buf.boot_door.mode = mode;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}


/****** MULTIPLE ACTUATORS */

/* set harvest fruits mode */
int8_t i2c_slavedspic_mode_harvest_fruits(uint8_t mode)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_HARVEST_FRUITS;
	buf.harvest_fruits.mode = mode;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

/* set dump fruits mode */
int8_t i2c_slavedspic_mode_dump_fruits(uint8_t mode)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_DUMP_FRUITS;
	buf.dump_fruits.mode = mode;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}


/* ARM-FIRES modes */

int8_t i2c_slavedspic_mode_hide_arm (uint8_t sucker_type) 
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_ARM;
	buf.arm.mode = I2C_SLAVEDSPIC_MODE_ARM_HIDE;
    buf.arm.sucker_type = sucker_type;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_ready_for_pickup_torch (uint8_t sucker_type)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_ARM;
	buf.arm.mode = I2C_SLAVEDSPIC_MODE_ARM_PICKUP_TORCH_READY;
    buf.arm.sucker_type = sucker_type;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_pickup_torch (uint8_t sucker_type)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_ARM;
	buf.arm.mode = I2C_SLAVEDSPIC_MODE_ARM_PICKUP_TORCH_DO;
    buf.arm.sucker_type = sucker_type;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}


int8_t i2c_slavedspic_mode_ready_for_pickup_fire (uint8_t sucker_type, uint8_t level)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_ARM;
	buf.arm.mode = I2C_SLAVEDSPIC_MODE_ARM_PICKUP_FIRE_READY;
    buf.arm.sucker_type = sucker_type;
    buf.arm.level = level;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_pickup_fire (uint8_t sucker_type, uint8_t level)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_ARM;
	buf.arm.mode = I2C_SLAVEDSPIC_MODE_ARM_PICKUP_FIRE_DO;
    buf.arm.sucker_type = sucker_type;
    buf.arm.level = level;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_store_fire (uint8_t sucker_type)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_ARM;
	buf.arm.mode = I2C_SLAVEDSPIC_MODE_ARM_STORE;
    buf.arm.sucker_type = sucker_type;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_load_fire (uint8_t sucker_type)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_ARM;
	buf.arm.mode = I2C_SLAVEDSPIC_MODE_ARM_LOAD_FIRE;
    buf.arm.sucker_type = sucker_type;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}


static void arm_x_to_ay (int16_t x, int16_t *a, int16_t *y)
{
/* XXX keep syncronized with slavedspic/actuators.c */
#define SHOULDER_JOIN_X   (165-21) // 144
#define SHOULDER_JOIN_Y   (130)
#define ARM_LENGTH        (219.0)
#define SUCKER_LENGTH_0	  (46.5)
#define SUCKER_LENGTH_180 (55.5)

    double a1, x1, y1;

	x1 = (double) (x - SHOULDER_JOIN_X);
    a1 = acos (x1 / ARM_LENGTH);
    y1 = ARM_LENGTH * sin(a1);

	//printf ("a1 = %f, x1 = %f, y1 = %f\n\r", DEG(a1), x1, y1);

    *a = (int16_t)DEG(a1);
    *y = (int16_t)y1 + SHOULDER_JOIN_Y; 

	//printf ("a = %d, y = %d\n\r", *a, *y);  
}

int8_t i2c_slavedspic_mode_putdown_fire 
        (uint8_t sucker_type, uint8_t level,
         int16_t x, int16_t *y, int16_t *a, int8_t sucker_angle)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_ARM;
	buf.arm.mode = I2C_SLAVEDSPIC_MODE_ARM_PUTDOWN_FIRE;
    buf.arm.sucker_type = sucker_type;
    buf.arm.level = level;
    buf.arm.x_lsb = (uint8_t)((uint16_t)x & 0x00FF);
    buf.arm.x_msb = (uint8_t)((uint16_t)(x >> 8) & 0x00FF);
    buf.arm.sucker_angle = sucker_angle;

    /* y,a final positions */
    arm_x_to_ay (x, a, y);

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}


int8_t i2c_slavedspic_mode_putdown_fire_inv
        (uint8_t sucker_type, uint8_t level, int16_t x, int16_t *y, int16_t *a)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_ARM;
	buf.arm.mode = I2C_SLAVEDSPIC_MODE_ARM_PUTDOWN_FIRE_INV;
    buf.arm.sucker_type = sucker_type;
    buf.arm.level = level;
    buf.arm.x_lsb = (uint8_t)((uint16_t)x & 0x00FF);
    buf.arm.x_msb = (uint8_t)((uint16_t)(x >> 8) & 0x00FF);
    buf.arm.sucker_angle = 0;

    /* y,a final positions */
    arm_x_to_ay (x, a, y);

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

int8_t i2c_slavedspic_mode_release_fire (uint8_t sucker_type)
{
	struct i2c_cmd_slavedspic_set_mode buf;

	/* fill cmd structure */
	buf.hdr.cmd = I2C_CMD_SLAVEDSPIC_SET_MODE;
	buf.mode = I2C_SLAVEDSPIC_MODE_ARM;
	buf.arm.mode = I2C_SLAVEDSPIC_MODE_ARM_RELEASE_FIRE;
    buf.arm.sucker_type = sucker_type;

	/* send command and return */
	return i2c_send_command(I2C_SLAVEDSPIC_ADDR, (uint8_t*)&buf, sizeof(buf));
}

  
/*******************************************************************************
 * Debug functions
 ******************************************************************************/
#if 0

uint8_t val[32];
uint8_t i=0;
uint8_t error = 0;

void i2c_test_read_event(uint8_t *rBuff, uint16_t size)
{
		error = 0;
		
		if(size == 1 && rBuff[0]==val[0])
			printf("%d val_rd = %d\n\r", i++, rBuff[0]);
		else{
			printf("%d Error lectura: leido %d\r\n", i++, rBuff[0]);
			error = 1;
			i2c_reset();
		}			
			
		val[0]++;
}

void i2c_test_write_event(uint16_t size)
{
	if(size ==1)
		printf("%d val_wr = %d\n\r", i, val[0]);
	else{
		printf("%d Error escritura: %d bytes escritos\n\r",i, size);
		error = 1;
		i2c_reset();
	}
}

void i2c_test(void)
{
	uint8_t ret;
	
	while(1){
		ret = i2c_write(0x21, 0x02, val, 1);	
		wait_ms(100);
		ret = i2c_read(0x21, 0x02, 1);
		wait_ms(100);
	
	}
}	
#endif

#if 0
	while(1){
		wait_ms(200);
		pepe ^=1;
		i2c_led_control(I2C_SLAVEDSPIC_ADDR,1,pepe);
		printf("slavedspic led = %d\r\n", slavedspic.led);
	}
#endif



