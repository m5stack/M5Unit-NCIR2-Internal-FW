#ifndef _I2C_GPIO_H_
#define _I2C_GPIO_H_
 
#include "main.h"
#include <stdbool.h>

 // 定义内联延时函数
#define delay_150ns() __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP")
#define delay_300ns() delay_150ns(); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP"); __asm("NOP")
#define delay_600ns() delay_300ns(); delay_300ns()
#define I2C2_Delay2us() delay_150ns()

#ifndef BIT
#define BIT(n)  (1UL << (n))
#endif


typedef struct i2c_msg {
	/** Data buffer in bytes */
	uint8_t		*buf;

	/** Length of buffer in bytes */
	uint32_t	len;

	/** Flags for this message */
	uint8_t		flags;
} i2c_msg_t;

/*
 * I2C_MSG_* are I2C Message flags.
 */

/** Write message to I2C bus. */
#define I2C_MSG_WRITE			(0U << 0U)

/** Read message from I2C bus. */
#define I2C_MSG_READ			BIT(0)

/** @cond INTERNAL_HIDDEN */
#define I2C_MSG_RW_MASK			BIT(0)
/** @endcond  */

/** Send STOP after this message. */
#define I2C_MSG_STOP			BIT(1)

/** RESTART I2C transaction for this message.
 *
 * @note Not all I2C drivers have or require explicit support for this
 * feature. Some drivers require this be present on a read message
 * that follows a write, or vice-versa.  Some drivers will merge
 * adjacent fragments into a single transaction using this flag; some
 * will not. */
#define I2C_MSG_RESTART			BIT(2)
 
#define	I2C2_GPIOx  GPIOA
#define Pin_SCL		GPIO_PIN_3
#define Pin_SDA		GPIO_PIN_4
 
void gpio_i2c_init(void);
int i2c_bitbang_transfer(struct i2c_msg *msgs, uint8_t num_msgs,
			   uint16_t slave_address);
#endif
