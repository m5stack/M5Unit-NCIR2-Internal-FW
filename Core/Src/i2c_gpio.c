#include "i2c_gpio.h"   
#include <stdbool.h>

#define Pin_SCL		GPIO_PIN_3
#define Pin_SDA		GPIO_PIN_4

#define Pin_SCL_L		HAL_GPIO_WritePin(I2C2_GPIOx,Pin_SCL,GPIO_PIN_RESET)
#define Pin_SCL_H		HAL_GPIO_WritePin(I2C2_GPIOx,Pin_SCL,GPIO_PIN_SET)
 
#define Pin_SDA_L		HAL_GPIO_WritePin(I2C2_GPIOx,Pin_SDA,GPIO_PIN_RESET)
#define Pin_SDA_H		HAL_GPIO_WritePin(I2C2_GPIOx,Pin_SDA,GPIO_PIN_SET)

#define sda_out_mode() 	
#define sda_in_mode() 	
 
#define Read_SDA_Pin	HAL_GPIO_ReadPin(I2C2_GPIOx,Pin_SDA)

#define DELAY_WAIT_SET  24
#define DELAY_HIGH_NS   100
#define DELAY_LOW_NS    60
#define T_SU_STP_US	    DELAY_HIGH_NS
#define T_BUF_US        DELAY_LOW_NS
#define T_SU_STA_US	    DELAY_LOW_NS
#define T_HD_STA_US	    DELAY_HIGH_NS

static void i2c_gpio_set_scl(int state)
{
	if (state) {
        Pin_SCL_H;
    } else {
        Pin_SCL_L;
    }
}

static void i2c_gpio_set_sda(int state)
{
    sda_out_mode();
	if (state) {
        Pin_SDA_H;
    } else {
        Pin_SDA_L;
    }
}

static int i2c_gpio_get_sda(void)
{
    sda_in_mode();
	int rc = Read_SDA_Pin;

	/* Default high as that would be a NACK */
	return rc != 0;
}

// 初始化IIC的IO口
void gpio_i2c_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};   // 定义GPIO结构体
	
    __HAL_RCC_GPIOA_CLK_ENABLE();  // 打开GPIOA口时钟
	
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pin = Pin_SCL ; // IIC对应IO口
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH; // 50MHZ
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pin = Pin_SDA ; // IIC对应IO口
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH; // 50MHZ
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
	
    // I2C总线的SDA和SCL两条信号线同时处于高电平时；表示空闲状态
    Pin_SDA_H;	
    Pin_SCL_H;
}

static void i2c_start(void)
{
	if (!i2c_gpio_get_sda()) {
		/*
		 * SDA is already low, so we need to do something to make it
		 * high. Try pulsing clock low to get slave to release SDA.
		 */
		i2c_gpio_set_scl(0);
        user_delaynus_tim(DELAY_LOW_NS);
		i2c_gpio_set_scl(1);
        user_delaynus_tim(T_SU_STA_US);
	}
	i2c_gpio_set_sda(0);
    user_delaynus_tim(T_HD_STA_US);

	i2c_gpio_set_scl(0);
    user_delaynus_tim(DELAY_LOW_NS);
}

static void i2c_repeated_start(void)
{
	i2c_gpio_set_sda(1);
	i2c_gpio_set_scl(1);
    user_delaynus_tim(DELAY_HIGH_NS);

	user_delaynus_tim(T_SU_STA_US);
	i2c_start();
}

static void i2c_stop(void)
{
	i2c_gpio_set_sda(0);
    user_delaynus_tim(DELAY_LOW_NS);

	i2c_gpio_set_scl(1);
    user_delaynus_tim(DELAY_HIGH_NS);

    user_delaynus_tim(T_SU_STP_US);
    i2c_gpio_set_sda(1);
    user_delaynus_tim(T_BUF_US);
}

static void i2c_write_bit(int bit)
{
	/* SDA hold time is zero, so no need for a delay here */
	i2c_gpio_set_sda(bit);
	i2c_gpio_set_scl(1);
	user_delaynus_tim(DELAY_HIGH_NS);
	i2c_gpio_set_scl(0);
	user_delaynus_tim(DELAY_LOW_NS);
}

static void i2c_send_ack(int bit)
{
	/* SDA hold time is zero, so no need for a delay here */
	i2c_gpio_set_sda(bit);
	user_delaynus_tim(DELAY_WAIT_SET);
	i2c_gpio_set_scl(1);
	user_delaynus_tim(DELAY_HIGH_NS);
	i2c_gpio_set_scl(0);
	user_delaynus_tim(DELAY_LOW_NS);
	i2c_gpio_set_sda(1);
}

static bool i2c_read_bit(void)
{
	bool bit;

	/* SDA hold time is zero, so no need for a delay here */
	// i2c_gpio_set_sda(1); /* Stop driving low, so slave has control */
	// i2c_gpio_set_sda(0); /* Stop driving low, so slave has control */

	sda_in_mode();
	sda_in_mode();
	i2c_gpio_set_scl(1);
	user_delaynus_tim(DELAY_HIGH_NS);

	bit = i2c_gpio_get_sda();

	i2c_gpio_set_scl(0);
	user_delaynus_tim(DELAY_LOW_NS);
	return bit;
}

static bool i2c_wait_ack(void)
{
	// uint32_t timeout = 0;
	bool bit;

	/* SDA hold time is zero, so no need for a delay here */
	i2c_gpio_set_sda(1); /* Stop driving low, so slave has control */

	sda_in_mode();
	i2c_gpio_set_scl(1);
	user_delaynus_tim(DELAY_HIGH_NS);

	bit = i2c_gpio_get_sda();

	i2c_gpio_set_scl(0);
	user_delaynus_tim(DELAY_LOW_NS);
	return bit;
}

static bool i2c_write_byte(uint8_t byte)
{
	uint8_t mask = 1 << 7;

	do {
		i2c_write_bit(byte & mask);
	} while (mask >>= 1);

	/* Return inverted ACK bit, i.e. 'true' for ACK, 'false' for NACK */
	return !i2c_wait_ack();
}

static uint8_t i2c_read_byte(void)
{
	unsigned int byte = 1U;

	do {
		byte <<= 1;
		byte |= i2c_read_bit();
	} while (!(byte & (1 << 8)));

	return byte;
}

int i2c_bitbang_transfer(struct i2c_msg *msgs, uint8_t num_msgs,
			   uint16_t slave_address)
{
	uint8_t *buf, *buf_end;
	unsigned int flags;
	int result = -1;

	if (!num_msgs) {
		return 0;
	}

	/* We want an initial Start condition */
	flags = I2C_MSG_RESTART;

	/* Make sure we're in a good state so slave recognises the Start */
	i2c_gpio_set_scl(1);
	// flags |= I2C_MSG_STOP;

	do {
		/* Stop flag from previous message? */
		// if (flags & I2C_MSG_STOP) {
		// 	i2c_stop();
		// }

		/* Forget old flags except start flag */
		flags &= I2C_MSG_RESTART;

		/* Start condition? */
		if (flags & I2C_MSG_RESTART) {
			i2c_start();
		} else if (msgs->flags & I2C_MSG_RESTART) {
			i2c_repeated_start();
		}

		/* Get flags for new message */
		flags |= msgs->flags;

		/* Send address after any Start condition */
		if (flags & I2C_MSG_RESTART) {
			unsigned int byte0 = slave_address << 1;

			byte0 |= ((flags & I2C_MSG_RW_MASK) == I2C_MSG_READ);
			// i2c_write_byte(byte0);
			// if (!i2c_wait_ack()) {
			// 	goto finish; /* No ACK received */
			// }

			if (!i2c_write_byte(byte0)) {
				goto finish; /* No ACK received */
			}
			flags &= ~I2C_MSG_RESTART;
		}

		/* Transfer data */
		buf = msgs->buf;
		buf_end = buf + msgs->len;
		if ((flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			/* Read */
			while (buf < buf_end) {
				*buf++ = i2c_read_byte();
				/* ACK the byte, except for the last one */
				i2c_send_ack(buf == buf_end);
			}
		} else {
			/* Write */
			while (buf < buf_end) {
				if (!i2c_write_byte(*buf++)) {
					goto finish; /* No ACK received */
				}
			}
		}

		/* Next message */
		msgs++;
		num_msgs--;
	} while (num_msgs);

	/* Complete without error */
	result = 0;
finish:
	i2c_stop();

	return result;
}
