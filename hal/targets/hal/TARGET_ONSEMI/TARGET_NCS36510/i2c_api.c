/**
 *******************************************************************************
 * @file    i2c.c
 * @brief Implementation of an i2c api
 * @internal
 * @author  ON Semiconductor
 * $Rev: 3525 $
 * $Date: 2015-07-20 15:24:25 +0530 (Mon, 20 Jul 2015) $
 ******************************************************************************
 * @copyright (c) 2012 ON Semiconductor. All rights reserved.
 * ON Semiconductor is supplying this software for use with ON Semiconductor
 * processor based microcontrollers only.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * ON SEMICONDUCTOR SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL,
 * INCIDENTAL, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 * @endinternal
 *
 * @ingroup i2c
 *
 */
#if DEVICE_I2C

#include "i2c.h"
#include "i2c_api.h"

#define I2C_READ_WRITE_BIT_MASK	0xFE

/* See i2c_api.h for details */
void i2c_init(i2c_t *obj, PinName sda, PinName scl)
{
	fI2cInit(obj, sda, scl);
}

/* See i2c_api.h for details */
void i2c_frequency(i2c_t *obj, int hz)
{
	fI2cFrequency(obj, hz);
}

/* See i2c_api.h for details */
int i2c_start(i2c_t *obj)
{
	return(fI2cStart(obj));
}

/* See i2c_api.h for details */
int i2c_stop(i2c_t *obj)
{
	return(fI2cStop(obj));
}

/* See i2c_api.h for details */
int i2c_read(i2c_t *obj, int address, char *data, int length, int stop)
{	/* TODO address parameter not usable */
	int Count, status;
	const char WriteData = (address | (~I2C_READ_WRITE_BIT_MASK)) & 0xFF;

	/* Send  start bit */
	status = fI2cStart(obj);
	if(status)
	{/* Error sending start bit */
		return status;
	}

	/* Send address | read */
	Count = fI2cWriteB(obj, &WriteData, 1);
	if(Count != 1)
	{/* Error sending address */
		return Count;
	}

	/* Send command/s */
	Count = fI2cReadB(obj, data, length);
	if(Count != length)
	{/* Error sending coomand/s */
		return Count;
	}
	if(stop)	/* Send stop bit if requested */
	{
		status = fI2cStop(obj);
		if(status)
		{/* Error sending stop bit */
			return status;
		}
	}
	return Count;
}

/* See i2c_api.h for details */
int i2c_write(i2c_t *obj, int address, const char *data, int length, int stop)
{
	int Count, status;
	const char WriteData = (address & I2C_READ_WRITE_BIT_MASK) & 0xFF;

	/* Send start bit */
	status = fI2cStart(obj);
	if(status)
	{/* Error sending start bit */
		return status;
	}

	/* Send address | write */
	Count = fI2cWriteB(obj, &WriteData, 1);
	if(Count != 1)
	{/* Error sending address */
		return Count;
	}

	/* Sens command, [data] */
	Count = fI2cWriteB(obj, data, length);
	if(Count != length)
	{/* Error sending address */
		return Count;
	}

	if(stop)	/* If stop requested */
	{/* Send stop bit */
		status = fI2cStop(obj);
		if(status)
		{/* Error sending stop bit */
			return status;
		}
	}
	return Count;
}

/* See i2c_api.h for details */
void i2c_reset(i2c_t *obj)
{
	(void)fI2cStop(obj);
}

/* See i2c_api.h for details */
int i2c_byte_read(i2c_t *obj, int last) /* TODO return size can be uint8_t */
{
	int Count;
	char data;
	Count = fI2cReadB(obj, &data, 1);
	if(Count != 1)
	{/* Error */
		return Count;
	}
	if(last)
	{/* ACK */
		obj->membase->CMD_REG = I2C_CMD_WDAT0;
	}
	else
	{/* No ACK */
		obj->membase->CMD_REG = I2C_CMD_WDAT1;
	}
	return data;
}

/* See i2c_api.h for details */
int i2c_byte_write(i2c_t *obj, int data)
{
	int Count;
	Count = fI2cWriteB(obj, (const char *)&data, 1);
	if(Count != 1)
	{
		return Count;
	}

	obj->membase->CMD_REG = I2C_CMD_VRFY_ACK; /* Verify ACK */

	while(obj->membase->STATUS.WORD & I2C_STATUS_CMD_FIFO_OFL_BIT); /* Wait till command overflow ends */

	if(obj->membase->STATUS.WORD & I2C_STATUS_BUS_ERR_BIT)
	{/* Bus error means NAK received */
		return 0;
	}
	else
	{/* ACK received */
		return 1;
	}
}

#endif /* DEVICE_I2C */