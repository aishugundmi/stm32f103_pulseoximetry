
#include "main.h"
#include "i2c.h"

#define MAX101_ADDRESS_7BIT		0x57
#define	read_sda_pin()			HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)



void i2c_delay()
{
	uint8_t t=10;
	while(t--)
	{
		;
	}
}

void i2c_stop(void)
{
	SCL_LOW();
	i2c_delay();
	SDA_LOW();
	i2c_delay();
	SCL_HIGH();
	i2c_delay();
	SDA_HIGH();
	i2c_delay();
}



int read_max101_register(uint8_t reg, uint8_t *data, int size)
{
	int ret = -1;

	//I2C START CONDITION https://i2c.info/wp-content/images/i2c.info/data-transfer.gif
	SCL_HIGH();
	i2c_delay();
	SDA_LOW();
	i2c_delay();
	SCL_LOW();


	//ADDRESSING IN WRITE MODE TO WRITE THE 8 BIT REGISTER VALUE TO THE MAX101
	uint8_t dev_address_write_mode = MAX101_ADDRESS_7BIT;
	uint8_t dev_address_read_mode = MAX101_ADDRESS_7BIT;

	dev_address_write_mode <<= 1;
	dev_address_write_mode += 0; //last bit zero for write, 1 for read

	dev_address_read_mode <<= 1;
	dev_address_read_mode += 1; //last bit zero for write, 1 for read

	//currently scl and sda will be low. Now send 8 BIT address to slave.
	//Rule:  Set or clear the data bits only when the SCK is low. While reading ACK, sample the ACK from the slave while the SCK is high.


	for(int i = 0; i < 8; i++) {

			i2c_delay(); //total scl low delay is 1+1 = 2ms

			if(dev_address_write_mode & (1<<(7-i))) { //If MSB bit is high, set data bit as high while SCL is low.
				SDA_HIGH();
			} else {
				SDA_LOW();
			}

			i2c_delay();  //We ensure the data transition happened in middle of the SCL_LOW

			//NOW MAKE SCL HIGH
			SCL_HIGH();
			i2c_delay(); //We kept 2 milli second to make the duty cycle same for scl low and scl high. May not be required.
			i2c_delay();
			SCL_LOW();
	}

	//8BITS are transmitted, now read the ACK LOW from the slave to ensure that the slave is responding...
	i2c_delay();
	SDA_HIGH();	//RELEASE SDA if it is LOW. So that slave can pull it down as an acknowledgement.
	i2c_delay();

	//Now we are going to sample the ACK on 9th bit SCK HIGH timing
	SCL_HIGH();
	i2c_delay();
	uint8_t ack = read_sda_pin(); // it should return 0 or 1 based on pin status. 0 means ACK is success.
	i2c_delay();
	SCL_LOW();

	if(ack)
	{
		i2c_stop();
		//return -1; //return on ack error
		goto error;
	}




	//------------------------ WE ARE HERE IF ADDRESSING IS SUCCESSFUL -----------------//

	//Now we have to write register address
		for(int i = 0; i < 8; i++) {

			i2c_delay(); //total scl low delay is 1+1 = 2ms

			if(reg & (1<<(7-i))) { //If MSB bit is high, set data bit as high while SCL is low.
				SDA_HIGH();
			} else {
				SDA_LOW();
			}

			i2c_delay();  //We ensure the data transition happened in middle of the SCL_LOW

			//NOW MAKE SCL HIGH
			SCL_HIGH();
			i2c_delay(); //We kept 2 milli second to make the duty cycle same for scl low and scl high. May not be required.
			i2c_delay();
			SCL_LOW();
	}
		//8BITS are transmitted, now read the ACK LOW from the slave to ensure that the slave is responding...
		i2c_delay();
		SDA_HIGH();	//RELEASE SDA if it is LOW. So that slave can pull it down as an acknowledgement.
		i2c_delay();

		//Now we are going to sample the ACK on 9th bit SCK HIGH timing
		SCL_HIGH();
		i2c_delay();
		ack = read_sda_pin(); // it should return 0 or 1 based on pin status. 0 means ACK is success.
		i2c_delay();
		SCL_LOW();


		if(ack)
		{
			i2c_stop();
			//return -1; //return on ack error for register address write
			goto error;
		}


	//------------------------ WE ARE HERE IF ADDRESSING IS SUCCESSFUL AND REGISTER WRITE IS SUCCESSFUL-----------------//

 	//Now we have to do REPEATED start
	//I2C START CONDITION https://i2c.info/wp-content/images/i2c.info/data-transfer.gif

	SCL_HIGH();
	i2c_delay();
	SDA_LOW();
	i2c_delay();
	SCL_LOW();


//--- We are now addressing the device in read mode..... After addressing is success, we will be reading data....///
	for(int i = 0; i < 8; i++) {

			i2c_delay(); //total scl low delay is 1+1 = 2ms

			if(dev_address_read_mode & (1<<(7-i))) { //If MSB bit is high, set data bit as high while SCL is low.
				SDA_HIGH();
			} else {
				SDA_LOW();
			}

			i2c_delay();  //We ensure the data transition happened in middle of the SCL_LOW

			//NOW MAKE SCL HIGH
			SCL_HIGH();
			i2c_delay(); //We kept 2 milli second to make the duty cycle same for scl low and scl high. May not be required.
			i2c_delay();
			SCL_LOW();
	}

	//8BITS are transmitted, now read the ACK LOW from the slave to ensure that the slave is responding...
	i2c_delay();
	SDA_HIGH();	//RELEASE SDA if it is LOW. So that slave can pull it down as an acknowledgement.
	i2c_delay();

	//Now we are going to sample the ACK on 9th bit SCK HIGH timing
	SCL_HIGH();
	i2c_delay();
	ack = read_sda_pin(); // it should return 0 or 1 based on pin status. 0 means ACK is success.
	i2c_delay();
	SCL_LOW();

	if(ack)
	{
		i2c_stop();
		//return -1; //return on ack error
		goto error;
	}

	//---Let  us read the data.... Now master have to give ACK to slave for all bytes except for the last byte.

	uint8_t last_byte_status = 0;

	for(int i = 0; i < size; i++)
	{
			if(i == (size -1 ))
			{
				last_byte_status = 1;
			}

			data[i] = 0; //init the data to be filled with zero. This is the pointer passed from the main application to which we will be filling the data bytes.

			for(int j = 0; j < 8; j++)
			{

				i2c_delay(); //total scl low delay is 1+1 = 2ms
				SDA_HIGH(); //we are reading, so keep data always high, then only we can detect if slave makes it zero for corresponding bits... :)
				i2c_delay();  //We ensure the data transition happened in middle of the SCL_LOW

				//NOW MAKE SCL HIGH
				SCL_HIGH();
				i2c_delay();
				uint8_t data_bit =  read_sda_pin();
				i2c_delay();
				SCL_LOW();
				if(data_bit)
				{
					data[i] |= (1<<(7-j)); //set bits from msb to lsb
				} //no need of else because we made the bits zero above at the time of init for each byte(data[i] = 0;)

			}

				//8BITS are read... Master write ACK LOW for all received bytes except last byte.
				i2c_delay();

				if (last_byte_status)
				{
					SDA_HIGH();	//RELEASE SDA if it is LOW. So that slave can pull it down as an acknowledgement.
				}
				else
				{
					SDA_LOW();
				}
				i2c_delay();

				SCL_HIGH();
				i2c_delay();
				i2c_delay();
				SCL_LOW();


	    }

	i2c_stop();
	return 0;

	error:
		return -1;
}



int write_max101_register(uint8_t reg, uint8_t data)
{
	int ret = -1;

	//I2C START CONDITION https://i2c.info/wp-content/images/i2c.info/data-transfer.gif
	SCL_HIGH();
	i2c_delay();
	SDA_LOW();
	i2c_delay();
	SCL_LOW();


	//ADDRESSING IN WRITE MODE TO WRITE THE 8 BIT REGISTER VALUE TO THE MAX101
	uint8_t dev_address_write_mode = MAX101_ADDRESS_7BIT;

	dev_address_write_mode <<= 1;
	dev_address_write_mode += 0; //last bit zero for write, 1 for read


	//currently scl and sda will be low. Now send 8 BIT address to slave.
	//Rule:  Set or clear the data bits only when the SCK is low. While reading ACK, sample the ACK from the slave while the SCK is high.


	for(int i = 0; i < 8; i++) {

			i2c_delay(); //total scl low delay is 1+1 = 2ms

			if(dev_address_write_mode & (1<<(7-i))) { //If MSB bit is high, set data bit as high while SCL is low.
				SDA_HIGH();
			} else {
				SDA_LOW();
			}

			i2c_delay();  //We ensure the data transition happened in middle of the SCL_LOW

			//NOW MAKE SCL HIGH
			SCL_HIGH();
			i2c_delay(); //We kept 2 milli second to make the duty cycle same for scl low and scl high. May not be required.
			i2c_delay();
			SCL_LOW();
	}

	//8BITS are transmitted, now read the ACK LOW from the slave to ensure that the slave is responding...
	i2c_delay();
	SDA_HIGH();	//RELEASE SDA if it is LOW. So that slave can pull it down as an acknowledgement.
	i2c_delay();

	//Now we are going to sample the ACK on 9th bit SCK HIGH timing
	SCL_HIGH();
	i2c_delay();
	uint8_t ack = read_sda_pin(); // it should return 0 or 1 based on pin status. 0 means ACK is success.
	i2c_delay();
	SCL_LOW();

	if(ack)
	{
		i2c_stop();
		//return -1; //return on ack error
		goto error;
	}




	//------------------------ WE ARE HERE IF ADDRESSING IS SUCCESSFUL -----------------//

	//Now we have to write register address
		for(int i = 0; i < 8; i++) {

			i2c_delay(); //total scl low delay is 1+1 = 2ms

			if(reg & (1<<(7-i))) { //If MSB bit is high, set data bit as high while SCL is low.
				SDA_HIGH();
			} else {
				SDA_LOW();
			}

			i2c_delay();  //We ensure the data transition happened in middle of the SCL_LOW

			//NOW MAKE SCL HIGH
			SCL_HIGH();
			i2c_delay(); //We kept 2 milli second to make the duty cycle same for scl low and scl high. May not be required.
			i2c_delay();
			SCL_LOW();
	}
		//8BITS are transmitted, now read the ACK LOW from the slave to ensure that the slave is responding...
		i2c_delay();
		SDA_HIGH();	//RELEASE SDA if it is LOW. So that slave can pull it down as an acknowledgement.
		i2c_delay();

		//Now we are going to sample the ACK on 9th bit SCK HIGH timing
		SCL_HIGH();
		i2c_delay();
		ack = read_sda_pin(); // it should return 0 or 1 based on pin status. 0 means ACK is success.
		i2c_delay();
		SCL_LOW();


		if(ack)
		{
			i2c_stop();
			//return -1; //return on ack error for register address write
			goto error;
		}


	//------------------------ WE ARE HERE IF ADDRESSING IS SUCCESSFUL AND REGISTER WRITE IS SUCCESSFUL-----------------//

		//Now we have to write data
				for(int i = 0; i < 8; i++) {

					i2c_delay(); //total scl low delay is 1+1 = 2ms

					if(data & (1<<(7-i))) { //If MSB bit is high, set data bit as high while SCL is low.
						SDA_HIGH();
					} else {
						SDA_LOW();
					}

					i2c_delay();  //We ensure the data transition happened in middle of the SCL_LOW

					//NOW MAKE SCL HIGH
					SCL_HIGH();
					i2c_delay(); //We kept 2 milli second to make the duty cycle same for scl low and scl high. May not be required.
					i2c_delay();
					SCL_LOW();
			}
				//8BITS are transmitted, now read the ACK LOW from the slave to ensure that the slave is responding...
				i2c_delay();
				SDA_HIGH();	//RELEASE SDA if it is LOW. So that slave can pull it down as an acknowledgement.
				i2c_delay();

				//Now we are going to sample the ACK on 9th bit SCK HIGH timing
				SCL_HIGH();
				i2c_delay();
				ack = read_sda_pin(); // it should return 0 or 1 based on pin status. 0 means ACK is success.
				i2c_delay();
				SCL_LOW();


				if(ack)
				{
					i2c_stop();
					//return -1; //return on ack error for register address write
					goto error;
				}


	i2c_stop();
	return 0;

	error:
		return -1;
}






