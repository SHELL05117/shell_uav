#include <i2c.h>
#include <utility>  // for std::exchange

I2C::I2C(uint8 address, uint8 bus) noexcept(false)
	: kI2CAddress(address), kI2CBus(bus), kI2CFileDescriptor(-1)
{
	open_device();
}

I2C::~I2C()
{
	close_device();
}

// 移动构造函数
I2C::I2C(I2C&& other) noexcept
	: kI2CAddress(other.kI2CAddress),
	  kI2CBus(other.kI2CBus),
	  kI2CFileDescriptor(std::exchange(other.kI2CFileDescriptor, -1))
{
	// 资源已转移，other 不再拥有文件描述符
}

// 移动赋值运算符
I2C& I2C::operator=(I2C&& other) noexcept
{
	if (this != &other)
	{
		// 先关闭当前资源
		close_device();
		
		// 转移资源
		kI2CAddress = other.kI2CAddress;
		kI2CBus = other.kI2CBus;
		kI2CFileDescriptor = std::exchange(other.kI2CFileDescriptor, -1);
	}
	return *this;
}

void I2C::open_device() noexcept(false)
{
	char fileNameBuffer[32];
	sprintf(fileNameBuffer, "/dev/i2c-%d", kI2CBus);

	kI2CFileDescriptor = open(fileNameBuffer, O_RDWR);

	if (kI2CFileDescriptor < 0)
		throw std::system_error(errno, std::generic_category(), "[i2c]Could not open the file");

	if (ioctl(kI2CFileDescriptor, I2C_SLAVE, kI2CAddress) < 0)
		throw std::system_error(errno, std::generic_category(), "[i2c]Could not open dev on bus");
}

void I2C::close_device()
{
	if (kI2CFileDescriptor > 0)
	{
		close(kI2CFileDescriptor);
		// WARNING - This is not quite right, need to check for error first
		kI2CFileDescriptor = -1;
	}
}

void I2C::change_address(uint8 address)
{
	if (ioctl(kI2CFileDescriptor, I2C_SLAVE, address) < 0)
		throw std::system_error(errno, std::generic_category(), "[i2c]Could not open dev on bus");
}

uint16 I2C::read_register16(uint16 reg) noexcept(false)
{
	int read = i2c_smbus_read_word_data(kI2CFileDescriptor, reg);
	int resp = ((read & 0xFF) << 8) + (read >> 8);

	if (read < 0)
		throw std::system_error(errno, std::generic_category(), "[i2c]Read_register16 error");
	return resp;
}

uint16 I2C::read_register_block(uint16 reg, uint8 *values) noexcept(false)
{
	int read = i2c_smbus_read_block_data(kI2CFileDescriptor, reg, values);
	int resp = ((read & 0xFF) << 8) + (read >> 8);
	if (read < 0)
		throw std::system_error(errno, std::generic_category(), "[i2c]Read_register_block error");

	return resp;
}

uint16 I2C::write_register16(uint8 device_register, uint16 value) noexcept(false)
{
	int write = ((value & 0xFF) << 8) + (value >> 8);
	int resp = i2c_smbus_write_word_data(kI2CFileDescriptor, device_register, write);
	if (resp < 0)
		throw std::system_error(errno, std::generic_category(), "[i2c]Write_register16 error");

	return resp;
}

uint16 I2C::write_register_block(uint8 device_register, uint8 length, uint8 *values) noexcept(false)
{
	int resp = i2c_smbus_write_block_data(kI2CFileDescriptor, device_register, length, values);
	if (resp < 0)
		throw std::system_error(errno, std::generic_category(), "[i2c]Write_register_block error");

	return resp;
}

int I2C::read(uint8 reg) noexcept(false)
{
	int read = i2c_smbus_read_byte_data(kI2CFileDescriptor, reg);
	if (read < 0)
		throw std::system_error(errno, std::generic_category(), "[i2c]Read_register16 error");

	return read;
}

int I2C::write_register8(uint8 device_register, uint8 value) noexcept(false)
{
	int resp = i2c_smbus_write_byte_data(kI2CFileDescriptor, device_register, value);
	if (resp < 0)
		throw std::system_error(errno, std::generic_category(), "[i2c]Write_register16 error");

	return resp;
}

int I2C::write(uint8 writeValue) noexcept(false)
{
	int write = i2c_smbus_write_byte(kI2CFileDescriptor, writeValue);
	if (write < 0)
		throw std::system_error(errno, std::generic_category(), "[i2c]Write_register16 error");

	return write;
}