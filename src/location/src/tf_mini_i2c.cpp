#include <tf_mini_i2c.h>

TFMiniPro::TFMiniPro(uint8 address, uint8 bus) noexcept(false)
{
	i2c_dev_ = new I2C(address, bus);
}

bool TFMiniPro::SendCommand(uint32 cmnd, uint32 param, uint8 *response)
{
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Step 1 - Build the command data to send to the device
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	const uint8 reply_length = cmnd & 0xff;
	const uint8 cmd_length = (cmnd >> 8) & 0xff;
	if (cmd_length < 4 || cmd_length > TFMP_MAX_CMD_LEN ||
	    reply_length > TFMP_DATA_LENGTH || (reply_length && !response)) return false;
	uint8 check_sum = 0;
	uint8 cmnd_data[TFMP_MAX_CMD_LEN] = {};
	cmnd_data[0] = TFMP_FRAME_HEADER;
	cmnd_data[1] = cmd_length;
	cmnd_data[2] = (cmnd >> 16) & 0xff;
	cmnd_data[3] = (cmnd >> 24) & 0xff;

	if (cmnd == TFMP_SET_FRAME_RATE)	   // If the command is to Set Frame Rate...
		memcpy(&cmnd_data[3], &param, 2);  // add the 2 byte Frame Rate parameter.
	else if (cmnd == TFMP_SET_BAUD_RATE)   // If the command is to Set Baud Rate...
		memcpy(&cmnd_data[3], &param, 4);  // add the 3 byte Baud Rate parameter.
	else if (cmnd == TFMP_SET_I2C_ADDRESS) // If the command to set I2C address...
		memcpy(&cmnd_data[3], &param, 1);  // copy the 1 byte Address parameter.

	check_sum = 0;
	// Add together all bytes but the last...
	for (uint8_t i = 0; i < (cmd_length - 1); i++)
		check_sum += cmnd_data[i];
	// and save it as the last byte of command data.
	cmnd_data[cmd_length - 1] = uint8_t(check_sum);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Step 2 - Send the command data array to the device
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	try
	{
		i2c_dev_->write_bytes(cmnd_data, cmd_length);
	}
	catch (std::system_error &e)
	{
		printf("error in TF Mini Pro configuration %s\n", e.what());
		return false;
	}

	// If no reply data expected, then return. Otherwise,
	// wait for device to process the command and continue.
	if (reply_length == 0)
		return true;
	else if (cmnd != TFMP_I2C_FORMAT_CM)
		usleep(200000);
	else
		// This RK/TFmini pair produced 0x2c instead of 0x59 on occasional
		// immediate reads. A bounded 1 ms command-to-read gap is bench-verified.
		usleep(1000);

	// - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Step 3 - Get command reply data back from the device.
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	try
	{
		i2c_dev_->read_bytes(response, reply_length);
	}
	catch (std::system_error &e)
	{
		printf("error reading channel%s\n", e.what());
		return false;
	}
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	// Step 4 - Perform a checksum test.
	// - - - - - - - - - - - - - - - - - - - - - - - - - - - -
	uint8 check_sum_response = 0; // 8 byte send command array
	for (uint8_t i = 0; i < (reply_length - 1); i++)
		check_sum_response += response[i];
	if (response[reply_length - 1] != (uint8_t)check_sum_response)
	{
		return false;
	}
	if (cmnd == TFMP_I2C_FORMAT_CM)
		return response[0] == 0x59 && response[1] == 0x59;
	if (reply_length < 4 || response[0] != TFMP_FRAME_HEADER ||
	    response[1] != reply_length || response[2] != cmnd_data[2]) return false;
	return true;
}

bool TFMiniPro::ChangeAddress(uint8 new_address)
{
	bool success = false;
	uint8 cmd_resp[5]; // array where the cmd response will be stored
	// Command to change address distance data in centimeters
	std::cout << "Changing TF Mini plus address..." << std::endl;
	if (SendCommand(TFMP_SET_I2C_ADDRESS, new_address, cmd_resp) != true)
		return success;

	// Save settings
	std::cout << "Saving settings..." << std::endl;
	if (SendCommand(TFMP_SAVE_SETTINGS, new_address, cmd_resp) != true)
		return success;
	usleep(500000);
	// Change device address
	i2c_dev_->change_address(new_address);
	return true;
}

float TFMiniPro::ReadData()
{
	float resp = -1;
	uint8 data[TFMP_DATA_LENGTH]; // array where the data response will be stored
	// Command device to ready distance data in centimeters
	if (SendCommand(TFMP_I2C_FORMAT_CM, 0, data) != true)
		return resp;

	uint16 dist = data[2] + (data[3] << 8);
	uint16 flux = data[4] + (data[5] << 8);
	// TFmini Plus standard frames: blind zone below 10 cm, maximum 1200 cm;
	// weak/overexposed returns must not refresh the localization height.
	if (dist < 10 || dist > 1200 || flux < 100 || flux == 65535) return -1.0f;
	resp = (float)dist;
	return resp / 100.0f;		  // Convert to meters and return
}

TFMiniPro::~TFMiniPro()
{
	delete i2c_dev_;
}
