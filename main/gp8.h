#pragma once
#include <string.h>
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#define TAG "DFRobot_GP8XXX_IIC"
#define I2C_MASTER_TIMEOUT_MS 1000

class GP8 {
  public:
    /**
     * @enum eOutPutRange_t
     * @brief Analog voltage output range select
     */
    typedef enum {
      eOutputRange2_5V  = 0,
      eOutputRange5V  = 1,
      eOutputRange10V = 2,
      eOutputRangeVCC   = 3
    } eOutPutRange_t;

    GP8() = default;

    /**
     * @fn setDACOutVoltage
     * @brief Set different channel output DAC values
     * @param data requires the output voltage value
     * @param channel output channel 0: channel 0; 1: Channel 1; 2: All channels
     * @return NONE
     */
    virtual void setDACOutVoltage(uint16_t data, uint8_t channel) =0;   
  protected:
    uint16_t _voltage = 0;
};

class GP8I2C: public GP8 {
  public:

    #define RESOLUTION_12_BIT 0x0FFF
    #define RESOLUTION_15_BIT 0x7FFF
    #define GP8XXX_CONFIG_CURRENT_REG                  uint8_t(0x02)
    #define DFGP8XXX_I2C_DEVICEADDR                    uint8_t(0x58)   //!< i2c address

    #define GP8XXX_STORE_TIMING_HEAD            0x02  ///< Store function timing start head
    #define GP8XXX_STORE_TIMING_ADDR            0x10  ///< The first address for entering store timing
    #define GP8XXX_STORE_TIMING_CMD1            0x03  ///< The command 1 to enter store timing
    #define GP8XXX_STORE_TIMING_CMD2            0x00  ///< The command 2 to enter store timing
    #define GP8XXX_STORE_TIMING_DELAY           10    ///< Store procedure interval delay time: 10ms, more than 7ms
    #define I2C_CYCLE_TOTAL                     5     ///< Total I2C communication cycle
    #define I2C_CYCLE_BEFORE                    1     ///< The first half cycle 2 of the total I2C communication cycle
    #define I2C_CYCLE_AFTER                     2     ///< The second half cycle 3 of the total I2C communication cycle

    /**
     * @brief DFRobot_GP8XXX constructor
     * @param resolution resolution
     * @param deviceAddr I2C address
     * @param pWire I2C object
     */
    GP8I2C(uint16_t resolution, uint8_t deviceAddr = DFGP8XXX_I2C_DEVICEADDR, int sda_io_num=22, int scl_io_num=23,
			i2c_port_t i2c_master_num=static_cast<i2c_port_t>(0))
		: _resolution(resolution),
		  _deviceAddr(deviceAddr),
	      i2c_master_num(i2c_master_num) {

        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = static_cast<gpio_num_t>(sda_io_num);  // SDA pin
        conf.scl_io_num = static_cast<gpio_num_t>(scl_io_num);  // SCL pin
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = 100000;  // 100 kHz
        i2c_param_config(i2c_master_num, &conf);
        i2c_driver_install(i2c_master_num, conf.mode, 0, 0, 0);
    }

    /**
     * @fn setDACOutRange
     * @brief Set the DAC output range
     * @param range DAC output range
     * @n     eOutputRange0_5V(0-5V)
     * @n     eOutputRange0_10V(0-10V)
     * @return NONE
     */

	void setDACOutRange(eOutPutRange_t range) {
		uint8_t data = 0x00;
		uint8_t reg = GP8XXX_CONFIG_CURRENT_REG >> 1; // Adjust this as needed based on your hardware setup
		switch (range) {
			case eOutputRange5V:
				break;
			case eOutputRange10V:
				data = 0x11;
				break;
			default:
				return; // Exit the function if the range is not handled
		}

		uint8_t write_buf[2] = {reg, data};
		esp_err_t ret = i2c_master_write_to_device(i2c_master_num, _deviceAddr, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
		if (ret != ESP_OK) {
			// Handle error
			ESP_LOGE(TAG, "Failed to write DAC output range");
		}
	}

    /**
     * @fn setDACOutVoltage
     * @brief Set different channel output DAC values
     * @param data value corresponding to the voltage value
     * @param channel output channel
     * @n 0: Channel 0 (valid when PWM0 output is configured)
     * @n 1: Channel 1 (valid when PWM1 output is configured)
     * @n 2: All channels (valid when configuring dual channel output)
     * @return NONE
     */
	void setDACOutVoltage(uint16_t voltage, uint8_t channel) {
		if (voltage > _resolution) {
			voltage = _resolution;
		}

		if (_resolution == RESOLUTION_12_BIT) {
			voltage = voltage << 4; // Align voltage to the upper bits
		} else if (_resolution == RESOLUTION_15_BIT) {
			voltage = voltage << 1; // Align voltage to the upper bits
		}

		sendData(voltage, channel);
	}

    
    /**
     * @fn store
     * @brief Save the set voltage inside the chip
     * @return NONE
     */
    void store(void);
    

  protected:
	esp_err_t sendData(uint16_t data, uint8_t channel) {
		uint8_t buff[4] = {uint8_t(data & 0xFF), uint8_t(data >> 8), uint8_t(data & 0xFF), uint8_t(data >> 8)};
		uint8_t write_buf[5]; // Adjust size if needed
		uint8_t reg;
		size_t len = 2; // Default length

		if (channel == 0) {
			reg = GP8XXX_CONFIG_CURRENT_REG;
		} else if (channel == 1) {
			reg = GP8XXX_CONFIG_CURRENT_REG << 1;
		} else if (channel == 2) {
			reg = GP8XXX_CONFIG_CURRENT_REG;
			len = 4; // Length is 4 for channel 2
		}

		write_buf[0] = reg;
		memcpy(&write_buf[1], buff, len); // Copy data into write buffer starting after the register byte

		esp_err_t ret = i2c_master_write_to_device(i2c_master_num, _deviceAddr, write_buf, len + 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
		ESP_RETURN_ON_ERROR(ret, TAG, "Failed to write data to device");
	}

    uint8_t writeRegister(uint8_t reg, void* pBuf, size_t size);
  
  protected:
    uint16_t _resolution=0;
    uint8_t _deviceAddr;
	i2c_port_t i2c_master_num;
};

class GP8211S: public GP8I2C {
  public:
    GP8211S(uint16_t resolution = RESOLUTION_15_BIT) : GP8I2C(resolution) {};
	void setDACOutVoltage(uint16_t data, uint8_t channel);
};
