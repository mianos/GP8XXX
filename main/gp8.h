#pragma once
#include <cstring>
#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

constexpr char TAG[] = "DFRobot_GP8XXX_IIC";
constexpr int I2C_MASTER_TIMEOUT_MS = 1000;

class GP8 {
  public:
    enum eOutPutRange_t {
      eOutputRange2_5V = 0,
      eOutputRange5V = 1,
      eOutputRange10V = 2,
      eOutputRangeVCC = 3
    };

    GP8() = default;
    virtual esp_err_t setVoltage(float voltage, uint8_t channel=0) = 0;   

  protected:
	eOutPutRange_t currentRange = eOutputRange5V;
};

class GP8I2C : public GP8 {
  protected:
    uint16_t _resolution=0;
	i2c_master_dev_handle_t dev_handle;

  public:
    static constexpr uint16_t RESOLUTION_12_BIT = 0x0FFF;
    static constexpr uint16_t RESOLUTION_15_BIT = 0x7FFF;
    static constexpr uint8_t GP8XXX_CONFIG_CURRENT_REG = 0x02;
    static constexpr uint8_t DFGP8XXX_I2C_DEVICEADDR = 0x58;

    GP8I2C(uint16_t resolution = RESOLUTION_15_BIT, uint8_t deviceAddr = DFGP8XXX_I2C_DEVICEADDR,
			gpio_num_t sda_io_num = static_cast<gpio_num_t>(22), gpio_num_t scl_io_num = static_cast<gpio_num_t>(23))
        : _resolution(resolution) {

		i2c_master_bus_config_t i2c_mst_config = {};
		i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
		i2c_mst_config.i2c_port = I2C_NUM_0;
		i2c_mst_config.scl_io_num = scl_io_num;
		i2c_mst_config.sda_io_num = sda_io_num;
		i2c_mst_config.flags.enable_internal_pullup = true;

		i2c_master_bus_handle_t bus_handle;
		ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

		i2c_device_config_t dev_cfg = {};
		dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
		dev_cfg.device_address = deviceAddr;
		dev_cfg.scl_speed_hz = 100000;

		ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    }

	esp_err_t writeRegister(uint8_t reg, uint8_t* pBuf, size_t size) {
		if (pBuf == NULL) {
			return ESP_ERR_INVALID_ARG;
		}

		constexpr size_t max_buffer_size = 10;
		if (size > max_buffer_size - 1) {
			return ESP_ERR_INVALID_SIZE;
		}
		uint8_t data[max_buffer_size];

		data[0] = reg;  // First byte is the register address
		memcpy(&data[1], pBuf, size);  // Copy the data to be written after the register address

		return i2c_master_transmit(dev_handle, data, size + 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
	}

	esp_err_t sendData(uint16_t data, uint8_t channel) {
		uint8_t buff[4] = {uint8_t(data & 0xFF), uint8_t(data >> 8), uint8_t(data & 0xFF), uint8_t(data >> 8)};
		uint8_t write_buf[5];
		uint8_t reg;
		size_t len = 2;

		if (channel == 0) {
			reg = GP8XXX_CONFIG_CURRENT_REG;
		} else if (channel == 1) {
			reg = GP8XXX_CONFIG_CURRENT_REG << 1;
		} else if (channel == 2) {
			reg = GP8XXX_CONFIG_CURRENT_REG;
			len = 4; // Length is 4 for channel 2
		} else {
			ESP_LOGE(TAG, "Unsupported channel %d", (unsigned)channel);
			return ESP_ERR_INVALID_ARG;
		}

		write_buf[0] = reg;
		memcpy(&write_buf[1], buff, len); // Copy data into write buffer starting after the register byte

		ESP_RETURN_ON_ERROR(i2c_master_transmit(dev_handle, write_buf, len + 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS), TAG, "Failed to write data to device");
		return ESP_OK;
	}

	void setDACOutRange(eOutPutRange_t range) {
	  uint8_t data = 0x00;
	  switch (range) {
		case eOutputRange5V:    
		  writeRegister(GP8XXX_CONFIG_CURRENT_REG >> 1, &data, 1);
		  currentRange = range;
		  ESP_LOGI(TAG, "Setting output range to 5V");
		  break;
		case eOutputRange10V:  
		  data = 0x11;
		  writeRegister(GP8XXX_CONFIG_CURRENT_REG >> 1, &data, 1);
		  currentRange = range;
		  ESP_LOGI(TAG, "Setting output range to 10V");
		  break;
		default:
		  break;
	  }
	}

#if 0

    static constexpr uint8_t GP8XXX_STORE_TIMING_HEAD = 0x02;
    static constexpr uint8_t GP8XXX_STORE_TIMING_ADDR = 0x10;
    static constexpr uint8_t GP8XXX_STORE_TIMING_CMD1 = 0x03;
    static constexpr uint8_t GP8XXX_STORE_TIMING_CMD2 = 0x00;
    static constexpr int GP8XXX_STORE_TIMING_DELAY = 10;

    void store() {
		ESP_LOGE(TAG, "store is not tested");
		return;
        // Ensure the I2C interface is properly initialized

        // Buffer to hold commands
        uint8_t commands[10];
        int index = 0;

        // Start the sequence to store settings
        commands[index++] = GP8XXX_STORE_TIMING_HEAD;
        commands[index++] = GP8XXX_STORE_TIMING_ADDR;
        commands[index++] = GP8XXX_STORE_TIMING_CMD1;

        // Send the first sequence
		i2c_master_transmit(dev_handle, commands, index, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        //i2c_master_write_to_device(i2c_master_num, _deviceAddr, commands, index, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        
        // Prepare the second sequence
        index = 0;
        commands[index++] = _deviceAddr << 1; // Assuming this needs to be shifted for addressing
        for (int i = 0; i < 8; ++i) {
            commands[index++] = GP8XXX_STORE_TIMING_CMD2;
        }

        // Send the second sequence
		i2c_master_transmit(dev_handle, commands, index, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        //i2c_master_write_to_device(i2c_master_num, _deviceAddr, commands, index, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

        // Delay for the specified interval
        vTaskDelay(pdMS_TO_TICKS(GP8XXX_STORE_TIMING_DELAY));

        // Final command sequence
        index = 0;
        commands[index++] = GP8XXX_STORE_TIMING_HEAD;
        commands[index++] = GP8XXX_STORE_TIMING_ADDR;
        commands[index++] = GP8XXX_STORE_TIMING_CMD2;

        // Execute the final sequence
		i2c_master_transmit(dev_handle, commands, index, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
        //i2c_master_write_to_device(i2c_master_num, _deviceAddr, commands, index, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    }
#endif
};

class GP8211S: public GP8I2C {
  public:
    GP8211S(uint16_t resolution = RESOLUTION_15_BIT) : GP8I2C(resolution) {}
	esp_err_t setVoltage(float voltage, uint8_t channel=0) {
		ESP_RETURN_ON_FALSE(channel == 0, ESP_ERR_INVALID_ARG, TAG, "Invalid channel: %d", channel);
		//  VOUT=5V* DATA/0x7FFF VOUT=10V* DATA/0x7FFF
		uint16_t oval;
		switch (_resolution) {
		case RESOLUTION_15_BIT:
			switch (currentRange) {
			case eOutputRange5V:
				oval = voltage * (double)0x7FFF / 5.0;
				break;
			case eOutputRange10V:
				oval = voltage * (double)0x7FFF / 10.0;
				break;
			default:
				ESP_LOGE(TAG, "usupported range");
				return ESP_ERR_INVALID_ARG;
			}
			sendData(oval << 1l, 0);
			ESP_LOGI(TAG, "output value %u", oval);
			break;
		default:
			// remember
			// voltage = voltage << 4; // Align voltage to the upper bits
			ESP_LOGE(TAG, "usupported resolution");
			return ESP_ERR_INVALID_ARG;
			break;
		}
		return ESP_OK;
	}
};
