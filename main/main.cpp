#include "esp_log.h"
#include <string>

#include "gp8.h"

extern "C" void app_main() {
    GP8I2C dac;
    bool increasing = true; // Direction flag
    uint16_t voltage = 0; // Start from the lowest value

	dac.setDACOutRange(GP8::eOutputRange10V);
    while (true) {
#if 1
        if (increasing) {
            if (voltage < 32767) { // Check if the voltage is less than the max value for 15-bit
                voltage += 100;
            } else {
                increasing = false; // Start decreasing
            }
        } else {
            if (voltage > 0) { // Ensure voltage does not underflow
                voltage -= 100;
            } else {
                increasing = true; // Start increasing
            }
        }
#else
		voltage = 15000;
#endif
        dac.setDACOutVoltage(voltage, 0); // Update the DAC output
//		ESP_LOGI(TAG, "voltage %d", voltage);
//        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
