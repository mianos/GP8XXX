#include "esp_log.h"
#include <string>

#include "gp8.h"

extern "C" void app_main() {
    GP8I2C dac;
	dac.setDACOutRange(GP8::eOutputRange10V);
	float voltage = 0.0;
    while (true) {  // Infinite loop to cycle voltages
        dac.setVoltage(voltage);
        ESP_LOGI(TAG, "Setting voltage to %g volts", voltage);
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5-second delay between updates

        voltage += 1.0;  // Increment voltage by 1V
        if (voltage > 10.0) {  // Reset voltage to 0 after reaching 10V
            voltage = 0.0;
        }
    }
}
