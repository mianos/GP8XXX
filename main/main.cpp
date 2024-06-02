#include "esp_log.h"
#include <string>

#include "gp8.h"

extern "C" void app_main() {
    GP8211S dac;
	dac.setDACOutRange(GP8::eOutputRange5V);
	float voltage = 2.0;
    while (true) {  // Infinite loop to cycle voltages
        ESP_LOGI(TAG, "Setting voltage to %g volts", voltage);
		dac.setVoltage(voltage);
        vTaskDelay(pdMS_TO_TICKS(10000));
#if 1
        voltage += 1.0;  // Increment voltage by 1V
        if (voltage > 5.0) {  // Reset voltage to 0 after reaching 10V
            voltage = 0.0;
        }
#endif
    }
}
