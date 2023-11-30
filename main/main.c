#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "nofrendo.h"
#include "esp_partition.h"

char *osd_getromdata() {
	char* romdata;
	const esp_partition_t* part;
	spi_flash_mmap_handle_t hrom;
	esp_err_t err;
	nvs_flash_init();
	part = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_NVS, NULL);
	if (part == NULL) {
		printf("Couldn't find rom part!\n");
	}
	err = esp_partition_mmap(part, 0, 3 * 1024 * 1024, SPI_FLASH_MMAP_DATA, (const void**)&romdata, &hrom);
	if (err != ESP_OK) {
		printf("Couldn't map rom part!\n");
	}
	printf("Initialized. ROM@%p\n", romdata);
	return (char*)romdata;
}

esp_err_t event_handler(void *ctx, system_event_t *event) {
	return ESP_OK;
}

void app_main(void) {
	printf("NoFrendo start!\n");
	nofrendo_main(0, NULL);
	printf("NoFrendo died? WtF?\n");
	asm("break.n 1");
}

