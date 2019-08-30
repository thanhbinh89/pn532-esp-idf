/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <esp_log.h>
#include <esp_log_internal.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "sdkconfig.h"

#include "pn532.h"

#define BLINK_GPIO CONFIG_BLINK_GPIO

#define PN532_SCK (32)
#define PN532_MOSI (26)
#define PN532_SS (25)
#define PN532_MISO (33)

static const char *TAG = "APP";

static pn532_t nfc;

void blink_task(void *pvParameter)
{
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    while (1)
    {
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(900 / portTICK_PERIOD_MS);
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void nfc_task(void *pvParameter)
{
    pn532_init_io(&nfc, PN532_SCK, PN532_MISO, PN532_MOSI, PN532_SS);
    pn532_begin(&nfc);

    uint32_t versiondata = pn532_getFirmwareVersion(&nfc);
    if (!versiondata)
    {
        ESP_LOGI(TAG, "Didn't find PN53x board");
        while (1)
        {
            vTaskDelay(1000 / portTICK_RATE_MS);
        }
    }
    // Got ok data, print it out!
    ESP_LOGI(TAG, "Found chip PN5 %x", (versiondata >> 24) & 0xFF);
    ESP_LOGI(TAG, "Firmware ver. %d.%d", (versiondata >> 16) & 0xFF, (versiondata >> 8) & 0xFF);

    // configure board to read RFID tags
    pn532_SAMConfig(&nfc);

    ESP_LOGI(TAG, "Waiting for an ISO14443A Card ...");

    while (1)
    {
        uint8_t success;
        uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0}; // Buffer to store the returned UID
        uint8_t uidLength;                     // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

        // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
        // 'uid' will be populated with the UID, and uidLength will indicate
        // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
        success = pn532_readPassiveTargetID(&nfc, PN532_MIFARE_ISO14443A, uid, &uidLength, 0);

        if (success)
        {
            // Display some basic information about the card
            ESP_LOGI(TAG, "Found an ISO14443A card");
            ESP_LOGI(TAG, "UID Length: %d bytes", uidLength);
            ESP_LOGI(TAG, "UID Value");
            esp_log_buffer_hexdump_internal(TAG, uid, uidLength, ESP_LOG_INFO);

            if (uidLength == 4)
            {
                // We probably have a Mifare Classic card ...
                ESP_LOGI(TAG, "Seems to be a Mifare Classic card (4 byte UID)");

                // Now we need to try to authenticate it for read/write access
                // Try with the factory default KeyA: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF
                ESP_LOGI(TAG, "Trying to authenticate block 4 with default KEYA value");
                uint8_t keya[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

                // Start with block 4 (the first block of sector 1) since sector 0
                // contains the manufacturer data and it's probably better just
                // to leave it alone unless you know what you're doing
                success = pn532_mifareclassic_AuthenticateBlock(&nfc, uid, uidLength, 4, 0, keya);

                if (success)
                {
                    ESP_LOGI(TAG, "Sector 1 (Blocks 4..7) has been authenticated");
                    uint8_t data[16];

                    // If you want to write something to block 4 to test with, uncomment
                    // the following line and this text should be read back in a minute
                    //memcpy(data, (const uint8_t[]){ 'a', 'd', 'a', 'f', 'r', 'u', 'i', 't', '.', 'c', 'o', 'm', 0, 0, 0, 0 }, sizeof data);
                    // success = mifareclassic_WriteDataBlock (4, data);

                    // Try to read the contents of block 4
                    success = pn532_mifareclassic_ReadDataBlock(&nfc, 4, data);

                    if (success)
                    {
                        // Data seems to have been read ... spit it out
                        ESP_LOGI(TAG, "Reading Block 4");
                        esp_log_buffer_hexdump_internal(TAG, data, 16, ESP_LOG_INFO);

                        // Wait a bit before reading the card again
                        vTaskDelay(1000 / portTICK_RATE_MS);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Ooops ... unable to read the requested block.  Try another key?");
                    }
                }
                else
                {
                    ESP_LOGI(TAG, "Ooops ... authentication failed: Try another key?");
                }
            }

            if (uidLength == 7)
            {
                // We probably have a Mifare Ultralight card ...
                ESP_LOGI(TAG, "Seems to be a Mifare Ultralight tag (7 byte UID)");

                // Try to read the first general-purpose user page (#4)
                ESP_LOGI(TAG, "Reading page 4");
                uint8_t data[32];
                success = pn532_mifareultralight_ReadPage(&nfc, 4, data);
                if (success)
                {
                    // Data seems to have been read ... spit it out
                    esp_log_buffer_hexdump_internal(TAG, data, 4, ESP_LOG_INFO);

                    // Wait a bit before reading the card again
                    vTaskDelay(1000 / portTICK_RATE_MS);
                }
                else
                {
                    ESP_LOGI(TAG, "Ooops ... unable to read the requested page!?");
                }
            }
        }
    }
}

void app_main()
{
    xTaskCreate(&blink_task, "blink_task", configMINIMAL_STACK_SIZE, NULL, 5, NULL);
    xTaskCreate(&nfc_task, "nfc_task", 4096, NULL, 4, NULL);
}
