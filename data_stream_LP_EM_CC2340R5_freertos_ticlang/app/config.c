/*
 * config.c
 *
 *  Created on: 11-July-2024
 *      Author: aman
 */

#include "config.h"
#include <ti/drivers/Power.h>


static NVS_Handle nvsHandle;
static NVS_Attrs regionAttrs;
static NVS_Params nvsParams;


uint8_t nvsWriteFlag = 0;
extern UART2_Handle uart;

configurableData_t nvsConfigData;

//It will store the config data with swaped endianness big-endian to little endian
configurableData_t swapedConfigData;


// Function to swap endianness of a 32-bit integer
uint32_t swapBytes(uint32_t value) {
    return ((value >> 24) & 0x000000FF) |
           ((value >> 8) & 0x0000FF00)  |
           ((value << 8) & 0x00FF0000)  |
           ((value << 24) & 0xFF000000);
}


// Function to parse and handle configuration data from UART
void parseConfigData(uint8_t *Data) {
        // Determine the command type and parse accordingly
    uint8_t commandType = *(Data + 3); // Get the command type
    uint8_t dataLength = *(Data + 1);

    nvsHandle = NVS_open(CONFIG_NVS_1, &nvsParams);
    if (nvsHandle == NULL) {
        while (1); // Handle NVS open failure
    }

    switch (commandType) {
        case 0x11: // Set BLE name
        {
            memset(nvsConfigData.deviceName, 0, sizeof(nvsConfigData.deviceName));
            memcpy(nvsConfigData.deviceName, (Data + 4), sizeof(nvsConfigData.deviceName));
            if (strlen(nvsConfigData.deviceName) == dataLength) {
                UART2_write(uart, "Ack", 3, NULL);
                NVS_write(nvsHandle, MEM_BYTES, (void*)&nvsConfigData, sizeof(nvsConfigData), NVS_WRITE_ERASE | NVS_WRITE_POST_VERIFY);
            } else {
                UART2_write(uart, "Nack", 4, NULL);
            }
            break;
        }
        case 0x21: // Set timeout
        {
            memcpy(&nvsConfigData.timeOut, (Data + 4), 4);
            if(dataLength == 4){
                UART2_write(uart, "Ack", 3, NULL);
                NVS_write(nvsHandle, MEM_BYTES, (void*)&nvsConfigData, sizeof(nvsConfigData), NVS_WRITE_ERASE | NVS_WRITE_POST_VERIFY);
            } else {
                UART2_write(uart, "Nack", 4, NULL);
            }
            break;
        }
        case 0x31: // Set passkey
        {
            memcpy(&nvsConfigData.passKey, (Data + 4), 4);
            if(dataLength == 4){
                UART2_write(uart, "Ack", 3, NULL);
                NVS_write(nvsHandle, MEM_BYTES, (void*)&nvsConfigData, sizeof(nvsConfigData), NVS_WRITE_ERASE | NVS_WRITE_POST_VERIFY);
            } else {
                UART2_write(uart, "Nack", 4, NULL);
            }
            break;
        }
        default:
        {
            NVS_close(nvsHandle);
            return;
        }
    }
    NVS_close(nvsHandle);
    Power_reset();
}


//Initialize and read NVS
void Config_init(void)
{
    NVS_init();
    NVS_Params_init(&nvsParams);
    nvsHandle = NVS_open(CONFIG_NVS_1, &nvsParams);
    if (nvsHandle == NULL)
    {
        while(1);
    }

    NVS_getAttrs(nvsHandle, &regionAttrs);
    //NVS_erase(nvsHandle, 0, regionAttrs.sectorSize);
    //NVS_write(nvsHandle, 0, (configurableData_t *)&nvsConfigData, sizeof(nvsConfigData), NVS_WRITE_ERASE | NVS_WRITE_POST_VERIFY);
    NVS_read(nvsHandle, MEM_BYTES,(configurableData_t *)&nvsConfigData, sizeof(nvsConfigData));
    NVS_close(nvsHandle);

    swapedConfigData.passKey=swapBytes(nvsConfigData.passKey);
    swapedConfigData.timeOut=swapBytes(nvsConfigData.timeOut);
//    swapedConfigData.deviceName=nvsConfigData.deviceName;
}

