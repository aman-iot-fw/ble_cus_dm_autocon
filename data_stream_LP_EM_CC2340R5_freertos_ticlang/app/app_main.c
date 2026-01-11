/******************************************************************************

@file  app_main.c

@brief This file contains the application main functionality
Group: WCS, BTS
Target Device: cc23xx

******************************************************************************

 Copyright (c) 2022-2024, Texas Instruments Incorporated
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:

 *  Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.

 *  Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

 *  Neither the name of Texas Instruments Incorporated nor the names of
    its contributors may be used to endorse or promote products derived
    from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

******************************************************************************


*****************************************************************************/

//*****************************************************************************
//! Includes
//*****************************************************************************
#include "ti_ble_config.h"
#include <ti/bleapp/ble_app_util/inc/bleapputil_api.h>
#include <ti/bleapp/menu_module/menu_module.h>
#include <health_toolkit/inc/debugInfo.h>
#include <app_main.h>
#include <ti/bleapp/profiles/data_stream/data_stream_profile.h>


#include "sys/types.h"


extern configurableData_t swapedConfigData;
uint8_t connectionFlag =0;


//*****************************************************************************
//! Defines
//*****************************************************************************

//*****************************************************************************
//! Globals
//*****************************************************************************

// Parameters that should be given as input to the BLEAppUtil_init function
BLEAppUtil_GeneralParams_t appMainParams =
{
    .taskPriority = 1,
    .taskStackSize = 1024,
    .profileRole = (BLEAppUtil_Profile_Roles_e)(HOST_CONFIG),
    .addressMode = DEFAULT_ADDRESS_MODE,
    .deviceNameAtt = attDeviceName,
//    .pDeviceRandomAddress = pRandomAddress,
};

BLEAppUtil_PeriCentParams_t appMainPeriCentParams =
{
#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
 .connParamUpdateDecision = DEFAULT_PARAM_UPDATE_REQ_DECISION,
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )
 .gapBondParams = &gapBondParams
#endif
};

//*****************************************************************************
//! Functions
//*****************************************************************************

/*********************************************************************
 * @fn      criticalErrorHandler
 *
 * @brief   Application task entry point
 *
 * @return  none
 */
void criticalErrorHandler(int32 errorCode , void* pInfo)
{
//    trace();
//
//#ifdef DEBUG_ERR_HANDLE
//
//    while (1);
//#else
//    SystemReset();
//#endif

}

/*********************************************************************
 * @fn      App_StackInitDone
 *
 * @brief   This function will be called when the BLE stack init is
 *          done.
 *          It should call the applications modules start functions.
 *
 * @return  none
 */

void App_StackInitDoneHandler(gapDeviceInitDoneEvent_t *deviceInitDoneData)
{
    bStatus_t status = SUCCESS;

    // Print the device ID address
//    MenuModule_printf(APP_MENU_DEVICE_ADDRESS, 0, "BLE ID Address: "
//                      MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_GREEN "%s" MENU_MODULE_COLOR_RESET,
//                      BLEAppUtil_convertBdAddr2Str(deviceInitDoneData->devAddr));
//
//    if ( appMainParams.addressMode > ADDRMODE_RANDOM)
//    {
//      // Print the RP address
//        MenuModule_printf(APP_MENU_DEVICE_RP_ADDRESS, 0,
//                     "BLE RP Address: "
//                     MENU_MODULE_COLOR_BOLD MENU_MODULE_COLOR_GREEN "%s" MENU_MODULE_COLOR_RESET,
//                     BLEAppUtil_convertBdAddr2Str(GAP_GetDevAddress(FALSE)));
//    }

//#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )
//    status = DataStream_start();
//    if ( status != SUCCESS )
//    {
//    // TODO: Call Error Handler
//    }
//#endif

#ifdef BLE_HEALTH
    status = DbgInf_init( DBGINF_DOMAIN_ALL );
    if ( status != SUCCESS )
    {
    // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG ) )
    // Any device that accepts the establishment of a link using
    // any of the connection establishment procedures referred to
    // as being in the Peripheral role.
    // A device operating in the Peripheral role will be in the
    // Peripheral role in the Link Layer Connection state.
    status = Peripheral_start();
    if(status != SUCCESS)
    {
    // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( BROADCASTER_CFG ) )
    // A device operating in the Broadcaster role is a device that
    // sends advertising events or periodic advertising events
    status = Broadcaster_start();
    if(status != SUCCESS)
    {
    // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( CENTRAL_CFG ) )
    // A device that supports the Central role initiates the establishment
    // of an active physical link. A device operating in the Central role will
    // be in the Central role in the Link Layer Connection state.
    // A device operating in the Central role is referred to as a Central.
    status = Central_start();
    if(status != SUCCESS)
    {
    // TODO: Call Error Handler
    }
#endif

#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( OBSERVER_CFG ) )
    // A device operating in the Observer role is a device that
    // receives advertising events or periodic advertising events
    status = Observer_start();
    if(status != SUCCESS)
    {
    // TODO: Call Error Handler
    }
#endif
//#if defined( HOST_CONFIG ) && ( HOST_CONFIG & ( PERIPHERAL_CFG | CENTRAL_CFG ) )
    status = Connection_start();
    if ( status != SUCCESS )
    {
    // TODO: Call Error Handler
    }
    status = Pairing_start();
    if ( status != SUCCESS )
    {
    // TODO: Call Error Handler
    }
    status = Data_start();
    if ( status != SUCCESS )
    {
    // TODO: Call Error Handler
    }
    status = DevInfo_start();
    if ( status != SUCCESS )
    {
    // TODO: Call Error Handler
    }
    status = DataStream_start();
    if(status != SUCCESS)
    {
        // TODO: Call Error Handler
    }
//#endif
}
//
//int ConnTick = 0;
//uint32_t thresHold=0;
//
//void timerCallback(LGPTimerLPF3_Handle lgptHandle, LGPTimerLPF3_IntMask interruptMask) {
//    ConnTick++;
//    thresHold=swapedConfigData.timeOut;
//    if (thresHold < 0 || thresHold > 300000) {
//        thresHold = B_APP_DEFAULT_CONN_TIMEOUT;
//    }
//
//    if(connectionFlag && !GPIO_read(CONFIG_GPIO_INPUT_PIN)){
//        if(ConnTick>=thresHold)
//        {
////            DSP_sendData((uint8_t *)"TIME_OUT", 7);
//            DS_TerminateCB(); //#
//            LGPTimerLPF3_stop(hTimer);
//            //Power_reset();
//        }
//    }
//    else{
//        ConnTick=thresHold;
//    }
//}

/*********************************************************************
 * @fn      appMain
 *
 * @brief   Application main function
 *
 * @return  none
 */
void appMain(void)
{
    Config_init();

    /* Call driver init functions */
    GPIO_init();

//    uint32_t counterTarget;
//     LGPTimerLPF3_Params params;
//     LGPTimerLPF3_Params_init(&params);
//     params.hwiCallbackFxn = timerCallback;
//     hTimer = LGPTimerLPF3_open(CONFIG_LGPTIMER_3, &params);
//     if(hTimer == NULL) {
//       //Log_error0("Failed to open LGPTimer");
//       //Task_exit();
//     }
//     counterTarget = 48000-1;  // 1 ms with a system clock of 48 MHz
//     LGPTimerLPF3_setInitialCounterTarget(hTimer, counterTarget, true);
//     LGPTimerLPF3_enableInterrupt(hTimer, LGPTimerLPF3_INT_TGT);
//     LGPTimerLPF3_start(hTimer, LGPTimerLPF3_CTL_MODE_UP_PER);



    // Call the BLEAppUtil module init function
    BLEAppUtil_init(&criticalErrorHandler, &App_StackInitDoneHandler,
                    &appMainParams, &appMainPeriCentParams);
}
