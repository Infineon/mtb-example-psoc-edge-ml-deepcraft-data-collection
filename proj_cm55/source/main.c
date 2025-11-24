/*******************************************************************************
* File Name        : main.c
*
* Description      : This source file contains the main routine for CM55 CPU
*
* Related Document: See README.md
*
*******************************************************************************
 * (c) 2025, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/*******************************************************************************
* Header Files
*******************************************************************************/

#include "cybsp.h"
#include <string.h>
#include <stdlib.h>
#include "protocol/protocol.h"
#include "usbd.h"
#include "common.h"
#include "build.h"
#include "board.h"
#include "system.h"
#include "clock.h"
#include "retarget_io_init.h"

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM55 application.It also initializes the 
* the device and board peripherals, retarget-io middleware, clock and streaming interface.
* 
* CM33 application enables the CM55 CPU and then the CM33 CPU enters 
* deep sleep.
* 
* Parameters:
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{

    /* Initialize the device and board peripherals */
    board_init_system();

    /* Initialize retarget-io middleware */
    init_retarget_io();

    /* Start clock */
    if(!clock_init())
    {
        halt_error(LED_CODE_CLOCK_ERROR);
    }

    /* Firmware version */
    protocol_Version firmware_version =
    {
            .major = 1,
            .minor = 2,
            .build = BUILD_DATE,
            .revision = BUILD_TIME
    };
    /* Serial UUID */
    uint8_t* serial = board_get_serial_uuid();

     /* Create a protocol instance */
    #ifdef USE_KIT_PSE84_AI
    protocol_t* protocol = protocol_create("PSOC Edge E84 AI Kit ", serial, firmware_version);
    #elif defined(USE_KIT_PSE84_EVAL_EPC2) || defined(USE_KIT_PSE84_EVAL_EPC4)
    protocol_t* protocol = protocol_create("PSOC Edge E84 Evaluation Kit ", serial, firmware_version);
    #endif  
    
    /* Add reset function */
    protocol->board_reset = board_reset;

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    /* Debug console print */
    printf("*********** Firmware Debug Console ***********\n\n");
    printf("Board Name             : %s\r\n", protocol->board.name);
    printf("Board Serial           : %02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X\n",
                serial[0], serial[1], serial[2], serial[3],
                serial[4], serial[5],
                serial[6], serial[7],
                serial[8], serial[9],
                serial[10], serial[11], serial[12], serial[13], serial[14], serial[15]);
    printf("Firmware Version       : %lu.%lu.%lu.%lu\n",
                (unsigned long int)protocol->board.firmware_version.major,
                (unsigned long int)protocol->board.firmware_version.minor,
                (unsigned long int)protocol->board.firmware_version.build,
                (unsigned long int)protocol->board.firmware_version.revision);
    printf("Protocol Version       : %lu.%lu.%lu.%lu\n\n",
                (unsigned long int)protocol->board.protocol_version.major,
                (unsigned long int)protocol->board.protocol_version.minor,
                (unsigned long int)protocol->board.protocol_version.build,
                (unsigned long int)protocol->board.protocol_version.revision);

    system_load_device_drivers(protocol);
    
    /* Initialize the streaming interface */
    usbd_t* usb = usbd_create(protocol);

    printf("Ready accepting commands.\r\n");

    set_led(false);

    for(;;)
    {
        protocol_process_request(protocol, &usb->istream, &usb->ostream);
    }
    
}

/* [] END OF FILE */
