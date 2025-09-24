/*******************************************************************************
* File Name        : main.c
*
* Description      : This source file contains the main routine for CM55 CPU
*
* Related Document: See README.md
*
*******************************************************************************
* Copyright 2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
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
    #ifdef KIT_PSE84_AI
    protocol_t* protocol = protocol_create("PSOC Edge E84 AI Kit ", serial, firmware_version);
    #else
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


    /* Initialize the streaming interface */
    usbd_t* usb = usbd_create(protocol);

    system_load_device_drivers(protocol);

    printf("Ready accepting commands.\r\n");

    set_led(false);

    for(;;)
    {
        protocol_process_request(protocol, &usb->istream, &usb->ostream);
    }
    
}

/* [] END OF FILE */
