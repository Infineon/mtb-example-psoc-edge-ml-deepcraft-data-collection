/******************************************************************************
* File Name:   board.c
*
* Description: This file provides basic board functionalities like
*         - reset
*         - serial
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

#include "retarget_io_init.h"
#include <string.h>
#include <stdlib.h>
#include <cybsp.h>

#include "protocol/protocol.h"
#include "board.h"

/*******************************************************************************
* Function Name: init_system
********************************************************************************
* Summary:
*   Initializes the system including device and board peripherals.
*******************************************************************************/
void board_init_system(void)
{
    cy_rslt_t result = CY_RSLT_SUCCESS;
    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board initialization failed. Stop program execution */
    if (CY_RSLT_SUCCESS != result)
    {
        handle_app_error();
    }

    /* Enable global interrupts */
    __enable_irq();
 }

/*******************************************************************************
* Function Name: board_get_serial_uuid
********************************************************************************
* Summary:
*   Retrieves the unique serial UUID of the board.
*
* Return:
*   Pointer to the serial UUID array.
*******************************************************************************/
uint8_t* board_get_serial_uuid(void)
{
    /* Create serial UUID {290DE5CB-460B-41BF-XXXX-XXXXXXXXXXXX}.
     Last part is silicon unique ID.*/
    uint64_t serial64 = Cy_SysLib_GetUniqueId();
    static uint8_t serial[16] = {
            0x29, 0x0d, 0xe5, 0xcb, 0x46, 0x0b, 0x41, 0xbf,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    memcpy(serial + 8, &serial64, 8);

    return serial;
}

/*******************************************************************************
* Function Name: board_reset
********************************************************************************
* Summary:
*   Resets the board.
*
* Parameters:
*   protocol: Pointer to the protocol object.
*******************************************************************************/
void board_reset(protocol_t* protocol)
{
    UNUSED(protocol);

    NVIC_SystemReset();
}

/* [] END OF FILE */
