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
* (c) 2025-2025, Infineon Technologies AG, or an affiliate of Infineon Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is owned by
* Infineon Technologies AG or one of its affiliates ("Infineon") and is protected
* by and subject to worldwide patent protection, worldwide copyright laws, and
* international treaty provisions. Therefore, you may use this Software only as
* provided in the license agreement accompanying the software package from which
* you obtained this Software. If no license agreement applies, then any use,
* reproduction, modification, translation, or compilation of this Software is
* prohibited without the express written permission of Infineon.
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING,
* BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF THIRD-PARTY RIGHTS AND
* IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A SPECIFIC USE/PURPOSE OR
* MERCHANTABILITY. Infineon reserves the right to make changes to the Software
* without notice. You are responsible for properly designing, programming, and
* testing the functionality and safety of your intended application of the
* Software, as well as complying with any legal requirements related to its
* use. Infineon does not guarantee that the Software will be free from intrusion,
* data theft or loss, or other breaches ("Security Breaches"), and Infineon
* shall have no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any application
* where a failure of the Product or any consequences of the use thereof can
* reasonably be expected to result in personal injury.
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
