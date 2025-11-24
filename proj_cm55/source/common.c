/******************************************************************************
* File Name: common.c
*
* Description:
*   This file contains common utility functions used across the project.
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

#include <cy_result.h>
#include <cy_utils.h>
#include <cybsp.h>

#include "common.h"

/*******************************************************************************
* Function Name: set_led
********************************************************************************
* Summary:
*   Sets the status LED.
*
* Parameters:
*   state: The desired state of the LED. 'true' for ON, 'false' for OFF.
*
*******************************************************************************/
void set_led(bool state)
{
    Cy_GPIO_Write(CYBSP_USER_LED_PORT, CYBSP_USER_LED_PIN, state ? CYBSP_LED_STATE_OFF : CYBSP_LED_STATE_ON);
}

/*******************************************************************************
* Function Name: halt_error
********************************************************************************
* Summary:
*  Flash the given error code indefinitely. This function never returns.
*
*  Blink pattern:
*  0:  .        10: ..-      20: .--.
*  1:  -        11: -.-      21: ---.
*  2:  ..       12: .--      22: ...-
*  3:  -.       13: ---      23: -..-
*  4:  .-       14: ....     24: .-.-
*  5:  --       15: -...     25: --.-
*  6:  ...      16: .-..     26: ..--
*  7:  -..      17: --..     27: -.--
*  8:  .-.      18: ..-.     28: .---
*  9:  --.      19: -.-.     29: ----
*******************************************************************************/
void halt_error(int code)
{
    __disable_irq();

    if(code < 0)
        code = -code;

    do {
        int i = code + 2;
        do {
            set_led(true);
            Cy_SysLib_Delay((i & 1) ? LED_LONG_ON_TIME : LED_SHORT_ON_TIME);
            set_led(false);
            Cy_SysLib_Delay(LED_OFF_TIME);
            i >>= 1;
        } while(i > 1);

        Cy_SysLib_Delay(LED_CODE_SEPARATOR_TIME);
    } while(true);
}

/* [] END OF FILE */
