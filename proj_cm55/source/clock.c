/******************************************************************************
* File Name:   clock.c
*
* Description: This file provides a clock.
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

#include "cybsp.h"
#include "clock.h"

/*******************************************************************************
* Static Variables
*******************************************************************************/
static uint32_t uptime_seconds = 0;

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*******************************************************************************
* Function Name: clock_interrupt_handler
********************************************************************************
* Summary:
*   Keeps track of total elapsed seconds.
*
* Parameters:
*   None
*******************************************************************************/
void clock_interrupt_handler(void)
{
    uptime_seconds++;
    uint32_t interrupts = Cy_TCPWM_GetInterruptStatusMasked(CYBSP_GENERAL_PURPOSE_TIMER_HW,
                                                            CYBSP_GENERAL_PURPOSE_TIMER_NUM);
    /*Clear the interrupt*/ 
    Cy_TCPWM_ClearInterrupt(CYBSP_GENERAL_PURPOSE_TIMER_HW, CYBSP_GENERAL_PURPOSE_TIMER_NUM, interrupts);
}

/*******************************************************************************
* Function Name: clock_get_tick
********************************************************************************
* Summary:
*   Returns the number of clock ticks elapsed since initialization.
*
* Return:
*   The number of clock ticks elapsed.
*
*******************************************************************************/
clock_tick_t clock_get_tick(void)
{
    /*Ensure it can be reread in case of a corner case.*/ 
    volatile uint32_t* p_sec = (uint32_t*)&uptime_seconds;

    uint32_t seconds_0 = *p_sec;
    uint32_t time = Cy_TCPWM_Counter_GetCounter(CYBSP_GENERAL_PURPOSE_TIMER_HW, CYBSP_GENERAL_PURPOSE_TIMER_NUM);
    uint32_t seconds_1 = *p_sec;

    if (seconds_0 == seconds_1) {
        return ((clock_tick_t)CLOCK_TICK_PER_SECOND * seconds_1) + (clock_tick_t)time;
    }

    /*If the latter differ from the first, then assume time is zero.*/ 
    return ((clock_tick_t)CLOCK_TICK_PER_SECOND * seconds_1);
}

/*******************************************************************************
* Function Name: clock_init
********************************************************************************
* Summary:
*   Initializes the clock.
*
* Return:
*   True if initialization is successful, otherwise false.
*
*******************************************************************************/
bool clock_init(void)
{

    cy_en_tcpwm_status_t status;
    Cy_TCPWM_GetInterruptStatusMasked(CYBSP_GENERAL_PURPOSE_TIMER_HW,
                                                            CYBSP_GENERAL_PURPOSE_TIMER_NUM);
    /*Initialize and enable TCPWM counter*/ 
    status = Cy_TCPWM_Counter_Init(CYBSP_GENERAL_PURPOSE_TIMER_HW,CYBSP_GENERAL_PURPOSE_TIMER_NUM, &CYBSP_GENERAL_PURPOSE_TIMER_config);
    if (status != CY_TCPWM_SUCCESS)
    {
        return false;
    }

    /*Assign the interrupt handler*/ 
    cy_stc_sysint_t irq_cfg = {
        .intrSrc = CYBSP_GENERAL_PURPOSE_TIMER_IRQ,
        .intrPriority = CLOCK_INTERRUPT_PRIORITY,
    };

    if (Cy_SysInt_Init(&irq_cfg, clock_interrupt_handler) != CY_SYSINT_SUCCESS)
    {
        return false;
    }

    NVIC_EnableIRQ(CYBSP_GENERAL_PURPOSE_TIMER_IRQ);

    /* Enable the initialized counter */
    Cy_TCPWM_Counter_Enable(CYBSP_GENERAL_PURPOSE_TIMER_HW,
            CYBSP_GENERAL_PURPOSE_TIMER_NUM);

    /* Start the counter */
    Cy_TCPWM_TriggerStart_Single(CYBSP_GENERAL_PURPOSE_TIMER_HW,
            CYBSP_GENERAL_PURPOSE_TIMER_NUM);
            
    return true;
}
/* [] END OF FILE */
