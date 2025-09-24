/******************************************************************************
* File Name:   dev_pdm_pcm.h
*
* Description: This file implements the PDM/PCM peripheral for PSOC Edge using 
* Peripheral Driver Layer (PDL) APIs.
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
#ifndef _PDM_PCM_H_
#define _PDM_PCM_H_

#include "cy_result.h"
#include "cycfg_peripherals.h"

/******************************************************************************
 * Macros
*****************************************************************************/
#define INCLUDE_STRINGS /* Use this when config strings are needed. */

#define MAX_CHANNEL_COUNT_PER_HANDLE    (2u)
#define MAX_HANDLE_COUNT                (2u)

/******************************************************************************
 * Type Definitions
*****************************************************************************/
typedef enum
{
    PDM_PCM_STATUS_SUCCESS = 0u,
    PDM_PCM_STATUS_BAD_PARAM,
    PDM_PCM_STATUS_FAIL,
}PDM_PCM_STATUS_t;

typedef enum
{
    MODE_MONO = 0u,
    MODE_STEREO,
}MODE_t;

typedef enum
{
    SAMPLE_RATE_8000 = 0u,
    SAMPLE_RATE_16000,
    SAMPLE_RATE_22050,
    SAMPLE_RATE_44100,
    SAMPLE_RATE_48000,
}SAMPLE_RATE;


typedef struct
{
    MODE_t mode;
    uint8_t channel_index_list[MAX_CHANNEL_COUNT_PER_HANDLE];
    cy_stc_pdm_pcm_channel_config_t channel_config[MAX_CHANNEL_COUNT_PER_HANDLE];
    cy_stc_sysint_t pdm_irq_cfg;
}PDM_PCM_CONFIG_t;

typedef struct pdm_pcm_t * pdm_pcm;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
PDM_PCM_STATUS_t pdm_pcm_set_gain (pdm_pcm handle, uint8_t channel_num, cy_en_pdm_pcm_gain_sel_t gain);
PDM_PCM_STATUS_t pdm_pcm_init_hw(SAMPLE_RATE sample_rate);
pdm_pcm pdm_pcm_create(PDM_PCM_CONFIG_t * config);
PDM_PCM_STATUS_t pdm_pcm_update_config(pdm_pcm handle, PDM_PCM_CONFIG_t * config);
PDM_PCM_STATUS_t pdm_pcm_start(pdm_pcm handle);
PDM_PCM_STATUS_t pdm_pcm_stop(pdm_pcm handle);

bool pdm_pcm_data_ready(pdm_pcm handle);
void pdm_pcm_discard_samples(pdm_pcm handle);
int16_t* pdm_pcm_get_full_buffer(pdm_pcm handle);

#ifdef INCLUDE_STRINGS
const char ** pdm_pcm_get_string_list_of_sample_rates(void);
uint8_t pdm_pcm_get_sample_rate_option_count(void);
const char ** pdm_pcm_get_string_list_of_gain_options(void);
uint8_t pdm_pcm_get_gain_option_count(void);
#endif

int pdm_pcm_get_frequency_from_frequency_index(SAMPLE_RATE frequency_index);
void pdm_pcm_clear_data_ready_flag(pdm_pcm handle);
uint32_t pdm_pcm_get_frame_count(pdm_pcm handle);


#endif /* _DEV_PDM_PCM_H_ */

/* [] END OF FILE */