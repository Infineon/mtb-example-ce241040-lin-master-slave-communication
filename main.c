/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code of LIN for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
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

/******************************************************************************
* Header Files
*******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "lin_master_slave.h"

/*******************************************************************************
* Global Variables
********************************************************************************/
uint8_t src_data[LIN_DATA_LENGTH_MAX];
uint8_t master_dst_data[LIN_DATA_LENGTH_MAX];
uint8_t slave_dst_data[LIN_DATA_LENGTH_MAX];

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*  1. Initialize bsp.
*  2. Configure source data.
*  3. Initialize LIN.
*  4. Send commands below each 500ms (LIN master only).
*     STEP1: Master transmits header.
*     STEP2: Master transmits header and response.
*     STEP3: Master transmits header and slave transmits response.
*
*
* Parameters:
*  none
*
* Return:
*  none
*
********************************************************************************/
int main(void)
{
    cy_rslt_t result;
    __enable_irq();

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Prepare source data to transmit */
    for (uint8_t i=0; i<LIN_DATA_LENGTH_MAX; i++)
    {
        src_data[i] = i+1UL;
    }

    /* Test mode configuration only for loop-back mode */
#if (LIN_MODE == LOOP_BACK_MODE)
    const cy_stc_lin_test_config_t lin_test_cfg =
    {
        .chidx = 0UL,
        .mode = TEST_MODE
    };

    Cy_LIN_TestMode_Enable(LIN0, &lin_test_cfg);
#endif

    /* Initialize interrupt handler */
#if (LIN_MODE == MASTER_MODE) | (LIN_MODE == LOOP_BACK_MODE)
    lin_master_init_isr();
#endif
#if (LIN_MODE == SLAVE_MODE) | (LIN_MODE == LOOP_BACK_MODE)
    lin_slave_init_isr();
#endif

    for(;;)
    {
#if (LIN_MODE == MASTER_MODE) | (LIN_MODE == LOOP_BACK_MODE)
        /* STEP1: Master transmits header */
        lin_master_set_command(ID_MASTER_TX_HEADER_SLAVE_RX_HEADER);
        Cy_SysLib_Delay(INTERVAL_MS);

        /* STEP2: Master transmits header and response */
        lin_master_set_command(ID_MASTER_TX_RESPONSE_SLAVE_RX_RESPONSE);
        Cy_SysLib_Delay(INTERVAL_MS);

        /* STEP3: Master transmits header and slave transmits response */
        lin_master_set_command(ID_MASTER_RX_RESPONSE_SLAVE_TX_RESPONSE);
        Cy_SysLib_Delay(INTERVAL_MS);
#endif
    }
}

/* [] END OF FILE */
