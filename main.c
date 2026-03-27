/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code of LIN for ModusToolbox.
*
* Related Document: See README.md
*
*
*******************************************************************************
* (c) 2024-2026, Infineon Technologies AG, or an affiliate of Infineon
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
*  3. Initialize LIN interrupt handler.
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
    /* LIN initialization is done in the function  */
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
