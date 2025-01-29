/******************************************************************************
* File Name:   lin_slave.c
*
* Description: This is the source code for PSOC4 HVMS LIN master related
*              function definition.
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
#include "lin_master_slave.h"
#include "cy_device_headers.h"

#if (LIN_MODE == MASTER_MODE) | (LIN_MODE == LOOP_BACK_MODE)
/*******************************************************************************
* Function Prototypes
********************************************************************************/
void lin_master_isr(void);

/*******************************************************************************
* Function Name: lin_master_init_isr
********************************************************************************
* Summary:
*  Initialize interrupt handler.
*
* Parameters:
*  none
*
* Return:
*  none
*
********************************************************************************/
void lin_master_init_isr(void)
{
    cy_en_sysint_status_t sysStatus;

    /* Register LIN interrupt handler and enable interrupt */
    /* Populate the configuration structure */
    const cy_stc_sysint_t master_irq_cfg =
    {
        .intrSrc = LIN_INTERRUPT_MASTER, /* LIN interrupt number */
        .intrPriority = 3UL
    };
    
    /* Hook the interrupt service routine and enable the interrupt */
    sysStatus = Cy_SysInt_Init(&master_irq_cfg, &lin_master_isr);
    if(CY_SYSINT_SUCCESS != sysStatus)
    {
        CY_ASSERT(0);
    }

    /* Enable interrupt in NVIC */
    NVIC_EnableIRQ(LIN_INTERRUPT_MASTER);
}

/*******************************************************************************
* Function Name: lin_master_isr
********************************************************************************
* Summary:
*  Arbitration of interrupt.
*  If LIN_INTR_RX_RESPONSE_DONE sets, it checks if the data from master is same
*  as source data.
*
* Parameters:
*  none
*
* Return:
*  none
*
********************************************************************************/
void lin_master_isr(void)
{
    uint32_t status;
    uint32_t maskStatus;
    cy_en_lin_status_t linStatus;

    Cy_LIN_GetInterruptMaskedStatus(LIN_MASTER_CHANNEL, &maskStatus);

    if (0U != (maskStatus & LIN_INTR_ALL_ERROR_MASK_MASTER))
    {
        /* There are some error */
        /* Handle error if needed. */

        /* Disable the channel to reset LIN status */
        Cy_LIN_Disable(LIN_MASTER_CHANNEL);

        /* Re-enable the channel */
        Cy_LIN_Enable(LIN_MASTER_CHANNEL);
    }
    else
    {
        if(maskStatus & LIN_INTR_TX_HEADER_DONE)
        {
            Cy_LIN_ClearInterrupt(LIN_MASTER_CHANNEL, LIN_INTR_TX_HEADER_DONE);
        }
        else if(maskStatus & LIN_INTR_TX_RESPONSE_DONE)
        {
            /* Transmitted source data to slave's LIN_CH_DATA0 and LIN_CH_DATA1 */
            Cy_LIN_ClearInterrupt(LIN_MASTER_CHANNEL, maskStatus);
        }
        else if(maskStatus & LIN_INTR_RX_RESPONSE_DONE)
        {
            /* Received slave's LIN_CH_DATA0 and LIN_CH_DATA1 */

            /* clear master_dst_data */
            memset(master_dst_data, 0, LIN_DATA_LENGTH_MAX*sizeof(master_dst_data[0]));

            /* Read master's LIN_CH_DATA0 and LIN_CH_DATA1 */
            while(1)
            {
                linStatus = Cy_LIN_ReadData(LIN_MASTER_CHANNEL, master_dst_data);
                if(linStatus == CY_LIN_SUCCESS)
                {
                    break;
                }
            }
            /* Check if src_data corresponds to master_dst_data. */
            status = memcmp(src_data, master_dst_data, (LIN_DATA_LENGTH_MAX*sizeof(master_dst_data[0])));
            if (status !=0)
            {
                CY_ASSERT(0);
            }

            /* Clear all accepted interrupt. */
            Cy_LIN_ClearInterrupt(LIN_MASTER_CHANNEL, maskStatus);
        }
    }
}

/*******************************************************************************
* Function Name: lin_master_set_command
********************************************************************************
* Summary:
*  Set master command for next communication.
*
* Parameters:
*  uint8_t id
*
* Return:
*  none
*
********************************************************************************/
void lin_master_set_command(uint8_t id)
{
    cy_en_lin_status_t linStatus;
    /* LIN operation */
    linStatus = Cy_LIN_SetInterruptMask(LIN_MASTER_CHANNEL, LIN_INTR_TX_HEADER_DONE |
                                        LIN_INTR_TX_RESPONSE_DONE |
                                        LIN_INTR_RX_RESPONSE_DONE |
                                        LIN_INTR_ALL_ERROR_MASK_SLAVE);
    if(CY_LIN_SUCCESS != linStatus)
    {
        CY_ASSERT(0);
    }
    linStatus = Cy_LIN_SetChecksumType(LIN_MASTER_CHANNEL,LIN_CHECKSUM_TYPE_EXTENDED);
    if(CY_LIN_SUCCESS != linStatus)
    {
        CY_ASSERT(0);
    }
    linStatus = Cy_LIN_SetHeader(LIN_MASTER_CHANNEL, id);
    if(CY_LIN_SUCCESS != linStatus)
    {
        CY_ASSERT(0);
    }
    linStatus = Cy_LIN_SetDataLength(LIN_MASTER_CHANNEL, LIN_DATA_LENGTH_MAX);
    if(CY_LIN_SUCCESS != linStatus)
    {
        CY_ASSERT(0);
    }

    if(ID_MASTER_TX_HEADER_SLAVE_RX_HEADER == id)
    {
        linStatus = Cy_LIN_SetCmd(LIN_MASTER_CHANNEL, LIN_CMD_TX_HEADER);
        if(CY_LIN_SUCCESS != linStatus)
        {
            CY_ASSERT(0);
        }
    }
    else if(ID_MASTER_TX_RESPONSE_SLAVE_RX_RESPONSE == id)
    {
        /* Write source data to master's LIN_CH_DATA0 and LIN_CH_DATA1 */
        Cy_LIN_WriteData(LIN_MASTER_CHANNEL, src_data, LIN_DATA_LENGTH_MAX);

        linStatus = Cy_LIN_SetCmd(LIN_MASTER_CHANNEL, LIN_CMD_TX_HEADER_TX_RESPONSE);
        if(CY_LIN_SUCCESS != linStatus)
        {
            CY_ASSERT(0);
        }
    }
    else if(ID_MASTER_RX_RESPONSE_SLAVE_TX_RESPONSE == id)
    {
        linStatus = Cy_LIN_SetCmd(LIN_MASTER_CHANNEL, LIN_CMD_TX_HEADER_RX_RESPONSE);
        if(CY_LIN_SUCCESS != linStatus)
        {
            CY_ASSERT(0);
        }
    }
}
#endif

/* [] END OF FILE */

