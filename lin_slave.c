/******************************************************************************
* File Name:   lin_slave.c
*
* Description: This is the source code for PSOC4 HVMS LIN slave related
*              function definition.
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
#include "lin_master_slave.h"
#include "cy_device_headers.h"

#if (LIN_MODE == SLAVE_MODE) | (LIN_MODE == LOOP_BACK_MODE)
/*******************************************************************************
* Function Prototypes
********************************************************************************/
void lin_slave_isr(void);

/*******************************************************************************
* Function Name: lin_slave_init_isr
********************************************************************************
* Summary:
*  1. Initialize interrupt handler.
*  2. Set slave first command.
*
* Parameters:
*  none
*
* Return:
*  none
*
********************************************************************************/
void lin_slave_init_isr(void)
{
    cy_en_sysint_status_t sysStatus;

    /* Register LIN interrupt handler and enable interrupt */
    /* Populate the configuration structure */
    const cy_stc_sysint_t slave_irq_cfg =
    {
        .intrSrc = LIN_INTERRUPT_SLAVE, /* LIN interrupt number */
        .intrPriority = 3UL
    };

    /* Hook the interrupt service routine and enable the interrupt */
    sysStatus = Cy_SysInt_Init(&slave_irq_cfg, &lin_slave_isr);
    if(CY_SYSINT_SUCCESS != sysStatus)
    {
        CY_ASSERT(0);
    }

    /* Enable interrupt in NVIC */
    NVIC_EnableIRQ(LIN_INTERRUPT_SLAVE);

    /* Set command to receive header from master */
    lin_slave_set_command(LIN_CMD_RX_HEADER_RX_RESPONSE);
}

/*******************************************************************************
* Function Name: lin_slave_isr
********************************************************************************
* Summary:
*  Arbitration of interrupt.
*
* Parameters:
*  none
*
* Return:
*  none
*
********************************************************************************/
void lin_slave_isr(void)
{
    uint32_t status;
    uint32_t maskStatus;
    uint8_t id;
    uint8_t parity;
    cy_en_lin_status_t linStatus;

    Cy_LIN_GetInterruptMaskedStatus(LIN_SLAVE_CHANNEL, &maskStatus);
    Cy_LIN_GetHeader(LIN_SLAVE_CHANNEL, &id, &parity);

    if (0U != (maskStatus & LIN_INTR_ALL_ERROR_MASK_SLAVE))
    {
        /* There are some error,Handle error if needed. */
        /* Disable the channel to reset LIN status */
        Cy_LIN_Disable(LIN_SLAVE_CHANNEL);

        /* Re-enable the channel */
        Cy_LIN_Enable(LIN_SLAVE_CHANNEL);
        Cy_LIN_SetInterruptMask(LIN_SLAVE_CHANNEL, LIN_INTR_RX_HEADER_DONE |
                                LIN_INTR_TX_RESPONSE_DONE |
                                LIN_INTR_RX_RESPONSE_DONE |
                                LIN_INTR_ALL_ERROR_MASK_SLAVE);
        Cy_LIN_SetCmd(LIN_SLAVE_CHANNEL, LIN_CMD_RX_HEADER_RX_RESPONSE);
    }
    else
    {
        if(maskStatus & LIN_INTR_RX_HEADER_DONE)
        {
            /* Clear Rx Header interrupt. */
            linStatus = Cy_LIN_ClearInterrupt(LIN_SLAVE_CHANNEL, LIN_INTR_RX_HEADER_DONE);

#if defined COMPONENT_PSOC4HVMS64K || COMPONENT_PSOC4HVMS128K
            Cy_GPIO_Inv(CYBSP_LED4_PORT, CYBSP_LED4_PIN);
            Cy_SysLib_Delay(BLINK_TIME_MS);
            Cy_GPIO_Inv(CYBSP_LED4_PORT, CYBSP_LED4_PIN);
#elif defined COMPONENT_PSOC4HVPA144K || COMPONENT_PSOC4HVPASPM10
#endif
            if(ID_MASTER_TX_HEADER_SLAVE_RX_HEADER == id)
            {
                lin_slave_set_command(LIN_CMD_RX_HEADER_RX_RESPONSE);
            }
            else if(ID_MASTER_TX_RESPONSE_SLAVE_RX_RESPONSE == id)
            {
                lin_slave_set_command(LIN_CMD_RX_RESPONSE);
            }
            else if(ID_MASTER_RX_RESPONSE_SLAVE_TX_RESPONSE == id)
            {
                lin_slave_set_command(LIN_CMD_TX_RESPONSE);
            }
        }
        else if(maskStatus & LIN_INTR_RX_RESPONSE_DONE)
        {
            memset(slave_dst_data, 0, LIN_DATA_LENGTH_MAX*sizeof(slave_dst_data[0]));
            while(1)
            {
                linStatus = Cy_LIN_ReadData(LIN_SLAVE_CHANNEL, slave_dst_data);
                if(linStatus == CY_LIN_SUCCESS)
                {
                    break;
                }
            }
            status = memcmp(src_data, slave_dst_data, (LIN_DATA_LENGTH_MAX*sizeof(slave_dst_data[0])));
            if (status !=0)
            {
                CY_ASSERT(0);
            }

            /* Clear all accepted interrupt. */
            Cy_LIN_ClearInterrupt(LIN_SLAVE_CHANNEL, maskStatus);

            if(ID_MASTER_TX_RESPONSE_SLAVE_RX_RESPONSE == id)
            {
#if defined COMPONENT_PSOC4HVMS64K || COMPONENT_PSOC4HVMS128K
                Cy_GPIO_Inv(CYBSP_LED7_PORT, CYBSP_LED7_PIN);
                Cy_SysLib_Delay(BLINK_TIME_MS);
                Cy_GPIO_Inv(CYBSP_LED7_PORT, CYBSP_LED7_PIN);
#elif defined COMPONENT_PSOC4HVPA144K || COMPONENT_PSOC4HVPASPM10
                Cy_GPIO_Inv(CYBSP_LED6_PORT, CYBSP_LED6_PIN);
                Cy_SysLib_Delay(BLINK_TIME_MS);
                Cy_GPIO_Inv(CYBSP_LED6_PORT, CYBSP_LED6_PIN);
#endif

                lin_slave_set_command(LIN_CMD_RX_HEADER_RX_RESPONSE);
            }
        }
        else if(maskStatus & LIN_INTR_TX_RESPONSE_DONE)
        {
             /* Clear all accepted interrupt. */
            Cy_LIN_ClearInterrupt(LIN_SLAVE_CHANNEL, maskStatus);

            if(ID_MASTER_RX_RESPONSE_SLAVE_TX_RESPONSE == id)
            {
#if defined COMPONENT_PSOC4HVMS64K || COMPONENT_PSOC4HVMS128K
                Cy_GPIO_Inv(CYBSP_LED6_PORT, CYBSP_LED6_PIN);
                Cy_SysLib_Delay(BLINK_TIME_MS);
                Cy_GPIO_Inv(CYBSP_LED6_PORT, CYBSP_LED6_PIN);
#elif defined COMPONENT_PSOC4HVPA144K || COMPONENT_PSOC4HVPASPM10
                Cy_GPIO_Inv(CYBSP_LED7_PORT, CYBSP_LED7_PIN);
                Cy_SysLib_Delay(BLINK_TIME_MS);
                Cy_GPIO_Inv(CYBSP_LED7_PORT, CYBSP_LED7_PIN);
#endif

                lin_slave_set_command(LIN_CMD_RX_HEADER_RX_RESPONSE);
            }
        }
    }
}

/*******************************************************************************
* Function Name: lin_slave_set_command
********************************************************************************
* Summary:
*  Set slave command for next communication.
*
* Parameters:
*  uint32_t command
*
* Return:
*  none
*
********************************************************************************/
void lin_slave_set_command(uint32_t command)
{
    cy_en_lin_status_t linStatus;
    /* LIN operation */
    linStatus = Cy_LIN_SetInterruptMask(LIN_SLAVE_CHANNEL, LIN_INTR_RX_HEADER_DONE |
                                        LIN_INTR_RX_RESPONSE_DONE |
                                        LIN_INTR_TX_RESPONSE_DONE |
                                        LIN_INTR_ALL_ERROR_MASK_SLAVE);
    if(CY_LIN_SUCCESS != linStatus)
    {
        CY_ASSERT(0);
    }

    linStatus = Cy_LIN_SetChecksumType(LIN_SLAVE_CHANNEL, LIN_CHECKSUM_TYPE_EXTENDED);
    if(CY_LIN_SUCCESS != linStatus)
    {
        CY_ASSERT(0);
    }

    linStatus = Cy_LIN_SetDataLength(LIN_SLAVE_CHANNEL, LIN_DATA_LENGTH_MAX);
    if(CY_LIN_SUCCESS != linStatus)
    {
        CY_ASSERT(0);
    }

    linStatus = Cy_LIN_SetCmd(LIN_SLAVE_CHANNEL, command);
    if(CY_LIN_SUCCESS != linStatus)
    {
        CY_ASSERT(0);
    }
}
#endif

/* [] END OF FILE */
