/******************************************************************************
* File Name:   lin_master_slave.h
*
* Description: This is the source code for PSOC4 HVMS LIN master and slave
*              related function declaration and macro definition.
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
#include "cycfg.h"
#include "cycfg_peripherals.h"

/*******************************************************************************
* Macros
********************************************************************************/
#define INTERVAL_MS (500UL)
#define BLINK_TIME_MS (100UL)

#define MASTER_MODE (1UL)
#define SLAVE_MODE  (2UL)
#define LOOP_BACK_MODE (3UL)

/* LIN_MODE defines which mode LIN ch works as */
/* Only ch1 is connected to internal LIN PHY */
#define LIN_MODE (MASTER_MODE)

#if (LIN_MODE == MASTER_MODE)
    #define LIN_MASTER_CHANNEL LIN0_CH1
    #define LIN_SLAVE_CHANNEL LIN0_CH0

    #define LIN_INTERRUPT_MASTER lin_interrupts_1_IRQn
    #define LIN_INTERRUPT_SLAVE lin_interrupts_0_IRQn
#elif (LIN_MODE == SLAVE_MODE) | (LIN_MODE == LOOP_BACK_MODE)
    #define LIN_MASTER_CHANNEL LIN0_CH0
    #define LIN_SLAVE_CHANNEL LIN0_CH1

    #define LIN_INTERRUPT_MASTER lin_interrupts_0_IRQn
    #define LIN_INTERRUPT_SLAVE lin_interrupts_1_IRQn
#endif

#define TEST_MODE (1UL)

/* Frame identifier */
#define ID_MASTER_TX_RESPONSE_SLAVE_RX_RESPONSE  (0x11)
#define ID_MASTER_RX_RESPONSE_SLAVE_TX_RESPONSE  (0x10)
#define ID_MASTER_TX_HEADER_SLAVE_RX_HEADER      (0x20)

/*******************************************************************************
* Global Variables
********************************************************************************/
extern uint8_t src_data[LIN_DATA_LENGTH_MAX];
extern uint8_t master_dst_data[LIN_DATA_LENGTH_MAX];
extern uint8_t slave_dst_data[LIN_DATA_LENGTH_MAX];

/*******************************************************************************
* Function Prototypes
********************************************************************************/
void lin_master_init_isr(void);
void lin_slave_init_isr(void);
void lin_master_set_command(uint8_t id);
void lin_slave_set_command(uint32_t command);

/* [] END OF FILE */
