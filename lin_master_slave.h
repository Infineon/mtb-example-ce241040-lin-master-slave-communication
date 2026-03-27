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
#define LIN_MODE (SLAVE_MODE)

#if defined COMPONENT_PSOC4HVMS64K || COMPONENT_PSOC4HVMS128K || COMPONENT_PSOC4HVPA144K
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
#elif defined COMPONENT_PSOC4HVPASPM10
    #if (LIN_MODE == MASTER_MODE)
        #define LIN_MASTER_CHANNEL LIN0_CH0
        #define LIN_SLAVE_CHANNEL LIN0_CH1

        #define LIN_INTERRUPT_MASTER lin_interrupts_0_IRQn
        #define LIN_INTERRUPT_SLAVE lin_interrupts_1_IRQn
    #elif (LIN_MODE == SLAVE_MODE) | (LIN_MODE == LOOP_BACK_MODE)
        #define LIN_MASTER_CHANNEL LIN0_CH1
        #define LIN_SLAVE_CHANNEL LIN0_CH0

        #define LIN_INTERRUPT_MASTER lin_interrupts_1_IRQn
        #define LIN_INTERRUPT_SLAVE lin_interrupts_0_IRQn
    #endif
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
