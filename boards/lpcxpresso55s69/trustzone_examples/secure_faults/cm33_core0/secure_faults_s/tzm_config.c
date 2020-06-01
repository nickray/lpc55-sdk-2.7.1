/*
 * Copyright 2017-2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/***********************************************************************************************************************
 * This file was generated by the MCUXpresso Config Tools. Any manual edits made to this file
 * will be overwritten if the respective MCUXpresso Config Tools is used to update this file.
 **********************************************************************************************************************/

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: TEE v1.0
processor: LPC55S69
package_id: LPC55S69JBD100
mcu_data: ksdk2_0
processor_version: 0.2.6
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

#include "fsl_common.h"
#include "tzm_config.h"

//********************************************************************
//*** Definitions ****************************************************
//********************************************************************
/* SAU region boundaries */
#define REGION_0_BASE 0
#define REGION_0_END 0x0FFFFFFFU
#define REGION_1_BASE 0x20000000U
#define REGION_1_END 0xFFFFFFFFU
#define REGION_2_BASE 0x1000FE00U
#define REGION_2_END 0x1000FFFFU

/* clang-format off */
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
tee:
- ahb:
  - regions:
    - relative_region: {memory: PROGRAM_FLASH, security: s_priv, start: '0x00000000', size: '0x00010000'}
    - relative_region: {memory: PROGRAM_FLASH, security: ns_user, start: '0x00010000', size: '0x00090000'}
    - relative_region: {memory: BootROM, security: ns_user, start: '0x00000000', size: '0x00020000'}
    - relative_region: {memory: SRAMX, security: ns_user, start: '0x00000000', size: '0x00008000'}
    - relative_region: {memory: SRAM0, security: s_priv, start: '0x00000000', size: '0x00008000'}
    - relative_region: {memory: SRAM0, security: ns_user, start: '0x00008000', size: '0x00008000'}
    - relative_region: {memory: SRAM1, security: s_priv, start: '0x00000000', size: '0x00008000'}
    - relative_region: {memory: SRAM1, security: ns_user, start: '0x00008000', size: '0x00008000'}
    - relative_region: {memory: SRAM2, security: ns_user, start: '0x00000000', size: '0x00010000'}
    - relative_region: {memory: SRAM3, security: ns_user, start: '0x00000000', size: '0x00010000'}
    - relative_region: {memory: SRAM4, security: ns_user, start: '0x00000000', size: '0x00004000'}
    - relative_region: {memory: USB_RAM, security: ns_user, start: '0x00000000', size: '0x00004000'}
  - masters:
    - ns_user: {id: HASH, id: MCM33C, id: MCM33S, id: PQ, id: SDIO, id: SDMA0, id: SDMA1, id: USBFSD, id: USBFSH}
  - peripherals:
    - ns_user: {id: ADC0, id: AHB_SECURE_CTRL, id: ANACTRL, id: CASPER, id: CRC_ENGINE, id: CTIMER0, id: CTIMER1, id: CTIMER2, id: CTIMER3, id: CTIMER4, id: DBGMAILBOX,
      id: DMA0, id: DMA1, id: FLASH, id: FLEXCOMM1, id: FLEXCOMM2, id: FLEXCOMM3, id: FLEXCOMM4, id: FLEXCOMM5, id: FLEXCOMM6, id: FLEXCOMM7, id: GINT0, id: GINT1,
      id: GPIO, id: HASHCRYPT, id: INPUTMUX, id: MAILBOX, id: MRT0, id: OSTIMER, id: PINT, id: PLU, id: PMC, id: POWERQUAD, id: PRINCE, id: PUF, id: RNG, id: RTC,
      id: SCT0, id: SDIF, id: SECGPIO, id: SECPINT, id: SPI8, id: SYSCTL, id: USB0, id: USBFSH, id: USBHSD, id: USBHSH, id: USBPHY, id: UTICK0, id: WWDT}
    - s_priv: {id: FLEXCOMM0, id: IOCON, id: SYSCON}
  - interrupts:
    - masking:
      - Non-masked: {id: acmp_capt_irq, id: adc_irq, id: casper_irq, id: ctimer0_irq, id: ctimer1_irq, id: ctimer2_irq, id: ctimer3_irq, id: ctimer4_irq, id: flexcomm0_irq,
        id: flexcomm1_irq, id: flexcomm2_irq, id: flexcomm3_irq, id: flexcomm4_irq, id: flexcomm5_irq, id: flexcomm6_irq, id: flexcomm7_irq, id: global_irq0, id: global_irq1,
        id: lspi_hs_irq, id: mailbox_irq, id: mrt_irq, id: os_event_irq, id: pin_int4, id: pin_int5, id: pin_int6, id: pin_int7, id: pin_irq0, id: pin_irq1, id: pin_irq2,
        id: pin_irq3, id: plu_irq, id: pq_irq, id: qddkey_irq, id: rtc_irq, id: sct_irq, id: sdio_irq, id: sdma0_irq, id: sdma1_irq, id: sec_hypervisor_call_irq,
        id: sec_int0, id: sec_int1, id: sec_vio_irq, id: sha_irq, id: sys_irq, id: usb0_irq, id: usb0_needclk_irq, id: usb1_irq, id: usb1_needclk_irq, id: usb1_utmi_irq,
        id: utick_irq}
    - security:
      - Secure: {id: acmp_capt_irq, id: adc_irq, id: casper_irq, id: ctimer0_irq, id: ctimer1_irq, id: ctimer2_irq, id: ctimer3_irq, id: ctimer4_irq, id: flexcomm0_irq,
        id: flexcomm1_irq, id: flexcomm2_irq, id: flexcomm3_irq, id: flexcomm4_irq, id: flexcomm5_irq, id: flexcomm6_irq, id: flexcomm7_irq, id: global_irq0, id: global_irq1,
        id: lspi_hs_irq, id: mailbox_irq, id: mrt_irq, id: os_event_irq, id: pin_int4, id: pin_int5, id: pin_int6, id: pin_int7, id: pin_irq0, id: pin_irq1, id: pin_irq2,
        id: pin_irq3, id: plu_irq, id: pq_irq, id: qddkey_irq, id: rtc_irq, id: sct_irq, id: sdio_irq, id: sdma0_irq, id: sdma1_irq, id: sec_hypervisor_call_irq,
        id: sec_int0, id: sec_int1, id: sec_vio_irq, id: sha_irq, id: sys_irq, id: usb0_irq, id: usb0_needclk_irq, id: usb1_irq, id: usb1_needclk_irq, id: usb1_utmi_irq,
        id: utick_irq}
  - pins_masks:
    - pio0:
      - Non-masked: {id: '0', id: '1', id: '10', id: '11', id: '12', id: '13', id: '14', id: '15', id: '16', id: '17', id: '18', id: '19', id: '2', id: '20', id: '21',
        id: '22', id: '23', id: '24', id: '25', id: '26', id: '27', id: '28', id: '29', id: '3', id: '30', id: '31', id: '4', id: '5', id: '6', id: '7', id: '8',
        id: '9'}
    - pio1:
      - Non-masked: {id: '0', id: '1', id: '10', id: '11', id: '12', id: '13', id: '14', id: '15', id: '16', id: '17', id: '18', id: '19', id: '2', id: '20', id: '21',
        id: '22', id: '23', id: '24', id: '25', id: '26', id: '27', id: '28', id: '29', id: '3', id: '30', id: '31', id: '4', id: '5', id: '6', id: '7', id: '8',
        id: '9'}
- sau:
  - enabled: 'true'
  - generate_code_for_disabled_regions: 'false'
  - regions:
    - region: {index: '0', enabled: 'true', security: ns, start: '0x00000000', size: '0x10000000'}
    - region: {index: '1', enabled: 'true', security: ns, start: '0x20000000', size: '0xE0000000'}
    - region: {index: '2', enabled: 'true', security: nsc, start: '0x1000FE00', size: '0x00000200'}
    - region: {index: '3', enabled: 'false', security: ns, start: '0x00000000', size: '0x00000020'}
    - region: {index: '4', enabled: 'false', security: ns, start: '0x00000000', size: '0x00000020'}
    - region: {index: '5', enabled: 'false', security: ns, start: '0x00000000', size: '0x00000020'}
    - region: {index: '6', enabled: 'false', security: ns, start: '0x00000000', size: '0x00000020'}
    - region: {index: '7', enabled: 'false', security: ns, start: '0x00000000', size: '0x00000020'}
- global_options:
  - no: {id: AIRCR_PRIS, id: AIRCR_BFHFNMINS, id: AIRCR_SYSRESETREQS, id: SCR_SLEEPDEEPS, id: SHCSR_SECUREFAULTENA, id: NSACR_CP2, id: NSACR_CP3, id: NSACR_CP4,
    id: NSACR_CP5, id: NSACR_CP6, id: NSACR_CP7, id: CPPWR_SU0, id: CPPWR_SUS0, id: CPPWR_SU1, id: CPPWR_SUS1, id: CPPWR_SU2, id: CPPWR_SUS2, id: CPPWR_SU3, id: CPPWR_SUS3,
    id: CPPWR_SU4, id: CPPWR_SUS4, id: CPPWR_SU5, id: CPPWR_SUS5, id: CPPWR_SU6, id: CPPWR_SUS6, id: CPPWR_SU7, id: CPPWR_SUS7, id: CPPWR_SU10, id: CPPWR_SUS10, id: CPPWR_SU11,
    id: CPPWR_SUS11, id: SEC_GPIO_MASK0_LOCK, id: SEC_GPIO_MASK1_LOCK, id: SEC_CPU1_INT_MASK0_LOCK, id: SEC_CPU1_INT_MASK1_LOCK, id: MASTER_SEC_LEVEL_LOCK, id: CPU0_LOCK_NS_VTOR,
    id: CPU0_LOCK_NS_MPU, id: CPU0_LOCK_S_VTAIRCR, id: CPU0_LOCK_S_MPU, id: CPU0_LOCK_SAU, id: CPU0_LOCK_REG_LOCK, id: CPU1_LOCK_NS_VTOR, id: CPU1_LOCK_NS_MPU, id: CPU1_LOCK_REG_LOCK,
    id: AHB_MISC_CTRL_REG_ENABLE_S_PRIV_CHECK, id: AHB_MISC_CTRL_REG_ENABLE_NS_PRIV_CHECK, id: AHB_MISC_CTRL_REG_DISABLE_VIOLATION_ABORT, id: AHB_MISC_CTRL_REG_DISABLE_SIMPLE_MASTER_STRICT_MODE,
    id: AHB_MISC_CTRL_REG_DISABLE_SMART_MASTER_STRICT_MODE, id: AHB_MISC_CTRL_REG_IDAU_ALL_NS}
  - yes: {id: NSACR_CP0, id: NSACR_CP1, id: NSACR_CP10, id: NSACR_CP11, id: AHB_MISC_CTRL_REG_ENABLE_SECURE_CHECKING, id: AHB_MISC_CTRL_REG_WRITE_LOCK}
- user_memory_regions:
  - user_memory_region: {id: Region_1, security: s_priv, name: Secure Code, start: '0x10000000', size: '0x0000FE00'}
  - user_memory_region: {id: Region_2, security: nsc_priv, name: Veneer Table, start: '0x1000FE00', size: '0x00000200'}
  - user_memory_region: {id: Region_3, security: s_priv, name: Secure Stack and Data, start: '0x30000000', size: '0x00008000'}
  - user_memory_region: {id: Region_4, security: ns_user, name: Non-secure Code, description: Privilege check is disabled so reset value (NS-User) can be used,
    start: '0x00010000', size: '0x00062000'}
  - user_memory_region: {id: Region_5, security: ns_user, name: Non-secure Stack and Data, description: 'Privilege check is disabled so reset value (NS-User) can
      be used. ', start: '0x20008000', size: '0x00008000'}
  - user_memory_region: {id: Region_6, security: s_priv, name: Wrong Non-secure Stack and Data, description: This region is configured to demonstrate misalignment  between SAU/IDAU and AHB secure controller configuration in test 5.,
    start: '0x20010000', size: '0x00008000'}
  - user_memory_region: {id: Region_7, security: ns_user, name: Non-secure Stack and Data, description: 'Privilege check is disabled so reset value (NS-User) can
      be used. ', start: '0x20018000', size: '0x0001B000'}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/
/* clang-format on */

/*!
 * @brief TrustZone initialization
 *
 * The function configures SAU and AHB.
 */
void BOARD_InitTrustZone()
{
    //####################################################################
    //### SAU configuration ##############################################
    //####################################################################

    /* Set SAU Control register: Disable SAU and All Secure */
    SAU->CTRL = 0;

    /* Set SAU region number */
    SAU->RNR = 0;
    /* Region base address */
    SAU->RBAR = REGION_0_BASE & SAU_RBAR_BADDR_Msk;
    /* Region end address */
    SAU->RLAR = ((REGION_0_END & SAU_RLAR_LADDR_Msk) | ((0U << SAU_RLAR_NSC_Pos) & SAU_RLAR_NSC_Msk)) |
                ((1U << SAU_RLAR_ENABLE_Pos) & SAU_RLAR_ENABLE_Msk);

    /* Set SAU region number */
    SAU->RNR = 0x00000001U;
    /* Region base address */
    SAU->RBAR = REGION_1_BASE & SAU_RBAR_BADDR_Msk;
    /* Region end address */
    SAU->RLAR = ((REGION_1_END & SAU_RLAR_LADDR_Msk) | ((0U << SAU_RLAR_NSC_Pos) & SAU_RLAR_NSC_Msk)) |
                ((1U << SAU_RLAR_ENABLE_Pos) & SAU_RLAR_ENABLE_Msk);

    /* Set SAU region number */
    SAU->RNR = 0x00000002U;
    /* Region base address */
    SAU->RBAR = REGION_2_BASE & SAU_RBAR_BADDR_Msk;
    /* Region end address */
    SAU->RLAR = ((REGION_2_END & SAU_RLAR_LADDR_Msk) | ((1U << SAU_RLAR_NSC_Pos) & SAU_RLAR_NSC_Msk)) |
                ((1U << SAU_RLAR_ENABLE_Pos) & SAU_RLAR_ENABLE_Msk);

    /* Force memory writes before continuing */
    __DSB();
    /* Flush and refill pipeline with updated permissions */
    __ISB();
    /* Set SAU Control register: Enable SAU and All Secure (applied only if disabled) */
    SAU->CTRL = 0x00000001U;

    //####################################################################
    //### AHB Configurations #############################################
    //####################################################################

    //--------------------------------------------------------------------
    //--- AHB Security Level Configurations ------------------------------
    //--------------------------------------------------------------------
    /* Configuration of AHB Secure Controller
     * Possible values for every memory sector or peripheral rule:
     *  0    Non-secure, user access allowed.
     *  1    Non-secure, privileged access allowed.
     *  2    Secure, user access allowed.
     *  3    Secure, privileged access allowed. */

    //--- Security level configuration of memories -----------------------
    AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_FLASH_MEM_RULE[0] = 0x00000033U;
    AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_FLASH_MEM_RULE[1] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_FLASH_MEM_RULE[2] = 0;
    AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_ROM_MEM_RULE[0]   = 0;
    AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_ROM_MEM_RULE[1]   = 0;
    AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_ROM_MEM_RULE[2]   = 0;
    AHB_SECURE_CTRL->SEC_CTRL_FLASH_ROM[0].SEC_CTRL_ROM_MEM_RULE[3]   = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAMX[0].MEM_RULE[0]                     = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM0[0].MEM_RULE[0]                     = 0x33333333U;
    AHB_SECURE_CTRL->SEC_CTRL_RAM0[0].MEM_RULE[1]                     = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM1[0].MEM_RULE[0]                     = 0x33333333U;
    AHB_SECURE_CTRL->SEC_CTRL_RAM1[0].MEM_RULE[1]                     = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM2[0].MEM_RULE[0]                     = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM2[0].MEM_RULE[1]                     = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM3[0].MEM_RULE[0]                     = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM3[0].MEM_RULE[1]                     = 0;
    AHB_SECURE_CTRL->SEC_CTRL_RAM4[0].MEM_RULE[0]                     = 0;
    AHB_SECURE_CTRL->SEC_CTRL_USB_HS[0].MEM_RULE[0]                   = 0;

    //--- Security level configuration of peripherals --------------------
    AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE0_MEM_CTRL0 = 0x00000033U;
    AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE0_MEM_CTRL1 = 0;
    AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE0_MEM_CTRL2 = 0;
    AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE1_MEM_CTRL0 = 0;
    AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE1_MEM_CTRL1 = 0;
    AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE1_MEM_CTRL2 = 0;
    AHB_SECURE_CTRL->SEC_CTRL_APB_BRIDGE[0].SEC_CTRL_APB_BRIDGE1_MEM_CTRL3 = 0;
    AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT8_SLAVE0_RULE                        = 0x03000000U;
    AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT8_SLAVE1_RULE                        = 0;
    AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT9_SLAVE0_RULE                        = 0;
    AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT9_SLAVE1_RULE                        = 0;
    AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT10[0].SLAVE0_RULE                    = 0;
    AHB_SECURE_CTRL->SEC_CTRL_AHB_PORT10[0].SLAVE1_RULE                    = 0;

    //--- Security level configuration of masters ------------------------
    AHB_SECURE_CTRL->MASTER_SEC_LEVEL        = 0;
    AHB_SECURE_CTRL->MASTER_SEC_ANTI_POL_REG = 0x3FFFFFFFU;

    //--------------------------------------------------------------------
    //--- Pins: Reading GPIO state ---------------------------------------
    //--------------------------------------------------------------------
    // Possible values for every pin:
    //  0b0    Deny
    //  0b1    Allow
    //--------------------------------------------------------------------
    AHB_SECURE_CTRL->SEC_GPIO_MASK0 = 0xFFFFFFFFU;
    AHB_SECURE_CTRL->SEC_GPIO_MASK1 = 0xFFFFFFFFU;

    //--------------------------------------------------------------------
    //--- Interrupts: Interrupt handling by Core1 ------------------------
    //--------------------------------------------------------------------
    // Possible values for every interrupt:
    //  0b0    Deny
    //  0b1    Allow
    //--------------------------------------------------------------------
    AHB_SECURE_CTRL->SEC_CPU_INT_MASK0 = 0xFFFFFFFFU;
    AHB_SECURE_CTRL->SEC_CPU_INT_MASK1 = 0xFFFFFFFFU;

    //--------------------------------------------------------------------
    //--- Interrupts: Interrupt security configuration -------------------
    //--------------------------------------------------------------------
    // Possible values for every interrupt:
    //  0b0    Secure
    //  0b1    Non-secure
    //--------------------------------------------------------------------
    NVIC->ITNS[0] = 0;
    NVIC->ITNS[1] = 0;

    //--------------------------------------------------------------------
    //--- Global Options -------------------------------------------------
    //--------------------------------------------------------------------
    SCB->AIRCR = (SCB->AIRCR & 0x000009FF7U) | 0x005FA0000U;
    SCB->SCR &= 0x0FFFFFFF7U;
    SCB->SHCSR &= 0x0FFF7FFFFU;
    SCB->NSACR                               = 0x00000C03U;
    SCnSCB->CPPWR                            = 0;
    AHB_SECURE_CTRL->SEC_MASK_LOCK           = 0x00000AAAU;
    AHB_SECURE_CTRL->MASTER_SEC_LEVEL        = (AHB_SECURE_CTRL->MASTER_SEC_LEVEL & 0x03FFFFFFFU) | 0x080000000U;
    AHB_SECURE_CTRL->MASTER_SEC_ANTI_POL_REG = (AHB_SECURE_CTRL->MASTER_SEC_ANTI_POL_REG & 0x03FFFFFFFU) | 0x080000000U;
    AHB_SECURE_CTRL->CPU0_LOCK_REG           = 0x800002AAU;
    AHB_SECURE_CTRL->CPU1_LOCK_REG           = 0x8000000AU;
    AHB_SECURE_CTRL->MISC_CTRL_REG           = (AHB_SECURE_CTRL->MISC_CTRL_REG & 0x0FFFF0003U) | 0x00000AAA4U;
    AHB_SECURE_CTRL->MISC_CTRL_DP_REG        = 0x0000AAA5U;
}
