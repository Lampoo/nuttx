/****************************************************************************************************
 * arch/arm/src/tiva/hardware/cc13x2_cc26x2/cc13x2_cc26x2_ddi0_osc.h
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Authors: Gregory Nutt <gnutt@nuttx.org>
 *
 * Technical content derives from a TI header file that has a compatible BSD license:
 *
 *   Copyright (c) 2015-2017, Texas Instruments Incorporated
 *   All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************************************/

#ifndef __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_DDI0_OSC_H
#define __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_DDI0_OSC_H

/****************************************************************************************************
 * Included Files
 ****************************************************************************************************/

#include <nuttx/config.h>
#include "hardware/cc13x2_cc26x2/cc13x2_cc26x2_ddi.h"
#include "hardware/tiva_memorymap.h"

/****************************************************************************************************
 * Pre-processor Definitions
 ****************************************************************************************************/

/* DDI0 OSC Register Offsets ************************************************************************/

#define TIVA_DDI0_OSC_CTL0_OFFSET                 0x0000  /* Control 0 */
#define TIVA_DDI0_OSC_CTL1_OFFSET                 0x0004  /* Control 1 */
#define TIVA_DDI0_OSC_RADCEXTCFG_OFFSET           0x0008  /* RADC External Configuration */
#define TIVA_DDI0_OSC_AMPCOMPCTL_OFFSET           0x000c  /* Amplitude Compensation Control */
#define TIVA_DDI0_OSC_AMPCOMPTH1_OFFSET           0x0010  /* Amplitude Compensation Threshold 1 */
#define TIVA_DDI0_OSC_AMPCOMPTH2_OFFSET           0x0014  /* Amplitude Compensation Threshold 2 */
#define TIVA_DDI0_OSC_ANABYPASSVAL1_OFFSET        0x0018  /* Analog Bypass Values 1 */
#define TIVA_DDI0_OSC_ANABYPASSVAL2_OFFSET        0x001c
#define TIVA_DDI0_OSC_ATESTCTL_OFFSET             0x0020  /* Analog Test Control */
#define TIVA_DDI0_OSC_ADCDOUBLERNANOAMPCTL_OFFSET 0x0024  /* ADC Doubler Nanoamp Control */
#define TIVA_DDI0_OSC_XOSCHFCTL_OFFSET            0x0028  /* XOSCHF Control */
#define TIVA_DDI0_OSC_LFOSCCTL_OFFSET             0x002c  /* Low Frequency Oscillator Control */
#define TIVA_DDI0_OSC_RCOSCHFCTL_OFFSET           0x0030  /* RCOSCHF Control */
#define TIVA_DDI0_OSC_RCOSCMFCTL_OFFSET           0x0034  /* RCOSC_MF Control */
#define TIVA_DDI0_OSC_STAT0_OFFSET                0x003c  /* Status 0 */
#define TIVA_DDI0_OSC_STAT1_OFFSET                0x0040  /* Status 1 */
#define TIVA_DDI0_OSC_STAT2_OFFSET                0x0044  /* Status 2 */

/* DDI0 OSC Register Addresses **********************************************************************/
/* Direct access */

#define TIVA_DDI0_OSC_CTL0                        (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_CTL0_OFFSET)
#define TIVA_DDI0_OSC_CTL1                        (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_CTL1_OFFSET)
#define TIVA_DDI0_OSC_RADCEXTCFG                  (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_RADCEXTCFG_OFFSET)
#define TIVA_DDI0_OSC_AMPCOMPCTL                  (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_AMPCOMPCTL_OFFSET)
#define TIVA_DDI0_OSC_AMPCOMPTH1                  (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_AMPCOMPTH1_OFFSET)
#define TIVA_DDI0_OSC_AMPCOMPTH2                  (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_AMPCOMPTH2_OFFSET)
#define TIVA_DDI0_OSC_ANABYPASSVAL1               (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_ANABYPASSVAL1_OFFSET)
#define TIVA_DDI0_OSC_ANABYPASSVAL2               (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_ANABYPASSVAL2_OFFSET)
#define TIVA_DDI0_OSC_ATESTCTL                    (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_ATESTCTL_OFFSET)
#define TIVA_DDI0_OSC_ADCDOUBLERNANOAMPCTL        (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_ADCDOUBLERNANOAMPCTL_OFFSET)
#define TIVA_DDI0_OSC_XOSCHFCTL                   (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_XOSCHFCTL_OFFSET)
#define TIVA_DDI0_OSC_LFOSCCTL                    (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_LFOSCCTL_OFFSET)
#define TIVA_DDI0_OSC_RCOSCHFCTL                  (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_RCOSCHFCTL_OFFSET)
#define TIVA_DDI0_OSC_RCOSCMFCTL                  (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_RCOSCMFCTL_OFFSET)
#define TIVA_DDI0_OSC_STAT0                       (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_STAT0_OFFSET)
#define TIVA_DDI0_OSC_STAT1                       (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_STAT1_OFFSET)
#define TIVA_DDI0_OSC_STAT2                       (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI0_OSC_STAT2_OFFSET)

/* Offsets may also be used in conjunction with access as described in cc13x2_cc26x2_ddi.h */

#define TIVA_DDI0_OSC_DIR                        (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI_DIR_OFFSET)
#define TIVA_DDI0_OSC_SET                        (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI_SET_OFFSET)
#define TIVA_DDI0_OSC_CLR                        (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI_CLR_OFFSET)
#define TIVA_DDI0_OSC_MASK4B                     (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI_MASK4B_OFFSET)
#define TIVA_DDI0_OSC_MASK8B                     (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI_MASK8B_OFFSET)
#define TIVA_DDI0_OSC_MASK16B                    (TIVA_AUX_DDI0_OSC_BASE + TIVA_DDI_MASK16B_OFFSET)

/* DDI0 OSC Bitfield Definitions ********************************************************************/

/* DDI0_OSC_CTL0 */

#define DDI0_OSC_CTL0_BYPASS_RCOSC_LF_CLK_QUAL   (1 << 28) /* Bit 28 */
#define DDI0_OSC_CTL0_BYPASS_XOSC_LF_CLK_QUAL    (1 << 29) /* Bit 29 */
#define DDI0_OSC_CTL0_XTAL_IS_24M                (1 << 31) /* Bit 31: Set based high frequency XTAL */
#  define DDI0_OSC_CTL0_XTAL_IS_24M_48M          (0)
#  define DDI0_OSC_CTL0_XTAL_IS_24M_24M          DDI0_OSC_CTL0_XTAL_IS_24M

/* Field: [27:26] DOUBLER_START_DURATION
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_CTL0_DOUBLER_START_DURATION_SHIFT                         26
#define DDI0_OSC_CTL0_DOUBLER_START_DURATION_MASK        (3 << DDI0_OSC_CTL0_DOUBLER_START_DURATION_SHIFT)
#  define DDI0_OSC_CTL0_DOUBLER_START_DURATION(n)        ((uint32_t)(n) << DDI0_OSC_CTL0_DOUBLER_START_DURATION_SHIFT)

#define DDI0_OSC_CTL0_CLK_LOSS_EN                (1 << 9)  /* Bit 9:  Enable SCLK_HF, SCLK_MF and SCLK_LF clock
                                                            * loss detection and indicators to the system
                                                            * controller */
#define DDI0_OSC_CTL0_XOSC_LF_DIG_BYPASS         (1 << 10) /* Bit 10: Bypass XOSC_LF and use the digital
                                                            * input clock from AON for the xosc_lf */
#  define DDI0_OSC_CTL0_XOSC_LF_DIG_32KHZ        (0)                              /* Use 32kHz XOSC as xosc_lf clock
                                                                                   * source */
#  define DDI0_OSC_CTL0_XOSC_LF_DIG_BYPASS       DDI0_OSC_CTL0_XOSC_LF_DIG_BYPASS /* Use digital input (from AON) as
                                                                                   * xosc_lf clock source */
#define DDI0_OSC_CTL0_XOSC_HF_POWER_MODE         (1 << 11) /* Bit 11 */
#define DDI0_OSC_CTL0_RCOSC_LF_TRIMMED           (1 << 12) /* Bit 12 */
#define DDI0_OSC_CTL0_HPOSC_MODE_EN              (1 << 14) /* Bit 14 */
#define DDI0_OSC_CTL0_CLK_DCDC_SRC_SEL           (1 << 24) /* Bit 24: Select DCDC clock source
                                                            * 0=CLK_DCDC is 48 MHz clock from RCOSC or XOSC/HPOSC
                                                            * 1=CLK_DCDC is always 48 MHz clock from RCOSC */
#define DDI0_OSC_CTL0_DOUBLER_RESET_DURATION     (1 << 25) /* Bit 25 */


/* Field:   [8:7] ACLK_TDC_SRC_SEL
 *
 * Source select for aclk_tdc.
 *
 * 00: RCOSC_HF (48MHz)
 * 01: RCOSC_HF (24MHz)
 * 10: XOSC_HF (24MHz)
 * 11: Not used
 */

#define DDI0_OSC_CTL0_ACLK_TDC_SRC_SEL_SHIFT                                7
#define DDI0_OSC_CTL0_ACLK_TDC_SRC_SEL_MASK        (3 << DDI0_OSC_CTL0_ACLK_TDC_SRC_SEL_SHIFT)
#  define DDI0_OSC_CTL0_ACLK_TDC_SRC_SEL(n)        ((uint32_t)(n) << DDI0_OSC_CTL0_ACLK_TDC_SRC_SEL_SHIFT)

/* Field:   [6:4] ACLK_REF_SRC_SEL
 *
 * Source select for aclk_ref
 *
 * 000: RCOSC_HF derived (31.25kHz)
 * 001: XOSC_HF derived (31.25kHz)
 * 010: RCOSC_LF (32kHz)
 * 011: XOSC_LF (32.768kHz)
 * 100: RCOSC_MF (2MHz)
 * 101-111: Not used
 */

#define DDI0_OSC_CTL0_ACLK_REF_SRC_SEL_SHIFT                                4
#define DDI0_OSC_CTL0_ACLK_REF_SRC_SEL_MASK        (7 << DDI0_OSC_CTL0_ACLK_REF_SRC_SEL_SHIFT)
#  define DDI0_OSC_CTL0_ACLK_REF_SRC_SEL(n)        ((uint32_t)(n) << DDI0_OSC_CTL0_ACLK_REF_SRC_SEL_SHIFT)

/* Field:   [3:2] SCLK_LF_SRC_SEL
 *
 * Source select for sclk_lf
 * ENUMs:
 * XOSCLF                   Low frequency XOSC
 * RCOSCLF                  Low frequency RCOSC
 * XOSCHFDLF                Low frequency clock derived from High Frequency
 *                          XOSC
 * RCOSCHFDLF               Low frequency clock derived from High Frequency
 *                          RCOSC
 */

#define DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_SHIFT                                 2
#define DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_MASK        (3 << DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_SHIFT)
#  define DDI0_OSC_CTL0_SCLK_LF_SRC_SEL(n)        ((uint32_t)(n) << DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_SHIFT)
#  define DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_XOSCLF                       0x0000000c
#  define DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_RCOSCLF                      0x00000008
#  define DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_XOSCHFDLF                    0x00000004
#  define DDI0_OSC_CTL0_SCLK_LF_SRC_SEL_RCOSCHFDLF                   0x00000000

#define DDI0_OSC_CTL0_SCLK_HF_SRC_SEL            (1 << 0)                      /* Bit 0:  Source select for sclk_hf */
#  define DDI0_OSC_CTL0_SCLK_HF_SRC_RCOSC        (0)                           /* High frequency RCOSC clock */
#  define DDI0_OSC_CTL0_SCLK_HF_SRC_XOSC         DDI0_OSC_CTL0_SCLK_HF_SRC_SEL /* High frequency XOSC clock */

/* DDI0_OSC_CTL1 */

/* Field: [22:18] RCOSCHFCTRIMFRACT
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_CTL1_RCOSCHFCTRIMFRACT_SHIFT                              18
#define DDI0_OSC_CTL1_RCOSCHFCTRIMFRACT_MASK        (31 << DDI0_OSC_CTL1_RCOSCHFCTRIMFRACT_SHIFT)
#  define DDI0_OSC_CTL1_RCOSCHFCTRIMFRACT(n)        ((uint32_t)(n) << DDI0_OSC_CTL1_RCOSCHFCTRIMFRACT_SHIFT)

/* Field:    [17] RCOSCHFCTRIMFRACT_EN
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_CTL1_RCOSCHFCTRIMFRACT_EN       (1 << 17) /* Bit 17 */

/* Field:   [1:0] XOSC_HF_FAST_START
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_CTL1_XOSC_HF_FAST_START_SHIFT                              0
#define DDI0_OSC_CTL1_XOSC_HF_FAST_START_MASK        (3 << DDI0_OSC_CTL1_XOSC_HF_FAST_START_SHIFT)
#  define DDI0_OSC_CTL1_XOSC_HF_FAST_START(n)        ((uint32_t)(n) << DDI0_OSC_CTL1_XOSC_HF_FAST_START_SHIFT)

/* DDI0_OSC_RADCEXTCFG */

/* Field: [31:22] HPM_IBIAS_WAIT_CNT
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_RADCEXTCFG_HPM_IBIAS_WAIT_CNT_SHIFT                       22
#define DDI0_OSC_RADCEXTCFG_HPM_IBIAS_WAIT_CNT_MASK        (0x3ff << DDI0_OSC_RADCEXTCFG_HPM_IBIAS_WAIT_CNT_SHIFT)
#  define DDI0_OSC_RADCEXTCFG_HPM_IBIAS_WAIT_CNT(n)        ((uint32_t)(n) << DDI0_OSC_RADCEXTCFG_HPM_IBIAS_WAIT_CNT_SHIFT)

/* Field: [21:16] LPM_IBIAS_WAIT_CNT
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_RADCEXTCFG_LPM_IBIAS_WAIT_CNT_SHIFT                       16
#define DDI0_OSC_RADCEXTCFG_LPM_IBIAS_WAIT_CNT_MASK        (0x3f << DDI0_OSC_RADCEXTCFG_LPM_IBIAS_WAIT_CNT_SHIFT)
#  define DDI0_OSC_RADCEXTCFG_LPM_IBIAS_WAIT_CNT(n)        ((uint32_t)(n) << DDI0_OSC_RADCEXTCFG_LPM_IBIAS_WAIT_CNT_SHIFT)

/* Field: [15:12] IDAC_STEP
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_RADCEXTCFG_IDAC_STEP_SHIFT                                12
#define DDI0_OSC_RADCEXTCFG_IDAC_STEP_MASK        (15 << DDI0_OSC_RADCEXTCFG_IDAC_STEP_SHIFT)
#  define DDI0_OSC_RADCEXTCFG_IDAC_STEP(n)        ((uint32_t)(n) << DDI0_OSC_RADCEXTCFG_IDAC_STEP_SHIFT)

/* Field:  [11:6] RADC_DAC_TH
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_RADCEXTCFG_RADC_DAC_TH_SHIFT                               6
#define DDI0_OSC_RADCEXTCFG_RADC_DAC_TH_MASK        (0x3f << DDI0_OSC_RADCEXTCFG_RADC_DAC_TH_SHIFT)
#  define DDI0_OSC_RADCEXTCFG_RADC_DAC_TH(n)        ((uint32_t)(n) << DDI0_OSC_RADCEXTCFG_RADC_DAC_TH_SHIFT)

#define DDI0_OSC_RADCEXTCFG_RADC_MODE_IS_SAR     (1 << 5)  /* Bit 5 */

/* DDI0_OSC_AMPCOMPCTL */

#define DDI0_OSC_AMPCOMPCTL_AMPCOMP_REQ_MODE     (1 << 30) /* Bit 30 */

/* Field: [29:28] AMPCOMP_FSM_UPDATE_RATE
 *
 * Internal. Only to be used through TI provided API.
 * ENUMs:
 * 250KHZ                   Internal. Only to be used through TI provided API.
 * 500KHZ                   Internal. Only to be used through TI provided API.
 * 1MHZ                     Internal. Only to be used through TI provided API.
 * 2MHZ                     Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_SHIFT                  28
#define DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_MASK        (3 << DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_SHIFT)
#  define DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE(n)        ((uint32_t)(n) << DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_SHIFT)
#  define DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_250KHZ         0x30000000
#  define DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_500KHZ         0x20000000
#  define DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_1MHZ           0x10000000
#  define DDI0_OSC_AMPCOMPCTL_AMPCOMP_FSM_UPDATE_RATE_2MHZ           0x00000000

#define DDI0_OSC_AMPCOMPCTL_AMPCOMP_SW_EN        (1 << 26) /* Bit 26 */
#define DDI0_OSC_AMPCOMPCTL_AMPCOMP_SW_CTRL      (1 << 27) /* Bit 27 */

/* Field: [23:20] IBIAS_OFFSET
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_AMPCOMPCTL_IBIAS_OFFSET_SHIFT                             20
#define DDI0_OSC_AMPCOMPCTL_IBIAS_OFFSET_MASK        (15 << DDI0_OSC_AMPCOMPCTL_IBIAS_OFFSET_SHIFT)
#  define DDI0_OSC_AMPCOMPCTL_IBIAS_OFFSET(n)        ((uint32_t)(n) << DDI0_OSC_AMPCOMPCTL_IBIAS_OFFSET_SHIFT)

/* Field: [19:16] IBIAS_INIT
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_AMPCOMPCTL_IBIAS_INIT_SHIFT                               16
#define DDI0_OSC_AMPCOMPCTL_IBIAS_INIT_MASK        (15 << DDI0_OSC_AMPCOMPCTL_IBIAS_INIT_SHIFT)
#  define DDI0_OSC_AMPCOMPCTL_IBIAS_INIT(n)        ((uint32_t)(n) << DDI0_OSC_AMPCOMPCTL_IBIAS_INIT_SHIFT)

/* Field:  [15:8] LPM_IBIAS_WAIT_CNT_FINAL
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_AMPCOMPCTL_LPM_IBIAS_WAIT_CNT_FINAL_SHIFT                  8
#define DDI0_OSC_AMPCOMPCTL_LPM_IBIAS_WAIT_CNT_FINAL_MASK        (0xff << DDI0_OSC_AMPCOMPCTL_LPM_IBIAS_WAIT_CNT_FINAL_SHIFT)
#  define DDI0_OSC_AMPCOMPCTL_LPM_IBIAS_WAIT_CNT_FINAL(n)        ((uint32_t)(n) << DDI0_OSC_AMPCOMPCTL_LPM_IBIAS_WAIT_CNT_FINAL_SHIFT)

/* Field:   [7:4] CAP_STEP
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_AMPCOMPCTL_CAP_STEP_SHIFT                                  4
#define DDI0_OSC_AMPCOMPCTL_CAP_STEP_MASK        (15 << DDI0_OSC_AMPCOMPCTL_CAP_STEP_SHIFT)
#  define DDI0_OSC_AMPCOMPCTL_CAP_STEP(n)        ((uint32_t)(n) << DDI0_OSC_AMPCOMPCTL_CAP_STEP_SHIFT)

/* Field:   [3:0] IBIASCAP_HPTOLP_OL_CNT
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_AMPCOMPCTL_IBIASCAP_HPTOLP_OL_CNT_SHIFT                    0
#define DDI0_OSC_AMPCOMPCTL_IBIASCAP_HPTOLP_OL_CNT_MASK        (15 << DDI0_OSC_AMPCOMPCTL_IBIASCAP_HPTOLP_OL_CNT_SHIFT)
#  define DDI0_OSC_AMPCOMPCTL_IBIASCAP_HPTOLP_OL_CNT(n)        ((uint32_t)(n) << DDI0_OSC_AMPCOMPCTL_IBIASCAP_HPTOLP_OL_CNT_SHIFT)

/* DDI0_OSC_AMPCOMPTH1 */

/* Field: [23:18] HPMRAMP3_LTH
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_AMPCOMPTH1_HPMRAMP3_LTH_SHIFT                             18
#define DDI0_OSC_AMPCOMPTH1_HPMRAMP3_LTH_MASK        (0x3f << DDI0_OSC_AMPCOMPTH1_HPMRAMP3_LTH_SHIFT)
#  define DDI0_OSC_AMPCOMPTH1_HPMRAMP3_LTH(n)        ((uint32_t)(n) << DDI0_OSC_AMPCOMPTH1_HPMRAMP3_LTH_SHIFT)

/* Field: [15:10] HPMRAMP3_HTH
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_AMPCOMPTH1_HPMRAMP3_HTH_SHIFT                             10
#define DDI0_OSC_AMPCOMPTH1_HPMRAMP3_HTH_MASK        (0x3f << DDI0_OSC_AMPCOMPTH1_HPMRAMP3_HTH_SHIFT)
#  define DDI0_OSC_AMPCOMPTH1_HPMRAMP3_HTH(n)        ((uint32_t)(n) << DDI0_OSC_AMPCOMPTH1_HPMRAMP3_HTH_SHIFT)

/* Field:   [9:6] IBIASCAP_LPTOHP_OL_CNT
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_AMPCOMPTH1_IBIASCAP_LPTOHP_OL_CNT_SHIFT                    6
#define DDI0_OSC_AMPCOMPTH1_IBIASCAP_LPTOHP_OL_CNT_MASK        (15 << DDI0_OSC_AMPCOMPTH1_IBIASCAP_LPTOHP_OL_CNT_SHIFT)
#  define DDI0_OSC_AMPCOMPTH1_IBIASCAP_LPTOHP_OL_CNT(n)        ((uint32_t)(n) << DDI0_OSC_AMPCOMPTH1_IBIASCAP_LPTOHP_OL_CNT_SHIFT)

/* Field:   [5:0] HPMRAMP1_TH
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_AMPCOMPTH1_HPMRAMP1_TH_SHIFT                               0
#define DDI0_OSC_AMPCOMPTH1_HPMRAMP1_TH_MASK        (0x3f << DDI0_OSC_AMPCOMPTH1_HPMRAMP1_TH_SHIFT)
#  define DDI0_OSC_AMPCOMPTH1_HPMRAMP1_TH(n)        ((uint32_t)(n) << DDI0_OSC_AMPCOMPTH1_HPMRAMP1_TH_SHIFT)

/* DDI0_OSC_AMPCOMPTH2 */

/* Field: [31:26] LPMUPDATE_LTH
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_AMPCOMPTH2_LPMUPDATE_LTH_SHIFT                            26
#define DDI0_OSC_AMPCOMPTH2_LPMUPDATE_LTH_MASK        (0x3f << DDI0_OSC_AMPCOMPTH2_LPMUPDATE_LTH_SHIFT)
#  define DDI0_OSC_AMPCOMPTH2_LPMUPDATE_LTH(n)        ((uint32_t)(n) << DDI0_OSC_AMPCOMPTH2_LPMUPDATE_LTH_SHIFT)

/* Field: [23:18] LPMUPDATE_HTH
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_AMPCOMPTH2_LPMUPDATE_HTH_SHIFT                            18
#define DDI0_OSC_AMPCOMPTH2_LPMUPDATE_HTH_MASK        (0x3f << DDI0_OSC_AMPCOMPTH2_LPMUPDATE_HTH_SHIFT)
#  define DDI0_OSC_AMPCOMPTH2_LPMUPDATE_HTH(n)        ((uint32_t)(n) << DDI0_OSC_AMPCOMPTH2_LPMUPDATE_HTH_SHIFT)

/* Field: [15:10] ADC_COMP_AMPTH_LPM
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_LPM_SHIFT                       10
#define DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_LPM_MASK        (0x3f << DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_LPM_SHIFT)
#  define DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_LPM(n)        ((uint32_t)(n) << DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_LPM_SHIFT)

/* Field:   [7:2] ADC_COMP_AMPTH_HPM
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_HPM_SHIFT                        2
#define DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_HPM_MASK        (0x3f << DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_HPM_SHIFT)
#  define DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_HPM(n)        ((uint32_t)(n) << DDI0_OSC_AMPCOMPTH2_ADC_COMP_AMPTH_HPM_SHIFT)

/* DDI0_OSC_ANABYPASSVAL1 */

/* Field: [19:16] XOSC_HF_ROW_Q12
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_ANABYPASSVAL1_XOSC_HF_ROW_Q12_SHIFT                       16
#define DDI0_OSC_ANABYPASSVAL1_XOSC_HF_ROW_Q12_MASK        (15 << DDI0_OSC_ANABYPASSVAL1_XOSC_HF_ROW_Q12_SHIFT)
#  define DDI0_OSC_ANABYPASSVAL1_XOSC_HF_ROW_Q12(n)        ((uint32_t)(n) << DDI0_OSC_ANABYPASSVAL1_XOSC_HF_ROW_Q12_SHIFT)

/* Field:  [15:0] XOSC_HF_COLUMN_Q12
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_ANABYPASSVAL1_XOSC_HF_COLUMN_Q12_SHIFT                     0
#define DDI0_OSC_ANABYPASSVAL1_XOSC_HF_COLUMN_Q12_MASK        (0xffff << DDI0_OSC_ANABYPASSVAL1_XOSC_HF_COLUMN_Q12_SHIFT)
#  define DDI0_OSC_ANABYPASSVAL1_XOSC_HF_COLUMN_Q12(n)        ((uint32_t)(n) << DDI0_OSC_ANABYPASSVAL1_XOSC_HF_COLUMN_Q12_SHIFT)

/* DDI0_OSC_ANABYPASSVAL2 */

/* Field:  [13:0] XOSC_HF_IBIASTHERM
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_ANABYPASSVAL2_XOSC_HF_IBIASTHERM_SHIFT                     0
#define DDI0_OSC_ANABYPASSVAL2_XOSC_HF_IBIASTHERM_MASK        (0x3fff << DDI0_OSC_ANABYPASSVAL2_XOSC_HF_IBIASTHERM_SHIFT)
#  define DDI0_OSC_ANABYPASSVAL2_XOSC_HF_IBIASTHERM(n)        ((uint32_t)(n) << DDI0_OSC_ANABYPASSVAL2_XOSC_HF_IBIASTHERM_SHIFT)

/* DDI0_OSC_ATESTCTL */

#define DDI0_OSC_ATESTCTL_SCLK_LF_AUX_EN        (1 << 31) /* Bit 31: Enable 32 kHz clock to AUX_COMPB */

/* Field: [15:14] TEST_RCOSCMF
 *
 * Test mode control for RCOSC_MF
 *
 * 0x0:  test modes disabled
 * 0x1:  boosted bias current into self biased inverter
 * 0x2:  clock qualification disabled
 * 0x3:  boosted bias current into self biased inverter + clock qualification
 * disabled
 */

#define DDI0_OSC_ATESTCTL_TEST_RCOSCMF_SHIFT                               14
#define DDI0_OSC_ATESTCTL_TEST_RCOSCMF_MASK        (3 << DDI0_OSC_ATESTCTL_TEST_RCOSCMF_SHIFT)
#  define DDI0_OSC_ATESTCTL_TEST_RCOSCMF(n)        ((uint32_t)(n) << DDI0_OSC_ATESTCTL_TEST_RCOSCMF_SHIFT)

/* Field: [13:12] ATEST_RCOSCMF
 *
 * ATEST control for RCOSC_MF
 *
 * 0x0:  ATEST disabled
 * 0x1:  ATEST enabled, VDD_LOCAL connected,  ATEST internal to **RCOSC_MF*
 * enabled to send out 2MHz clock.
 * 0x2:  ATEST disabled
 * 0x3:  ATEST enabled, bias current connected, ATEST internal to **RCOSC_MF*
 * enabled to send out 2MHz clock.
 */

#define DDI0_OSC_ATESTCTL_ATEST_RCOSCMF_SHIFT                              12
#define DDI0_OSC_ATESTCTL_ATEST_RCOSCMF_MASK        (3 << DDI0_OSC_ATESTCTL_ATEST_RCOSCMF_SHIFT)
#  define DDI0_OSC_ATESTCTL_ATEST_RCOSCMF(n)        ((uint32_t)(n) << DDI0_OSC_ATESTCTL_ATEST_RCOSCMF_SHIFT)

/* DDI0_OSC_ADCDOUBLERNANOAMPCTL */

#define DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_SH_VBUF_EN       (1 << 4)  /* Bit 4 */
#define DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_SH_MODE_EN       (1 << 5)  /* Bit 5 */
#define DDI0_OSC_ADCDOUBLERNANOAMPCTL_SPARE23              (1 << 23) /* Bit 23 */
#define DDI0_OSC_ADCDOUBLERNANOAMPCTL_NANOAMP_BIAS_ENABLE  (1 << 24) /* Bit 24 */

/* Field:   [1:0] ADC_IREF_CTRL
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_IREF_CTRL_SHIFT                   0
#define DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_IREF_CTRL_MASK        (3 << DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_IREF_CTRL_SHIFT)
#  define DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_IREF_CTRL(n)        ((uint32_t)(n) << DDI0_OSC_ADCDOUBLERNANOAMPCTL_ADC_IREF_CTRL_SHIFT)

/* DDI0_OSC_XOSCHFCTL */

#define DDI0_OSC_XOSCHFCTL_TCXO_MODE            (1 << 12) /* Bit 12: Enable qualifications to TXCO clock (if BYPASS=1) */
#define DDI0_OSC_XOSCHFCTL_TCXO_MODE_XOSC_HF_EN (1 << 13) /* Bit 13: Enable XOSC_HF (if TCX0_MODE=1) */

/* Field:   [9:8] PEAK_DET_ITRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_XOSCHFCTL_PEAK_DET_ITRIM_SHIFT                             8
#define DDI0_OSC_XOSCHFCTL_PEAK_DET_ITRIM_MASK        (3 << DDI0_OSC_XOSCHFCTL_PEAK_DET_ITRIM_SHIFT)
#  define DDI0_OSC_XOSCHFCTL_PEAK_DET_ITRIM(n)        ((uint32_t)(n) << DDI0_OSC_XOSCHFCTL_PEAK_DET_ITRIM_SHIFT)

#define DDI0_OSC_XOSCHFCTL_BYPASS               (1 << 6)  /* Bit 6 */

/* Field:   [4:2] HP_BUF_ITRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_XOSCHFCTL_HP_BUF_ITRIM_SHIFT                               2
#define DDI0_OSC_XOSCHFCTL_HP_BUF_ITRIM_MASK        (7 << DDI0_OSC_XOSCHFCTL_HP_BUF_ITRIM_SHIFT)
#  define DDI0_OSC_XOSCHFCTL_HP_BUF_ITRIM(n)        ((uint32_t)(n) << DDI0_OSC_XOSCHFCTL_HP_BUF_ITRIM_SHIFT)

/* Field:   [1:0] LP_BUF_ITRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_XOSCHFCTL_LP_BUF_ITRIM_SHIFT                               0
#define DDI0_OSC_XOSCHFCTL_LP_BUF_ITRIM_MASK        (3 << DDI0_OSC_XOSCHFCTL_LP_BUF_ITRIM_SHIFT)
#  define DDI0_OSC_XOSCHFCTL_LP_BUF_ITRIM(n)        ((uint32_t)(n) << DDI0_OSC_XOSCHFCTL_LP_BUF_ITRIM_SHIFT)

/* DDI0_OSC_LFOSCCTL */

/* Field: [23:22] XOSCLF_REGULATOR_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_LFOSCCTL_XOSCLF_REGULATOR_TRIM_SHIFT                      22
#define DDI0_OSC_LFOSCCTL_XOSCLF_REGULATOR_TRIM_MASK        (3 << DDI0_OSC_LFOSCCTL_XOSCLF_REGULATOR_TRIM_SHIFT)
#  define DDI0_OSC_LFOSCCTL_XOSCLF_REGULATOR_TRIM(n)        ((uint32_t)(n) << DDI0_OSC_LFOSCCTL_XOSCLF_REGULATOR_TRIM_SHIFT)

/* Field: [21:18] XOSCLF_CMIRRWR_RATIO
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_LFOSCCTL_XOSCLF_CMIRRWR_RATIO_SHIFT                       18
#define DDI0_OSC_LFOSCCTL_XOSCLF_CMIRRWR_RATIO_MASK        (15 << DDI0_OSC_LFOSCCTL_XOSCLF_CMIRRWR_RATIO_SHIFT)
#  define DDI0_OSC_LFOSCCTL_XOSCLF_CMIRRWR_RATIO(n)        ((uint32_t)(n) << DDI0_OSC_LFOSCCTL_XOSCLF_CMIRRWR_RATIO_SHIFT)

/* Field:   [9:8] RCOSCLF_RTUNE_TRIM
 *
 * Internal. Only to be used through TI provided API.
 * ENUMs:
 * 6P0MEG                   Internal. Only to be used through TI provided API.
 * 6P5MEG                   Internal. Only to be used through TI provided API.
 * 7P0MEG                   Internal. Only to be used through TI provided API.
 * 7P5MEG                   Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_SHIFT                          8
#define DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_MASK        (3 << DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_SHIFT)
#  define DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM(n)        ((uint32_t)(n) << DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_SHIFT)
#  define DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_6P0MEG                0x00000300
#  define DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_6P5MEG                0x00000200
#  define DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_7P0MEG                0x00000100
#  define DDI0_OSC_LFOSCCTL_RCOSCLF_RTUNE_TRIM_7P5MEG                0x00000000

/* Field:   [7:0] RCOSCLF_CTUNE_TRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM_SHIFT                          0
#define DDI0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM_MASK        (0xff << DDI0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM_SHIFT)
#  define DDI0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM(n)        ((uint32_t)(n) << DDI0_OSC_LFOSCCTL_RCOSCLF_CTUNE_TRIM_SHIFT)

/* DDI0_OSC_RCOSCHFCTL */

/* Field:  [15:8] RCOSCHF_CTRIM
 *
 * Internal. Only to be used through TI provided API.
 */

#define DDI0_OSC_RCOSCHFCTL_RCOSCHF_CTRIM_SHIFT                             8
#define DDI0_OSC_RCOSCHFCTL_RCOSCHF_CTRIM_MASK        (0xff << DDI0_OSC_RCOSCHFCTL_RCOSCHF_CTRIM_SHIFT)
#  define DDI0_OSC_RCOSCHFCTL_RCOSCHF_CTRIM(n)        ((uint32_t)(n) << DDI0_OSC_RCOSCHFCTL_RCOSCHF_CTRIM_SHIFT)

/* DDI0_OSC_RCOSCMFCTL */

/* Field:  [15:9] RCOSC_MF_CAP_ARRAY
 *
 * Adjust RCOSC_MF capacitor array.
 *
 * 0x0:  nominal frequency, 0.625pF
 * 0x40:  highest frequency, 0.125pF
 * 0x3f:  lowest frequency, 1.125pF
 */

#define DDI0_OSC_RCOSCMFCTL_RCOSC_MF_CAP_ARRAY_SHIFT                        9
#define DDI0_OSC_RCOSCMFCTL_RCOSC_MF_CAP_ARRAY_MASK        (0x7f << DDI0_OSC_RCOSCMFCTL_RCOSC_MF_CAP_ARRAY_SHIFT)
#  define DDI0_OSC_RCOSCMFCTL_RCOSC_MF_CAP_ARRAY(n)        ((uint32_t)(n) << DDI0_OSC_RCOSCMFCTL_RCOSC_MF_CAP_ARRAY_SHIFT)

#define DDI0_OSC_RCOSCMFCTL_RCOSC_MF_REG_SEL    (1 << 8)  /* Bit 8:  Select alternate regulator type */

/* Field:   [7:6] RCOSC_MF_RES_COARSE
 *
 * Select coarse resistor for frequency adjustment.
 *
 * 0x0:  400kohms, default
 * 0x1:  300kohms, min
 * 0x2:  600kohms, max
 * 0x3:  500kohms
 */

#define DDI0_OSC_RCOSCMFCTL_RCOSC_MF_RES_COARSE_SHIFT                       6
#define DDI0_OSC_RCOSCMFCTL_RCOSC_MF_RES_COARSE_MASK        (3 << DDI0_OSC_RCOSCMFCTL_RCOSC_MF_RES_COARSE_SHIFT)
#  define DDI0_OSC_RCOSCMFCTL_RCOSC_MF_RES_COARSE(n)        ((uint32_t)(n) << DDI0_OSC_RCOSCMFCTL_RCOSC_MF_RES_COARSE_SHIFT)

/* Field:   [5:4] RCOSC_MF_RES_FINE
 *
 * Select fine resistor for frequency adjustment.
 *
 * 0x0:  11kohms, minimum resistance, max freq
 * 0x1:  13kohms
 * 0x2:  16kohms
 * 0x3:  20kohms, max resistance, min freq
 */

#define DDI0_OSC_RCOSCMFCTL_RCOSC_MF_RES_FINE_SHIFT                         4
#define DDI0_OSC_RCOSCMFCTL_RCOSC_MF_RES_FINE_MASK        (3 << DDI0_OSC_RCOSCMFCTL_RCOSC_MF_RES_FINE_SHIFT)
#  define DDI0_OSC_RCOSCMFCTL_RCOSC_MF_RES_FINE(n)        ((uint32_t)(n) << DDI0_OSC_RCOSCMFCTL_RCOSC_MF_RES_FINE_SHIFT)

/* Field:   [3:0] RCOSC_MF_BIAS_ADJ
 *
 * Adjusts bias current to RCOSC_MF.
 *
 * 0x8 minimum current
 * 0x0 default current
 * 0x7 maximum current
 */

#define DDI0_OSC_RCOSCMFCTL_RCOSC_MF_BIAS_ADJ_SHIFT                         0
#define DDI0_OSC_RCOSCMFCTL_RCOSC_MF_BIAS_ADJ_MASK        (15 << DDI0_OSC_RCOSCMFCTL_RCOSC_MF_BIAS_ADJ_SHIFT)
#  define DDI0_OSC_RCOSCMFCTL_RCOSC_MF_BIAS_ADJ(n)        ((uint32_t)(n) << DDI0_OSC_RCOSCMFCTL_RCOSC_MF_BIAS_ADJ_SHIFT)

/* DDI0_OSC_STAT0 */

/* Field: [30:29] SCLK_LF_SRC
 *
 * Indicates source for the sclk_lf
 * ENUMs:
 * XOSCLF                   Low frequency XOSC
 * RCOSCLF                  Low frequency RCOSC
 * XOSCHFDLF                Low frequency clock derived from High Frequency
 *                          XOSC
 * RCOSCHFDLF               Low frequency clock derived from High Frequency
 *                          RCOSC
 */

#define DDI0_OSC_STAT0_SCLK_LF_SRC_SHIFT                                   29
#define DDI0_OSC_STAT0_SCLK_LF_SRC_MASK        (3 << DDI0_OSC_STAT0_SCLK_LF_SRC_SHIFT)
#  define DDI0_OSC_STAT0_SCLK_LF_SRC(n)        ((uint32_t)(n) << DDI0_OSC_STAT0_SCLK_LF_SRC_SHIFT)
#  define DDI0_OSC_STAT0_SCLK_LF_SRC_XOSCLF                          0x60000000
#  define DDI0_OSC_STAT0_SCLK_LF_SRC_RCOSCLF                         0x40000000
#  define DDI0_OSC_STAT0_SCLK_LF_SRC_XOSCHFDLF                       0x20000000
#  define DDI0_OSC_STAT0_SCLK_LF_SRC_RCOSCHFDLF                      0x00000000

#define DDI0_OSC_STAT0_ADC_DATA_READY           (1 << 7)  /* Bit 7:  Indicates when adc_data is ready */
#define DDI0_OSC_STAT0_ADC_THMET                (1 << 8)  /* Bit 8:  ADC_THMET */
#define DDI0_OSC_STAT0_XOSC_HF_HP_BUF_EN        (1 << 10) /* Bit 10: XOSC_HF_HP_BUF_EN */
#define DDI0_OSC_STAT0_XOSC_HF_LP_BUF_EN        (1 << 11) /* Bit 11: XOSC_HF_LP_BUF_EN */
#define DDI0_OSC_STAT0_XB_48M_CLK_EN            (1 << 13) /* Bit 13: Indicates that the 48MHz clock from
                                                           * the DOUBLER is enabled */
#define DDI0_OSC_STAT0_XOSC_HF_EN               (1 << 15) /* Bit 15: Indicates that XOSC_HF is enabled */
#define DDI0_OSC_STAT0_SCLK_LF_LOSS             (1 << 16) /* Bit 16: Indicates sclk_lf is lost */
#define DDI0_OSC_STAT0_SCLK_HF_LOSS             (1 << 17) /* Bit 17:  Indicates sclk_hf is lost */
#define DDI0_OSC_STAT0_CLK_DCDC_RDY_ACK         (1 << 18) /* Bit 18: CLK_DCDC ready acknowledge */
#define DDI0_OSC_STAT0_CLK_DCDC_RDY             (1 << 19) /* Bit 19: CLK_DCDC ready */
#define DDI0_OSC_STAT0_XOSC_LF_EN               (1 << 20) /* Bit 20: XOSC_LF enable */
#define DDI0_OSC_STAT0_RCOSC_LF_EN              (1 << 21) /* Bit 21: RCOSC_LF enable */
#define DDI0_OSC_STAT0_RCOSC_HF_EN              (1 << 22) /* Bit 22: RSOSC_HF enable */
#define DDI0_OSC_STAT0_SCLK_HF_SRC              (1 << 28) /* Bit 28:  Indicates source for sclk_hf */
#  define DDI0_OSC_STAT0_SCLK_HF_SRC_RCOSC      (0)  /* High frequency RCOSC clock */
#  define DDI0_OSC_STAT0_SCLK_HF_SRC_XOSC       DDI0_OSC_STAT0_SCLK_HF_SRC /* High frequency XOSC */

/* Field:   [6:1] ADC_DATA
 *
 * adc_data
 */

#define DDI0_OSC_STAT0_ADC_DATA_SHIFT                                       1
#define DDI0_OSC_STAT0_ADC_DATA_MASK        (0x3f << DDI0_OSC_STAT0_ADC_DATA_SHIFT)
#  define DDI0_OSC_STAT0_ADC_DATA(n)        ((uint32_t)(n) << DDI0_OSC_STAT0_ADC_DATA_SHIFT)

#define DDI0_OSC_STAT0_PENDINGSCLKHFSWITCHING   (1 << 0)  /* Bit 0:  Indicates when SCLK_HF clock source
                                                           *         is ready to be switched */

/* DDI0_OSC_STAT1 */

/* Field: [31:28] RAMPSTATE
 *
 * AMPCOMP FSM State
 * ENUMs:
 * FAST_START_SETTLE        FAST_START_SETTLE
 * FAST_START               FAST_START
 * DUMMY_TO_INIT_1          DUMMY_TO_INIT_1
 * IDAC_DEC_W_MEASURE       IDAC_DECREMENT_WITH_MEASURE
 * IBIAS_INC                IBIAS_INCREMENT
 * LPM_UPDATE               LPM_UPDATE
 * IBIAS_DEC_W_MEASURE      IBIAS_DECREMENT_WITH_MEASURE
 * IBIAS_CAP_UPDATE         IBIAS_CAP_UPDATE
 * IDAC_INCREMENT           IDAC_INCREMENT
 * HPM_UPDATE               HPM_UPDATE
 * HPM_RAMP3                HPM_RAMP3
 * HPM_RAMP2                HPM_RAMP2
 * HPM_RAMP1                HPM_RAMP1
 * INITIALIZATION           INITIALIZATION
 * RESET                    RESET
 */

#define DDI0_OSC_STAT1_RAMPSTATE_SHIFT                                     28
#define DDI0_OSC_STAT1_RAMPSTATE_MASK        (15 << DDI0_OSC_STAT1_RAMPSTATE_SHIFT)
#  define DDI0_OSC_STAT1_RAMPSTATE(n)        ((uint32_t)(n) << DDI0_OSC_STAT1_RAMPSTATE_SHIFT)
#  define DDI0_OSC_STAT1_RAMPSTATE_FAST_START_SETTLE                 0xe0000000
#  define DDI0_OSC_STAT1_RAMPSTATE_FAST_START                        0xd0000000
#  define DDI0_OSC_STAT1_RAMPSTATE_DUMMY_TO_INIT_1                   0xc0000000
#  define DDI0_OSC_STAT1_RAMPSTATE_IDAC_DEC_W_MEASURE                0xb0000000
#  define DDI0_OSC_STAT1_RAMPSTATE_IBIAS_INC                         0xa0000000
#  define DDI0_OSC_STAT1_RAMPSTATE_LPM_UPDATE                        0x90000000
#  define DDI0_OSC_STAT1_RAMPSTATE_IBIAS_DEC_W_MEASURE               0x80000000
#  define DDI0_OSC_STAT1_RAMPSTATE_IBIAS_CAP_UPDATE                  0x70000000
#  define DDI0_OSC_STAT1_RAMPSTATE_IDAC_INCREMENT                    0x60000000
#  define DDI0_OSC_STAT1_RAMPSTATE_HPM_UPDATE                        0x50000000
#  define DDI0_OSC_STAT1_RAMPSTATE_HPM_RAMP3                         0x40000000
#  define DDI0_OSC_STAT1_RAMPSTATE_HPM_RAMP2                         0x30000000
#  define DDI0_OSC_STAT1_RAMPSTATE_HPM_RAMP1                         0x20000000
#  define DDI0_OSC_STAT1_RAMPSTATE_INITIALIZATION                    0x10000000
#  define DDI0_OSC_STAT1_RAMPSTATE_RESET                             0x00000000

/* Field: [27:22] HPM_UPDATE_AMP
 *
 * XOSC_HF amplitude during HPM_UPDATE state.
 * When amplitude compensation of XOSC_HF is enabled in high performance mode,
 * this value is the amplitude of the crystal oscillations measured by the
 * on-chip oscillator ADC, divided by 15 mV.  For example, a value of 0x20
 * would indicate that the amplitude of the crystal is approximately 480 mV.
 * To enable amplitude compensation, AON_WUC OSCCFG must be set to a non-zero
 * value.
 */

#define DDI0_OSC_STAT1_HPM_UPDATE_AMP_SHIFT                                22
#define DDI0_OSC_STAT1_HPM_UPDATE_AMP_MASK        (0x3f << DDI0_OSC_STAT1_HPM_UPDATE_AMP_SHIFT)
#  define DDI0_OSC_STAT1_HPM_UPDATE_AMP(n)        ((uint32_t)(n) << DDI0_OSC_STAT1_HPM_UPDATE_AMP_SHIFT)

/* Field: [21:16] LPM_UPDATE_AMP
 *
 * XOSC_HF amplitude during LPM_UPDATE state
 * When amplitude compensation of XOSC_HF is enabled in low power  mode, this
 * value is the amplitude of the crystal oscillations measured by the on-chip
 * oscillator ADC, divided by 15 mV.  For example, a value of 0x20 would
 * indicate that the amplitude of the crystal is approximately 480 mV.  To
 * enable amplitude compensation, AON_WUC OSCCFG must be set to a non-zero
 * value.
 */

#define DDI0_OSC_STAT1_LPM_UPDATE_AMP_SHIFT                                16
#define DDI0_OSC_STAT1_LPM_UPDATE_AMP_MASK        (0x3f << DDI0_OSC_STAT1_LPM_UPDATE_AMP_SHIFT)
#  define DDI0_OSC_STAT1_LPM_UPDATE_AMP(n)        ((uint32_t)(n) << DDI0_OSC_STAT1_LPM_UPDATE_AMP_SHIFT)

#define DDI0_OSC_STAT1_CLK_DCDC_GOOD            (1 << 0)  /* Bit 0:  CLK_DCDC_GOOD */
#define DDI0_OSC_STAT1_CLK_CHP_GOOD             (1 << 1)  /* Bit 1:  CLK_CHP_GOOD */
#define DDI0_OSC_STAT1_ACLK_REF_GOOD            (1 << 2)  /* Bit 2:  ACLK_REF_GOOD */
#define DDI0_OSC_STAT1_ACLK_TDC_GOOD            (1 << 3)  /* Bit 3:  ACLK_TDC_GOOD */
#define DDI0_OSC_STAT1_ACLK_ADC_GOOD            (1 << 4)  /* Bit 4:  ACLK_ADC_GOOD */
#define DDI0_OSC_STAT1_SCLK_LF_GOOD             (1 << 5)  /* Bit 5:  SCLK_LF_GOOD */
#define DDI0_OSC_STAT1_SCLK_MF_GOOD             (1 << 6)  /* Bit 6:  SCLK_MF_GOOD */
#define DDI0_OSC_STAT1_SCLK_HF_GOOD             (1 << 7)  /* Bit 7:  SCLK_HF_GOOD */
#define DDI0_OSC_STAT1_CLK_DCDC_EN              (1 << 8)  /* Bit 8:  CLK_DCDC_EN */
#define DDI0_OSC_STAT1_CLK_CHP_EN               (1 << 9)  /* Bit 9:  CLK_CHP_EN */
#define DDI0_OSC_STAT1_ACLK_REF_EN              (1 << 10) /* Bit 10: ACLK_REF_EN */
#define DDI0_OSC_STAT1_ACLK_TDC_EN              (1 << 11) /* Bit 11: ACLK_TDC_EN */
#define DDI0_OSC_STAT1_ACLK_ADC_EN              (1 << 12) /* Bit 12: ACLK_ADC_EN */
#define DDI0_OSC_STAT1_SCLK_MF_EN               (1 << 13) /* Bit 13: SCLK_MF_EN */
#define DDI0_OSC_STAT1_SCLK_HF_EN               (1 << 14) /* Bit 14: SCLK_HF_EN */
#define DDI0_OSC_STAT1_FORCE_RCOSC_HF           (1 << 15) /* Bit 15: force_rcosc_hf */

/* DDI0_OSC_STAT2 */

/* Field: [31:26] ADC_DCBIAS
 *
 * DC Bias read by RADC during SAR mode
 * The value is an unsigned integer. It is used for debug only.
 */

#define DDI0_OSC_STAT2_ADC_DCBIAS_SHIFT                                    26
#define DDI0_OSC_STAT2_ADC_DCBIAS_MASK        (0x3f << DDI0_OSC_STAT2_ADC_DCBIAS_SHIFT)
#  define DDI0_OSC_STAT2_ADC_DCBIAS(n)        ((uint32_t)(n) << DDI0_OSC_STAT2_ADC_DCBIAS_SHIFT)

#define DDI0_OSC_STAT2_HPM_RAMP3_THMET          (1 << 23) /* Bit 23: Indication of threshold is met for hpm_ramp3 */
#define DDI0_OSC_STAT2_HPM_RAMP2_THMET          (1 << 24) /* Bit 24: Indication of threshold is met for hpm_ramp2 */
#define DDI0_OSC_STAT2_HPM_RAMP1_THMET          (1 << 25) /* Bit 25: Indication of threshold is met for hpm_ramp1 */

/* Field: [15:12] RAMPSTATE
 *
 * xosc_hf amplitude compensation FSM
 *
 * This is identical to STAT1.RAMPSTATE. See that description for encoding.
 */

#define DDI0_OSC_STAT2_RAMPSTATE_SHIFT                                     12
#define DDI0_OSC_STAT2_RAMPSTATE_MASK        (15 << DDI0_OSC_STAT2_RAMPSTATE_SHIFT)
#  define DDI0_OSC_STAT2_RAMPSTATE(n)        ((uint32_t)(n) << DDI0_OSC_STAT2_RAMPSTATE_SHIFT)

#define DDI0_OSC_STAT2_XOSC_HF_RF_FREQGOOD      (1 << 0)  /* Bit 0:  Frequency of xosc_hf is within +/- 20 ppm
                                                           *         and xosc_hf is good for radio operations. */
#define DDI0_OSC_STAT2_XOSC_HF_FREQGOOD         (1 << 1)  /* Bit 1:  Frequency of xosc_hf is good to use for
                                                           *         the digital clocks */
#define DDI0_OSC_STAT2_XOSC_HF_AMPGOOD          (1 << 2)  /* Bit 2:  Amplitude of xosc_hf is within the
                                                           *         required threshold (set by DDI) */
#define DDI0_OSC_STAT2_AMPCOMP_REQ              (1 << 3)  /* Bit 3:  AMPCOMP_REQ */

#endif /* __ARCH_ARM_SRC_TIVA_HARDWARE_CC13X2_CC26X2_CC13X2_CC26X2_DDI0_OSC_H */
