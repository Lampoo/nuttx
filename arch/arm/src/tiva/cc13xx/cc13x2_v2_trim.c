/****************************************************************************
 * arch/arm/src/tiva/cc13xx/cc13x_start.c
 *
 *   Copyright (C) 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * This is a port of TI's setup.c file (revision 49363) which has a fully
 * compatible BSD license:
 *
 *    Copyright (c) 2015-2017, Texas Instruments Incorporated
 *    All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1) Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *
 *  2) Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3) Neither the name NuttX nor the names of its contributors may be used to
 *     endorse or promote products derived from this software without specific
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************/

/******************************************************************************
 * Included Files
 ******************************************************************************/

#include <nuttx/config.h>

/* Hardware headers */

#include "../inc/hw_types.h"
#include "../inc/hw_memmap.h"
#include "../inc/hw_adi.h"
#include "../inc/hw_adi_2_refsys.h"
#include "../inc/hw_adi_3_refsys.h"
#include "../inc/hw_adi_4_aux.h"

/* Temporarily adding these defines as they are missing in hw_adi_4_aux.h */

#define ADI_4_AUX_LPMBIAS_OFFSET                                         0x0000000e
#define ADI_4_AUX_LPMBIAS_LPM_TRIM_IOUT_MASK                           0x0000003f
#define ADI_4_AUX_LPMBIAS_LPM_TRIM_IOUT_SHIFT                                    0
#define ADI_4_AUX_COMP_LPM_BIAS_WIDTH_TRIM_MASK                        0x00000038
#define ADI_4_AUX_COMP_LPM_BIAS_WIDTH_TRIM_SHIFT                                 3
#include "../inc/hw_aon_ioc.h"
#include "../inc/hw_aon_pmctl.h"
#include "../inc/hw_aon_rtc.h"
#include "../inc/hw_ddi_0_osc.h"
#include "../inc/hw_ddi.h"
#include "../inc/hw_ccfg.h"
#include "../inc/hw_fcfg1.h"
#include "../inc/hw_flash.h"
#include "../inc/hw_prcm.h"
#include "../inc/hw_vims.h"

/* Driverlib headers */

#include "aux_sysif.h"
#include "chipinfo.h"
#include "setup.h"
#include "setup_rom.h"

/******************************************************************************
 *
 * Defined CPU delay macro with microseconds as input
 * Quick check shows: (To be further investigated)
 * At 48 MHz RCOSC and VIMS.CONTROL.PREFETCH = 0, there is 5 cycles
 * At 48 MHz RCOSC and VIMS.CONTROL.PREFETCH = 1, there is 4 cycles
 * At 24 MHz RCOSC and VIMS.CONTROL.PREFETCH = 0, there is 3 cycles
 *
 ******************************************************************************/

#define CPU_DELAY_MICRO_SECONDS( x ) \
   CPUdelay(((uint32_t)((( x ) * 48.0 ) / 5.0 )) - 1 )

/******************************************************************************
 *
 * Function declarations
 *
 ******************************************************************************/

static void trim_coldreset(void);
static void trim_wakeup_fromshutdown(uint32_t fcfg1_revision);
static void trim_wakeup_frompowerdown(void);

/******************************************************************************
 *
 * Perform the necessary trim of the device which is not done in boot code
 *
 * This function should only execute coming from ROM boot. The current
 * implementation does not take soft reset into account. However, it does no
 * damage to execute it again. It only consumes time.
 *
 ******************************************************************************/

void cc13x2_cc26x2_trim_device(void)
{
  uint32_t fcfg1_revision;
  uint32_t aon_sysresetctrl;

  /* Get layout revision of the factory configuration area (Handle undefined
   * revision as revision = 0)
   */

  fcfg1_revision = getreg32(TIVA_FCFG1_FCFG1_REVISION);
  if (fcfg1_revision == 0xffffffff)
    {
      fcfg1_revision = 0;
    }

  /* This driverlib version and setup file is for the CC13x2, CC26x2 PG2.0 or
   * later chips. Halt if violated
   */

  ThisLibraryIsFor_CC13x2_CC26x2_HwRev20AndLater_HaltIfViolated();

  /* Enable standby in flash bank */

  regval  = getreg32(TIVA_FLASH_CFG);
  regval &= ~FLASH_CFG_DIS_STANDBY;
  putreg32(regval, TIVA_FLASH_CFG)

  /* Select correct CACHE mode and set correct CACHE configuration */

#if ( CCFG_BASE == CCFG_BASE_DEFAULT )
  SetupSetCacheModeAccordingToCcfgSetting();
#else
  NOROM_SetupSetCacheModeAccordingToCcfgSetting();
#endif

  /* 1. Check for powerdown 2. Check for shutdown 3. Assume cold reset if none
   * of the above. It is always assumed that the application will freeze the
   * latches in AON_IOC when going to powerdown in order to retain the values
   * on the IOs. NB. If this bit is not cleared before proceeding to
   * powerdown, the IOs will all default to the reset configuration when
   * restarting.
   */

  if ((getreg32(TIVA_AON_IOC_IOCLATCH) & AON_IOC_IOCLATCH_EN) == 0)
    {
      /* NB. This should be calling a ROM implementation of required trim and
       * compensation e.g.
       * trim_wakeup_frompowerdown()
       */

      trim_wakeup_frompowerdown();
    }
  /* Check for shutdown When device is going to shutdown the hardware will
   * automatically clear the SLEEPDIS bit in the SLEEP register in the
   * AON_PMCTL module. It is left for the application to assert this bit when
   * waking back up, but not before the desired IO configuration has been
   * re-established.
   */

  else if ((getreg32(TIVA_AON_PMCTL_SLEEPCTL) & AON_PMCTL_SLEEPCTL_IO_PAD_SLEEP_DIS) == 0)
    {
      /* NB. This should be calling a ROM implementation of required trim and
       * compensation e.g. trim_wakeup_fromshutdown() -->
       * trim_wakeup_frompowerdown();
       */

      trim_wakeup_fromshutdown(fcfg1_revision);
      trim_wakeup_frompowerdown();
    }
  else
    {
      /* Consider adding a check for soft reset to allow debugging to skip this
       * section!!! NB. This should be calling a ROM implementation of
       * required trim and compensation e.g. trim_coldreset() -->
       * trim_wakeup_fromshutdown() -->
       * trim_wakeup_frompowerdown()
       */

      trim_coldreset();
      trim_wakeup_fromshutdown(fcfg1_revision);
      trim_wakeup_frompowerdown();
    }

  /* Set VIMS power domain control. PDCTL1VIMS = 0 ==> VIMS power domain is
   * only powered when CPU power domain is powered
   */

  putreg32(0, TIVA_PRCM_PDCTL1VIMS);

  /* Configure optimal wait time for flash FSM in cases where flash pump wakes
   * up from sleep
   */

  regval  = getreg32(TIVA_FLASH_FPAC1);
  regval &= ~FLASH_FPAC1_PSLEEPTDIS_MASK;
  regval |= (0x139 << FLASH_FPAC1_PSLEEPTDIS_SHIFT);
  putreg32(regval, TIVA_FLASH_FPAC1);

  /* And finally at the end of the flash boot process: SET BOOT_DET bits in
   * AON_PMCTL to 3 if already found to be 1 Note: The BOOT_DET_x_CLR/SET bits
   * must be manually cleared
   */

  if (((getreg32(TIVA_AON_PMCTL_RESETCTL) &
        (AON_PMCTL_RESETCTL_BOOT_DET_1_MASK | AON_PMCTL_RESETCTL_BOOT_DET_0_MASK)) >>
       AON_PMCTL_RESETCTL_BOOT_DET_0_SHIFT) == 1)
    {
      aon_sysresetctrl = (getreg32(TIVA_AON_PMCTL_RESETCTL) &
                            ~(AON_PMCTL_RESETCTL_BOOT_DET_1_CLR_MASK |
                              AON_PMCTL_RESETCTL_BOOT_DET_0_CLR_MASK |
                              AON_PMCTL_RESETCTL_BOOT_DET_1_SET_MASK |
                              AON_PMCTL_RESETCTL_BOOT_DET_0_SET_MASK |
                              AON_PMCTL_RESETCTL_MCU_WARM_RESET_MASK));

      putreg32(aon_sysresetctrl | AON_PMCTL_RESETCTL_BOOT_DET_1_SET_MASK,
               TIVA_AON_PMCTL_RESETCTL);
      putreg32(aon_sysresetctrl, TIVA_AON_PMCTL_RESETCTL);
    }

  /* Make sure there are no ongoing VIMS mode change when leaving
   * cc13x2_cc26x2_trim_device() (There should typically be no wait time here,
   * but need to be sure)
   */

  while ((getreg32(TIVA_VIMS_STAT0 & VIMS_STAT_MODE_CHANGING) != 0)
    {
      /* Do nothing - wait for an eventual ongoing mode change to complete. */

    }
}

/******************************************************************************
 *
 * Description:
 *   Trims to be applied when coming from POWER_DOWN (also called when
 *   coming from SHUTDOWN and PIN_RESET).
 *
 * Returned Value:
 *   None
 *
 ******************************************************************************/

static void trim_wakeup_frompowerdown(void)
{
  /* Currently no specific trim for Powerdown */

}

/******************************************************************************
 *
 * Description:
 *   Trims to be applied when coming from SHUTDOWN (also called when
 *   coming from PIN_RESET).
 *
 * \param fcfg1_revision
 *
 * Returned Value:
 *   None
 *
 ******************************************************************************/

static void trim_wakeup_fromshutdown(uint32_t fcfg1_revision)
{
  uint32_t ccfg_ModeConfReg;

  /* Check in CCFG for alternative DCDC setting */

  if ((getreg32(TIVA_CCFG_SIZE_AND_DIS_FLAGS) &
       CCFG_SIZE_AND_DIS_FLAGS_DIS_ALT_DCDC_SETTING) == 0)
    {
      /* ADI_3_REFSYS:DCDCCTL5[3] (=DITHER_EN) = CCFG_MODE_CONF_1[19]
       * (=ALT_DCDC_DITHER_EN) ADI_3_REFSYS:DCDCCTL5[2:0](=IPEAK ) =
       * CCFG_MODE_CONF_1[18:16](=ALT_DCDC_IPEAK ) Using a single 4-bit masked
       * write since layout is equal for both source and destination
       */

      regval = getreg32(TIVA_CCFG_MODE_CONF_1);
      regval = (0xf0 | regval >> CCFG_MODE_CONF_1_ALT_DCDC_IPEAK_SHIFT);
      putreg8((uint8_t)regval, TIVA_ADI3_MASK4B + (ADI_3_REFSYS_DCDCCTL5_OFFSET * 2));
    }

  /* TBD - Temporarily removed for CC13x2 / CC26x2 */

  /* Force DCDC to use RCOSC before starting up XOSC. Clock loss detector does
   * not use XOSC until SCLK_HF actually switches and thus DCDC is not
   * protected from clock loss on XOSC in that time frame. The force must be
   * released when the switch to XOSC has happened. This is done in
   * OSCHfSourceSwitch().
   */

  regval = DDI_0_OSC_CTL0_CLK_DCDC_SRC_SEL_MASK |
          (DDI_0_OSC_CTL0_CLK_DCDC_SRC_SEL_MASK >> 16);
  putreg32(regval, TIVA_AUX_DDI0_OSCMASK16B + (DDI_0_OSC_CTL0_OFFSET << 1) + 4);

  /* Dummy read to ensure that the write has propagated */

  (void)getret16(TIVA_AUX_DDI0_OSCCTL0);

  /* read the MODE_CONF register in CCFG */

  ccfg_ModeConfReg = getreg32(TIVA_CCFG_MODE_CONF);

  /* First part of trim done after cold reset and wakeup from shutdown: -Adjust
   * the VDDR_TRIM_SLEEP value. -Configure DCDC.
   */

  SetupAfterColdResetWakeupFromShutDownCfg1(ccfg_ModeConfReg);

  /* Second part of trim done after cold reset and wakeup from shutdown:
   * -Configure XOSC.
   */

#if ( CCFG_BASE == CCFG_BASE_DEFAULT )
  SetupAfterColdResetWakeupFromShutDownCfg2(fcfg1_revision,
                                            ccfg_ModeConfReg);
#else
  NOROM_SetupAfterColdResetWakeupFromShutDownCfg2(fcfg1_revision,
                                                  ccfg_ModeConfReg);
#endif

  {
    uint32_t trimreg;
    uint32_t trimvalue;

        /*--- Propagate the LPM_BIAS trim --- */

    trimreg = getreg32(TIVA_FCFG1_DAC_BIAS_CNF);
    trimvalue = ((trimreg & FCFG1_DAC_BIAS_CNF_LPM_TRIM_IOUT_MASK) >>
                     FCFG1_DAC_BIAS_CNF_LPM_TRIM_IOUT_SHIFT);

    regval = ((trimvalue << ADI_4_AUX_LPMBIAS_LPM_TRIM_IOUT_SHIFT) &
              ADI_4_AUX_LPMBIAS_LPM_TRIM_IOUT_MASK);
    putreg8((uint8_t)regval, TIVA_AUX_ADI4_LPMBIAS);

    /* Set LPM_BIAS_BACKUP_EN according to FCFG1 configuration */

    if (trimreg & FCFG1_DAC_BIAS_CNF_LPM_BIAS_BACKUP_EN)
      {
        putreg8(ADI_3_REFSYS_AUX_DEBUG_LPM_BIAS_BACKUP_EN,
                TIVA_ADI3_SET + ADI_3_REFSYS_AUX_DEBUG_OFFSET);
      }
    else
      {
        putreg8(ADI_3_REFSYS_AUX_DEBUG_LPM_BIAS_BACKUP_EN,
                TIVA_ADI3_CLR + ADI_3_REFSYS_AUX_DEBUG_OFFSET);
      }

    /* Set LPM_BIAS_WIDTH_TRIM according to FCFG1 configuration */

    {
      uint32_t widthTrim =
        ((trimreg & FCFG1_DAC_BIAS_CNF_LPM_BIAS_WIDTH_TRIM_MASK) >>
         FCFG1_DAC_BIAS_CNF_LPM_BIAS_WIDTH_TRIM_SHIFT);

      /* Set LPM_BIAS_WIDTH_TRIM = 3
       * Set mask (bits to be written) in [15:8]
       * Set value (in correct bit pos) in [7:0]
       */

      regval16 = ((ADI_4_AUX_COMP_LPM_BIAS_WIDTH_TRIM_MASK << 8) |
                  (widthTrim << ADI_4_AUX_COMP_LPM_BIAS_WIDTH_TRIM_SHIFT));
      putreg16(regval16, TIVA_AUX_ADI4_MASK8B + (ADI_4_AUX_COMP_OFFSET * 2));
    }
  }

  /* Third part of trim done after cold reset and wakeup from shutdown:
   * -Configure HPOSC. -Setup the LF clock.
   */

#if ( CCFG_BASE == CCFG_BASE_DEFAULT )
  SetupAfterColdResetWakeupFromShutDownCfg3(ccfg_ModeConfReg);
#else
  NOROM_SetupAfterColdResetWakeupFromShutDownCfg3(ccfg_ModeConfReg);
#endif

  /* Set AUX into power down active mode */

  AUXSYSIFOpModeChange(AUX_SYSIF_OPMODE_TARGET_PDA);

  /* Disable EFUSE clock */

  regval  = getreg32(TIVA_FLASH_CFG);
  regval |= FLASH_CFG_DIS_EFUSECLK;
  putreg32(regval, TIVA_FLASH_CFG)
}

/******************************************************************************
 *
 * Description:
 *   Trims to be applied when coming from PIN_RESET.
 *
 * Returned Value:
 *   None
 *
 ******************************************************************************/

static void trim_coldreset(void)
{
  /* Currently no specific trim for Cold Reset */

}
