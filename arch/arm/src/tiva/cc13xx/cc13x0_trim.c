/******************************************************************************
 *  Filename:       setup.c
 *  Revised:        2018-06-26 13:51:40 +0200 (Tue, 26 Jun 2018)
 *  Revision:       52217
 *
 *  Description:    Setup file for CC13xx/CC26xx devices.
 *
 *  Copyright (c) 2015-2017, Texas Instruments Incorporated
 *  All rights reserved.
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

/* Hardware headers */

#include "../inc/hw_types.h"
#include "../inc/hw_memmap.h"
#include "../inc/hw_adi.h"
#include "../inc/hw_adi_2_refsys.h"
#include "../inc/hw_adi_3_refsys.h"
#include "../inc/hw_aon_ioc.h"
#include "../inc/hw_aon_sysctl.h"
#include "../inc/hw_aon_wuc.h"
#include "../inc/hw_aux_wuc.h"
#include "../inc/hw_ccfg.h"
#include "../inc/hw_fcfg1.h"
#include "../inc/hw_flash.h"
#include "../inc/hw_prcm.h"
#include "../inc/hw_vims.h"

/* Driverlib headers */

#include "aon_wuc.h"
#include "aux_wuc.h"
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

static void TrimAfterColdReset(void);
static void TrimAfterColdResetWakeupFromShutDown(uint32_t fcfg1_revision);
static void TrimAfterColdResetWakeupFromShutDownWakeupFromPowerDown(void);

/******************************************************************************
 *
 * Perform the necessary trim of the device which is not done in boot code
 *
 * This function should only execute coming from ROM boot. The current
 * implementation does not take soft reset into account. However, it does no
 * damage to execute it again. It only consumes time.
 *
 ******************************************************************************/

void cc13x0_trim_device(void)
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

  /* This driverlib version and setup file is for CC13x0 PG2.0 and later. Halt
   * if violated
   */

  ThisLibraryIsFor_CC13x0_HwRev20AndLater_HaltIfViolated();

  /* Enable standby in flash bank */

  HWREGBITW(TIVA_FLASH_CFG, FLASH_CFG_DIS_STANDBY_BITN) = 0;

  /* Clock must always be enabled for the semaphore module (due to ADI/DDI HW
   * workaround)
   */

  HWREG(TIVA_AON_WUC_MODCLKEN1) = AUX_WUC_MODCLKEN1_SMPH;

  /* Warm resets on CC13x0 and CC26x0 complicates software design because much
   * of our software expect that initialization is done from a full system
   * reset. This includes RTC setup, oscillator configuration and AUX setup. To
   * ensure a full reset of the device is done when customers get e.g. a
   * Watchdog reset, the following is set here:
   */

  HWREGBITW(TIVA_PRCM_WARMRESET, PRCM_WARMRESET_WR_TO_PINRESET_BITN) =
    1;

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

  if (!(HWREGBITW(TIVA_AON_IOC_IOCLATCH, AON_IOC_IOCLATCH_EN_BITN)))
    {
      /* NB. This should be calling a ROM implementation of required trim and
       * compensation e.g.
       * TrimAfterColdResetWakeupFromShutDownWakeupFromPowerDown()
       */

      TrimAfterColdResetWakeupFromShutDownWakeupFromPowerDown();
    }

  /* Check for shutdown When device is going to shutdown the hardware will
   * automatically clear the SLEEPDIS bit in the SLEEP register in the
   * AON_SYSCTL module. It is left for the application to assert this bit when
   * waking back up, but not before the desired IO configuration has been
   * re-established.
   */

  else
    if (!
        (HWREGBITW
         (TIVA_AON_SYSCTL_SLEEPCTL,
          AON_SYSCTL_SLEEPCTL_IO_PAD_SLEEP_DIS_BITN)))
    {
      /* NB. This should be calling a ROM implementation of required trim and
       * compensation e.g. TrimAfterColdResetWakeupFromShutDown() -->
       * TrimAfterColdResetWakeupFromShutDownWakeupFromPowerDown();
       */

      TrimAfterColdResetWakeupFromShutDown(fcfg1_revision);
      TrimAfterColdResetWakeupFromShutDownWakeupFromPowerDown();
    }
  else
    {
      /* Consider adding a check for soft reset to allow debugging to skip this
       * section!!! NB. This should be calling a ROM implementation of
       * required trim and compensation e.g. TrimAfterColdReset() -->
       * TrimAfterColdResetWakeupFromShutDown() -->
       * TrimAfterColdResetWakeupFromShutDownWakeupFromPowerDown()
       */

      TrimAfterColdReset();
      TrimAfterColdResetWakeupFromShutDown(fcfg1_revision);
      TrimAfterColdResetWakeupFromShutDownWakeupFromPowerDown();

    }

  /* Set VIMS power domain control. PDCTL1VIMS = 0 ==> VIMS power domain is
   * only powered when CPU power domain is powered
   */

  putreg32(0, TIVA_PRCM_PDCTL1VIMS);

  /* Configure optimal wait time for flash FSM in cases where flash pump wakes
   * up from sleep
   */

  HWREG(TIVA_FLASH_FPAC1) = (getreg32(TIVA_FLASH_FPAC1) &
                                       ~FLASH_FPAC1_PSLEEPTDIS_M) |
    (0x139 << FLASH_FPAC1_PSLEEPTDIS_S);

  /* And finally at the end of the flash boot process: SET BOOT_DET bits in
   * AON_SYSCTL to 3 if already found to be 1 Note: The BOOT_DET_x_CLR/SET bits
   * must be manually cleared
   */

  if (((getreg32(TIVA_AON_SYSCTL_RESETCTL) &
        (AON_SYSCTL_RESETCTL_BOOT_DET_1_M | AON_SYSCTL_RESETCTL_BOOT_DET_0_M))
       >> AON_SYSCTL_RESETCTL_BOOT_DET_0_S) == 1)
    {
      aon_sysresetctrl = (getreg32(TIVA_AON_SYSCTL_RESETCTL) &
                            ~(AON_SYSCTL_RESETCTL_BOOT_DET_1_CLR_M |
                              AON_SYSCTL_RESETCTL_BOOT_DET_0_CLR_M |
                              AON_SYSCTL_RESETCTL_BOOT_DET_1_SET_M |
                              AON_SYSCTL_RESETCTL_BOOT_DET_0_SET_M));
      HWREG(TIVA_AON_SYSCTL_RESETCTL) =
        aon_sysresetctrl | AON_SYSCTL_RESETCTL_BOOT_DET_1_SET_M;
      HWREG(TIVA_AON_SYSCTL_RESETCTL) = aon_sysresetctrl;
    }

  /* Make sure there are no ongoing VIMS mode change when leaving
   * cc13x0_trim_device() (There should typically be no wait time here, but
   * need to be sure)
   */

  while (HWREGBITW(TIVA_VIMS_STAT, VIMS_STAT_MODE_CHANGING_BITN))
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

static void TrimAfterColdResetWakeupFromShutDownWakeupFromPowerDown(void)
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

static void TrimAfterColdResetWakeupFromShutDown(uint32_t fcfg1_revision)
{
  uint32_t ccfg_ModeConfReg;
  uint32_t mp1rev;

  /* Force AUX on and enable clocks No need to save the current status of the
   * power/clock registers. At this point both AUX and AON should have been
   * reset to 0x0.
   */

  HWREG(TIVA_AON_WUC_AUXCTL) = AON_WUC_AUXCTL_AUX_FORCE_ON;

  /* Wait for power on on the AUX domain */

  while (!
         (HWREGBITW
          (TIVA_AON_WUC_PWRSTAT, AON_WUC_PWRSTAT_AUX_PD_ON_BITN)));

  /* Enable the clocks for AUX_DDI0_OSC and AUX_ADI4 */

  HWREG(TIVA_AON_WUC_MODCLKEN0) = AUX_WUC_MODCLKEN0_AUX_DDI0_OSC |
    AUX_WUC_MODCLKEN0_AUX_ADI4;

  /* Check in CCFG for alternative DCDC setting */

  if ((getreg32(TIVA_CCFG_SIZE_AND_DIS_FLAGS) &
       CCFG_SIZE_AND_DIS_FLAGS_DIS_ALT_DCDC_SETTING) == 0)
    {
      /* ADI_3_REFSYS:DCDCCTL5[3] (=DITHER_EN) = CCFG_MODE_CONF_1[19]
       * (=ALT_DCDC_DITHER_EN) ADI_3_REFSYS:DCDCCTL5[2:0](=IPEAK ) =
       * CCFG_MODE_CONF_1[18:16](=ALT_DCDC_IPEAK ) Using a single 4-bit masked
       * write since layout is equal for both source and destination
       */

      HWREGB(TIVA_ADI3_MASK4B + (ADI_3_REFSYS_DCDCCTL5_OFFSET * 2)) = (0xf0 |
                                                                          (getreg32(TIVA_CCFG_MODE_CONF_1)
                                                                           >>
                                                                           CCFG_MODE_CONF_1_ALT_DCDC_IPEAK_S));

    }

  *Enable for JTAG
    to be powered down(will still be powered on if debugger is connected)
      */AONWUCJtagPowerOff();

  /* read the MODE_CONF register in CCFG */

  ccfg_ModeConfReg = getreg32(TIVA_CCFG_MODE_CONF);

  /* First part of trim done after cold reset and wakeup from shutdown:
   * -Configure cc13x0 boost mode. -Adjust the VDDR_TRIM_SLEEP value.
   * -Configure DCDC.
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

  /* Increased margin between digital supply voltage and VDD BOD during
   * standby. VTRIM_UDIG: signed 4 bits value to be incremented by 2 (max = 7)
   * VTRIM_BOD: unsigned 4 bits value to be decremented by 1 (min = 0) This
   * applies to chips with mp1rev < 542 for cc13x0 and for mp1rev < 527 for
   * cc26x0
   */

  mp1rev =
    ((getreg32(TIVA_FCFG1_TRIM_CAL_REVISION) &
      FCFG1_TRIM_CAL_REVISION_MP1_M) >> FCFG1_TRIM_CAL_REVISION_MP1_S);
  if (mp1rev < 542)
    {
      uint32_t ldoTrimReg = getreg32(TIVA_FCFG1_BAT_RC_LDO_TRIM);
      uint32_t vtrim_bod = ((ldoTrimReg & FCFG1_BAT_RC_LDO_TRIM_VTRIM_BOD_M) >> FCFG1_BAT_RC_LDO_TRIM_VTRIM_BOD_S);     /* bit[27:24]
                                                                                                                         * unsigned
                                                                                                                         */

      uint32_t vtrim_udig = ((ldoTrimReg & FCFG1_BAT_RC_LDO_TRIM_VTRIM_UDIG_M) >> FCFG1_BAT_RC_LDO_TRIM_VTRIM_UDIG_S);  /* bit[19:16]
                                                                                                                         * signed
                                                                                                                         * but
                                                                                                                         * treated
                                                                                                                         * as
                                                                                                                         * unsigned
                                                                                                                         */

      if (vtrim_bod > 0)
        {
          vtrim_bod -= 1;
        }
      if (vtrim_udig != 7)
        {
          if (vtrim_udig == 6)
            {
              vtrim_udig = 7;
            }
          else
            {
              vtrim_udig = ((vtrim_udig + 2) & 0xf);
            }
        }
      HWREGB(TIVA_ADI2_SOCLDOCTL0) =
        (vtrim_udig << ADI_2_REFSYS_SOCLDOCTL0_VTRIM_UDIG_S) |
        (vtrim_bod << ADI_2_REFSYS_SOCLDOCTL0_VTRIM_BOD_S);
    }

  /* Third part of trim done after cold reset and wakeup from shutdown:
   * -Configure HPOSC. -Setup the LF clock.
   */

#if ( CCFG_BASE == CCFG_BASE_DEFAULT )
  SetupAfterColdResetWakeupFromShutDownCfg3(ccfg_ModeConfReg);
#else
  NOROM_SetupAfterColdResetWakeupFromShutDownCfg3(ccfg_ModeConfReg);
#endif

  /* Allow AUX to power down */

  AUXWUCPowerCtrl(AUX_WUC_POWER_DOWN);

  /* Leaving on AUX and clock for AUX_DDI0_OSC on but turn off clock for
   * AUX_ADI4
   */

  HWREG(TIVA_AON_WUC_MODCLKEN0) = AUX_WUC_MODCLKEN0_AUX_DDI0_OSC;

  /* Disable EFUSE clock */

  HWREGBITW(TIVA_FLASH_CFG, FLASH_CFG_DIS_EFUSECLK_BITN) = 1;
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

static void TrimAfterColdReset(void)
{
  /* Currently no specific trim for Cold Reset */

}
