;*****************************  APP_RFSETUP.A51  ****************************
;*           #######
;*           ##  ##
;*           #  ##    ####   #####    #####  ##  ##   #####
;*             ##    ##  ##  ##  ##  ##      ##  ##  ##
;*            ##  #  ######  ##  ##   ####   ##  ##   ####
;*           ##  ##  ##      ##  ##      ##   #####      ##
;*          #######   ####   ##  ##  #####       ##  #####
;*                                           #####
;*          Z-Wave, the wireless language.
;*
;*              Copyright (c) 2001
;*              Zensys A/S
;*              Denmark
;*
;*              All Rights Reserved
;*
;*    This source file is subject to the terms and conditions of the
;*    Zensys Software License Agreement which restricts the manner
;*    in which it may be used.
;*
;*---------------------------------------------------------------------------
;*
;* Description: Application ZW010x RF setting setup table
;*
;* Author:   Johann Sigfredsson
;*
;* Last Changed By:  $Author: efh $
;* Revision:         $Revision: 9763 $
;* Last Changed:     $Date: 2008-01-10 11:28:42 +0100 (Thu, 10 Jan 2008) $
;*
;****************************************************************************
#ifdef ZW020x
#include <ZW_RF020x.h>
#endif
#ifdef ZW030x
#include <ZW_RF030x.h>
#endif



;****************************************************************************
;*  WARNING:
;****************************************************************************
        NAME  ZW_APP_RF_TABLE

        CSEG  AT FLASH_APPL_TABLE_OFFSET

        ORG   FLASH_APPL_TABLE_OFFSET + FLASH_APPL_MAGIC_VALUE_OFFS
        DB    RF_MAGIC_VALUE

; Which RF frequency do we want to use, EU (868.42MHz), US (908.42MHz), ANZ (921.42MHz), HK (919.82MHz), MY (868.2MHz) or IN (865.4MHz) ?
        ORG   FLASH_APPL_TABLE_OFFSET + FLASH_APPL_FREQ_OFFS
#ifdef LIBDEFAULTS
        DB    0xFF
#else
#if   defined(US)
        DB    RF_US                ; US frequency (908.42MHz)
#elif defined(EU)
        DB    RF_EU                ; EU frequency (868.42MHz)
#elif defined(ANZ)
        DB    RF_ANZ               ; Australia/NewZealand frequency (921.42MHz)
#elif defined(HK)
        DB    RF_HK                ; Hongkong frequency (919.82MHz)
#elif defined(MY)
        DB    RF_MY                ; Malaysia frequency (868.2MHz)
#elif defined(IN)
        DB    RF_IN                ; India frequency (865.4MHz)
#elif defined(RU)
        DB    RF_RU                ; Russian Federation frequency (869.2MHz)
#else
        DB    RF_IL                ; Israel frequency (916.0MHz)
#endif /* MY */
#endif

#ifdef ZW010x
; What should the capacity match array register contain when in RX mode
        ORG   FLASH_APPL_TABLE_OFFSET + FLASH_APPL_RX_MATCH_OFFS
        DB    APP_DEFAULT_RX_MATCH

; What should the capacity match array register contain when in TX mode
        ORG   FLASH_APPL_TABLE_OFFSET + FLASH_APPL_TX_MATCH_OFFS
        DB    APP_DEFAULT_TX_MATCH
#endif


; What should the power level be when in normal power TX mode
        ORG   FLASH_APPL_TABLE_OFFSET + FLASH_APPL_NORM_POWER_OFFS
        DB    APP_DEFAULT_NORM_POWER

; What should the power level be when in low power TX mode
        ORG   FLASH_APPL_TABLE_OFFSET + FLASH_APPL_LOW_POWER_OFFS
        DB    APP_DEFAULT_LOW_POWER

#if defined(ZW020x) || defined(ZW030x)
; What should the PLL-stepup value be when in TX mode (change it to zero when using external PA)
        ORG   FLASH_APPL_TABLE_OFFSET + FLASH_APPL_PLL_STEPUP_OFFS
        DB    APP_DEFAULT_PLL_STEPUP
#endif

        END
