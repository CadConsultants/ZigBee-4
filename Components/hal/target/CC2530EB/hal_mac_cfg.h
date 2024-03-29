/**************************************************************************************************
  Filename:       hal_mac_cfg.h
  Revised:        $Date: 2009-03-27 14:32:42 -0700 (Fri, 27 Mar 2009) $
  Revision:       $Revision: 19584 $

  Description:    Describe the purpose and contents of the file.


  Copyright 2007-2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED �AS IS� WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

#ifndef HAL_MAC_CFG_H
#define HAL_MAC_CFG_H

/*
 *   Board Configuration File for low-level MAC
 *  --------------------------------------------
 *   Manufacturer : Texas Instruments
 *   Part Number  : CC2530EB
 *   Processor    : Texas Instruments CC2530
 *
 */


/* ------------------------------------------------------------------------------------------------
 *                                  Board Specific Configuration
 * ------------------------------------------------------------------------------------------------
 */
#define HAL_MAC_RSSI_OFFSET                         -73   /* no units */
#if defined (HAL_PA_LNA) || defined (HAL_PA_LNA_CC2590)
#define HAL_MAC_RSSI_LNA_HGM_OFFSET                 -12   /* TBD: place holder */
#define HAL_MAC_RSSI_LNA_LGM_OFFSET                 -6    /* TBD: place holder */
#endif


/**************************************************************************************************
*/
#endif
