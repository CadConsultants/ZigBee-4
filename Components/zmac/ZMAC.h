/**************************************************************************************************
  Filename:       ZMAC.h
  Revised:        $Date: 2009-03-10 08:35:17 -0700 (Tue, 10 Mar 2009) $
  Revision:       $Revision: 19360 $

  Description:    This file contains the ZStack MAC Porting Layer.


  Copyright 2004-2007 Texas Instruments Incorporated. All rights reserved.

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

#ifndef ZMAC_H
#define ZMAC_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */

#include "ZComDef.h"
#include "zmac_internal.h"

/*********************************************************************
 * MACROS
 */

/* Maximum length of the beacon payload */
#ifndef ZMAC_MAX_BEACON_PAYLOAD_LEN
  #define ZMAC_MAX_BEACON_PAYLOAD_LEN    (7 + Z_EXTADDR_LEN)
#endif

/*********************************************************************
 * CONSTANTS
 */

#if defined( MAC_API_H )
 #define ZMAC_CHAN_MASK ( \
   MAC_CHAN_11_MASK | \
   MAC_CHAN_12_MASK | \
   MAC_CHAN_13_MASK | \
   MAC_CHAN_14_MASK | \
   MAC_CHAN_15_MASK | \
   MAC_CHAN_16_MASK | \
   MAC_CHAN_17_MASK | \
   MAC_CHAN_18_MASK | \
   MAC_CHAN_19_MASK | \
   MAC_CHAN_20_MASK | \
   MAC_CHAN_21_MASK | \
   MAC_CHAN_22_MASK | \
   MAC_CHAN_23_MASK | \
   MAC_CHAN_24_MASK | \
   MAC_CHAN_25_MASK | \
   MAC_CHAN_26_MASK | \
   MAC_CHAN_27_MASK | \
   MAC_CHAN_28_MASK )
#else
 #define ZMAC_CHAN_MASK  0x07FFF800
#endif

/*********************************************************************
 * TYPEDEFS
 */

/* ZMAC event header type */
typedef struct
{
  uint8   Event;              /* ZMAC event */
  uint8   Status;             /* ZMAC status */
} ZMacEventHdr_t;

/* Common security type */
typedef struct
{
  uint8 KeySource[ZMAC_KEY_SOURCE_MAX_LEN];
  uint8 SecurityLevel;
  uint8 KeyIdMode;
  uint8 KeyIndex;
}ZMacSec_t;

/* PAN descriptor type */
typedef struct
{
  zAddrType_t   CoordAddress;
  uint16        CoordPANId;
  uint16        SuperframeSpec;
  uint8         LogicalChannel;
  uint8         ChannelPage;
  uint8         GTSPermit;
  uint8         LinkQuality;
  uint32        TimeStamp;
  uint8         SecurityFailure;
  ZMacSec_t     Sec;
} ZMacPanDesc_t;

/* Communication status indication type */
typedef struct
{
  ZMacEventHdr_t hdr;
  zAddrType_t    SrcAddress;
  zAddrType_t    DstAddress;
  uint16         PANId;
  uint8          Reason;
  ZMacSec_t      Sec;
} ZMacCommStatusInd_t;

/* SYNC */

typedef struct
{
  uint8 LogicalChannel;     /* The logical channel to use */
  uint8 ChannelPage;        /* The channel page to use */
  uint8 TrackBeacon;        /* Set to TRUE to continue tracking beacons after synchronizing with the
                               first beacon.  Set to FALSE to only synchronize with the first beacon */
}ZMacSyncReq_t;

/* DATA TYPES */

/* Data request parameters type */
typedef struct
{
  zAddrType_t   DstAddr;
  uint16        DstPANId;
  uint8         SrcAddrMode;
  uint8         Handle;
  uint8         TxOptions;
  uint8         Channel;
  uint8         Power;
  ZMacSec_t     Sec;
  uint8         msduLength;
  uint8        *msdu;
} ZMacDataReq_t;

/* Data confirm type */
typedef struct
{
  ZMacEventHdr_t hdr;
  uint8          msduHandle;
  ZMacDataReq_t  *pDataReq;
  uint32         Timestamp;
  uint16         Timestamp2;
  uint8          retries;
  uint8          mpduLinkQuality;
  uint8          correlation;
  int8           rssi;
} ZMacDataCnf_t;


/* ASSOCIATION TYPES */

/* Associate request type */
typedef struct
{
  uint8         LogicalChannel;
  uint8         ChannelPage;
  zAddrType_t   CoordAddress;
  uint16        CoordPANId;
  uint8         CapabilityInformation;
  ZMacSec_t     Sec;
} ZMacAssociateReq_t;

/* Associate response type */
typedef struct
{
  ZLongAddr_t		 DeviceAddress;
  uint16			   AssocShortAddress;
  uint8          Status;
  ZMacSec_t      Sec;
} ZMacAssociateRsp_t;

/* Associate indication parameters type */
typedef struct
{
  ZMacEventHdr_t hdr;
  ZLongAddr_t    DeviceAddress;
  uint8          CapabilityInformation;
  ZMacSec_t      Sec;
} ZMacAssociateInd_t;

/* Associate confim type */
typedef struct
{
  ZMacEventHdr_t hdr;
  uint16         AssocShortAddress;
  ZMacSec_t      Sec;
} ZMacAssociateCnf_t;

/* Disassociate request type */
typedef struct
{
  zAddrType_t		DeviceAddress;
  uint16        DevicePanId;
  uint8         DisassociateReason;
  uint8         TxIndirect;
  ZMacSec_t     Sec;
} ZMacDisassociateReq_t;

/* Rx enable confirm type */
typedef struct
{
  ZMacEventHdr_t hdr;
} ZMacRxEnableCnf_t;

/* SCAN */
/* Scan request type */
typedef struct
{
  uint32         ScanChannels;
  uint8          ScanType;			
  uint8          ScanDuration;
  uint8          ChannelPage;
  uint8          MaxResults;
  ZMacSec_t      Sec;
  union
  {
    uint8        *pEnergyDetect;
    ZMacPanDesc_t *pPanDescriptor;
  }Result;
} ZMacScanReq_t;

/* Scan confirm type */
typedef struct
{
  ZMacEventHdr_t hdr;
  uint8          ScanType;
  uint8          ChannelPage;
  uint32		     UnscannedChannels;
  uint8          ResultListSize;
  union
  {
    uint8         *pEnergyDetect;
    ZMacPanDesc_t *pPanDescriptor;
  }Result;
} ZMacScanCnf_t;


/* START */
/* Start request type */
typedef struct
{
  uint32        StartTime;
  uint16        PANID;
  uint8         LogicalChannel;
  uint8         ChannelPage;
  uint8         BeaconOrder;
  uint8         SuperframeOrder;
  uint8         PANCoordinator;
  uint8         BatteryLifeExt;
  uint8         CoordRealignment;
  ZMacSec_t     RealignSec;
  ZMacSec_t     BeaconSec;
} ZMacStartReq_t;

/* Start confirm type */
typedef struct
{
  ZMacEventHdr_t hdr;
} ZMacStartCnf_t;

/* POLL */
/* Roll request type */
typedef struct
{
  zAddrType_t CoordAddress;
  uint16      CoordPanId;
  ZMacSec_t   Sec;
} ZMacPollReq_t;

/* Poll confirm type */
typedef struct
{
  ZMacEventHdr_t hdr;
} ZMacPollCnf_t;

/* MAC_MLME_POLL_IND type */
typedef struct
{
  ZMacEventHdr_t  hdr;
  uint16          srcShortAddr;   /* Short address of the device sending the data request */
  uint16          srcPanId;       /* Pan ID of the device sending the data request */
} ZMacPollInd_t;

/* ORPHAN */
/* Orphan response type */
typedef struct
{
  ZLongAddr_t    OrphanAddress;
  uint16         ShortAddress;
  uint8          AssociatedMember;
  ZMacSec_t      Sec;
} ZMacOrphanRsp_t;

/* Orphan indication type */
typedef struct
{
  ZMacEventHdr_t hdr;
  ZLongAddr_t    OrphanAddress;
  ZMacSec_t      Sec;
} ZMacOrphanInd_t;

#if defined (MT_MAC_FUNC) || defined (MT_MAC_CB_FUNC)

/* Sync loss indication type */
typedef struct
{
  ZMacEventHdr_t hdr;
  uint16         PANId;
  uint8          LogicalChannel;
  uint8          ChannelPage;
  ZMacSec_t      Sec;
} ZMacSyncLossInd_t;

/* Data indication parameters type */
typedef struct
{
  ZMacEventHdr_t hdr;
  zAddrType_t    SrcAddr;
  zAddrType_t    DstAddr;
  uint32         Timestamp;
  uint16         Timestamp2;
  uint16         SrcPANId;
  uint16         DstPANId;
  uint8          mpduLinkQuality;
  uint8          Correlation;
  uint8          Rssi;
  uint8          Dsn;
  ZMacSec_t      Sec;
  uint8          msduLength;
  uint8         *msdu;
} ZMacDataInd_t;

/* Disassociate indication type */
typedef struct
{
  ZMacEventHdr_t hdr;
  ZLongAddr_t    DeviceAddress;
  uint8          DisassociateReason;
  ZMacSec_t      Sec;
} ZMacDisassociateInd_t;

/* Disassociate confirm type */
typedef struct
{
  ZMacEventHdr_t hdr;
  zAddrType_t    DeviceAddress;
  uint16         panID;
} ZMacDisassociateCnf_t;

/* Beacon notify indication type */
typedef struct
{
  ZMacEventHdr_t hdr;
  uint8          BSN;
  ZMacPanDesc_t *pPanDesc;
  uint8          PendAddrSpec;
  uint8         *AddrList;
  uint8          sduLength;
  uint8         *sdu;
} ZMacBeaconNotifyInd_t;

/* Purge confirm type */
typedef struct
{
  ZMacEventHdr_t hdr;
  uint8          msduHandle;
} ZMacPurgeCnf_t;
#endif

typedef enum
{
  TX_PWR_MAX,
  TX_PWR_MINUS_1,
  TX_PWR_MINUS_2,
  TX_PWR_MINUS_3,
  TX_PWR_MINUS_4,
  TX_PWR_MINUS_5,
  TX_PWR_MINUS_6,
  TX_PWR_MINUS_7,
  TX_PWR_MINUS_8,
  TX_PWR_MINUS_9,
  TX_PWR_MINUS_10,
  TX_PWR_MINUS_11,
  TX_PWR_MINUS_12,
  TX_PWR_MINUS_13,
  TX_PWR_MINUS_14,
  TX_PWR_MINUS_15
} ZMacTransmitPower_t;

typedef struct
{
  byte protocolID;
  byte stackProfile;    // 4 bit in native
  byte protocolVersion; // 4 bit in native
  byte reserved;        // 2 bit in native
  byte routerCapacity;  // 1 bit in native
  byte deviceDepth;     // 4 bit in native
  byte deviceCapacity;  // 1 bit in native
  byte extendedPANID[Z_EXTADDR_LEN];
  byte txOffset[3];
  byte updateId;
} beaconPayload_t;

/*********************************************************************
 * GLOBAL VARIABLES
 */
#define NWK_CMD_ID_LEN sizeof( byte )

/*********************************************************************
 * FUNCTIONS
 */

  /*
   * Initialize.
   */
  extern ZMacStatus_t ZMacInit( void );

  /*
   * Send a MAC Data Frame packet.
   */
  extern ZMacStatus_t ZMacDataReq( ZMacDataReq_t *param );

  /*
   * Request an association with a coordinator.
   */
  extern ZMacStatus_t ZMacAssociateReq( ZMacAssociateReq_t *param );

  /*
   * Request to send an association response message.
   */
  extern ZMacStatus_t ZMacAssociateRsp( ZMacAssociateRsp_t *param );

  /*
   * Request to send a disassociate request message.
   */
  extern ZMacStatus_t ZMacDisassociateReq( ZMacDisassociateReq_t *param );

  /*
   * Gives the MAC extra processing time.
   * Returns false if its OK to sleep.
   */
  extern byte ZMacUpdate( void );

  /*
   * Read a MAC PIB attribute.
   */
  extern ZMacStatus_t ZMacGetReq( ZMacAttributes_t attr, byte *value );

  /*
   * This function allows the next higher layer to respond to
   * an orphan indication message.
   */
  extern ZMacStatus_t ZMacOrphanRsp( ZMacOrphanRsp_t *param );

  /*
   * This function is called to request MAC data request poll.
   */
  extern ZMacStatus_t ZMacPollReq( ZMacPollReq_t *param );

  /*
   * Reset the MAC.
   */
  extern ZMacStatus_t ZMacReset( byte SetDefaultPIB );

  /*
   * This function is called to perform a network scan.
   */
  extern ZMacStatus_t ZMacScanReq( ZMacScanReq_t *param );

  /*
   * Write a MAC PIB attribute.
   */
  extern ZMacStatus_t ZMacSetReq( ZMacAttributes_t attr, byte *value );

  /*
   * This function is called to tell the MAC to transmit beacons
   * and become a coordinator.
   */
  extern ZMacStatus_t ZMacStartReq( ZMacStartReq_t *param );

  /*
   * This function is called to request a sync to the current
   * networks beacons.
   */
  extern ZMacStatus_t ZMacSyncReq( ZMacSyncReq_t *param );

  /*
   * This function requests to reset mac state machine and
   * transaction.
   */
  extern ZMacStatus_t ZMacCleanReq( void );

  /*
   * This function is called to request MAC to purge a message.
   */
  extern ZMacStatus_t ZMacPurgeReq( byte msduHandle );

  /*
   * This function is called to enable AUTOPEND and source address matching.
   */
  extern ZMacStatus_t ZMacSrcMatchEnable (uint8 addrType, uint8 numEntries);

 /*
  * This function is called to add a short or extended address to source address table.
  */
  extern ZMacStatus_t ZMacSrcMatchAddEntry (zAddrType_t *addr, uint16 panID);

  /*
   * This function is called to delete a short or extended address from source address table.
   */
  extern ZMacStatus_t ZMacSrcMatchDeleteEntry (zAddrType_t *addr, uint16 panID);

  /*
   * This funciton is called to enabled/disable acknowledging all packets with pending bit set
   */
  extern ZMacStatus_t ZMacSrcMatchAckAllPending (uint8 option);

  /*
   * This function is called to check if acknowledging all packets with pending bit set is enabled.
   */
  extern ZMacStatus_t ZMacSrcMatchCheckAllPending (void);

  /*
   * This function is called to request MAC to power on the radio hardware and wake up.
   */
  extern void ZMacPwrOnReq ( void );

  /*
   * This function returns the current power mode of the MAC.
   */
  extern uint8 ZMac_PwrMode(void);

  /*
   * This function is called to request MAC to set the transmit power level.
   */
  extern ZMacStatus_t ZMacSetTransmitPower( ZMacTransmitPower_t level );

  /*
   * This function is called to send out an empty msg
   */
  extern void ZMacSendNoData( uint16 DstAddr, uint16 DstPANId );

  /*
   * This callback function is called for every MAC message that is received
   * over-the-air or generated locally by MAC for the application.
   */
  extern uint8 (*pZMac_AppCallback)( uint8 *msgPtr );

  /*
   * This function returns true if the MAC state is idle.
   */
  extern uint8 ZMacStateIdle( void );

/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* ZMAC_H */
