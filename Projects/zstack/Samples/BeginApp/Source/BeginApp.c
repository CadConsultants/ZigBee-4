/**************************************************************************************************
  Filename:       BeginApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Begin Application (no Profile).


  Copyright 2004-2009 Texas Instruments Incorporated. All rights reserved.

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
  PROVIDED ?AS IS? WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, 
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

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends "Hello World" to another "Begin"
  application every 15 seconds.  The application will also
  receive "Hello World" packets.

  The "Hello World" messages are sent/received as MSG type message.

  This applications doesn't have a profile, so it handles everything
  directly - itself.

  Key control:
    SW1:
    SW2:  initiates end device binding
    SW3:
    SW4:  initiates a match description request
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "AF.h"
#include "ZDApp.h"
#include "ZDObject.h"
#include "ZDProfile.h"

#include "BeginApp.h"
#include "DebugTrace.h"

#if !defined( WIN32 )
  #include "OnBoard.h"
#endif

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_IR.h"
#include "hal_key.h"
//#include "hal_uart.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t BeginApp_ClusterList[BEGINAPP_MAX_CLUSTERS] =
{
  BEGINAPP_CLUSTERID
};

const SimpleDescriptionFormat_t BeginApp_SimpleDesc =
{
  BEGINAPP_ENDPOINT,              //  int Endpoint;
  BEGINAPP_PROFID,                //  uint16 AppProfId[2];
  BEGINAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  BEGINAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  BEGINAPP_FLAGS,                 //  int   AppFlags:4;
  BEGINAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)BeginApp_ClusterList,  //  byte *pAppInClusterList;
  BEGINAPP_MAX_CLUSTERS,          //  byte  AppNumInClusters;
  (cId_t *)BeginApp_ClusterList   //  byte *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in BeginApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t BeginApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
byte BeginApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // BeginApp_Init() is called.
devStates_t BeginApp_NwkState;


byte BeginApp_TransID;  // This is the unique message ID (counter)

afAddrType_t BeginApp_DstAddr;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void BeginApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg );
void BeginApp_HandleKeys( byte shift, byte keys );
void BeginApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void BeginApp_SendTheMessage( void );

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      BeginApp_Init
 *
 * @brief   Initialization function for the Begin App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
uint8 T1_FLAG1 = 0x01;
int T1_CNT1 = 0;
uint16 IR_Encoding[100];
int IR_CNT = sizeof(IR_Encoding) / sizeof(IR_Encoding[0]);
void BeginApp_Init( byte task_id )
{
  BeginApp_TaskID = task_id;
  BeginApp_NwkState = DEV_INIT;
  BeginApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

  BeginApp_DstAddr.addrMode = (afAddrMode_t)AddrNotPresent;
  BeginApp_DstAddr.endPoint = 0;
  BeginApp_DstAddr.addr.shortAddr = 0;

  // Fill out the endpoint description.
  BeginApp_epDesc.endPoint = BEGINAPP_ENDPOINT;
  BeginApp_epDesc.task_id = &BeginApp_TaskID;
  BeginApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&BeginApp_SimpleDesc;
  BeginApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &BeginApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( BeginApp_TaskID );

  // Update the display
#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "BeginApp", HAL_LCD_LINE_1 );
#endif
    
  ZDO_RegisterForZDOMsg( BeginApp_TaskID, End_Device_Bind_rsp );
  ZDO_RegisterForZDOMsg( BeginApp_TaskID, Match_Desc_rsp );
  
    //PERCFG &= ~0x20;
    //P1SEL |= 0x10;
    //P2DIR = (P2DIR & ~0xc0) | 0x80; 

    //T3CCTL1 = 0x34;
    //T3CC1 = 70;
    //T3CC0 = 210;
    //T3CTL = (T3CTL & ~0xE0) | 0x40; //divide with 4
    //T3CTL = (T3CTL | 0x02) & ~0x01; //set mode modulo

    //T3CTL |= 0x10; //start T3

    ////T1 interrupt
    //TIMIF |= 0x40;
    //IRCON &= ~0x02;
    //EA = 1;
    //T1IE = 1;
    //
    //IRCTL |= 0x01;

    ////Timer1
    //PERCFG &= ~0x40;
    //P0SEL |= 0x08;

    //T1CCTL1 = 0x64;

    //T1CC1L = IR_Encoding[0];
    //T1CC0L = 47;

    //T1CC1H = 0;
    //T1CC0H = 0;

    //T1CCTL0 |= 0x44;

    //T1CTL = 0x02;
}


/*********************************************************************
 * @fn      BeginApp_ProcessEvent
 *
 * @brief   Begin Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
UINT16 BeginApp_ProcessEvent( byte task_id, UINT16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  afDataConfirm_t *afDataConfirm;

  // Data Confirmation message fields
  byte sentEP;
  ZStatus_t sentStatus;
  byte sentTransID;       // This should match the value sent
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( BeginApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        case ZDO_CB_MSG:
          BeginApp_ProcessZDOMsgs( (zdoIncomingMsg_t *)MSGpkt );
          break;
          
        case KEY_CHANGE:
          BeginApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        case AF_DATA_CONFIRM_CMD:
          // This message is received as a confirmation of a data packet sent.
          // The status is of ZStatus_t type [defined in ZComDef.h]
          // The message fields are defined in AF.h
          afDataConfirm = (afDataConfirm_t *)MSGpkt;
          sentEP = afDataConfirm->endpoint;
          sentStatus = afDataConfirm->hdr.status;
          sentTransID = afDataConfirm->transID;
          (void)sentEP;
          (void)sentTransID;

          // Action taken when confirmation is received.
          if ( sentStatus != ZSuccess )
          {
            // The data wasn't delivered -- Do something
          }
          break;

        case AF_INCOMING_MSG_CMD:
          BeginApp_MessageMSGCB( MSGpkt );
          break;

        case ZDO_STATE_CHANGE:
          BeginApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( (BeginApp_NwkState == DEV_ZB_COORD)
              || (BeginApp_NwkState == DEV_ROUTER)
              || (BeginApp_NwkState == DEV_END_DEVICE) )
          {
            HalLedSet( HAL_LED_3, HAL_LED_MODE_FLASH );  
            // Start sending "the" message in a regular interval.
            osal_start_timerEx( BeginApp_TaskID,
                                BEGINAPP_SEND_MSG_EVT,
                              BEGINAPP_SEND_MSG_TIMEOUT );
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( BeginApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in BeginApp_Init()).
  if ( events & BEGINAPP_SEND_MSG_EVT )
  {
    // Send "the" message
    BeginApp_SendTheMessage();

    // Setup to send message again
    osal_start_timerEx( BeginApp_TaskID,
                        BEGINAPP_SEND_MSG_EVT,
                      BEGINAPP_SEND_MSG_TIMEOUT );

    // return unprocessed events
    return (events ^ BEGINAPP_SEND_MSG_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */

/*********************************************************************
 * @fn      BeginApp_ProcessZDOMsgs()
 *
 * @brief   Process response messages
 *
 * @param   none
 *
 * @return  none
 */
void BeginApp_ProcessZDOMsgs( zdoIncomingMsg_t *inMsg )
{
  switch ( inMsg->clusterID )
  {
    case End_Device_Bind_rsp:
      if ( ZDO_ParseBindRsp( inMsg ) == ZSuccess )
      {
        // Light LED
        HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
      }
#if defined(BLINK_LEDS)
      else
      {
        // Flash LED to show failure
        HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
      }
#endif
      break;

    case Match_Desc_rsp:
      {
        ZDO_ActiveEndpointRsp_t *pRsp = ZDO_ParseEPListRsp( inMsg );
        if ( pRsp )
        {
          if ( pRsp->status == ZSuccess && pRsp->cnt )
          {
            BeginApp_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
            BeginApp_DstAddr.addr.shortAddr = pRsp->nwkAddr;
            // Take the first endpoint, Can be changed to search through endpoints
            BeginApp_DstAddr.endPoint = pRsp->epList[0];

            // Light LED
            HalLedSet( HAL_LED_4, HAL_LED_MODE_ON );
          }
          osal_mem_free( pRsp );
        }
      }
      break;
  }
}

/*********************************************************************
 * @fn      BeginApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_4
 *                 HAL_KEY_SW_3
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void BeginApp_HandleKeys( byte shift, byte keys )
{
  zAddrType_t dstAddr;
  
  // Shift is used to make each button/switch dual purpose.
  if ( shift )
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }
    if ( keys & HAL_KEY_SW_2 )
    {
    }
    if ( keys & HAL_KEY_SW_3 )
    {
    }
    if ( keys & HAL_KEY_SW_4 )
    {
    }
  }
  else
  {
    if ( keys & HAL_KEY_SW_1 )
    {
    }

    if ( keys & HAL_KEY_SW_2 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );

      // Initiate an End Device Bind Request for the mandatory endpoint
      dstAddr.addrMode = Addr16Bit;
      dstAddr.addr.shortAddr = 0x0000; // Coordinator
      ZDP_EndDeviceBindReq( &dstAddr, NLME_GetShortAddr(), 
                            BeginApp_epDesc.endPoint,
                            BEGINAPP_PROFID,
                            BEGINAPP_MAX_CLUSTERS, (cId_t *)BeginApp_ClusterList,
                            BEGINAPP_MAX_CLUSTERS, (cId_t *)BeginApp_ClusterList,
                            FALSE );
    }

    if ( keys & HAL_KEY_SW_3 )
    {
    }

    if ( keys & HAL_KEY_SW_4 )
    {
      HalLedSet ( HAL_LED_4, HAL_LED_MODE_OFF );
      // Initiate a Match Description Request (Service Discovery)
      dstAddr.addrMode = AddrBroadcast;
      dstAddr.addr.shortAddr = NWK_BROADCAST_SHORTADDR;
      ZDP_MatchDescReq( &dstAddr, NWK_BROADCAST_SHORTADDR,
                        BEGINAPP_PROFID,
                        BEGINAPP_MAX_CLUSTERS, (cId_t *)BeginApp_ClusterList,
                        BEGINAPP_MAX_CLUSTERS, (cId_t *)BeginApp_ClusterList,
                        FALSE );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      BeginApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void BeginApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  switch ( pkt->clusterId )
  {
    case BEGINAPP_CLUSTERID:
      // "the" message
#if defined( LCD_SUPPORTED )
      HalLcdWriteScreen( (char*)pkt->cmd.Data, "rcvd" );
#elif defined( WIN32 )
      WPRINTSTR( pkt->cmd.Data );
#endif
      break;
  }
}

/*********************************************************************
 * @fn      BeginApp_SendTheMessage
 *
 * @brief   Send "the" message.
 *
 * @param   none
 *
 * @return  none
 */
void BeginApp_SendTheMessage( void )
{
  int a=BeginApp_TransID%5;
  char theMessage[5][30] = {"Hello World","iniil","asfweofj","9jfjf0","jowef98383"};

  if ( AF_DataRequest( &BeginApp_DstAddr, &BeginApp_epDesc,
                       BEGINAPP_CLUSTERID,
                       (byte)osal_strlen( theMessage[a] ) + 1,
                       (byte *)&theMessage[a],
                       &BeginApp_TransID,
                       AF_DISCV_ROUTE, AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
    // Successfully requested to be sent.
  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
*********************************************************************/
