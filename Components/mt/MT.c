/***************************************************************************************************
  Filename:       MTEL.c
  Revised:        $Date: 2008-10-10 09:56:24 -0700 (Fri, 10 Oct 2008) $
  Revision:       $Revision: 18257 $

  Description:    MonitorTest Event Loop functions.  Everything in the
                  MonitorTest Task (except the serial driver).

  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

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

 ***************************************************************************************************/


/***************************************************************************************************
 * INCLUDES
 ***************************************************************************************************/
#include "ZComDef.h"
#include "MT.h"
#include "MT_APP.h"
#include "MT_DEBUG.h"
#include "MT_UTIL.h"
#include "MT_SYS.h"
#include "MT_SAPI.h"
#include "AF.h"

#include "OnBoard.h"
#include "OSAL.h"
#include "OSAL_Memory.h"
#include "OSAL_Nv.h"

#include "DebugTrace.h"
#include "ZMAC.h"

#if !defined ( NONWK )
  #include "NLMEDE.h"
  #include "nwk_bufs.h"
  #include "ZDObject.h"
  #include "ssp.h"
  #include "nwk_util.h"
#endif

#if defined( MT_MAC_FUNC ) || defined( MT_MAC_CB_FUNC )
  #include "MT_MAC.h"
#endif
#if defined( MT_NWK_FUNC ) || defined( MT_NWK_CB_FUNC )
  #include "MT_NWK.h"
  #include "nwk.h"
  #include "nwk_bufs.h"
#endif
#if defined( MT_AF_FUNC ) || defined( MT_AF_CB_FUNC )
  #include "MT_AF.h"
#endif
#if defined( MT_USER_TEST_FUNC )
  #include "AF.h"
#endif
#if defined( MT_ZDO_FUNC )
  #include "MT_ZDO.h"
#endif
#if defined (MT_SAPI_FUNC)
	#include "MT_SAPI.h"
#endif
#if defined( APP_TP )
 #include "TestProfile.h"
#endif
#if defined( APP_TP2 )
 #include "TestProfile2.h"
#endif

#if defined(APP_TGEN)
  #include "TrafficGenApp.h"
#endif
#if defined(APP_DEBUG)
	#include "DebugApp.h"
#endif
#if defined (NWK_TEST)
	#include "HWTTApp.h"
#endif

/* HAL */
#include "hal_uart.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"

/***************************************************************************************************
 * MACROS
 ***************************************************************************************************/
#define MTEL_DEBUG_INFO( nParams, p1, p2, p3 ) DEBUG_INFO( COMPID_MTEL, nParams, p1, p2, p3 )

#define MT_ERROR_SRSP_LEN   3

/***************************************************************************************************
 * CONSTANTS
 ***************************************************************************************************/
mtProcessMsg_t mtProcessIncoming[] =
{
  NULL,

#if defined (MT_SYS_FUNC)
  MT_SysCommandProcessing,
#else
  NULL,
#endif

#if defined (MT_MAC_FUNC)
  MT_MacCommandProcessing,
#else
  NULL,
#endif

#if defined (MT_NWK_FUNC)
  MT_NwkCommandProcessing,
#else
  NULL,
#endif

#if defined (MT_AF_FUNC)
  MT_AfCommandProcessing,
#else
  NULL,
#endif

#if defined (MT_ZDO_FUNC)
  MT_ZdoCommandProcessing,
#else
  NULL,
#endif

#if defined (MT_SAPI_FUNC)
  MT_SapiCommandProcessing,
#else
  NULL,
#endif

#if defined (MT_UTIL_FUNC)
  MT_UtilCommandProcessing,
#else
  NULL,
#endif

#if defined (MT_DEBUG_FUNC)
  MT_DebugCommandProcessing,
#else
  NULL,
#endif

#if defined (MT_APP_FUNC)
  MT_AppCommandProcessing,
#else
  NULL,
#endif

};

/***************************************************************************************************
 * TYPEDEFS
 ***************************************************************************************************/

/***************************************************************************************************
 * GLOBAL VARIABLES
 ***************************************************************************************************/
UINT16 save_cmd;

byte MT_TaskID;
byte debugThreshold;
byte debugCompId;

/***************************************************************************************************
 * EXTERNAL FUNCTIONS
 ***************************************************************************************************/
extern unsigned int mac_sim_eventLoop( void );

#ifdef MACSIM
extern void MACSIM_TranslateMsg( byte *buf, byte bLen );  /*  Used to pass Zignet message */
#endif


/***************************************************************************************************
 * LOCAL VARIABLES
 ***************************************************************************************************/

/***************************************************************************************************
 * LOCAL FUNCTIONS
 ***************************************************************************************************/
void MT_MsgQueueInit( void );
void MT_ResetMsgQueue( void );
byte MT_QueueMsg( byte *msg , byte len );
void MT_ProcessQueue( void );

#if defined ( MT_USER_TEST_FUNC )
void MT_ProcessAppUserCmd( byte *pData );
#endif

/***************************************************************************************************
 * @fn      MT_Init()
 *
 * @brief   Initialize MT.
 *
 * @param   uint8 taskId - taskId
 *
 * @return  void
 ***************************************************************************************************/
void MT_Init(uint8 taskID)
{
  MT_TaskID = taskID;
  debugThreshold = 0;
  debugCompId = 0;
}

/***************************************************************************************************
 * @fn      MT_BuildSPIMsg
 *
 * @brief
 *
 *   Format an SPI message.
 *
 * @param   UINT16 cmd - command id
 * @param   byte *msg - pointer to message buffer
 * @param   byte dataLen - length of data field
 * @param   byte *pData - pointer to data field
 *
 * @return  void
 ***************************************************************************************************/
void MT_BuildSPIMsg( uint8 cmdType, uint8 cmdId, byte *msg, byte dataLen, byte *pData )
{
  /* Fill in the CMD and Data Length */
  *msg++ = dataLen;
  *msg++ = cmdType;
  *msg++ = cmdId;

  /* Fill in the data */
  if ( pData )
  {
    osal_memcpy( msg, pData, dataLen );
  }
}

/***************************************************************************************************
 * @fn      MT_BuildAndSendZToolResponse
 *
 * @brief   Build and send a ZTOOL msg
 * @param   uint8 cmdType - include type and subsystem
 *          uint8 cmdId - command ID
 *          byte dataLen
 *          byte *pData
 *
 * @return  void
 ***************************************************************************************************/
void MT_BuildAndSendZToolResponse(uint8 cmdType, uint8 cmdId, uint8 dataLen, uint8 *pData)
{
  uint8 *msg_ptr;

  /* Allocate memory including SOP and FCS */
  msg_ptr = MT_TransportAlloc((mtRpcCmdType_t)(cmdType & 0xE0), dataLen);

  if (msg_ptr)
  {
    /* Build the message */
    MT_BuildSPIMsg(cmdType, cmdId, msg_ptr, dataLen, pData);
    /* Send out the msg */
    MT_TransportSend(msg_ptr);
  }
}

/***************************************************************************************************
 * @fn      MT_ProcessIncoming
 *
 * @brief  Process Incoming Message.
 *
 * @param   byte *pBuf - pointer to event message
 *
 * @return  void
 ***************************************************************************************************/
void MT_ProcessIncoming(uint8 *pBuf)
{
  mtProcessMsg_t  func;
  uint8           rsp[MT_ERROR_SRSP_LEN];

  /* pre-build response message:  | status | cmd0 | cmd1 | */
  rsp[1] = pBuf[MT_RPC_POS_CMD0];
  rsp[2] = pBuf[MT_RPC_POS_CMD1];

  /* check length */
  if (pBuf[MT_RPC_POS_LEN] > MT_RPC_DATA_MAX)
  {
    rsp[0] = MT_RPC_ERR_LENGTH;
  }
  /* check subsystem range */
  else if ((rsp[1] & MT_RPC_SUBSYSTEM_MASK) < MT_RPC_SYS_MAX)
  {
    /* look up processing function */
    func = mtProcessIncoming[rsp[1] & MT_RPC_SUBSYSTEM_MASK];
    if (func)
    {
      /* execute processing function */
      rsp[0] = (*func)(pBuf);
    }
    else
    {
      rsp[0] = MT_RPC_ERR_SUBSYSTEM;
    }
  }
  else
  {
    rsp[0] = MT_RPC_ERR_SUBSYSTEM;
  }

  /* if error and this was an SREQ, send error message */
  if ((rsp[0] != MT_RPC_SUCCESS) && ((rsp[1] & MT_RPC_CMD_TYPE_MASK) == MT_RPC_CMD_SREQ))
  {
    MT_BuildAndSendZToolResponse(((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_RES0), 0, MT_ERROR_SRSP_LEN, rsp);
  }
}


/***************************************************************************************************
 * @fn      MTProcessAppRspMsg
 *
 * @brief   Process the User App Response Message
 *
 * @param   data - output serial buffer.  The first byte must be the
 *          endpoint that send this message.
 * @param   len - data length
 *
 * @return  none
 ***************************************************************************************************/
void MTProcessAppRspMsg( byte *pData, byte len )
{
  /* Send out Reset Response message */
  MT_BuildAndSendZToolResponse( ((uint8)MT_RPC_CMD_SRSP | (uint8)MT_RPC_SYS_APP), MT_APP_RSP, len, pData );
}


/***************************************************************************************************
 * @fn      MT_ReverseBytes
 *
 * @brief
 *
 *   Reverses bytes within an array
 *
 * @param   data - ptr to data buffer to reverse
 * @param    len - number of bytes in buffer
 *
 * @return  void
 ***************************************************************************************************/
void MT_ReverseBytes( byte *pData, byte len )
{
  byte i,j;
  byte temp;

  for ( i = 0, j = len-1; len > 1; len-=2 )
  {
    temp = pData[i];
    pData[i++] = pData[j];
    pData[j--] = temp;
  }
}


/***************************************************************************************************
 * @fn      MT_Word2Buf
 *
 * @brief   Copy a uint16 array to a byte array, little endian.
 *
 * @param   pBuf - byte array
 * @param   pWord - uint16 array
 * @param   len - length of uint16 array
 *
 * @return  pointer to end of byte array
 ***************************************************************************************************/
uint8 *MT_Word2Buf( uint8 *pBuf, uint16 *pWord, uint8 len )
{
  while ( len-- > 0 )
  {
    *pBuf++ = LO_UINT16( *pWord );
    *pBuf++ = HI_UINT16( *pWord );
    pWord++;
  }

  return pBuf;
}
/***************************************************************************************************
 * @fn      MT_BuildEndpointDesc
 *
 * @brief   Build endpoint descriptor and simple descriptor structure from incoming buffer
 *
 * @param   pBuf - byte array
 *
 * @return  epDesc - pointer to the endpoint descriptor
 ***************************************************************************************************/
uint8 MT_BuildEndpointDesc( uint8 *pBuf, void *param )
{
  uint8 i;
  uint8 ret = ZSuccess;
  endPointDesc_t *epDesc;

  epDesc = (endPointDesc_t *)param;
  /* check if this endpoint is already registered */
  if ( afFindEndPointDesc( *pBuf ) != NULL )
  {
    ret = ZApsDuplicateEntry;
  }
  else if ( epDesc )
  {
    epDesc->endPoint = *pBuf;

    /* Ignore the latency reqs */
    epDesc->latencyReq = noLatencyReqs;

    /* allocate memory for the simple descriptor */
    epDesc->simpleDesc = (SimpleDescriptionFormat_t *) osal_mem_alloc(sizeof(SimpleDescriptionFormat_t));
    if (epDesc->simpleDesc)
    {
      /* Endpoint */
      epDesc->simpleDesc->EndPoint = *pBuf++;

      /* AppProfId */
      epDesc->simpleDesc->AppProfId = BUILD_UINT16(pBuf[0], pBuf[1]);
      pBuf += sizeof(uint16);

      /* AppDeviceId */
      epDesc->simpleDesc->AppDeviceId = BUILD_UINT16(pBuf[0],pBuf[1]);
      pBuf += sizeof(uint16);

      /* AppDevVer */
      epDesc->simpleDesc->AppDevVer = (*pBuf++) & AF_APP_DEV_VER_MASK ;

      /* LatencyReq */
      pBuf++;

      /* AppNumInClusters */
      epDesc->simpleDesc->AppNumInClusters = *pBuf++;
      if (epDesc->simpleDesc->AppNumInClusters)
      {
        epDesc->simpleDesc->pAppInClusterList = (uint16 *)
                  osal_mem_alloc((epDesc->simpleDesc->AppNumInClusters)*sizeof(uint16));
        if ( epDesc->simpleDesc->pAppInClusterList )
        {
          for (i=0; i<(epDesc->simpleDesc->AppNumInClusters); i++)
          {
            epDesc->simpleDesc->pAppInClusterList[i] = BUILD_UINT16(*pBuf, *(pBuf+1));
            pBuf += 2;
          }
        }
        else
        {
          ret = ZMemError;
        }
      }

      /* AppNumOutClusters */
      epDesc->simpleDesc->AppNumOutClusters = *pBuf++;
      if (epDesc->simpleDesc->AppNumOutClusters)
      {
        epDesc->simpleDesc->pAppOutClusterList = (uint16 *)
                          osal_mem_alloc((epDesc->simpleDesc->AppNumOutClusters)*sizeof(uint16));
        if (epDesc->simpleDesc->pAppOutClusterList)
        {
          for (i=0; i<(epDesc->simpleDesc->AppNumOutClusters); i++)
          {
            epDesc->simpleDesc->pAppOutClusterList[i] = BUILD_UINT16(*pBuf, *(pBuf+1));
            pBuf += 2;
          }
        }
        else
        {
          ret = ZMemError;
        }
      }

      /* if any list cannot be allocated...free all */
      if ( ret == ZMemError )
      {
        if (epDesc->simpleDesc->pAppInClusterList)
        {
          osal_mem_free(epDesc->simpleDesc->pAppInClusterList);
        }

        if (epDesc->simpleDesc->AppNumOutClusters)
        {
          osal_mem_free(epDesc->simpleDesc->pAppOutClusterList);
        }

        osal_mem_free(epDesc->simpleDesc);
      }
    }
    else
    {
      ret = ZMemError;
    }
  }

  return ret;
}
/***************************************************************************************************
***************************************************************************************************/
