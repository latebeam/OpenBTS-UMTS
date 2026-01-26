/*
 * Copyright 2026 Late Beam
 *
 * This software is distributed under the terms of the GNU General Public
 * License version 3. See the COPYING and NOTICE files in the current
 * directory for licensing information.
 *
 * This use of this software may be subject to additional restrictions.
 * See the LEGAL file in the main directory for details.
 */

#ifndef __RANCONTROLMSG_H
#define __RANCONTROLMSG_H

#include <cstdint>

#pragma pack(push, 1)



enum eRanControlMsgType {
    eCellSetupIndMsg = 1,
    eRRCMsg,
    eRRCConnectionReleaseMsg,
    eRRCConnectionAcceptMsg,
    eRRCConnectionRejectMsg,
    eRttMeasMsg,
    eRRCConnectionReleaseCcchMsg,
    eCpichTxPowerMsg
};

struct sRanControlMsgHeaderType {
    uint32_t msgLen;
    uint32_t magicNr; // identify interface
    uint32_t msgType;
};

struct sCellSetupIndMsgType {
    sRanControlMsgHeaderType sRanControlMsgHeader;
    uint8_t cellIdLen;
    uint8_t cellId[0];
};

#define UE_ID_TYPE_URNTI 1
#define UE_ID_TYPE_CRNTI 2

// from 25.331:
#define RRC_PDU_NUM_DL_DCCH 1 
#define RRC_PDU_NUM_UL_DCCH 2
#define RRC_PDU_NUM_DL_CCCH 3
#define RRC_PDU_NUM_UL_CCCH 4

struct sRRCMsgType {
    sRanControlMsgHeaderType sRanControlMsgHeader;
    uint8_t ueIdType;
    uint32_t URNTI;
    uint16_t CRNTI;
    uint32_t rrcPduNum;
//    uint32_t rbId;
    uint32_t rrcMsgBitLen;
    uint32_t rrcMsgLen;
    uint8_t  rrcMsgData[0];

};

struct sRRCConnectionReleaseMsgType {
    sRanControlMsgHeaderType sRanControlMsgHeader;
    uint8_t ueIdType;
    uint32_t URNTI;
    uint16_t CRNTI;
};

struct sRRCConnectionAcceptMsgType {
    sRanControlMsgHeaderType sRanControlMsgHeader;
    uint8_t ueIdType;
    uint32_t URNTI;
    uint16_t CRNTI;
};

struct sRRCConnectionRejectMsgType {
    sRanControlMsgHeaderType sRanControlMsgHeader;
    uint8_t ueIdType;
    uint32_t URNTI;
    uint16_t CRNTI;
};

struct sRttMeasMsgType {
    sRanControlMsgHeaderType sRanControlMsgHeader;
    uint8_t ueIdType;
    uint32_t URNTI;
    uint16_t CRNTI;
    int rttValue;
};

struct sRRCConnectionReleaseCcchMsgType {
    sRanControlMsgHeaderType sRanControlMsgHeader;
    uint32_t URNTI;
};

struct sCpichTxPowerMsg {
    sRanControlMsgHeaderType sRanControlMsgHeader;
    int cpichTxPower;
};

#pragma pack(pop)

#endif  // __RANCONTROLMSG_H
