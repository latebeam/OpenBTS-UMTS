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

#ifndef RANCONTROL_H
#define RANCONTROL_H

#include "URRC.h"
#include "MemoryLeak.h"
#include "Utils.h"
#include "ByteVector.h"
#include "ScalarTypes.h"
#include "Threads.h"
#include "Interthread.h"

#include "RANControlMsg.h"

#define RANCONTROL_LOOP_TEST 0

class TCPClient;
class sCellSetupIndMsgType;
class sRRCMsgType;


using namespace UMTS;

namespace Control {

union RANControlMessageUnion {
    sCellSetupIndMsgType *cellSetupMsg;
    sRRCMsgType          *rrcMsg;
};


enum eInternalRANControlMessageType {
    eRANControlMessageNone = 0,
    eRANControlMessageCellSetupInd,
    eRANControlMessageRRC   
};

// internal wrapper
class RANControlMessage {
    public:
    uint32_t               rbNum;
    uint32_t               msgLen;
    eInternalRANControlMessageType    msgType; 
    RANControlMessageUnion msg;

    RANControlMessage(): 
    rbNum(0),msgLen(0),msgType(eRANControlMessageNone)
    {
    }
};

class RANControl
{
private:

    void handleRRCReqMsg(const sRRCMsgType *rrcMsg);
    void handleReceiveMessages(const sRanControlMsgHeaderType &header, const std::string &msg);

    sCellSetupIndMsgType* createCellSetupIndMsg(const std::string &trx_identifier,bool transmitting,uint32_t &msgLen);
    sRRCMsgType*          createRRCIndMsg(uint32_t URNTI,uint16_t CRNTI, const uint32_t rrcPduNum, const ByteVector &bv,uint32_t &msgLen);
    
#ifdef RANCONTROL_LOOP_TEST    
    // For testing purposes
    void                  createDLDirectTransferTest(ByteVector &bv);
    sRRCMsgType*          createRRCReqMsgTest(uint32_t URNTI,uint16_t CRNTI, const ByteVector &bv,uint32_t &msgLen);
#endif
    uint32_t stringToUint32(const std::string& str);
    std::vector<uint8_t> hexStringToBytes(const std::string hexStr);
    UEInfo * findUE(const uint32_t URNTI, const uint16_t CRNTI);
    /* data */
    Thread mRANControlReceiveThread;
    Thread mRANControlSendThread;
    mutable Mutex mLock;					///< Lock for thread-safe access.

    InterthreadQueue <RANControlMessage> mRANControlSendQueue;

    volatile bool mEnabled;
    volatile bool mRunning;

    TCPClient    *mTCPClient;
    unsigned int mRANControlPort;	    // Remote Port
	std::string  mRANControlIP;	        // Remote Address
    std::string  mCellId;               // Cell identification for the instance, not 3GPP related
    unsigned int mId;                   // Id of the instance, not indexing    
    
public:
    RANControl(/* args */);
    ~RANControl();
    void setParams(unsigned int iIndex,const std::string sCellId, std::string sIp, unsigned int iPort);
    bool enabled();
    void start();
    void receiveLoop();
    void sendLoop();

    void cellSetupInd(bool transmitting);
    void sendRRCIndMsg(UEInfo *uep, const BitVector &bitVector);
    void rrcUplinkMessageInd(UEInfo *uep, const ByteVector &byteVector,unsigned rbNum);
};


void *RANControlReceiveLoopAdapter(Control::RANControl*);
void *RANControlSendLoopAdapter(Control::RANControl*);

} // namespace Control

#endif // RANCONTROL_H
