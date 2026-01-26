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

#include <UMTSConfig.h>
#include "Logger.h"
#include "TCPSocket.h"
#include "RANControl.h"


#include "URRCMessages.h"

#include <iostream>
#include <string>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <algorithm>




using namespace Control;

RANControl::RANControl(/* args */) :
mId(-1), mEnabled(false), mRunning(false),
mRANControlPort(0), mRANControlIP(""), mCellId("")
{    
}
RANControl::~RANControl()
{
}
void RANControl::setParams(unsigned int iIndex,
     const std::string sCellId,
     std::string sIp,
     unsigned int iPort)
{
    mId = iIndex;
    mRANControlPort = iPort;
    mRANControlIP = sIp;
    mCellId = sCellId;

    if (mId >= 0)
        mEnabled = true;

    LOG(INFO) << format("mCellId = %s\n",mCellId.c_str());
}
bool RANControl::enabled() 
{   
    return mEnabled;
}
void RANControl::start() 
{   
    if (mRunning) return;	
    mRANControlSendThread.start((void*(*)(void*))Control::RANControlSendLoopAdapter,this);
    mRANControlReceiveThread.start((void*(*)(void*))Control::RANControlReceiveLoopAdapter,this);
    
}

void* Control::RANControlReceiveLoopAdapter(RANControl *controller)
{   
    controller->receiveLoop();
    return NULL;
}

void* Control::RANControlSendLoopAdapter(RANControl *controller)
{   
    controller->sendLoop();
    return NULL;
}

// sendLoop receives data from server and sends them to stack (RRC)
void RANControl::receiveLoop()
{   	
    while(!mRunning) 
    {
        LOG(DEBUG) << format("wait for connect");
        // 500 ms
        usleep(500*1000);
    }
    
    while (mRunning) {        
		// Receive is non blocking
        size_t headerSize = sizeof(sRanControlMsgHeaderType);
        std::string strHeader = mTCPClient->receiveData(headerSize);        
        if(strHeader.size() == headerSize)
        {
            sRanControlMsgHeaderType header;
            memcpy(&header,strHeader.c_str(),headerSize);
            uint32_t restOfTheMsgSize = ntohl(header.msgLen) - headerSize;
            std::string strData = mTCPClient->receiveData(restOfTheMsgSize);
            LOG(INFO) << "strData Size: " << strData.size();
            handleReceiveMessages(header, strHeader + strData);
        }
        else {
            // 10 ms            
            usleep(10*1000);            
        }           				
	}
}

// sendLoop sends data to server from 3G stack
void RANControl::sendLoop()
{    
    // Connect first
    LOG(INFO) << format("RAN CONTROL sendLoop starting ip:%s port:%u\n",mRANControlIP.c_str(),mRANControlPort);
    mTCPClient = new TCPClient(mRANControlIP,mRANControlPort);
    while(true)
    {
        LOG(INFO) << format("RAN CONTROL sendLoop connecting...\n");
        if(mTCPClient->connectToServer())
        {
            LOG(INFO) << format("RAN CONTROL sendLoop connected\n");
            mRunning = true;
            break;
        }
        // Connecting once in 2 second TODO: configurable?
        sleep(2);        
    } 
    LOG(INFO) << format("RAN CONTROL sendLoop sending...\n");    
	while (mRunning) {
        //  Read from queue and send        
        RANControlMessage *senderMsg = mRANControlSendQueue.read();
        switch(senderMsg->msgType) {
            case eRANControlMessageCellSetupInd:
            {
                LOG(INFO) << format("RAN CONTROL sendLoop CellSetupIndMsg\n");                
                mTCPClient->sendBinaryData((const char *)senderMsg->msg.cellSetupMsg,senderMsg->msgLen);
                free (senderMsg->msg.cellSetupMsg);
                delete (senderMsg);                
            }
            break;
            case eRANControlMessageRRC:
            {
                LOG(DEBUG) << format("eRRCMsg sending len = %u\n",senderMsg->msgLen);
                mTCPClient->sendBinaryData((const char *)senderMsg->msg.rrcMsg,senderMsg->msgLen);

#if RANCONTROL_LOOP_TEST
                if(senderMsg->rbNum == SRB3)
                {
                    LOG(INFO) << "SRB3\n";
                    ByteVector bv(6);
                    uint32_t msgLen = 0;
                    createDLDirectTransferTest(bv);
                    LOG(INFO) << format("1 bv size = %u\n",bv.size());
                    sRRCMsgType* rrcReq = createRRCReqMsgTest(senderMsg->msg.rrcMsg->URNTI,
                        senderMsg->msg.rrcMsg->CRNTI,bv,msgLen);
                    LOG(INFO) << format("msgLen = %u\n",msgLen);
                    if(rrcReq != NULL)
                    {
                        handleRRCReqMsg(rrcReq);
                        free(rrcReq);

                    }
                }
#endif                
                // Free for RRC data               
                free (senderMsg->msg.rrcMsg);
                delete (senderMsg);
            }
            break;
            case eRANControlMessageNone:
            {
                 LOG(WARNING) << format("sendLoop msgType eRANControlMessageNone\n");
            }
            break;
            default:
            {
                LOG(WARNING) << format("sendLoop msgType default\n");

            }
            break;
        }        
        // 1 ms
        usleep(1000);
    }
    
} 

void RANControl::handleRRCReqMsg(const sRRCMsgType *rrcMsg)
{
    UEInfo *uep = NULL;
    unsigned ueid;

   uint16_t CRNTI = ntohs(rrcMsg->CRNTI);
   uint32_t URNTI = ntohl(rrcMsg->URNTI);
   uint32_t rrcPduNum = ntohl(rrcMsg->rrcPduNum);

    LOG(DEBUG) << "URNTI     = "<< format("%x",URNTI);
    LOG(DEBUG) << "CRNTI     = "<< format("%x",CRNTI);
    LOG(DEBUG) << "rrcPduNum = "<< format("%x",rrcPduNum);
    // 0 = URNTI
    if(URNTI != 0)
    {         
        uep = gRrc.findUe(0,URNTI);
    }
    if(uep == NULL)
    {
        if(CRNTI != 0)
        {
            uep = gRrc.findUe(1,CRNTI);
        }
    }    
	if (uep == NULL) {
		LOG(INFO) << "Could not find UE with id ";
		return;
	}
    uint32_t rrcMsgLen = ntohl(rrcMsg->rrcMsgLen);
    
    LOG(DEBUG) << format("rrcMsgLen = %u\n",rrcMsgLen);
    ByteVector bvRRCMsg(rrcMsgLen);
    bvRRCMsg.setAppendP(0,0);
    bvRRCMsg.append((const char*)&(rrcMsg->rrcMsgData),rrcMsgLen);
    LOG(DEBUG) << "Sending\n";

    // TODO: Add support for SRB2 when needed
    if (rrcPduNum == RRC_PDU_NUM_DL_DCCH) {
        uep->ueWriteHighSide(SRB3,bvRRCMsg,"RAN Control SRB3");
    } else if (rrcPduNum == RRC_PDU_NUM_DL_CCCH) {
        uep->ueWriteHighSide(SRB0,bvRRCMsg,"RAN Control SRB0");
    }
    
    LOG(DEBUG) << "Sent\n";
}

void RANControl::handleReceiveMessages(const sRanControlMsgHeaderType &header, const std::string &dataMsg)
{
    uint32_t msgLen = ntohl(header.msgLen);
    uint32_t msgType = ntohl(header.msgType);

    switch (msgType)
    {
        case eRRCConnectionReleaseMsg:
        {
            sRRCConnectionReleaseMsgType rrcConnectionReleaseMsg;
            memcpy(&rrcConnectionReleaseMsg,dataMsg.c_str(),dataMsg.size());
            LOG(INFO) << "ueIdType: " << (int) rrcConnectionReleaseMsg.ueIdType;            

            uint32_t URNTI = ntohl(rrcConnectionReleaseMsg.URNTI);
            uint16_t CRNTI = ntohs(rrcConnectionReleaseMsg.CRNTI);            
            LOG(INFO) << "CRNTI     : " <<  CRNTI;
            LOG(INFO) << "URNTI     : " <<  URNTI;

            UEInfo *uep = NULL;
            uep = findUE(URNTI,CRNTI);
            if (uep == NULL) {
                LOG(INFO) << "Could not find UE with id ";
                return;
            }
            LOG(INFO) << "UE found";
            sendRrcConnectionRelease(uep);

        }
        break;
        case eRRCConnectionReleaseCcchMsg:
        {
            sRRCConnectionReleaseCcchMsgType rrcConnectionReleaseCcchMsg;
            memcpy(&rrcConnectionReleaseCcchMsg,dataMsg.c_str(),dataMsg.size());         

            uint32_t URNTI = ntohl(rrcConnectionReleaseCcchMsg.URNTI);
            LOG(INFO) << "URNTI     : " <<  URNTI;

            sendRrcConnectionReleaseCcch(URNTI);
        }
        break;
        case eRRCConnectionAcceptMsg:
        {
            sRRCConnectionAcceptMsgType rrcConnectionAcceptMsg;
            memcpy(&rrcConnectionAcceptMsg,dataMsg.c_str(),dataMsg.size());
            LOG(INFO) << "ueIdType  : " << (int) rrcConnectionAcceptMsg.ueIdType;

            uint32_t URNTI = ntohl(rrcConnectionAcceptMsg.URNTI);
            uint16_t CRNTI = ntohs(rrcConnectionAcceptMsg.CRNTI);
            
            LOG(INFO) << "CRNTI     : " <<  CRNTI;
            LOG(INFO) << "URNTI     : " <<  URNTI;
            
            UEInfo *uep = NULL;
            uep = findUE(URNTI,CRNTI);
            if (uep == NULL) {
                LOG(INFO) << "Could not find UE with id ";
                return;
            }
            LOG(INFO) << "UE found";
            continueRrcConnectionSetup(uep);

        }
        break;
        case eRRCConnectionRejectMsg:
        {
            sRRCConnectionRejectMsgType rrcConnectionRejectMsg;
            memcpy(&rrcConnectionRejectMsg,dataMsg.c_str(),dataMsg.size());
            LOG(INFO) << "ueIdType: " << (int) rrcConnectionRejectMsg.ueIdType;
            uint32_t URNTI = ntohl(rrcConnectionRejectMsg.URNTI);
            uint16_t CRNTI = ntohs(rrcConnectionRejectMsg.CRNTI);
            
            LOG(INFO) << "CRNTI     : " <<  CRNTI;
            LOG(INFO) << "URNTI     : " <<  URNTI;
            
            UEInfo *uep = NULL;
            // We have to find URNTI
            uep = findUE(URNTI,0);
            if (uep == NULL) {
                LOG(INFO) << "Could not find UE with id ";
                return;
            }

            sendRrcConnectionReject(uep);
        }
        break;
        case eRRCMsg:
        {
            sRRCMsgType *rrcMsg = NULL;
            rrcMsg = (sRRCMsgType *) malloc(dataMsg.size());
            memcpy(rrcMsg,dataMsg.c_str(),dataMsg.size());
            uint32_t rrcPduNum  = ntohl(rrcMsg->rrcPduNum);
            uint32_t rrcMsgBitLen = ntohl(rrcMsg->rrcMsgBitLen);
            uint32_t rrcMsgLen  = ntohl(rrcMsg->rrcMsgLen);
            
            LOG(INFO) << "rrcPduNum    : " << (int) rrcPduNum;
            LOG(INFO) << "rrcMsgBitLen : " << (int) rrcMsgBitLen;
            LOG(INFO) << "rrcMsgLen    : " << (int) rrcMsgLen;
            handleRRCReqMsg(rrcMsg);
            free(rrcMsg);
        }
        break;
        case eCpichTxPowerMsg:
        {
            sCpichTxPowerMsg cpichTxPowerMsg;
            memcpy(&cpichTxPowerMsg,dataMsg.c_str(),dataMsg.size());
            int cpichTxPower = cpichTxPowerMsg.cpichTxPower;
            gConfig.set("UMTS.CPICH.TxPower", std::to_string(cpichTxPower));
            gConfig.purge();
            gNodeB.regenerateBeacon();
        }
        break;
        default:
        {
             LOG(INFO) << "Not supported type = " << (int) msgType;
        }
        break;
    }

}

sCellSetupIndMsgType* RANControl::createCellSetupIndMsg(const std::string &trx_identifier, bool transmitting, uint32_t &msgLen)
{    
    size_t trx_identifier_size =  (trx_identifier.size() > 6) ? 6 : trx_identifier.size();
    LOG(INFO) << format("RAN CONTROL createCellSetupIndMsg trx_identifier_size = %d\n",trx_identifier_size);
    msgLen = sizeof(sCellSetupIndMsgType) + trx_identifier_size;
    sCellSetupIndMsgType* msg = (sCellSetupIndMsgType*) malloc(msgLen);

    // Header
    msg->sRanControlMsgHeader.msgLen = htonl(msgLen);
    msg->sRanControlMsgHeader.magicNr = htonl(0xA0);
    msg->sRanControlMsgHeader.msgType = htonl(eRanControlMsgType::eCellSetupIndMsg);    
    //Identifier
    msg->cellIdLen = trx_identifier_size;
    LOG(DEBUG) << "msg->cellIdLen = " << (int)msg->cellIdLen;
    
    std::vector<uint8_t> cellIdBytes = hexStringToBytes(trx_identifier);
    for (size_t i = 0;i < cellIdBytes.size();i++)
    {
        msg->cellId[i] = cellIdBytes[i];
    }
 
    LOG(DEBUG) << format("RAN CONTROL createCellSetupIndMsg Message size = %d\n",msgLen);

    return msg;
}


sRRCMsgType* RANControl::createRRCIndMsg( uint32_t URNTI,uint16_t CRNTI, const uint32_t rrcPduNum, const ByteVector &bv,uint32_t &msgLen)
{    
    LOG(DEBUG) << "URNTI = " << format("%x",URNTI) << "\n";
    LOG(DEBUG) << "CRNTI = " << format("%x",CRNTI) << "\n";
    LOG(DEBUG) << "bv size = " << bv.size();

    // Whole message len and allocation
    msgLen = sizeof(sRRCMsgType) + bv.size();
    sRRCMsgType* msg = (sRRCMsgType*) malloc(msgLen);
     // Header
    msg->sRanControlMsgHeader.msgLen =  htonl(msgLen);
    msg->sRanControlMsgHeader.magicNr = htonl(0xA0);
    msg->sRanControlMsgHeader.msgType = htonl(eRanControlMsgType::eRRCMsg);
    // Data
    msg->rrcMsgBitLen = htonl(bv.sizeBits());
    msg->rrcMsgLen = htonl(bv.size());

    if ((URNTI != 0) && (CRNTI != 0))
    {
        msg->ueIdType = UE_ID_TYPE_URNTI | UE_ID_TYPE_CRNTI;
    } else if ((URNTI != 0) && (CRNTI == 0)) {
        msg->ueIdType = UE_ID_TYPE_URNTI;
    } else {
        msg->ueIdType = UE_ID_TYPE_CRNTI;
    }
    msg->URNTI = htonl(URNTI);
    msg->CRNTI = htons(CRNTI);
    msg->rrcPduNum = htonl(rrcPduNum);
    memcpy(&(msg->rrcMsgData[0]),bv.begin(),bv.size());
    std::string tmpRrcMessage;
    for (int i = 0;i < bv.size();i++)
    {
        tmpRrcMessage.append(format("%02x",msg->rrcMsgData[i]));
    }
    LOG(INFO) << "rrcMsgData: " << tmpRrcMessage << "\n";
    LOG(DEBUG) << format("createRRCIndMsg Message size = %d\n",msgLen);
    return msg;
}



void RANControl::cellSetupInd(bool transmitting)
{
    uint32_t msgLen = 0;
    sCellSetupIndMsgType *msg = createCellSetupIndMsg(mCellId,transmitting,msgLen);
    RANControlMessage *senderMsg = new RANControlMessage();
    // Fill internal message
    senderMsg->msgType = eRANControlMessageCellSetupInd;
    senderMsg->msgLen = msgLen;
    senderMsg->msg.cellSetupMsg = msg;

    mRANControlSendQueue.write(senderMsg);
}

 void RANControl::rrcConnectionSetupReqInd(UEInfo *uep, const BitVector &bitVector)
 {
    LOG(INFO) << "RRC Connection Setup Request bitVector = " << bitVector.str();
    uint32_t msgLen = 0;
    ByteVector byteVector((bitVector.size()/8) + 1);
    byteVector.setAppendP(0,0);
    byteVector.append(bitVector);
    sRRCMsgType* msg = createRRCIndMsg(uep->mURNTI,uep->mCRNTI,RRC_PDU_NUM_UL_CCCH,byteVector,msgLen);
    RANControlMessage *senderMsg = new RANControlMessage();
    // Fill internal message
    senderMsg->rbNum    = 0; // SRB0
    senderMsg->msgType = eRANControlMessageRRC;
    senderMsg->msgLen  = msgLen;
    senderMsg->msg.rrcMsg = msg;

    mRANControlSendQueue.write(senderMsg);   

 }
void RANControl::rrcUplinkMessageInd(UEInfo *uep, const ByteVector &byteVector, unsigned rbNum)
{   
    LOG(DEBUG) << format("rbNum = %d\n",rbNum); 
    uint32_t msgLen = 0;
    sRRCMsgType* msg = createRRCIndMsg(uep->mURNTI,uep->mCRNTI,RRC_PDU_NUM_UL_DCCH,byteVector,msgLen);
    RANControlMessage *senderMsg = new RANControlMessage();
    // Fill internal message
    senderMsg->rbNum    = rbNum;
    senderMsg->msgType = eRANControlMessageRRC;
    senderMsg->msgLen  = msgLen;
    senderMsg->msg.rrcMsg = msg;

    mRANControlSendQueue.write(senderMsg);    
}

uint32_t RANControl::stringToUint32(const std::string& str) {
    if (str.size() < 4) {
        throw std::runtime_error("String too short to convert to uint32_t");
    }

    uint32_t value;
    std::memcpy(&value, str.data(), sizeof(uint32_t));
    return value;
}

std::vector<uint8_t> RANControl::hexStringToBytes(const std::string hexStr)
{
    std::vector<uint8_t> bytes;
    std::string tmpHexStr;

    tmpHexStr = hexStr;

    // Remove ":"
    tmpHexStr.erase(std::remove(tmpHexStr.begin(), tmpHexStr.end(), ':'), tmpHexStr.end());
    // Ensure even length
    if (tmpHexStr.length() % 2 != 0) {
        LOG(ERR) << "Invalid hex string length!" << std::endl;
        return bytes;
    }

    for (size_t i = 0; i < tmpHexStr.length(); i += 2) {
        std::string byteString = tmpHexStr.substr(i, 2);
        uint8_t byte = static_cast<uint8_t>(std::stoul(byteString, nullptr, 16));
        bytes.push_back(byte);
    }

    return bytes;
}

UEInfo * RANControl::findUE(const uint32_t URNTI, const uint16_t CRNTI)
{
    UEInfo *uep = NULL;
    if( URNTI != 0)
    { 
        //LOG(INFO) << "Could not find UE with id "<< ueid;
        uep = gRrc.findUe(0,URNTI);
    }
    if(uep == NULL)
    {
        if(CRNTI != 0)
        {
            uep = gRrc.findUe(1,CRNTI);
        }
    }    
    if (uep == NULL) {
        LOG(INFO) << "Could not find UE with id ";
        return NULL;
    }
    return uep;
}


#if RANCONTROL_LOOP_TEST  
void RANControl::createDLDirectTransferTest(ByteVector &bv)
{
    // 14 40 04 0a 30 04
    bv.setAppendP(0,0);
    LOG(INFO) << format("1 bv size = %u\n",bv.size());
    bv.appendByte(0x14);
    bv.appendByte(0x40);
    bv.appendByte(0x04);
    LOG(INFO) << format("2 bv size = %u\n",bv.size());
    bv.appendByte(0x0a);
    bv.appendByte(0x30);
    bv.appendByte(0x04);
    LOG(INFO) << format("3 bv size = %u\n",bv.size());
    
}
sRRCMsgType* RANControl::createRRCReqMsgTest( uint32_t URNTI,uint16_t CRNTI, const ByteVector &bv,uint32_t &msgLen)
{    
    msgLen = sizeof(sRRCMsgType) + bv.size();
    sRRCMsgType* msg = (sRRCMsgType*) malloc(msgLen);
    // Header
    msg->sRanControlMsgHeader.msgLen =  htonl(msgLen);
    msg->sRanControlMsgHeader.magicNr = htonl(0xA0);
    msg->sRanControlMsgHeader.msgType = htonl(eRanControlMsgType::eRRCMsg);
    // Data
    msg->rrcMsgBitLen = htonl(bv.sizeBits());
    msg->rrcMsgLen = htonl(bv.size());
    msg->URNTI = htonl(URNTI); 
    msg->CRNTI = htonl(CRNTI);    
    memcpy(&(msg->rrcMsgData[0]),bv.begin(),bv.size());
    LOG(INFO) << "rrcMsgData: " << format("%x",msg->rrcMsgData[0]) << "\n";
   
    LOG(INFO) << format("Message size = %d\n",msgLen);

    return msg;
}
#endif

