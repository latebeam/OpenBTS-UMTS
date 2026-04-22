#ifndef UMEASINTERFACE_H
#define UMEASINTERFACE_H

#include <cstdint>
#include <functional>
#include "Interthread.h"

namespace UMTS {

struct RttMeasMsg {
    uint32_t mHandle;
    float mRTT;
};

class MeasInterface {

public:

    virtual RttMeasMsg *allocRttMeasMsg() = 0;
    virtual void freeRttMeasMsg(RttMeasMsg *rttMeasMsg) = 0;

    virtual void writeRttMeasMsg(RttMeasMsg *rttMeasMsg) = 0;
    virtual RttMeasMsg *readRttMeasMsg() = 0;

};

} // namespace UMTS

#endif // UMEASINTERFACE_H
