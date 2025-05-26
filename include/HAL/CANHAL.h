#pragma once


class CANHAL {
public:
    virtual bool init() = 0;
    virtual bool sendFrame(const void *p_frame) = 0;
    virtual bool receiveFrame(void *p_frame) = 0;
    virtual ~CANHAL() {}
};