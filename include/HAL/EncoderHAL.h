#pragma once
#include <cstdint>
class EncoderHAL {
public:
    explicit EncoderHAL() = default;

    virtual void init() = 0;
    virtual int64_t getCount() = 0;
    virtual void setCount(int64_t value) = 0;
    virtual void cleaCount() = 0;
};