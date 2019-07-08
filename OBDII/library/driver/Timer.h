/**
 * See the file LICENSE for redistribution information.
 *
 * Copyright (c) 2009-2016 ObdDiag.Net. All rights reserved.
 *
 */

#ifndef __TIMER_H__
#define __TIMER_H__

#include <cstdint>
#include <stm32f0xx.h>

using namespace std;


class Timer {
public:
    const static int TIMER0 = 0;
    const static int TIMER1 = 1;
    const static int TIMER2 = 2;
    static void configure();
    static Timer* instance(int timerNum);
    void start(uint32_t interval);
    bool isExpired() const;
    uint32_t value() const;
protected:
    Timer(int timerNum);
    TIM_TypeDef* timer_;
};

class LongTimer : public Timer {
public:
    static LongTimer* instance();
private:
    LongTimer();
};

// For use with Rx/Tx LEDs
typedef void (*PeriodicCallbackT)();
class PeriodicTimer {
public:
    PeriodicTimer(PeriodicCallbackT callback);
    void start(uint32_t interval);
    void stop();
};

#endif //__TIMER_H__
