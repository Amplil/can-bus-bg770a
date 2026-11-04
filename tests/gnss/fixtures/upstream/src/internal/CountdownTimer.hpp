/*
 * CountdownTimer.hpp
 * Copyright (C) Seeed K.K.
 * MIT License
 */

#ifndef COUNTDOWNTIMER_HPP
#define COUNTDOWNTIMER_HPP

#include <Arduino.h>

namespace wiocellular::internal
{
    class CountdownTimer
    {
    private:
        int Timeout_;
        uint32_t Start_;

    public:
        CountdownTimer(int timeout) : Timeout_{timeout}
        {
            Start_ = millis();
        }

        bool isTimeout() const
        {
            if (Timeout_ >= 0)
            {
                return millis() - Start_ >= static_cast<uint32_t>(Timeout_);
            }
            else
            {
                return false;
            }
        }

        int remaining() const
        {
            if (Timeout_ >= 0)
            {
                const auto elapsed = millis() - Start_;
                return elapsed < static_cast<uint32_t>(Timeout_) ? Timeout_ - elapsed : 0;
            }
            else
            {
                return -1;
            }
        }
    };

}

#endif // COUNTDOWNTIMER_HPP
