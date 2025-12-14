//
// Created by jamie on 11/12/2020.
//

#include "AccurateWaiter.h"

AccurateWaiter::AccurateWaiter():
TimerEvent(get_us_ticker_data())
{

}


void AccurateWaiter::handler()
{
	// This signals the RTOS that the waiting thread is ready to wake up.
	flags.set(1);
}

void AccurateWaiter::wait_for(std::chrono::microseconds duration)
{
	// set up timer event to occur
	insert(duration);

	// wait for event flag and then clear it
	flags.wait_all(1);
}

void AccurateWaiter::wait_until(TickerDataClock::time_point timePoint)
{
	// set up timer event to occur
	insert_absolute(timePoint);

	// wait for event flag and then clear it
	flags.wait_all(1);
}
