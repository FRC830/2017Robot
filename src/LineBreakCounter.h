/*
 * LineBreakCounter.h
 *
 *  Created on: Feb 21, 2017
 *      Author: RatPack
 */

#ifndef LINE_BREAK_COUNTER_H_
#define LINE_BREAK_COUNTER_H_

#include "WPILib.h"

class LineBreakCounter : public PIDSource {
private:
	//int ticks_per_rev;	//number of ticks in a revolution
public:
	Counter * counter;	//counter to track linebreak signals
	LineBreakCounter(int dio_channel);
	virtual double PIDGet();
	virtual ~LineBreakCounter();
};

#endif
