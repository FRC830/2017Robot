/*
 * LineBreakCounter.cpp
 *
 *  Created on: Feb 21, 2017
 *      Author: RatPack
 */
#include "WPILib.h"
#include "LineBreakCounter.h"

LineBreakCounter::LineBreakCounter(int dio_channel){
	counter = new Counter(dio_channel);
	counter->Reset();
}

double LineBreakCounter::PIDGet(){
	//period measures time in seconds between signals from the line break
	//if there are n signals per revolution, period * n = seconds per revolution
	//we want revolutions per second, so we take the reciprocal of this
	return 1.0 / (counter->GetPeriod());
}

LineBreakCounter::~LineBreakCounter() {}
