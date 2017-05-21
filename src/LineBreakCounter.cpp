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
float Last_RPS = 0;
bool counter_error = false;

double LineBreakCounter::PIDGet(){
	//period measures time in seconds between signals from the line break
	//if there are n signals per revolution, period * n = seconds per revolution
	//we want revolutions per second, so we take the reciprocal of this

	float RPS = 1.0 / counter->GetPeriod();
	if (RPS > 100) {
		RPS = Last_RPS;
		counter_error = true;
	}
	SmartDashboard::PutNumber("RPS", RPS);
	SmartDashboard::PutBoolean("Excessive Counter Error", counter_error);

	//return 1.0 / (counter->GetPeriod());

	Last_RPS = RPS;
	counter_error = false;
	return RPS;
}

LineBreakCounter::~LineBreakCounter() {}
