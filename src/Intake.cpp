/*
 * Intake.cpp
 *
 *  Created on: Oct 14, 2017
 *      Author: Carmen
 */

#include "WPILib.h"
#include "Intake.h"

Intake::Intake(VictorSP *intakeMotor) {
	intake = intakeMotor;
	hasIntaken = false;
	state = NOTHING;
	time = new Timer();
}

void Intake::intakeBall() {
	state = INTAKE;
	//time->Reset();
}

void Intake::outputBall() {
	state = OUTPUT;
}

void Intake::stopIntake() {
	state = NOTHING;
}

void Intake::intakeOverRide() {
	//well we'll see i guess
	state = OVERIDE;
}

void Intake::update() {

	switch (state) {
		case INTAKE:
		case	 OVERIDE:
			if (hasIntaken == false) {
				time->Stop();
				time->Reset();
				time->Start();
				hasIntaken = true;
			}
			intake->Set(1.0);

			if(time->Get() > 20 && state !=OVERIDE) {
				intake->Set(0.0);
			}
			break;

		case OUTPUT:
			intake->Set(-1.0);
			break;

		case NOTHING:
		default:
			intake->Set(0.0);
			time->Stop();
			time->Reset();
			hasIntaken = false;
			break;
	}
}


Intake::~Intake() {}











