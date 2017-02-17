/*
 * Shooter.cpp
 *
 *  Created on: Feb 12, 2017
 *      Author: RatPack
 */

#include "Shooter.h"
#include "Timer.h"

Shooter::Shooter(VictorSP * intakeMotor, VictorSP* shooterMotor, DigitalOutput* ballCounter /*probably some sort of switch*/) {
	// TODO Auto-generated constructor stub
	intake = intakeMotor;
	shooter = shooterMotor;
	ballCount = ballCounter;
	state = NOTHING;

}
void Shooter::startTimer() {
	timer.Stop();
	timer.Reset();
	timer.Start();
}

void Shooter::shoot() {
	state = SHOOTING;
}

void Shooter::intakeBall() {
	state = TOINTAKE;
	numberOfBalls = 0;
	startTimer();
}

void Shooter::stopIntake() {
	state = NOTHING;
}

void Shooter::update() {
	if (state == SHOOTING) {
		shooter->Set(1.0);
	}
	else if (state == TOINTAKE) {
		intake->Set(-1.0);
		state = INTAKE;
	}
	else if (state == INTAKE) {
		if (timer.Get() > 5.0) {
			intake->Set(0.0);
		}
		if (ballCount->Get()) {
			numberOfBalls += 1;
		}
	}
	else {
		intake->Set(0.0);
		shooter->Set(0.0);
	}

}

Shooter::~Shooter() {
	// TODO Auto-generated destructor stub
}

