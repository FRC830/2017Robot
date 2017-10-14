/*
 * Shooter.cpp
 *
 *  Created on: Feb 12, 2017
 *      Author: RatPack
 */

#include "Shooter.h"
#include "Timer.h"
#include "WPIlib.h"


Shooter::Shooter(VictorSP* shooterMotor, /*Spark *ball_output,*/ LineBreakCounter * shoot_speed /*probably some sort of switch*/) {
	// TODO Auto-generated constructor stub
	p = 0.1f;
	i = 0.0;
	d = 0.0;

	//speed = 0.0;

	shooter = shooterMotor;
	shootSpeed = shoot_speed;
	speedPID = new PIDController(p,i,d, shootSpeed, shooter);
	speedPID->SetInputRange(0,1000);
	speedPID->SetOutputRange(0.0,1.0);
	speedPID->SetAbsoluteTolerance(5); //subject to change
	speedPID->Enable();

	state = NOTHING;

}
void Shooter::disablePID() {
	speedPID->Disable();
}

void Shooter::setPIDValues(float p, float i, float d) {
	speedPID->SetPID(p,i,d);
}

void Shooter::shoot() {
	state = SHOOTING;
	speedPID->SetSetpoint(SmartDashboard::GetNumber("revolutions",65));//20
}

void Shooter::manualShoot() {
	state = MANUAL_SHOOT;
}


void Shooter::stopShoot() {
	state = NOTHING;
}


void Shooter::update() {
	SmartDashboard::PutNumber("shooting state", state);
	SmartDashboard::PutNumber("shooter speed", shooter->Get());
	if (state == SHOOTING || state == MANUAL_SHOOT) {
		if (state == SHOOTING){
			SmartDashboard::PutNumber("PID error",speedPID->GetError());
			SmartDashboard::PutString("state1", "shooting state");
		}
		else {
			shooter->Set(0.8);
		}

		state = NOTHING;
	}

	else if (state == NOTHING) {
		shooter->Set(0.0);
		speedPID->SetSetpoint(0.0);

	}

}

Shooter::~Shooter() {
	// TODO Auto-generated destructor stub
}

