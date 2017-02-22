/*
 * Shooter.cpp
 *
 *  Created on: Feb 12, 2017
 *      Author: RatPack
 */

#include "Shooter.h"
#include "Timer.h"

Shooter::Shooter(VictorSP * intakeMotor, VictorSP* shooterMotor, Spark *ball_output, LineBreakCounter * shoot_speed /*probably some sort of switch*/) {
	// TODO Auto-generated constructor stub
	p = 0.1;
	i = 0.0;
	d = 0.0;
	intake = intakeMotor;
	shooter = shooterMotor;
	shootSpeed = shoot_speed;
	ballOutput = ball_output;
	speedPID = new PIDController(p,i,d, shootSpeed, shooter);
	speedPID->SetInputRange(0,1000);
	speedPID->SetOutputRange(0,1);
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
	startTimer();
}

void Shooter::stopIntake() {
	state = NOTHING;
}

void Shooter::update() {
	if (state == SHOOTING) {
		ballOutput->Set(1.0);
		speedPID->SetSetpoint(SmartDashboard::GetNumber("revolutions",20));//20
		SmartDashboard::PutNumber("PID error",speedPID->GetError());

	}
	else if (state == TOINTAKE) {
		intake->Set(1.0);
		//state = INTAKE;
	}
	/*else if (state == INTAKE) {
		if (timer.Get() > 5.0) {
			intake->Set(0.0);
		}
	}*/
	else {
		intake->Set(0.0);
		speedPID->SetSetpoint(0.0);
		ballOutput->Set(0.0);
	}

}

Shooter::~Shooter() {
	// TODO Auto-generated destructor stub
}

