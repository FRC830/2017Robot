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
	has_intaken = false;
	intake = intakeMotor;
	shooter = shooterMotor;
	shootSpeed = shoot_speed;
	ballOutput = ball_output;
	speedPID = new PIDController(p,i,d, shootSpeed, shooter);
	shooterTimer = new Timer();
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

void Shooter::shoot() {
	state = SHOOTING;
}

void Shooter::intakeBall() {
	state = TOINTAKE;
}

void Shooter::stopIntake() {
	state = NOTHING;
}

void Shooter::stopShoot() {
	state = NOTHING;
}

void Shooter::update() {
	SmartDashboard::PutNumber("state", state);
	if (state == SHOOTING) {
		state = NOTHING;
		ballOutput->Set(1.0);
		speedPID->SetSetpoint(SmartDashboard::GetNumber("revolutions",20));//20
		SmartDashboard::PutNumber("PID error",speedPID->GetError());
		SmartDashboard::PutString("state1", "shooting state");

	}
	else if (state == TOINTAKE) {
		state = NOTHING;
		float time = shooterTimer->Get();
		SmartDashboard::PutNumber("Intake timer", time );

		if (has_intaken == false) {
			intake->Set(1.0);
			shooterTimer->Start();
			has_intaken = true;
		}
		else {
			if (shooterTimer->Get() > 10.0) {
				intake->Set(0);
				shooterTimer->Stop();
			}
			else {
				intake->Set(1);
			}
		}
		//state = INTAKE;
	}
	else if (state == NOTHING) {
		intake->Set(0.0);
		shooter->Set(0.0);
		speedPID->SetSetpoint(0.0);
		ballOutput->Set(0.0);
		has_intaken = false;
		shooterTimer->Stop();
		shooterTimer->Reset();
	}

}

Shooter::~Shooter() {
	// TODO Auto-generated destructor stub
}

