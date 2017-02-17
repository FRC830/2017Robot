/*
 * Shooter.h
 *
 *  Created on: Feb 12, 2017
 *      Author: RatPack
 */
#include "WPIlib.h"

#ifndef SHOOTER_H_
#define SHOOTER_H_

class Shooter {
public:
	Shooter(VictorSP * intakeMotor, VictorSP* shooterMotor, DigitalOutput* ballCounter /*probably some sort of swtich*/);
	VictorSP * intake;
	VictorSP * shooter;
	DigitalOutput * ballCount;

	enum State {NOTHING, INTAKE, TOINTAKE, SHOOTING};

	bool hasBall();

	void shoot();
	void intakeBall();
	void stopIntake();
	void update();

	Timer timer;
	State state;

	virtual ~Shooter();
private:
	void startTimer();
	int numberOfBalls = 0;
};

#endif /* SHOOTER_H_ */
