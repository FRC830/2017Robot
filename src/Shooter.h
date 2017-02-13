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
	Shooter(VictorSP * intakeMotor, VictorSP* shooterMotor /*probably some sort of swtich*/);
	VictorSP * intake;
	VictorSP * shooter;

	enum State {NOTHING, INTAKE, SHOOTING};

	bool hasBall();

	void shoot();
	void intakeBall();
	void stopIntake();
	void update();

	Timer timer;
	State state;

	virtual ~Shooter();
};

#endif /* SHOOTER_H_ */
