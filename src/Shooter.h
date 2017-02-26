/*
 * Shooter.h
 *
 *  Created on: Feb 12, 2017
 *      Author: RatPack
 */
#include "WPIlib.h"
#include "LineBreakCounter.h"

#ifndef SHOOTER_H_
#define SHOOTER_H_

class Shooter {
public:
	Shooter(VictorSP * intakeMotor, VictorSP* shooterMotor, Spark *ball_output, LineBreakCounter* shoot_speed /*probably some sort of swtich*/);
	VictorSP * intake;
	VictorSP * shooter;
	Spark *ballOutput;
	LineBreakCounter * shootSpeed;
	PIDController * speedPID;
	float p,i,d;
	//float speed;

	enum State {NOTHING, TOINTAKE, SHOOTING};

	bool hasBall();

	void shoot();
	void intakeBall();
	void stopIntake();
	void stopShoot();
	void update();

	void disablePID();
	void setPIDValues(float p, float i, float d);

	Timer *shooterTimer;
	State state;

	virtual ~Shooter();
private:
	int numberOfBalls = 0;
	bool has_intaken = false;
};

#endif /* SHOOTER_H_ */
