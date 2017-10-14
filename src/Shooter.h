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
	Shooter(VictorSP* shooterMotor, LineBreakCounter* shoot_speed);
	VictorSP * shooter;
	LineBreakCounter * shootSpeed;
	PIDController * speedPID;
	float p,i,d;
	//float speed;

	enum State {NOTHING, SHOOTING, MANUAL_SHOOT};

	void shoot();
	void stopShoot();
	void update();
	void manualShoot();
	void setPIDValues(float p, float i, float d);

	State state;

	virtual ~Shooter();
private:
	void disablePID();

};

#endif /* SHOOTER_H_ */
