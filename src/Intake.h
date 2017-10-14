/*
 * Intake.h
 *
 *  Created on: Oct 14, 2017
 *      Author: Carmen
 */
#include "Lib830.h"
#include "WPIlib.h"

#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

class Intake {
public:
	Intake(VictorSP*intakeMotor);
	VictorSP*intake;

	enum State {
			INTAKE, OUTPUT, OVERIDE, NOTHING
	};

	void outputBall();
	void intakeBall();
	void stopIntake();
	void intakeOverRide();

	void update();

	Timer *time;
	State state;

	virtual ~Intake();

private:
	bool hasIntaken;


};



#endif /* SRC_INTAKE_H_ */
