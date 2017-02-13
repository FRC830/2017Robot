/*
 * Shooter.cpp
 *
 *  Created on: Feb 12, 2017
 *      Author: RatPack
 */

#include "Shooter.h"

Shooter::Shooter(VictorSP * intakeMotor, VictorSP* shooterMotor /*probably some sort of swtich*/) {
	// TODO Auto-generated constructor stub
	intake = intakeMotor;
	shooter = shooterMotor;
	state = NOTHING;

}

Shooter::~Shooter() {
	// TODO Auto-generated destructor stub
}

