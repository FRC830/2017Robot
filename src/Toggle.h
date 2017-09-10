/*
 * Toggle.h
 *
 *  Created on: Aug 16, 2017
 *      Author: RatPack
 */
#include "WPIlib.h"
#ifndef SRC_TOGGLE_H_
#define SRC_TOGGLE_H_

class Toggle {
	public:

	Toggle(bool toggle_state =false):pre_state(false),toggle_state(toggle_state) {};

	bool pre_state;
	bool toggle_state;
	bool toggle( bool button_state) {
		if (button_state && (pre_state != button_state)) {
			toggle_state = !toggle_state;
		}
		pre_state = button_state;
		SmartDashboard::PutBoolean("toggle_state", toggle_state);
		return toggle_state;
	}

	operator bool () {
		return toggle_state;
	}

};




#endif /* SRC_TOGGLE_H_ */


