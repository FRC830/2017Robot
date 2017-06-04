/*2017 Competition Robot SAURON
 * Pilot drives left joystick
 * Copilot X button stirs the wiffle balls
 * Copilot B button shoots the balls out the bottom
 * Copilot A button intakes the balls
 * Copilot Y button toggles LEDs
 * DPad up turns shooter on
 */
#include "WPILib.h"
#include "CameraServer.h"
#include "Lib830.h"
#include "AnalogGyro.h"
#include <GripPipeline.h>
#include <vision/VisionRunner.h>
#include "vision/VisionPipeline.h"
#include "thread"
#include "unistd.h"
#include "input/GamepadF310.h"
#include "util/Algorithms.h"
#include "math.h"
#include "Shooter.h"

using namespace Lib830;

class Robot: public IterativeRobot
{
	float previousTurn = 0;
	float previousSpeed = 0;
public:
	enum AutoMode {LEFT_SIDE, RIGHT_SIDE, CENTER, BASELINE, NOTHING, BAD_GYRO, CENTER_SHOOT};
private:
	//drivetrain motors
	static const int LEFT_PWM_ONE = 0; // 0 is 10 on electrical board
	static const int LEFT_PWM_TWO = 1;
	static const int RIGHT_PWM_ONE = 9;
	static const int RIGHT_PWM_TWO = 8;

	static const int CLIMBER_PWM = 3; //CB

	static const int BALL_OUTPUTPWM = 2;
	static const int SHOOTER_PWM =  5;//SH
	static const int INTAKE_PWM = 7;//IT
	static const int COUNTER_DIO = 1;

	static const int ANALOG_GYRO = 0;

	static const int RED_LED_DIO = 5;
	static const int GREEN_LED_DIO = 6;
	static const int BLUE_LED_DIO = 7;

	static const int CLIMBING_SWITCH_DIO = 0;

	bool highGoalCorrect = false;


	//drivetrain
	RobotDrive * drive;

	Spark* climber;

	GamepadF310 * pilot;
	GamepadF310 * copilot;

	Timer timer;

	frc::AnalogGyro *gyro;

	LiveWindow *lw = LiveWindow::GetInstance();

	DigitalLED * LED;

	DigitalOutput * climbingSwitch;

	SendableChooser<AutoMode*> *chooser;
	static const int TICKS_TO_ACCEL = 10;

	Shooter * shooter;

	Spark *agitator;

	void arcadeDrive(double speed, double turn, bool squaredinputs = false, bool inverted_control = false) {
		if (inverted_control == true) {
			speed = -speed;
			turn = -turn;
		}
		SmartDashboard::PutNumber("speed", speed);
		SmartDashboard::PutNumber("turn", turn);
		drive->ArcadeDrive(speed, -turn, squaredinputs);
	}

	static void CameraPeriodic()
	{
		CameraServer * server;
		grip::GripPipeline * pipeline;

		pipeline = new grip::GripPipeline();

		cs::UsbCamera camera;
		cv::Mat image;
		cv::Mat image_temp;
		cv::Mat hsl_output;
		int g_frame = 0;

		cs::CvSink sink;
		cs::CvSource outputStream;

		server = CameraServer::GetInstance();

		camera = server->StartAutomaticCapture();
		camera.SetResolution(320,240);

		sink = server->GetVideo();
		outputStream = server->PutVideo("Processed", 320, 240);
		//outputStream.
		//camera.SetExposureManual(80);


		while(1) {
			bool working = sink.GrabFrame(image_temp);
			SmartDashboard::PutBoolean("working", working);

			if (working) {
				g_frame ++;
				image = image_temp;
			}
			if (g_frame < 1) {
				continue;
			}

			//pipeline->hslThreshold(image, hue, sat, lum, hsl_output);
			bool target = SmartDashboard::GetBoolean("target bool", false);
			if (target == true) {
				pipeline->GuideLines(image);
			}
			else {
				pipeline->Process(image);
			}
			//outputStream.PutFrame(*pipeline->gethslThresholdOutput());
			outputStream.PutFrame(image);
		}


	}

	void RobotInit()
	{
		drive = new RobotDrive(
			new VictorSP(LEFT_PWM_ONE),
			new VictorSP(LEFT_PWM_TWO),
			new VictorSP(RIGHT_PWM_ONE),
			new VictorSP(RIGHT_PWM_TWO)
		);

		LED = new DigitalLED(RED_LED_DIO, GREEN_LED_DIO, BLUE_LED_DIO);
		LED->Set(1, 0, 0.5);

		/*drive = new RobotDrive(
			new VictorSP(LEFT_PWM_TWO), //8
			new VictorSP(LEFT_PWM_ONE) //9
		); */

		pilot = new GamepadF310(0);
		copilot = new GamepadF310(1);
		climber = new Spark(CLIMBER_PWM);

		gyro = new frc::AnalogGyro(ANALOG_GYRO);

		climbingSwitch = new DigitalOutput(CLIMBING_SWITCH_DIO);

		//autonChooser
		chooser = new SendableChooser<AutoMode*>();
		chooser->AddDefault("baseline", new AutoMode(BASELINE));
		chooser->AddObject("Left Side", new AutoMode(LEFT_SIDE));
		chooser->AddObject("Right Side", new AutoMode(RIGHT_SIDE));
		chooser->AddObject("Center", new AutoMode(CENTER));
		chooser->AddObject("default", new AutoMode(NOTHING));
		chooser->AddObject("bad gyro", new AutoMode(BAD_GYRO));
		chooser->AddObject("center and shoot", new AutoMode(CENTER_SHOOT));
		SmartDashboard::PutData("Auto Modes", chooser);
		
		SmartDashboard::PutNumber("P",9);
		SmartDashboard::PutNumber("I",2);
		SmartDashboard::PutNumber("D",2);

		SmartDashboard::PutNumber("revolutions",65);

		shooter = new Shooter(
				new VictorSP(INTAKE_PWM),
				new VictorSP(SHOOTER_PWM),
				//new Spark(BALL_OUTPUTPWM),
				new LineBreakCounter(COUNTER_DIO)
		);

		agitator = new Spark(BALL_OUTPUTPWM);

		std::thread visionThread(CameraPeriodic);
		visionThread.detach();

	}



	/**
	 * This autonomous (along with the chooser code above) shows how to select between different autonomous modes
	 * using the dashboard. The sendable chooser code works with the Java SmartDashboard. If you prefer the LabVIEW
	 * Dashboard, remove all of the chooser code and uncomment the GetString line to get the auto name from the text box
	 * below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the if-else structure below with additional strings.
	 * If using the SendableChooser make sure to add them to the chooser code above as well.
	 */
	float ProcessTargetTurn(float min_turn) {
		float turn = 0.0;
		if (SmartDashboard::GetBoolean("target acquired", false) == true) {
			float center = 160;
			float mid_point = SmartDashboard::GetNumber("x value between bars",center);
			turn = (center - mid_point) / -140;

			float max_turn_speed = 0.3;
			SmartDashboard::PutNumber("max_turn_speed", max_turn_speed);
			float min_turn_speed = min_turn;

			if (fabs(turn) < min_turn_speed) {
				if (turn < 0) {
					turn = -min_turn_speed;
				}
				else {
					turn = min_turn_speed;
				}
			}
			else if (fabs(turn) > max_turn_speed) {
				if (turn < 0) {
					turn = -max_turn_speed;
				}
				else {
					turn = max_turn_speed;
				}
			}
		}
		SmartDashboard::PutNumber("process turn speed", turn);
		return turn;
	}

	void AutonomousInit()
	{
		timer.Reset();
		timer.Start();
		gyro->Reset();
		float p = SmartDashboard::GetNumber("P",9.0);
		float i = SmartDashboard::GetNumber("I",1.1);
		float d = SmartDashboard::GetNumber("D",0.0);

		shooter->setPIDValues(p,i,d);
		//shooter->

		//autonChooser
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);

	}

	const float WEIGHT_AFTER_ONE_SECOND = .0001;

	float prev_time = 0;
	float prev_speed = 0;

	float prev_turn = 0;

	float prev_process_success_time = 0;
	float prev_process_success_turn = 0;
	bool process_success = false;
	double gyro_offset = 0;

	void GyroReset() {
		gyro_offset = gyro->GetAngle();
	}

	double GyroGetAngle() {
		return gyro->GetAngle()-gyro_offset;
	}

	void AutonomousPeriodic()

	{
		float time = timer.Get();
		//float cur_weight = pow(WEIGHT_AFTER_ONE_SECOND, time - prev_time);
		//float prev_weight = 1 - cur_weight;

		float prev_weight = 0.4;

		float speed = 0;

		double angle = gyro->GetAngle();
		agitator->Set(0);

		float turn = angle /-17.0;
		float straight_time = 1;

		float processed_turn = ProcessTargetTurn(0.1);
		//arcadeDrive(0.0, 0.0);

		AutoMode mode = BASELINE;

		if(chooser->GetSelected()) {
			mode = *chooser->GetSelected();
		}

		if (mode == LEFT_SIDE || mode == RIGHT_SIDE || mode == CENTER || mode == CENTER_SHOOT) {
			if (mode != CENTER || CENTER_SHOOT) {
				straight_time = 2.8;
			}
			if (time < straight_time) {
				speed = 0.3;
				//arcadeDrive(speed, turn);
			}
			else if (time >= straight_time && time < straight_time + 5) {
				speed = 0.3;
				if (processed_turn !=0 && time > straight_time + 1 && time < straight_time + 8) {
					process_success = true;
					gyro->Reset();
					turn = processed_turn;
				}
				else if (mode == LEFT_SIDE && (time - straight_time < 1)) {
					turn = (angle - 60) / -60.0; //opposite turn angle
					if (turn > 0.4) {
						turn = 0.4;
					}
					speed = 0;
				}
				else if (mode == RIGHT_SIDE && (time - straight_time < 1)) {
					turn = (angle + 60) / -60.0;
					if (turn < -0.4) {
						turn = -0.4;
					}
					speed = 0;
				}
			}
			else if (time > straight_time + 7.5) {
				if (mode != CENTER_SHOOT) {
					if (time >= straight_time + 7.5 && time < straight_time + 8.2){
						speed = -0.4;
					}
					else if (time >= straight_time + 8.5 ){
						speed = 0.3;
						if (processed_turn !=0) {
							gyro->Reset();
							turn = processed_turn;
						}
					}
				}
				else {
					float shoot_time = (2* straight_time) + 7.5;
					if (time < shoot_time) {
						speed = -0.3;
						shooter->manualShoot();

					}
					else if (time > shoot_time && time < shoot_time + 1) {
						turn = (angle + 90) /-100;
						shooter->manualShoot();
						speed = 0;
					}
					else if (time > shoot_time + 1) {
						turn = 0;
						speed = 0;
						shooter->manualShoot();
						agitator->Set(0.5);
					}

				}
			}
			else {
				turn = 0;
				speed = 0;
			}
		}
		else if (mode == BASELINE) {
			if (time < 3)
				//arcadeDrive(0.2, turn, false);
				speed = 0.6;
			else {
				speed = 0;
				turn = 0;
			}
		}
		else if (mode == BAD_GYRO) {
			speed = 0.3;
			if (time > 0 && time <= 6) {
				if (processed_turn !=0) {
					process_success = true;
					turn = processed_turn;
					prev_process_success_turn = processed_turn;
					prev_process_success_time = time;
					//speed = 0.3;
				}
				else if (process_success && ((time - prev_process_success_time) < 0.25)) {
					turn = prev_process_success_turn;
				}
				else {
					turn = 0;
				}
				//arcadeDrive(speed, turn, false);
			}
			else {
				turn = 0;
				speed = 0;
			}
		}


		//float accel_turn = (prev_weight * prev_turn) + (cur_weight * turn);
		float accel_turn = accel(prev_turn, turn, 9);
		//float accel_speed = accel(prev_speed, speed, 10);
		arcadeDrive(speed, accel_turn, false);
		prev_turn = turn;
		//prev_time = time;



		SmartDashboard::PutNumber("processed turn", processed_turn);
		SmartDashboard::PutNumber("turn", turn);
		SmartDashboard::PutNumber("previous weight", prev_weight);

		SmartDashboard::PutNumber("accel_turn", accel_turn);
		SmartDashboard::PutNumber("time", time);
		SmartDashboard::PutNumber("gyro angle", angle);
		shooter->update();

	}

	void TeleopInit()
	{
		gyro->Reset();
		LED->Disable();
		float p = SmartDashboard::GetNumber("P",9);
		float i = SmartDashboard::GetNumber("I",1.1);
		float d = SmartDashboard::GetNumber("D",0);

		shooter->setPIDValues(p,i,d);
	}
	bool invert = false;
	bool x_was_pressed = false;

	bool was_shooting = false;
	bool toggle_shoot = false;
	//bool shooting = false;
	void TeleopPeriodic()
	{

		float targetSpeed = pilot->LeftY();

		if (fabs(targetSpeed) < 0.13) {
			targetSpeed = 0;
		}


		double bVoltage = DriverStation::GetInstance().GetBatteryVoltage();
		if (bVoltage < 8.0) {
			targetSpeed = targetSpeed/2;
		}

		float speed = accel(previousSpeed, targetSpeed, TICKS_TO_ACCEL);
		previousSpeed = speed;



		double angle = gyro->GetAngle();

		SmartDashboard::PutNumber("gyro angle", angle);


		/*float targetTurn = pilot->LeftX();
		float turn = Lib830::accel(previousTurn, targetTurn, 30);
		previousTurn = targetTurn; */
		float targetTurn;

		if (pilot->ButtonState(Lib830::GamepadF310::BUTTON_RIGHT_BUMPER)) {
			targetTurn = ProcessTargetTurn(0.1);
		}
		else if (fabs(pilot->LeftTrigger()) > 0.5) {
			targetTurn = -angle/10.0;
		}
		else {
			targetTurn = pilot->RightX()/1.5;
			gyro->Reset();
		}
		if (pilot->ButtonState(GamepadF310::BUTTON_LEFT_BUMPER)) {
			highGoalCorrect = true;
			SmartDashboard::PutBoolean("target bool", highGoalCorrect);
		}
		else {
			highGoalCorrect = false;
			SmartDashboard::PutBoolean("target bool", highGoalCorrect);
		}

		float turn = accel(previousTurn, targetTurn, 10);
		previousTurn = targetTurn;
		SmartDashboard::PutNumber("real turn", turn);

		bool x_pressed = pilot->ButtonState(GamepadF310::BUTTON_X);

		if (x_pressed != x_was_pressed && x_pressed) {
			//arcadeDrive(speed, turn, false, true);
			invert = !invert;
		}
		x_was_pressed = x_pressed;

		if (invert == true) {
			LED->Set(0.5,0.0,0.5);
		}
		else {
			LED->SetAllianceColor();
		}
		SmartDashboard::PutBoolean("invert", invert);
		arcadeDrive(speed, turn, false, invert);

		//Climb with right trigger when the limit switch isn't down; override limit switch with down on Dpad
		SmartDashboard::PutBoolean("Climber Switch", climbingSwitch->Get());
		if (climbingSwitch->Get() == false || copilot->DPadDown()) {
			climber->Set(-(copilot->RightTrigger()));
		}

		//change lights to green with Y button on copilot
		if (copilot->ButtonState(GamepadF310::BUTTON_Y)) {
			LED->Set(0,1,0);
		}

		//Pull balls in with copilot A
		if (copilot->ButtonState(GamepadF310::BUTTON_A)) {
			shooter->intakeBall();
		}

		//Push balls out with copilot B
		if (copilot->ButtonState(GamepadF310::BUTTON_B)) {
			shooter->outputBall();
		}

		//Dpad up toggles shooter
		bool shooting = copilot->DPadUp();

		if (shooting != was_shooting && shooting == true) {
			toggle_shoot = !toggle_shoot;
		}


		was_shooting = shooting;

		if (toggle_shoot == true) {
			shooter->shoot();
		}

		//Left trigger turns on agitator
		if (copilot->LeftTrigger() > 0.5) {
			agitator->Set(0.5);
		}

		//X turns on input, shooter, and agitator
		if (copilot->ButtonState(GamepadF310::BUTTON_X)){
			shooter->shoot();
			agitator->Set(0.5);
			SmartDashboard::PutString("agitator string", "it should be working");
		} else if (copilot->LeftTrigger() < 0.5) {
			agitator->Set(0);
			SmartDashboard::PutString("agitator string", "it is not working");
		}
		SmartDashboard::PutNumber("agitator", agitator->Get());
		shooter->update();

	}
	void TestPeriodic()
	{
		lw->Run();
	}
	void DisabledInit() {
		timer.Start();
	}
	void DisabledPeriodic() {
		arcadeDrive(0.0,0.0);
//		DigitalLED::Color imperfectYellow = {1,0.6,0};
//		LED->Alternate(imperfectYellow, {0,0,1});
		//LED->Set(, , );
		float angle = gyro->GetAngle();
		SmartDashboard::PutNumber("gyro angle", angle);

	}
	void RobotPeriodic() {
		//float angle = gyro->GetAngle();
		//SmartDashboard::PutNumber("gyro angle", angle);
		SmartDashboard::PutNumber("left y", pilot->LeftY());
		SmartDashboard::PutData("gryo", gyro);
		//float angle = gyro->GetAngle();
		//SmartDashboard::PutNumber("gyro angle", angle);

	}
};

START_ROBOT_CLASS(Robot)
