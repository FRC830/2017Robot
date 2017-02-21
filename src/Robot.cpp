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

using namespace Lib830;

class Robot: public IterativeRobot
{
	float previousTurn = 0;
	float previousSpeed = 0;
public:
	enum AutoMode {LEFT_SIDE, RIGHT_SIDE, CENTER, BASELINE, NOTHING, BAD_GYRO};
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

	static const int ANALOG_GYRO = 0;

	static const int RED_LED_DIO = 5;
	static const int GREEN_LED_DIO = 6;
	static const int BLUE_LED_DIO = 7;

	static const int CLIMBING_SWITCH_DIO = 0;


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
		outputStream = server->PutVideo("Processed", 400, 400);
		//outputStream.
		//camera.SetExposureManual(80);


		while(1) {
			bool working = sink.GrabFrame(image_temp);
			SmartDashboard::PutBoolean("working", working);
			/*if (set_exposure && !previous_exposure) {
				camera.SetExposureManual(10);
			}
			else if(!set_exposure && previous_exposure) {
				camera.SetExposureManual(80);
			}
			previous_exposure = set_exposure; */
			if (working) {
				g_frame ++;
				image = image_temp;
			}
			if (g_frame < 1) {
				continue;
			}

			//pipeline->hslThreshold(image, hue, sat, lum, hsl_output);
			pipeline->Process(image);
			//outputStream.PutFrame(*pipeline->gethslThresholdOutput());
			outputStream.PutFrame(image);
		}


		//hue = SmartDashboard::GetNumber("P:", 0.075);
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
		SmartDashboard::PutData("Auto Modes", chooser);
		
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

		//autonChooser
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);

	}

	const float WEIGHT_AFTER_ONE_SECOND = .1;

	float prev_time = 0;
	float prev_speed = 0;
	float prev_turn = 0;

	float prev_process_success_time = 0;
	float prev_process_success_turn = 0;
	float process_success = 0;

	void AutonomousPeriodic()

	{
		float time = timer.Get();
		float prev_weight = pow(WEIGHT_AFTER_ONE_SECOND, time - prev_time);
		float cur_weight = 1 - prev_weight;

		float speed = 0;

		double angle = gyro->GetAngle();

		float turn = angle /-17.0;

		float processed_turn = ProcessTargetTurn(0.1);
		//arcadeDrive(0.0, 0.0);

		AutoMode mode = BASELINE;

		if(chooser->GetSelected()) {
			mode = *chooser->GetSelected();
		}

		if (mode == LEFT_SIDE || mode == RIGHT_SIDE || mode == CENTER) {
			speed = 0.3;
			if (time < 1) {
				if (mode != CENTER) {
					speed = 0.6;
				}
				//arcadeDrive(speed, turn);
			}
			else if (time >= 1 && time < 7) {
				if (processed_turn !=0) {
					process_success += 1;
					if (process_success > 1) {
						process_success = 1;
					}
					prev_process_success_turn = processed_turn;
					prev_process_success_time = time;
					//speed = 0.3;
				}
				else if (process_success > 0 && ((time - prev_process_success_time) < 0.25)) {
					turn = prev_process_success_turn;
				}
				else if (mode == LEFT_SIDE) {
					turn = (angle - 60) / -60.0; //opposite turn angle
				}
				else if (mode == RIGHT_SIDE) {
					turn = (angle + 30) / -60.0;
				}
				//arcadeDrive(speed, turn, false);
			}
			else if(time < 5) {
				if (mode == LEFT_SIDE || mode == RIGHT_SIDE) {
					speed = 0.5;
					turn = processed_turn;
					//arcadeDrive(0.5, processed_turn);
				}
			}
			else if (time > 7) {
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
			if (time < 2) {
				turn = 0.2;
				//arcadeDrive(speed, turn, false);
			}
			else if (time > 2 && time <= 6) {
				if (processed_turn != 0) {
					turn = processed_turn;
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


		float accel_turn = (prev_weight * prev_turn) + (cur_weight * turn);
		//float accel_turn = turn;
		arcadeDrive(speed, accel_turn, false);
		prev_turn = turn;
		prev_time = time;



		SmartDashboard::PutNumber("processed turn", processed_turn);
		SmartDashboard::PutNumber("turn", turn);
		SmartDashboard::PutNumber("previous weight", prev_weight);

		SmartDashboard::PutNumber("accel_turn", accel_turn);
		SmartDashboard::PutNumber("time", time);
		SmartDashboard::PutNumber("gyro angle", angle);

	}

	void TeleopInit()
	{
		gyro->Reset();
		LED->Disable();


	}
	bool invert = false;
	void TeleopPeriodic()
	{

		float targetSpeed = pilot->LeftY();
		if (fabs(targetSpeed) < 0.13) {
			targetSpeed = 0;
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

		float turn = accel(previousTurn, targetTurn, 10);
		previousTurn = targetTurn;
		SmartDashboard::PutNumber("real turn", turn);

		if (pilot->ButtonState(GamepadF310::BUTTON_X)) {
			//arcadeDrive(speed, turn, false, true);
			invert = !invert;
		}

		if (invert == true) {
			LED->Set(0.5,0.0,0.5);
		}
		else {
			LED->SetAllianceColor();
		}

		arcadeDrive(speed, turn, false, invert);

		SmartDashboard::PutBoolean("Climber Switch", climbingSwitch->Get());
		if (climbingSwitch->Get() == false) {
			climber->Set(copilot->LeftTrigger()-copilot->RightTrigger());
		}

		//LED->SetAllianceColor();
		//LED->Set(0,1,0);
		if (copilot->ButtonState(GamepadF310::BUTTON_B)) {
			LED->Set(0,1,0);

		}

//		LED->Set(copilot->LeftTrigger(), copilot->RightTrigger(), fabs(copilot->LeftY()));
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
		DigitalLED::Color imperfectYellow = {1,0.6,0};
		LED->Alternate(imperfectYellow, {0,0,1});
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
