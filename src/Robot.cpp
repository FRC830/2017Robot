#include "WPILib.h"
#include "CameraServer.h"
#include "Lib830.h"
#include "AnalogGyro.h"

using namespace Lib830;

class Robot: public IterativeRobot
{
	float previousTurn = 0;
	float previousSpeed = 0;
public:
	enum AutoMode {LEFT_SIDE, RIGHT_SIDE, CENTER, BASELINE, NOTHING};
private:
	//drivetrain motors
	static const int LEFT_PWM_ONE = 9;
	static const int LEFT_PWM_TWO = 8;
	static const int RIGHT_PWM_ONE = 0;
	static const int RIGHT_PWM_TWO = 1;

	static const int ANALOG_GYRO = 0;

	//drivetrain
	RobotDrive * drive;

	GamepadF310 * pilot;
	GamepadF310 * copilot;

	Timer timer;

	frc::AnalogGyro *gyro;

	CameraServer * camera;
	LiveWindow *lw = LiveWindow::GetInstance();

	static const int TICKS_TO_ACCEL = 10;

	//auton chooser
	SendableChooser<AutoMode*> *chooser;

	void arcadeDrive(double speed, double turn, bool squaredinputs = false) {
		drive->ArcadeDrive(speed, -turn, squaredinputs);
	}

	void RobotInit()
	{
		drive = new RobotDrive(
			new VictorSP(LEFT_PWM_ONE),
			new VictorSP(LEFT_PWM_TWO),
			new VictorSP(RIGHT_PWM_ONE),
			new VictorSP(RIGHT_PWM_TWO)
		);

		pilot = new GamepadF310(0);
		copilot = new GamepadF310(1);

		gyro = new frc::AnalogGyro(ANALOG_GYRO);

		//autonChooser
		chooser = new SendableChooser<AutoMode*>();
		chooser->AddDefault("baseline", new AutoMode(BASELINE));
		chooser->AddObject("Left Side", new AutoMode(LEFT_SIDE));
		chooser->AddObject("Right Side", new AutoMode(RIGHT_SIDE));
		chooser->AddObject("Center", new AutoMode(CENTER));
		chooser->AddObject("default", new AutoMode(NOTHING));
		SmartDashboard::PutData("Auto Modes", chooser);
		
		//camera stuff
		camera = CameraServer::GetInstance();
		camera->StartAutomaticCapture();

	}

	void AutonomousInit()
	{
		timer.Reset();
		timer.Start();
		gyro->Reset();

		//autonChooser
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);

	}

	void AutonomousPeriodic()
	{
		float time = timer.Get();

		double angle = gyro->GetAngle();
		double turn = angle /-17.0;


		switch(*chooser->GetSelected()){
			case LEFT_SIDE:
				if(time < 3){
					drive->ArcadeDrive(0.5, -0.5, 1);
				}
				break;

			case RIGHT_SIDE:
				if(time < 3){
					drive->ArcadeDrive(0.5, 0.5, 1);
				}
				break;

			case CENTER:
				if (time < 2.0){
					drive->ArcadeDrive(0.5, turn, 1);
				}
				break;

			case BASELINE:
				if (time < 5){
					arcadeDrive(0.5, turn, 1);
				}
				break;
			default:
				arcadeDrive(0.0,turn, false);
		}

		//camera->StartAutomaticCapture();
	}

	void TeleopInit()
	{
		gyro->Reset();
	}

	void TeleopPeriodic()
	{
		float targetTurn = pilot->RightX();
		float turn = accel(previousTurn, targetTurn, TICKS_TO_ACCEL);
		previousTurn = turn;

		float targetForward = pilot->LeftY();
		float speed = accel(previousSpeed, targetForward, TICKS_TO_ACCEL);
		previousSpeed = speed;

		drive->ArcadeDrive(speed, -turn, true);
		double angle = gyro->GetAngle();

		SmartDashboard::PutNumber("gyro angle", angle);




	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
