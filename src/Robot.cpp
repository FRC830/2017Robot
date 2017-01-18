#include "WPILib.h"
#include "CameraServer.h"
#include "Lib830.h"

using namespace Lib830;

class Robot: public IterativeRobot
{
	float previousTurn = 0;
	float previousSpeed = 0;
public:
	enum AutoMode {LEFT_SIDE, RIGHT_SIDE, CENTER, BASELINE};
private:
	//drivetrain motors
	static const int LEFT_PWM_ONE = 0;
	static const int LEFT_PWM_TWO = 1;
	static const int RIGHT_PWM_ONE = 2;
	static const int RIGHT_PWM_TWO = 3;

	//gear shifting
	static const DoubleSolenoid::Value LOW = DoubleSolenoid::kForward;
	static const DoubleSolenoid::Value HIGH = DoubleSolenoid::kReverse;

	//drivetrain
	RobotDrive * drive;
	VictorSP * frontLeft;
	VictorSP * backLeft;
	VictorSP * frontRight;
	VictorSP * backRight;
	DoubleSolenoid * gear_shift;

	GamepadF310 * pilot;
	GamepadF310 * copilot;

	Timer * timer;

	CameraServer * camera;
	LiveWindow *lw = LiveWindow::GetInstance();

	//auton chooser
	SendableChooser<AutoMode*> *chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;


	void RobotInit()
	{
		drive = new RobotDrive(
			new VictorSP(LEFT_PWM_ONE),
			new VictorSP(LEFT_PWM_TWO),
			new VictorSP(RIGHT_PWM_ONE),
			new VictorSP(RIGHT_PWM_TWO)
		);

		//gear shifting
		enum PCM_id {
			GEAR_SHIFT_SOL_FORWARD = 0,
			GEAR_SHIFT_SOL_REVERSE = 1,
		};
		gear_shift = new DoubleSolenoid(GEAR_SHIFT_SOL_FORWARD, GEAR_SHIFT_SOL_REVERSE);

		pilot = new GamepadF310(0);
		copilot = new GamepadF310(1);

		//autonChooser
		chooser = new SendableChooser<AutoMode*>();
		chooser->AddDefault(autoNameDefault, new AutoMode(BASELINE));
		chooser->AddObject(autoNameCustom, new AutoMode(LEFT_SIDE));
		chooser->AddObject(autoNameCustom, new AutoMode(RIGHT_SIDE));
		chooser->AddObject(autoNameCustom, new AutoMode(CENTER));
		SmartDashboard::PutData("Auto Modes", chooser);
		
		//camera stuff
		camera = CameraServer::GetInstance();
		camera->StartAutomaticCapture();

	}

	void AutonomousInit()
	{
		timer->Reset();
		timer->Start();

		gear_shift->Set(LOW);

		//autonChooser
		autoSelected = *((std::string*)chooser->GetSelected());
		std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

	}

	void AutonomousPeriodic()
	{
		float time = timer->Get();

		switch(autoSelected){
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
				if (time < 4.0){
					drive->ArcadeDrive(0.5, 0, 1);
				}
			break;

			default:
				if (time < 5){
					drive->ArcadeDrive(0.4, 0.0, 1);
				}
			break;
		}

		camera->StartAutomaticCapture();
	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		float targetTurn = pilot->RightX();
		float turn = accel(previousTurn, targetTurn, 20);
		previousTurn = turn;
		float targetForward = pilot->LeftY();
		float speed = accel(previousSpeed, targetForward, 30);
		previousSpeed = speed;

		drive->ArcadeDrive(speed, turn);

		if (pilot->LeftTrigger() > 0.5){
			gear_shift->Set(LOW);
		} else {
			gear_shift->Set(HIGH);
		}
		SmartDashboard::PutString("gear", gear_shift->Get() == LOW ? "low" : "high");

	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
