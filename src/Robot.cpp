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
	static const int LEFT_PWM_ONE = 9;
	static const int LEFT_PWM_TWO = 8;
	static const int RIGHT_PWM_ONE = 0;
	static const int RIGHT_PWM_TWO = 1;

	static const int TEST_PWM = 3;

	//drivetrain
	RobotDrive * drive;

	Spark * testMotor;

	GamepadF310 * pilot;
	GamepadF310 * copilot;

	Timer * timer;

	CameraServer * camera;
	LiveWindow *lw = LiveWindow::GetInstance();

	//auton chooser
	SendableChooser<AutoMode*> *chooser;
	const std::string autoNameDefault = "Baseline";
	const std::string autoNameCustom = "Left Side";
	//const std::string autoNameCustom = "Right Side";
	//const std::string autoNameCustom = "Center";
	std::string autoSelected;


	void RobotInit()
	{
		drive = new RobotDrive(
			new VictorSP(LEFT_PWM_ONE),
			new VictorSP(LEFT_PWM_TWO),
			new VictorSP(RIGHT_PWM_ONE),
			new VictorSP(RIGHT_PWM_TWO)
		);
		/*drive = new RobotDrive (
			new VictorSP(LEFT_PWM_ONE),
			new VictorSP(LEFT_PWM_TWO)
		);*/

		pilot = new GamepadF310(0);
		copilot = new GamepadF310(1);

		testMotor = new Spark(3);

		//autonChooser
		chooser = new SendableChooser<AutoMode*>();
		chooser->AddDefault(autoNameDefault, new AutoMode(BASELINE));
		chooser->AddObject("Left Side", new AutoMode(LEFT_SIDE));
		chooser->AddObject("Right Side", new AutoMode(RIGHT_SIDE));
		chooser->AddObject("Center", new AutoMode(CENTER));
		SmartDashboard::PutData("Auto Modes", chooser);
		
		//camera stuff
		camera = CameraServer::GetInstance();
		camera->StartAutomaticCapture();

	}

	void AutonomousInit()
	{
		timer->Reset();
		timer->Start();

		//autonChooser
		autoSelected = *((std::string*)chooser->GetSelected());
		std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

	}

	void AutonomousPeriodic()
	{
		float time = timer->Get();

		/*switch(autoSelected){
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
					drive->ArcadeDrive(0.5, 0, 1);
				}
			break;

			default:
				if (time < 5){
					drive->ArcadeDrive(0.4, 0.0, 1);
				}
			break;
		}*/

		//camera->StartAutomaticCapture();
	}

	void TeleopInit()
	{

	}

	void TeleopPeriodic()
	{
		float targetTurn = pilot->RightX();
		float turn = accel(previousTurn, targetTurn, 5);
		previousTurn = turn;

		float targetForward = pilot->LeftY();
		float speed = accel(previousSpeed, targetForward, 5);
		previousSpeed = speed;

		drive->ArcadeDrive(speed, turn, true);




	}

	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
