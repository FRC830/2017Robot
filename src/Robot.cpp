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

	LiveWindow *lw = LiveWindow::GetInstance();

	SendableChooser<AutoMode*> *chooser;
	static const int TICKS_TO_ACCEL = 10;


	void arcadeDrive(double speed, double turn, bool squaredinputs = false) {
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
					arcadeDrive(0.5, -0.5, 1);
				}
				break;

			case RIGHT_SIDE:
				if(time < 3){
					arcadeDrive(0.5, 0.5, 1);
				}
				break;

			case CENTER:
				if (time < 2.75){
					arcadeDrive(0.5, turn, 1);
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

	}

	void TeleopInit()
	{
		gyro->Reset();

	}

	void TeleopPeriodic()
	{

		float targetSpeed = pilot->LeftY();
		float speed = accel(previousSpeed, targetSpeed, TICKS_TO_ACCEL);
		previousSpeed = speed;

		double angle = gyro->GetAngle();

		SmartDashboard::PutNumber("gyro angle", angle);


		/*float targetTurn = pilot->LeftX();
		float turn = Lib830::accel(previousTurn, targetTurn, 30);
		previousTurn = targetTurn; */
		double mid_point = SmartDashboard::GetNumber("x value between bars",0);
		float targetTurn;

		if (pilot ->ButtonState(Lib830::GamepadF310::BUTTON_RIGHT_BUMPER)) {
			targetTurn = (160.0 - mid_point) /-60.0;
		}
		else {
			targetTurn = pilot->RightX();
		}
		float turn = Lib830::accel(previousTurn, targetTurn, 10);
		previousTurn = targetTurn;

		arcadeDrive(speed/1.5, turn/2.0, true);

	}
	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
