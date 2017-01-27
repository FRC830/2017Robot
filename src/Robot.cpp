#include "WPILib.h"
#include "CameraServer.h"
#include <GripPipeline.h>
#include <vision/VisionRunner.h>
#include "vision/VisionPipeline.h"
#include "thread"
#include "unistd.h"
#include "input/GamepadF310.h"
#include "util/Algorithms.h"

class Robot: public IterativeRobot {

private:
	enum AutoMode {ONE, TWO, THREE};
	static const int LEFT_PWM = 3;
	static const int RIGHT_PWM = 4;
	static bool set_exposure;
	static bool previous_exposure;

	LiveWindow *lw = LiveWindow::GetInstance();

	//VisionRunner<grip::GripPipeline> * runner;
	//frc::VisionPipeline * v_pipeline;

	//grip::GripPipeline * pipeline;
	SendableChooser<AutoMode*> *chooser;
	RobotDrive * drive;
	VictorSP * left_motor;
	VictorSP * right_motor;
	Lib830::GamepadF310 * pilot;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;


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

			//cvtColor(image, grey, CV_RGB2GRAY);

			//pipeline->hslThreshold(image, hue, sat, lum, hsl_output);
			pipeline->Process(image);
			//outputStream.PutFrame(*pipeline->gethslThresholdOutput());
			outputStream.PutFrame(image);
		}


		//hue = SmartDashboard::GetNumber("P:", 0.075);


	}

	void RobotInit()
	{
		chooser = new SendableChooser<AutoMode*>();
		drive = new RobotDrive(
				new VictorSP(LEFT_PWM),
				new VictorSP(RIGHT_PWM)
		);

		pilot = new Lib830::GamepadF310(0);
		//runner = new VisionRunner();
		chooser->AddDefault(autoNameDefault, new AutoMode(ONE));
		chooser->AddObject(autoNameCustom, new AutoMode(TWO));
		SmartDashboard::PutData("Auto Modes", chooser);


		//outputStream = server->PutVideo("Processed", 400, 400 );
		//set_exposure = false;
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
		autoSelected = *((std::string*)chooser->GetSelected());
		//std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);
		std::cout << "Auto selected: " << autoSelected << std::endl;

		if(autoSelected == autoNameCustom){
			//Custom Auto goes here
		} else {
			//Default Auto goes here
		}
	}

	void AutonomousPeriodic()
	{
		if(autoSelected == autoNameCustom){
			//Custom Auto goes here
		} else {
			//Default Auto goes here
		}

	}

	void TeleopInit()
	{
		//std::thread visionThread();
		//visionThread.detach();
	}
	float previousForward = 0;
	float previousTurn = 0;

	void TeleopPeriodic()
	{
		float targetForward = -(pilot->RightY());
		float forward = Lib830::accel(previousForward, targetForward, 20);
		previousForward = targetForward;

		/*float targetTurn = pilot->LeftX();
		float turn = Lib830::accel(previousTurn, targetTurn, 30);
		previousTurn = targetTurn; */

		double mid_point = SmartDashboard::GetNumber("x value between bars",0);

		float targetTurn = (160.0 - mid_point) /-60.0;
		float turn = Lib830::accel(previousTurn, targetTurn, 10);
		previousTurn = targetTurn;

		/*if (pilot->ButtonState(Lib830::GamepadF310::BUTTON_RIGHT_BUMPER)) {
			turn = (160.0 - mid_point )/ -60.0;
			set_exposure = true;
		}
		else {
			set_exposure = false;
		} */

		drive->ArcadeDrive(forward/2.0, turn, true);


	}
	void TestPeriodic()
	{
		lw->Run();
	}
};

bool Robot::set_exposure = false;
bool Robot::previous_exposure = false;

START_ROBOT_CLASS(Robot)
