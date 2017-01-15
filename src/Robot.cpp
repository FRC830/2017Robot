#include "WPILib.h"
#include "CameraServer.h"
#include <GripPipeline.h>
#include <vision/VisionRunner.h>
#include "vision/VisionPipeline.h"
#include "thread"
#include "unistd.h"


class Robot: public IterativeRobot {

private:
	enum AutoMode {ONE, TWO, THREE};

	LiveWindow *lw = LiveWindow::GetInstance();

	//VisionRunner<grip::GripPipeline> * runner;
	//frc::VisionPipeline * v_pipeline;

	//grip::GripPipeline * pipeline;
	SendableChooser<AutoMode*> *chooser;
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
		double hue[] = {0.0, 37.16723549488058};
		double sat[] = {137.58992805755395, 255.0};
		double lum[] = {59.62230215827338, 255.0};

		server = CameraServer::GetInstance();

		server->StartAutomaticCapture().SetResolution(320,240);

		sink = server->GetVideo();
		outputStream = server->PutVideo("Processed", 400, 400);

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

			//cvtColor(image, grey, CV_RGB2GRAY);

			//pipeline->hslThreshold(image, hue, sat, lum, hsl_output);
			pipeline->Process(image);
			outputStream.PutFrame(*pipeline->gethslThresholdOutput());
		}


		//hue = SmartDashboard::GetNumber("P:", 0.075);


	}

	void RobotInit()
	{
		chooser = new SendableChooser<AutoMode*>();
		//runner = new VisionRunner();
		chooser->AddDefault(autoNameDefault, new AutoMode(ONE));
		chooser->AddObject(autoNameCustom, new AutoMode(TWO));
		SmartDashboard::PutData("Auto Modes", chooser);


		//outputStream = server->PutVideo("Processed", 400, 400 );
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

	void TeleopPeriodic()
	{
		//pipeline-> Process(hsl_output);
		//outputStream.PutFrame(hsl_output);
		//server->GetVideo();


	}
	void TestPeriodic()
	{
		lw->Run();
	}
};

START_ROBOT_CLASS(Robot)
