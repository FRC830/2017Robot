/*
 * GripPipeline.h
 *
 *  Created on: Jan 10, 2017
 *      Author: RatPack
 */
#include "WPILib.h"
#ifndef SRC_GRIPPIPELINE_H_
#define SRC_GRIPPIPELINE_H_
#pragma once

#include "vision/VisionPipeline.h"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <map>
#include <vector>
#include <string>
#include <math.h>

namespace grip {

/**
* GripPipeline class.
*
* An OpenCV pipeline generated by GRIP.
*/
class GripPipeline {
	public:
		GripPipeline();
		void Process(cv::Mat &source0);
		void setsource0(cv::Mat &source0);
		cv::Mat* gethslThresholdOutput();
		std::vector<std::vector<cv::Point> >* getfindContoursOutput();
		void hslThreshold(cv::Mat &, double [], double [], double [], cv::Mat &);
		void findContours(cv::Mat &, bool , std::vector<std::vector<cv::Point> > &);
		void DrawContours(cv::Mat &imageOutput, std::vector<std::vector<cv::Point>> &points, int idx, const cv::Scalar &color);

	private:
			cv::Mat hslThresholdOutput;
			cv:: Mat source0;
			std::vector<std::vector<cv::Point> > findContoursOutput;
};


} // end namespace grip







#endif /* SRC_GRIPPIPELINE_H_ */
