/*
 * GripPipeline.cpp
 *
 *  Created on: Jan 10, 2017
 *      Author: RatPack
 */
#include "GripPipeline.h"
/**
* Initializes a GripPipeline.
*/

namespace grip {

GripPipeline::GripPipeline() {
}
/**
* Runs an iteration of the Pipeline and updates outputs.
*
* Sources need to be set before calling this method.
*
*/
void GripPipeline::Process(cv::Mat &source0){
	//Step HSL_Threshold0:
	//input
	cv::Mat hslThresholdInput = source0;
	double hslThresholdHue[] = {45.32374100719424, 100.13651877133105};
	double hslThresholdSaturation[] = {57.32913669064748, 255.0};
	double hslThresholdLuminance[] = {133.00359712230215, 255.0};
	hslThreshold(hslThresholdInput, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, this->hslThresholdOutput);
	//Step Find_Contours0:
	//input
	cv::Mat findContoursInput = hslThresholdOutput;
	bool findContoursExternalOnly = true;  // default Boolean
	findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);

	//cv::Mat drawOutputContours = source0;
	cv::Scalar color(255,128,0);
	std::sort(findContoursOutput.begin(), findContoursOutput.end(), FindContourArea);

	findContoursOutput.resize(2);

	//DrawContours(source0,findContoursOutput,-1, color);
	DrawContours(source0,findContoursOutput,-1, color);

	//cv::Mat newMoment;
	/*double moment10,moment01, moment00;
	cv::Point pt1;
	cv::Point pt2;
	cv::Moments moment;
	cv::cvMoments(findCountoursOutput, &moments);*/
	/*moment10 = moment.m10;
	moment01 = moment.m01;
	moment00 = moment.m00;
	double x = (int)(moment10/moment00);
	double y = (int)(moment01/moment00);
	pt1 = cv::Point(x,y);

	cv::line(source0, pt1, pt1, color, 5); */
	std::vector<cv::Moments> mu( findContoursOutput.size());
	for (int i = 0; i < (int)(findContoursOutput.size()); i++) {
		mu[i] = moments(findContoursOutput[i],false);
	}
	std::vector<cv::Point2f> mc(findContoursOutput.size());
	for (int i =  0; i < (int)(findContoursOutput.size()); i++) {
		mc[i] = cv::Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
	}
	cv::line(source0, mc[0], mc[1], color, 5);
}

/**
 * This method is a generated setter for source0.
 * @param source the Mat to set
 */
void GripPipeline::setsource0(cv::Mat &source0){
	source0.copyTo(this->source0);
}
/**
 * This method is a generated getter for the output of a HSL_Threshold.
 * @return Mat output from HSL_Threshold.
 */
cv::Mat* GripPipeline::gethslThresholdOutput(){
	return &(this->hslThresholdOutput);
}
/**
 * This method is a generated getter for the output of a Find_Contours.
 * @return ContoursReport output from Find_Contours.
 */
std::vector<std::vector<cv::Point> >* GripPipeline::getfindContoursOutput(){
	return &(this->findContoursOutput);
}

	/**
	 * Segment an image based on hue, saturation, and luminance ranges.
	 *
	 * @param input The image on which to perform the HSL threshold.
	 * @param hue The min and max hue.
	 * @param sat The min and max saturation.
	 * @param lum The min and max luminance.
	 * @param output The image in which to store the output.
	 */
	//void hslThreshold(Mat *input, double hue[], double sat[], double lum[], Mat *out) {
	void GripPipeline::hslThreshold(cv::Mat &input, double hue[], double sat[], double lum[], cv::Mat &out) {
		cv::cvtColor(input, out, cv::COLOR_BGR2HLS);
		cv::inRange(out, cv::Scalar(hue[0], lum[0], sat[0]), cv::Scalar(hue[1], lum[1], sat[1]), out);
	}

	/**
	 * Finds contours in an image.
	 *
	 * @param input The image to find contours in.
	 * @param externalOnly if only external contours are to be found.
	 * @param contours vector of contours to put contours in.
	 */
	void GripPipeline::findContours(cv::Mat &input, bool externalOnly, std::vector<std::vector<cv::Point> > &contours) {
		std::vector<cv::Vec4i> hierarchy;
		contours.clear();
		int mode = externalOnly ? cv::RETR_EXTERNAL : cv::RETR_LIST;
		int method = cv::CHAIN_APPROX_SIMPLE;
		cv::findContours(input, contours, hierarchy, mode, method);
	}

	void GripPipeline::DrawContours(cv::Mat &imageOutput, std::vector<std::vector<cv::Point>> &points, int idx, const cv::Scalar &color) {

		cv::drawContours(imageOutput, points, idx, color, 5);
	}
	bool GripPipeline::FindContourArea(std::vector<cv::Point> &contour1, std::vector<cv::Point> &contour2) {
		double i = fabs(cv::contourArea(cv::Mat(contour1)));
		double j = fabs(cv::contourArea(cv::Mat(contour2)));
		return (i > j);
	}


} // end grip namespace





