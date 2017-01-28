/*
 * GripPipeline.cpp
 *
 *  Created on: Jan 10, 2017
 *      Author: RatPack
 */
#include "GripPipeline.h"
#include "WPILib.h"
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
	/*double hslThresholdHue[] = {40, 80};
	double hslThresholdSaturation[] = {57, 127};
	double hslThresholdLuminance[] = {225, 255}; */
	 double hslThresholdHue[] = {0,180};
     double hslThresholdSaturation[] = {28, 255.0};
     double hslThresholdLuminance[] = {240, 255.0};


	double filterContoursMaxVertecies = 55;
	double filterContoursMinVertecies = 1;

	hslThreshold(hslThresholdInput, hslThresholdHue, hslThresholdSaturation, hslThresholdLuminance, this->hslThresholdOutput);
	cv::Mat findContoursInput = hslThresholdOutput;
	bool findContoursExternalOnly = true;  // default Boolean

	findContours(findContoursInput, findContoursExternalOnly, this->findContoursOutput);
	cv::Scalar color(255,128,0);
//	cv::Scalar color(255,128,0);

	std::vector<std::vector<cv::Point>> smoothContours(findContoursOutput.size());
	for (int i = 0; i < (int) (findContoursOutput.size()); i++) {
		cv::approxPolyDP(findContoursOutput[i], smoothContours[i], 4, true);
	}

	filterContourVertecies(smoothContours, filterContoursMaxVertecies, filterContoursMinVertecies, this->filterContourVerteciesOutput);

	smoothContours = filterContourVerteciesOutput;

	//DrawContours(source0,findContoursOutput,-1, color);

	//DrawContours(source0,smoothContours,-1, color);

	cv::Scalar color_2(255,255,0);
	std::vector<cv::Rect> boundRect(smoothContours.size());



	for (int i = 0; i < (int)(smoothContours.size()); /*nothing*/) {
		boundRect[i] = cv::boundingRect(cv::Mat(smoothContours[i]));
		double width = boundRect[i].width;
		double height = boundRect[i].height;
		double ratio = width/height;
		if ( ratio < 0.30 || ratio > 0.55) {
			smoothContours.erase (smoothContours.begin() + i);
			boundRect.erase(boundRect.begin() + i);
		}
		else {
			i++;
		}
	}

	if (smoothContours.size() < 2 || boundRect.size() < 2) {
		return;
	}
	else {
		std::sort(smoothContours.begin(), smoothContours.end(), FindContourArea);
		boundRect.resize(2);
		smoothContours.resize(2);
	}

	for (int i = 0; i < 2; i++ ) {
		boundRect[i] = cv::boundingRect(cv::Mat(smoothContours[i]));
	}


	for (int i = 0; i < (int)(smoothContours.size()); i++) {
		cv::rectangle(source0, boundRect[i].tl(), boundRect[i].br(), color_2, 2);
	}

	//DrawContours(source0, smoothContours,-1, color);

	SmartDashboard::PutNumber("ratio 1", double(boundRect[0].width)/ double(boundRect[0].height) );
	SmartDashboard::PutNumber("ratio 2", double(boundRect[1].width)/double(boundRect[1].height) );

	cv::Point center;

	if (boundRect.size() == 2) {
		cv::Point top_left = boundRect[0].tl();
		cv::Point bottom_right = boundRect[1].br();
		center = (top_left + bottom_right)/2;
		cv::line(source0, top_left, bottom_right, color_2, 3);
		cv::line(source0, center, center, color, 5);
	}

	/*std::vector<cv::Moments> mu( smoothContours.size());
	for (int i = 0; i < (int)(smoothContours.size()); i++) {
		mu[i] = moments(smoothContours[i],false);
	}
	std::vector<cv::Point2f> mc(smoothContours.size());
	for (int i =  0; i < (int)(smoothContours.size()); i++) {
		mc[i] = cv::Point2f(mu[i].m10/mu[i].m00, mu[i].m01/mu[i].m00);
	}
	cv::line(source0, mc[0], mc[1], color, 5);
	cv::Point mid_point = (mc[0] + mc[1])/2;
	cv::line(source0, mid_point, mid_point,color_2,5 ); */



	cv::Scalar color_3 (0,255,255);
	cv::line(source0, cv::Point2f(160,0), cv::Point2f(160,240),color_3,2 );
	cv::line(source0, cv::Point2f(0,120), cv::Point2f(320,120), color_3,2);


	SmartDashboard::PutNumber("x value between bars", center.x);
	SmartDashboard::PutNumber("middle x value", 160);
	//SmartDashboard::PutNumber("height", height); //240
	//SmartDashboard::PutNumber("width", width); //320
	//return mid_point.x;

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

std::vector<std::vector<cv::Point> >* GripPipeline::getfilterContoursOutput() {
	return &(this->filterContourVerteciesOutput);
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

		cv::drawContours(imageOutput, points, idx, color, 3);
	}
	bool GripPipeline::FindContourArea(std::vector<cv::Point> &contour1, std::vector<cv::Point> &contour2) {
		double i = fabs(cv::contourArea(cv::Mat(contour1)));
		double j = fabs(cv::contourArea(cv::Mat(contour2)));
		return (i > j);
	}

	void GripPipeline::filterContourVertecies(std::vector <std::vector <cv::Point>> &contours, double maxVertexCount, double minVertexCount, std::vector<std::vector <cv::Point>> &output) {
		output.clear();
		for (std::vector <cv::Point> contour: contours) {
			if (contour.size() < minVertexCount || contour.size() > maxVertexCount) {
				continue;
			}
			output.push_back(contour);
		}
	}


} // end grip namespace





