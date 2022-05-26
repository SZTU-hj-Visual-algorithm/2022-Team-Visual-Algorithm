#include <iostream>
#include "energy_predict.h"
#include <opencv2/opencv.hpp>


using namespace cv;

int main()
{
	energy_pre E_predicter;
	cv::Mat src;
	VideoCapture cap("../3低曝光红.mp4");
	
	while(1)
	{
		cap.read(src);
//		if (E_predicter.limit_s == 0)
//		{
//			E_predicter.limit_s = (double)getTickCount();
//		}
//		else
//		{
//			E_predicter.limit_t = ((double)getTickCount() - E_predicter.limit_s)/getTickFrequency();
//		}
//
//		if (E_predicter.limit_t>3.3)
//		{
//			E_predicter.hit = false;
//			E_predicter.reset();
//		}
		
		if (E_predicter.energy_detect(src))
		{
//			std::cout<<"预测yaw角："<<E_predicter.E_yaw<<std::endl;
//			std::cout<<"预测pitch角："<<E_predicter.E_pitch<<std::endl;
		}
		else
		{
//			imshow("Src",src);
		}

		if (cv::waitKey(5) == 27)
		{
			break;
		}
	}
	return 0;
}



