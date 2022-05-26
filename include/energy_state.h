#pragma once
//
// Created by liyankuan on 2022/1/17.
//

#ifndef ENERGY_ENERGY_H
#define ENERGY_ENERGY_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include "ArmorDetector.hpp"
#include "robot_state.h"

//#define PI       3.1415926535897932384626433
//#define SPEED    28.00//单位m，子弹速度

//enum EnemyColor { RED = 0, BLUE = 1 };

//extern cv::Mat  quan_src;
//extern float quan_ab_pitch;
//extern float quan_ab_yaw;
//extern float quan_ab_roll;
//extern float quan_speed;

struct energy_cnt
{
	std::vector<cv::Point>cnt;
	int index;
};


class energy:public robot_state
{
private:
	
	double wh_min_ratio = 1.25;
	double wh_max_ratio = 1.85;


public:
	energy() = default;
	
	
	//开始转的点
	cv::Point start_p = {-1,-1};
	//上一时刻的点
	cv::Point last_p;//这个last_p是用来算转动方向的所以不用重置，也不需要初值
	cv::Point last_dt_p = { -1,-1 };
	//用来算圆心和半径的点
	std::vector<cv::Point2f>three_points;
	//半径
	double radius = 0;
	//大符圆心
	cv::Point R_center = { -1,-1 };
	
	int count = 0;
	bool lose_aim = false;
	
	int direct =-1;//1为顺时针，0为逆时针
	
	int hited = 0;//用来记录是否有已打中的大符
	bool hit = false;//记录本次大符识别是否打中
	
	
	cv::Rect roi;
	//时间量
	long int start_time = -1;
	long int last_time = 0;//这两个是gettickcount得到的运行次数整数
	double t = 0;
	double dt = 0;
	
	//大符装甲板宽高
	double w_std = 0.230;//单位是米
	double h_std = 0.127;
	//存储大符装甲板矩形的四个角点
	cv::Point2f p[4];
	
	//上一时刻角速度,在代码里有些地方是作为当前时刻的角度存在的
	double l_angle = 0.0;
	
	//上一时刻的角度,在代码里有些地方是作为当前时刻的角度存在的
	double last_angle = 0.0;
	
//	int cr_count = 0;
	//换象限判断，判断当前点在圆心左边还是右边
	int pos = 0;//0在左,1在右
	
	int get_direct(cv::Point &now_p);//获取大符转动方向
	
	
	cv::Point detect_aim(cv::Mat& src);//识别要击打的装甲板
	
	cv::Mat set_image(cv::Mat &src);
	
	void make_safe(cv::Rect &rect, cv::Mat &src);
	
	
	static inline bool center_area(std::vector<cv::Point> contour1,std::vector<cv::Point> contour2)
	{
		return (cv::contourArea(contour1) > cv::contourArea(contour2));
	}
	
	//还需要一个清零预测参数的函数,这个函数在energy_predict里
};
#endif //ENERGY_ENERGY_H
