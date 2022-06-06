//
// Created by liyankuan on 2022/2/11.
//
//这个文件是用来将所有大符所需的函数集合起来的文件
#include "energy_predict.h"
#include "energy_state.h"
#include <opencv2/opencv.hpp>
#include <iostream>

//#define BEIZHU

using namespace cv;
using namespace std;


bool energy_pre::energy_detect(Mat &src, int color) {
	
	Rect r(210,200,840,824);
	Mat img = src(r).clone();
	this->enermy_color = color;
	//std::cout<<"已击中："<<hited<<std::endl;
	//if (hited / 3 == 5) {

	//	return false;
	//}
//	cout<<img.size<<endl;
//	image = set_image(img);

	image = img;
	//目标大符装甲板检测
	Aim_armor = detect_aim(image);
	if (Aim_armor.x == 0 && Aim_armor.y == 0) {
//		reset();
		lose_aim = true;
		return false;
	}
//	else
//	{
//		circle(src, Aim_armor, 3, Scalar(255, 0, 0), 5);
//		imshow("image",image);
//		return true;
//	}
	
//	Eigen::Vector3d aim2 = pnp_get_pc(p);
	//double del_ta = pow(SPEED, 4) + 2 * 9.8 * aim2(1, 0) * SPEED * SPEED - 9.8 * 9.8 * aim2(2,0)*aim2(2,0);
	
	//double t_2 = (9.8 * aim2(1, 0) + SPEED * SPEED - sqrt(del_ta)) / (0.5 * 9.8 * 9.8);

	//double height = 0.5 * 9.8 * t_2;

//	E_pitch = atan2(aim2(1, 0), aim2(2, 0)) / CV_PI * 180.0 - 0/*- quan_ab_pitch*/;
//	E_yaw = atan2(aim2(0, 0), aim2(2, 0)) / CV_PI * 180.0 - 0/*- quan_ab_yaw*/;

//	last_dt_p = Aim_armor;
	//Eigen::Vector3d ap_g = {ap_pre(0,0),ap_pre(1,0) - height,depth};
	//E_pitch = atan2(ap_g(1,0),ap_g(2,0));
	//E_yaw = atan2(ap_g(0,0),ap_g(2,0));
	
	return true;
}


bool energy_pre::energy_predict_aim(long int now_time, bool small_energy) {
	if (start_time == -1)
	{
		start_time = now_time;//大符开始计时的时刻
		last_time = now_time;//上一时刻
	}
	else
	{
		t = ((double)(now_time - start_time)) / getTickFrequency();
//		cout<<"总时间："<<t<<endl;
		if (cal_dela_angle())
		{
		    if (hit)
		    {
		        //		cout<<"累计时间："<<t<<endl;
		        //		cout<<"hit(上一发是否打中了)："<<hit<<endl;
		        hit_reset();
		        hit = false;//上一时刻发弹打中了，不用return，继续这一时刻的预测即可
		    }
		    else
		    {
		        hit = false;
		        hited = 0;
		        reset();
		        return false;
		    }

		}
//		std::cout<<"总时间："<<t<<endl;
		dt = ((double) (now_time - last_time)) / getTickFrequency();
//		std::cout<<"时间："<<dt<<std::endl;
		last_time = now_time;
	}
	

	
	if (start_p.x == -1 && start_p.y == -1)
	{
		start_p = Aim_armor;
		start_c = R_center;
	}
	
	if (count == 0) {
		last_p = Aim_armor;
		count++;
	} else if (count == 10) {
		direct = get_direct(Aim_armor);
		
		count++;
	} else if (count< 10)
	{
		count++;
	}
	
	double angle, depth;
	Eigen::Vector3d ap,ap_c;
	ap = pnp_get_pc(pp,0.220,0.122);
	ap_c = pnp_get_pc(p,w_std,h_std);
	if (ap_c(2,0) > 10.5)
	{
		depth = ap_c(2, 0)*0.007 + depth;
	}
	else
	{
		depth = ap_c(2,0);
	}
	
	//上一时刻的点设置好
	if (last_dt_p.x == -1 && last_dt_p.y == -1) {
		last_dt_p = Aim_armor;
		return false;
	}
	else {
		if ((R_center.x!=0)&&(R_center.y!=0)&&(depth>5)&&(depth<13))
		{
			angle = measured(Aim_armor);
//			cout<<"观测值："<<angle<<endl;
			cout << "转动方向：" << direct << endl;
		
			double predict_angle;
			predict(t, dt, true, small_energy);//更新步前必要的更新参数用
			double cor = correct(angle);//更新步
			//std::cout<<"更新角度："<<cor<<std::endl;
			cv::Point pre_aim;
			double p_t =sqrt(ap(0,0)*ap(0,0)+ap(1,0)*ap(1,0)+depth*depth) / SPEED;
			predict_angle = predict(t , p_t + shoot_delay, false,small_energy);
//			cout<<predict_angle<<endl;
			double pred_ang = predict_angle - cor;
			//std::cout<<"预测角度："<<pred_ang<<std::endl;
			pre_aim = angle2_xy(Aim_armor,pred_ang);
//			std::cout<<"predict_aim:"<<pre_aim<<std::endl;
			cv::Point mubiao = gravity_finish(pre_aim, ap, depth);
			circle(image,mubiao,8,Scalar(255,255,0),-1);
			imshow("image",image);
			last_dt_p = Aim_armor;
			return true;
		}	
		else
		{
			return false;
		}
	}
}


