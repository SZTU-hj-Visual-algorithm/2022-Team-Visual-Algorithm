//
// Created by liyankuan on 2022/1/17.
//
#include "energy_predict.h"


//
void energy_pre::reset()//没打中大符，大符时间重置故所有参数都要重置
{
	H = 1;
	Q = 10;
	R = 0.2;
	sigma = Q;
	start_p = { -1,-1 };
	start_c = {-1,-1};
	last_dt_p = { -1,-1 };
	measure_angle = 0.10;
	angle_k_1 = 0.10;
	start_time = -1;
	last_time = 0;
	t=0;
	dt=0;
	flip_angle = false;
	R_center.x = -1;
	R_center.y=-1;
//	count = 0;
}

void energy_pre::hit_reset()//打中大符后所需的重置函数
{
	H = 1;
	Q = 10;
	R = 0.2;
	sigma = Q;
	start_c = {-1,-1};
	start_p = {-1,-1};
	last_dt_p = {-1,-1};
	measure_angle = 0.10;
	angle_k_1 = 0.10;
//	start_angle = angle_k;
	last_time = 0;
	flip_angle = false;
}



cv::Point energy_pre::angle2_xy(cv::Point &now_xy, double pred_angle)
{
	double angle = atan2(now_xy.y - R_center.y, now_xy.x - R_center.x);
	double pre_x, pre_angle, pre_y;
	if (direct == 1)
	{
		//1为顺时针
		pre_angle = keep_pi(angle + pred_angle);
		
	}
	else
	{
		//0为逆时针
		pre_angle = keep_pi(angle - pred_angle);
	}
	pre_x = radius*cos(pre_angle) + R_center.x;
	pre_y = radius*sin(pre_angle) + R_center.y;
	
	
	
	return cv::Point(pre_x, pre_y);
}


energy_pre::energy_pre()
{
	H = 1;
	Q = 10;
	R = 0.2;
	sigma = Q;
	
	F_MAT=(cv::Mat_<double>(3, 3) << 1583.14676, 0.000000000000, 633.90210, 0.000000000000, 1582.31678, 528.53893, 0.000000000000, 0.000000000000, 1.000000000000);
	C_MAT=(cv::Mat_<double>(1, 5) << -0.08767, 0.19792, 0.00021, 0.00068, 0.00000);
	
	cv::cv2eigen(F_MAT,F_EGN);
	cv::cv2eigen(C_MAT,C_EGN);
}



energy_pre::energy_pre(ArmorDetector &armor)
{
	H = 1;
	Q = 10;
	R = 0.2;
	sigma = Q;

	F_MAT=(cv::Mat_<double>(3, 3) << 1583.14676, 0.000000000000, 633.90210, 0.000000000000, 1582.31678, 528.53893, 0.000000000000, 0.000000000000, 1.000000000000);
	C_MAT=(cv::Mat_<double>(1, 5) << -0.08767, 0.19792, 0.00021, 0.00068, 0.00000);

	cv::cv2eigen(F_MAT,F_EGN);
	cv::cv2eigen(C_MAT,C_EGN);
	this->enermy_color = armor.enermy_color;
}

double energy_pre::predict(double t, double dt, bool pre_not)
{
	if (pre_not)
	{
		double F1 = 0.913 * sin(1.942 * t) + 1.177;
		double F2 = 1.177 * t - 913.0000 * cos(0.971 * t) * cos(0.971 * t) / 971.000 + 913.000 / 971.000;
		F = 1 + F1 * dt / F2;
		
		angle_k = F*angle_k;
		angle_k_1 = angle_k - start_angle;
//		angle_k_1 = F*angle_k_1;
		return angle_k_1;
	}
	else
	{
		t = (t + t + dt) / 2;
		double F1 = 0.913 * sin(1.942 * t) + 1.177;
		double F2 = 1.177 * t - ((913.00000 * cos(0.971 * t) * cos(0.971 * t)) / 971.0000)+ (913.00000 / 971.00000);
		//std::cout << "时间" << t << std::endl;
		//std::cout << "F2" << F2 << std::endl;
		F = 1 + F1 * dt / F2;
		//std::cout << "F" << F << std::endl;
		//让预测不影响卡尔曼的迭代
		return F * angle_k - start_angle;
	}
}

double energy_pre::correct(double measure)//这里先不管了，以后再想，摆了
{
	sigma = F * sigma * F + Q;
	
	K = sigma * H / (H * sigma * H + R);
	
	angle_k_1 = angle_k_1 + K * (measure - H * angle_k_1);
	
	sigma = (1 - K * H) * sigma;
	
	//得出绝对角度的后验值
	angle_k = angle_k_1 + start_angle;
	
	return angle_k_1;
}



//double energy_pre::keep_pi_2(double angle)
//{
//	if (angle > 90)
//	{
//		angle = 90.0 - (angle - 90.0);
//		return angle;
//	}
//	else if (angle < 0)
//	{
//		angle = -angle;
//		return angle;
//	}
//	return angle;
//}

double energy_pre::keep_pi(double angle)
{
	if (angle > CV_PI)
	{
		return -(CV_PI + CV_PI - angle);
	}
	else if (angle < -CV_PI)
	{
		return CV_PI + CV_PI + angle;
	}
	else
	{
		return angle;
	}
}

Eigen::Vector3d energy_pre::pnp_get_pc(const cv::Point2f p[4])
{
	cv::Point2f lu, ld, ru, rd;
	std::vector<cv::Point3d> ps = {
			{-w_std / 2 , -h_std / 2, 0.},
			{w_std / 2 , -h_std / 2, 0.},
			{w_std / 2 , h_std / 2, 0.},
			{-w_std / 2 , h_std / 2, 0.}
	};
	if (p[0].y < p[1].y) {
		lu = p[0];   ////左上
		ld = p[1];	////左下
	}
	else {
		lu = p[1];
		ld = p[0];
	}
	if (p[2].y < p[3].y) {
		ru = p[2];   ////右上
		rd = p[3];	////右下
	}
	else {
		ru = p[3];
		rd = p[2];
	}
	
	std::vector<cv::Point2f> pu;
	pu.push_back(lu);
	pu.push_back(ru);
	pu.push_back(rd);
	pu.push_back(ld);
	
	cv::Mat rvec;
	cv::Mat tvec;
	Eigen::Vector3d tv;
	
	
	cv::solvePnP(ps, pu, F_MAT, C_MAT, rvec, tvec);
	
	
	cv::cv2eigen(tvec, tv);//这个转为了啥
	
	return tv;
}

cv::Point energy_pre::gravity_finish(cv::Point& pp,Eigen::Vector3d &ap)
{
	double height;
	
	//----------用到目标点而不是预测点是因为要获取目标的距离------------
	double depth = ap(2,0);
//	std::cout<<depth<<std::endl;
	//------------------------------------------------------------------
	
	Eigen::Vector3d p_pre = {(double)pp.x,(double)pp.y,1.0};
	Eigen::Vector3d ap_pre = pu_to_pc(p_pre,depth);
	
	double del_ta = pow(SPEED, 4) + 2 * 9.8 * ap(1, 0) * SPEED * SPEED - 9.8 * 9.8 * depth*depth;
	
	double t_2 = (9.8 * ap(1, 0) + SPEED * SPEED - sqrt(del_ta)) / (0.5 * 9.8 * 9.8);
	
	height = 0.5 * 9.8 * t_2;
//	std::cout<<"抬枪补偿:"<<height<<std::endl;
	Eigen::Vector3d ap_g = {ap_pre(0,0),ap_pre(1,0) - height,depth};
	E_pitch = atan2(ap(1,0) - height, ap(2,0))/CV_PI*180.0;
	E_yaw = atan2(ap(0,0) , ap(2,0))/CV_PI*180.0;
	
	Eigen::Vector3d ap_pu = pc_to_pu(ap_g,depth);//ap_g(2,0)是距离
	
	return cv::Point((int)ap_pu(0,0),(int)ap_pu(1,0));
}

double energy_pre::measured(cv::Point& xy) //重写
{
	int offset_x = R_center.x - start_c.x;
	int offset_y = R_center.y - start_c.y;
	cv::Point _start_p = {start_p.x+offset_x,start_p.y+offset_y};
	double dis_st = sqrt((xy.x - _start_p.x) * (xy.x - _start_p.x) + (xy.y - _start_p.y) * (xy.y - _start_p.y)) / 2;
	//防止出现无穷大数据，因为asin函数如果入参大于1就会出现无穷大数据
	if (dis_st > radius)
	{
		dis_st = radius;
	}
	
	double angle = 2 * asin(dis_st/radius);
	
	
	if (angle > 2.95)
	{
		flip_angle = true;
	}
	if (flip_angle)
	{
		angle = 2 * CV_PI - angle;
	}
	measure_angle = angle;
	//std::cout <<"观测值"<< measure_angle << std::endl;
	
	return measure_angle;
}

