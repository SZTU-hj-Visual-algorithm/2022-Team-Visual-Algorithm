#include "KAl.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>


#define draw_circle
#define draw_height
#define show_circle

kal_filter KAL::init()
{
	//cv::FileStorage fin("/home/hsy/����/zhenhe/linux_project6/other/calib_no_4_12801(1).yml", cv::FileStorage::READ);
	//fin["F"] >> F_MAT;//�ڲξ���
	//fin["C"] >> C_MAT;//�������
	F_MAT=(Mat_<double>(3, 3) << 1564.40096, 0.000000000000, 641.93179, 0.000000000000, 1564.32777, 523.40759, 0.000000000000, 0.000000000000, 1.000000000000);
	C_MAT=(Mat_<double>(1, 5) << -0.07930, 0.21700, 0.00045, 0.00033, 0.00000);
	
	cv2eigen(F_MAT, F);
	cv2eigen(C_MAT, C);
	
	kal_filter kf;//��ʼ��kalman�˲�
	
	return kf;
}

void KAL::reset()
{
	t = -1;
	//last_ct.x = -1;
	//last_ct.y = -1;
	//stop_predict = 0;
//	send_flag = 0;
}

void KAL::sp_reset(kal_filter& kf)
{
	kf.Xk_1[1] = 0.0001;
	kf.Xk_1[2] = 0.001;
	kf.Xk_1[4] = 0.00000001;
	kf.Xk_1[5] = 0.00001;
}

bool KAL::predict(RotatedRect &detection, kal_filter& kf, double time)
{
	//t=-1;
	if (detection.size.empty())
	{
		reset();
		
		//printf("no aim!!");
		return false;
	}
	//����ͼ���ϼ�����Ŀ�����ڵ����������˲��õ��������ֵ--------------------------------
	Point bp = detection.center;
	//���Ŀ����������------------------------------------------------------
	double w, h;
	if (type == 1)
	{
		w = 0.130;
		h = 0.055;
	}
	else
	{
		w = 0.225;
		h = 0.055;
	}
	
	cv::Point2f pts[4];
	detection.points(pts);
	std::sort(pts, pts + 4, [](const cv::Point2f& p1, const cv::Point2f& p2) { return p1.x < p2.x; });
	Eigen::Vector3d m_pc = pnp_get_pc(pts, w, h);
	
	if (depth > 5.5)
	{
		depth = 109.41*pow(MIN(detection.size.width,detection.size.height), -1.066);
	}
	
	depth = (1-filter)*m_pc(2,0) + filter*depth;
	


	Mat eular = (Mat_<float>(3,1)<<-ab_pitch/180.0*CV_PI,-ab_yaw/180.0*CV_PI,ab_roll/180.0*CV_PI);
	Mat rotated_mat;
	Eigen::Matrix<double,3,3> rotated_matrix;
	cv::Rodrigues(eular,rotated_mat);
	cv2eigen(rotated_mat,rotated_matrix);
	Eigen::Vector3d m_pd = rotated_matrix*m_pc;
	
	if (t == -1)
	{
		t = time;
		double height = get_gravity(m_pc);
//		double ra_yaw = atan2(m_pc(0,0),m_pc(2,0)) / CV_PI*180.0;
//		double ra_pitch = atan2(m_pc(1,0) - height,m_pc(2,0)) / CV_PI*180.0;
//		send.yaw = ra_yaw - ab_yaw;
//		send.pitch = ra_pitch - ab_pitch;
        get_send(m_pc,height);
		kf.Xk_1[0] = m_pd(0,0);
		kf.Xk_1[3] = m_pd(1,0);
		//last_yaw = ra_yaw - ab_yaw;
		circle(_src,bp,7,Scalar(0,0,255),4);
		imshow("src_kal",_src);
//		last_aim_pitch = ra_pitch - ab_pitch;
//		last_aim_yaw = ra_yaw - ab_yaw;
		return true;
	}


	Eigen::Matrix<double, 2, 1> measured;
	measured << m_pd(0,0), m_pd(1,0);
	
	double delata_t = ((time - t)*0.55)/(double)getTickFrequency();
	//printf("delata_t:%lf\n",delata_t);
	t = time;
	
	Eigen::Matrix<double, 6, 1> pred = kf.predict(delata_t,false);//������һʱ�̵ĺ������Ԥ�����һʱ�̵��������ֵ
	
	Eigen::Matrix<double, 6, 1> corrected = kf.correct(measured);//��������Э������󣬲�����������
	
	
	//-----------------------------------------------------------------------------------------
	
	double predict_time = m_pc.norm() / SPEED + shoot_delay;
	//printf("speed:%lf\n",SPEED);
	//printf("pre_time:%lf\n",predict_time);
	
	
	Eigen::Matrix<double, 6, 1> pre_xy = kf.predict(predict_time,true);
	
	shoot_delay = (fabs(pre_xy[1])/1.25)*shoot_delay_init;
	//printf("shoot_delay:%lf\n",shoot_delay);
	
	
	Eigen::Vector3d pos3 = {pre_xy[0],pre_xy[3],m_pd(2,0)};
	//printf("x_v:%lf\tx_a:%lf\ny_v:%lf\ty_a:%lf\n",pre_xy[1],pre_xy[2],pre_xy[4],pre_xy[5]);
	//printf("x:%lf\ty:%lf\n",pre_xy[0],pre_xy[3]);
	pos3 = rotated_matrix.inverse()*pos3;
	pos3 = {pos3[0],pos3[1],depth};


#ifdef draw_circle
	//������----------------------------------------------------------------------
	Eigen::Vector3d pos2 = pc_to_pu(pos3);
	//circle(_src, bp, 7, Scalar(0, 0, 255), 3);
	//由角度预测得到的目标点在图像上的映射位置
	circle(_src, Point((int)pos2[0],(int)pos2[1]), 7, Scalar(255, 0, 0), 3);
	//---------------------------------------------------------------------
#endif
	
	
	//-----------------------------------------------------------------------------------
	
	if (SPEED == 0.0)
	{
		SPEED = 26.5;
	}
	//-----̧ǹ����----------------------------------------------------------------------
	//printf("pos3.No3:%f\t%f\n",pos3[2]);
	
	double height = get_gravity(pos3);
	//printf("height:%f\n",height);
	//------------------------------------------------------------------------------------
#ifdef draw_height
	Eigen::Vector3d ap_pre{ pos3(0,0) , pos3(1,0) - height , pos3(2,0) };
	
	Eigen::Vector3d pu_pre = pc_to_pu(ap_pre);
	
	circle(_src, Point((int)pu_pre(0, 0), (int)pu_pre(1, 0)), 7, Scalar(255, 255, 0), 3);
#endif
#ifdef show_circle
	imshow("src_kal",_src);
#endif

//	send.yaw = atan2(pos3(0, 0), pos3(2, 0))/CV_PI * 180.0 - ab_yaw;//atan2�ĽǶȷ�Χ��-180~180���պ����нǶȶ��и���,������
//
//	double xishu = 0.78 * (pos3(2,0)/1.92);
//	send.pitch = atan2(pos3(1, 0) - height*xishu , pos3(2, 0))/CV_PI * 180.0 - ab_pitch;
    get_send(pos3,height);
	//-----------------------------------------------------------------------------------
	
	return true;
	
}

Eigen::Vector3d KAL::pnp_get_pc(const cv::Point2f p[4], const double& w, const double& h)
{
	cv::Point2f lu, ld, ru, rd;
	std::vector<cv::Point3d> ps = {
			{-w / 2 , -h / 2, 0.},
			{w / 2 , -h / 2, 0.},
			{w / 2 , h / 2, 0.},
			{-w / 2 , h / 2, 0.}
	};
	if (p[0].y < p[1].y) {
		lu = p[0];   ////����
		ld = p[1];	////����
	}
	else {
		lu = p[1];
		ld = p[0];
	}
	if (p[2].y < p[3].y) {
		ru = p[2];   ////����
		rd = p[3];	////����
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
	
	
	cv::cv2eigen(tvec, tv);//���תΪ��ɶ
	
	return tv;
}

float KAL::keep_pi(float angle)
{
	if (angle > 180.0)
	{
		return -(360.0 - angle);
	}
	else if (angle < -180.0)
	{
		return 360.0 + angle;
	}
	else
	{
		return angle;
	}
}
