//
// Created by liyankuan on 2022/1/17.
//
#include "energy_state.h"
#include "energy_predict.h"
#include <vector>
#include <Eigen/Dense>

using namespace cv;
using namespace std;


int energy::get_direct(cv::Point &now)
{
	//设1为顺时针，设0为逆时针
	double symbol = (now.y - R_center.y) * (last_p.y - R_center.y);
	double now_ang = atan2(now.y - R_center.y, now.x - R_center.x);
	double five_ang = atan2(last_p.y - R_center.y, last_p.x - R_center.x);
	if (symbol < 0)//是否在x轴的交界处
	{
		if ((now_ang - five_ang) < 0)
		{
			direct = abs(now_ang) > CV_PI / 2 && abs(five_ang) > CV_PI / 2 ? 1 : 0;
		}
		else
		{
			direct = abs(now_ang) > CV_PI / 2 && abs(five_ang) > CV_PI / 2 ? 0 : 1;
		}
	}
	else
	{
		if ((now_ang - five_ang) < 0)
		{
			direct = 0;
		}
		else
		{
			direct = 1;
		}
	}
	return direct;
}

Point energy::detect_aim(Mat& img)
{
	Mat binary = cv::Mat(img.size(), CV_8UC1, cv::Scalar(0));
//	Mat binary_2 = cv::Mat(img.size(), CV_8UC1, cv::Scalar(0));
	Mat thres_src;
	Mat kernel2 = getStructuringElement(MORPH_CROSS,Size(13,13));
	Mat kernel1 = getStructuringElement(MORPH_CROSS,Size(9,9));
	
	uchar* mat_head = (uchar*)img.data;
	uchar* bin_head = (uchar*)binary.data;
	
	int img_all = img.rows * img.cols;
	Mat max_color;
	
	if (enermy_color == RED)
	{
		for (int i = 0; i < img_all; i++)
		{
			if (*(mat_head + 2) - *mat_head > 45)
				*bin_head = 255;
			mat_head += 3;
			bin_head++;
		}
//		Mat imag;
//		cvtColor(img, imag, COLOR_BGR2GRAY);
//		threshold(imag, thres_src, 38, 255, THRESH_BINARY);
//		imshow("binary",binary);
//		max_color = binary & thres_src;
		max_color = binary.clone()/* & thres_src*/;
//		medianBlur(max_color,max_color,7);
		
		erode(max_color,max_color,kernel1);
		dilate(max_color,max_color,kernel2);
		imshow("max_color",max_color);
	}
	else if (enermy_color == BLUE)
	{
		for (int i = 0; i < img_all; i++)
		{
			if (*mat_head - *(mat_head + 2) > 49)
				*bin_head = 255;
			mat_head += 3;
			bin_head++;
		}
//		Mat imag;
//		cvtColor(img, imag, COLOR_BGR2GRAY);
//		threshold(imag, thres_src, 39, 255, THRESH_BINARY);
//		max_color = binary & thres_src;
		max_color = binary.clone()/* & thres_src*/;
//		medianBlur(max_color,max_color,7);
		
		erode(max_color,max_color,kernel1);
		dilate(max_color,max_color,kernel2);
		imshow("max_color",max_color);
	}
	
	vector<vector<Point>>cnts;
	vector<vector<Point>>cntses;
	vector<vector<Point>>centers;
	vector<vector<Point>>aims;
	vector<Vec4i>heri;
	int hit_c = 0;
	findContours(max_color, cnts,heri, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
	int center_target = 0;
	bool find_c = false;
	
	for (int i = 0; i < heri.size(); i++)
	{
		if (heri[i][3] != -1)//判断父轮廓
		{
			if (heri[i][1] == -1 && heri[i][0] == -1) {
				cntses.push_back(cnts[i]);
			} else {
				hit_c++;
			}
		}
		else if (heri[i][3] == -1)
		{
			if (heri[i][2] == -1)
			{
				RotatedRect c_rect = minAreaRect(cnts[i]);
				RotatedRect temp_rect = minAreaRect(cnts[center_target]);
				Point en_center = c_rect.center;
				double temp_area = temp_rect.size.width*temp_rect.size.height;
				double area = c_rect.size.width *c_rect.size.height;
				if (/*(area > temp_area) &&*/ ((c_rect.size.width / c_rect.size.height )<1.15) && ((c_rect.size.width / c_rect.size.height )>0.85))
				{
//					Mat mean, stdDev;
//					double avg,stddev;
//					meanStdDev(img(cnt_r),mean,stdDev);
//					double en_dis = sqrt((en_center.x - R_center.x)*(en_center.x - R_center.x)+(en_center.y - R_center.y)*(en_center.y - R_center.y));
//					double dis_dela = sqrt((c_rect.center.x - R_center.x)*(c_rect.center.x-R_center.x) + (c_rect.center.y - R_center.y)*(c_rect.center.y-R_center.y));

//					if (dis_dela < 50)
//					{
					centers.push_back(cnts[i]);
					find_c = true;
//					}
				}
			}
		}
		
	}
//	imshow("img",img);
	if (hit_c/3 - hited/3 == 1) {
		hit = true;
		hited = hit_c;
		//printf("yidazhong_energy                                                       :%d\n",hited);
	}
	std::sort(centers.begin(),centers.end(),center_area);
	
	
//	}
	RotatedRect final_rect;
	//对上一层筛选后得到的目标进一步进行旋转矩形外包，比较长宽比进行筛选
	int symbol = 0;
	if (cntses.size() > 1)
	{
		for (int i = 0; i < cntses.size(); i++)
		{
			RotatedRect ro_rect = minAreaRect(cntses[i]);
			Rect cnt_r = boundingRect(cntses[i]);
			double w = ro_rect.size.width;
			double h = ro_rect.size.height;
			double wh_compare = w > h ? w / h : h / w;
			if ((wh_compare > wh_min_ratio) && (wh_compare < wh_max_ratio))
			{
				double rect_area = w*h;
				if ((rect_area/contourArea(cntses[i]) > 0.78) && (rect_area/contourArea(cntses[i]) < 1.23))
				{
					//printf("mianji                                          :%lf\n",contourArea(cntses[i]));
					if ((contourArea(cntses[i]) > 900)&&(contourArea(cntses[i])<1500))
					{
						Mat mean, stdDev;
						double avg,stddev;
						meanStdDev(img(cnt_r),mean,stdDev);
						avg = mean.ptr<double>(0)[0];
						stddev = stdDev.ptr<double>(0)[0];
						//printf("avg                                          :%lf\n",avg);
						//printf("stddev                                          :%lf\n",stddev);
						if ((avg < 43.00)&&(stddev > 15.8))
						{
							if (symbol == 1)
							{
								aims.push_back(cntses[i]);
							}
							else
							{
								aims.push_back(cntses[i]);
								symbol= 1;
							}
						}
					
					}
				}
				
			}
		}
		if (symbol == 1)
		{
			sort(aims.begin(),aims.end(), center_area);
			final_rect = minAreaRect(aims[0]);
		}
	}
	else if (cntses.size() == 1)
	{
		final_rect = minAreaRect(cntses[0]);
//		printf("only one\n");
	}
	else
	{
		final_rect = RotatedRect();
//		printf("no aim\n");
		return Point(0,0);
	}
	
	Point aim = final_rect.center;
	if (final_rect.size.height > final_rect.size.width)
	{
	    pnp_flip_wh = true;
	}
	else
	{
	    pnp_flip_wh = false;
	}
	RotatedRect center_r;
	
	if (find_c)
	{
		//std::cout<<"can find_c"<<std::endl;
		Rect center_rect = boundingRect(centers[center_target]);
		int c_x = center_rect.x;
		int c_y = center_rect.y;
		int c_w = center_rect.width;
		int c_h = center_rect.height;
		Point want_center;
		want_center.x = (c_x + c_x + c_w) / 2;
		want_center.y = (c_y + c_y + c_h) / 2;
		double want_dis = sqrt((want_center.x-aim.x)*(want_center.x-aim.x)+(want_center.y-aim.y)*(want_center.y-aim.y));
		if ((want_dis < 270)&&(want_dis > 85))
		{
			R_center.x = want_center.x;
			R_center.y = want_center.y;
			center_r = minAreaRect(centers[center_target]);
			rectangle(img,center_rect,Scalar(255,0,0),5);
		}
		
	}
	radius = sqrt((R_center.x-aim.x)*(R_center.x-aim.x)+(R_center.y-aim.y)*(R_center.y-aim.y));
	center_r.points(p);
	final_rect.points(pp);
	circle(img, aim, 6, Scalar(255, 0, 0), 5);
	for (int i=0;i<4;i++)
	{
		line(img,pp[i],pp[(i+1)%4],Scalar(255,0,0),4);
	}
	imshow("image",img);
	return aim;
}


cv::Mat energy::set_image(cv::Mat &src)
{
	if (R_center.x ==-1 ||R_center.y==-1||last_dt_p.x == -1||last_dt_p.y == -1)
	{
		return src;
	}
	else
	{
		
		if (lose_aim)
		{
			lose_aim = false;
			
			roi.width = roi.width+roi.width;
			roi.height = roi.height+roi.height/2;
			roi.x = roi.x - 50;
			roi.y = roi.y-50;

			make_safe(roi,src);
			Mat img = src(roi).clone();
			return img;
		
//			return src;
		}
		int width = R_center.x-last_dt_p.x;
		width = width >0? width : -width;
		int height = R_center.y-last_dt_p.y;
		height = height>0? height : -height;
		if (width>0&&height>0)
		{
			roi.width = width*2*1.2;
			roi.height = height*2*1.5;
			roi.x = R_center.x-width;
			roi.y = R_center.y - height;
//			offset_x =roi.x;
//			offset_y = roi.y;
			make_safe(roi,src);
			Mat img = src(roi).clone();
			return img;
		}
		else
		{
			return src;
		}
		
	}
}

void energy::make_safe(Rect &rect,Mat &src)
{
	if (rect.x < 0)
	{
		rect.x = 0;
	}
	if (rect.y < 0)
	{
		rect.y = 0;
	}
	if (rect.width+rect.x > src.cols)
	{
		rect.width = src.cols - rect.x;
	}
	if (rect.height+rect.y > src.rows)
	{
		rect.height = src.rows - rect.y;
	}
}





