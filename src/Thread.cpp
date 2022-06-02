#include "Thread.hpp"
#include <cstdio>
#include <opencv2/opencv.hpp>

using namespace cv;

bool is_continue = true;

typedef struct form
{
	RotatedRect ROT;
	int Armor_type;
	float a[4];
	int is_get;
	int mode;
	int da_is_get;
	double time;
}form;
double time_ar_ka;
double time_src_ar;


form send_data;

Mat ka_src_get;

SerialPort port("/dev/ttyS0");

void* Build_Src(void* PARAM)
{
	Mat get_src;
	auto camera_warper = new Camera;
	printf("camera_open\n");
	if (camera_warper->init())
	{
		printf("1\n");
		while (is_continue && !(waitKey(10) == 27))
		{
			if (camera_warper->read_frame_rgb())
			{
				//printf("1\n");
				get_src = cv::cvarrToMat(camera_warper->ipiimage).clone();
				time_src_ar = (double)getTickCount();
				Rect roi = Rect(0+200, 0+200, get_src.cols-200, get_src.rows-200);
				get_src = get_src(roi);
				pthread_mutex_lock(&mutex_new);
				{
					get_src.copyTo(src);
					is_start = true;
					pthread_cond_signal(&cond_new);
					pthread_mutex_unlock(&mutex_new);
					imshow("src",src);

					camera_warper->release_data();
				}
				//camera_warper->record_start();
				//camera_warper->camera_record();
			}
			else
			{
				src = cv::Mat();
			}
			
		}
		camera_warper->~Camera();
		is_continue = false;
		is_start = true;
		pthread_cond_signal(&cond_new);
	}
	else
	{
		printf("No camera!!\n");
		is_continue = false;
	}
}

void* Armor_Kal(void* PARAM)
{
	ArmorDetector shibie = ArmorDetector();
	shibie.enemy_color = RED;
	RotatedRect mubiao;
	Mat src_copy;

	energy_pre E_predicter(shibie);

	port.initSerialPort();
	double time_ar;
	sleep(2);
	printf("Armor_open\n");
	while (is_continue)
	{
		pthread_mutex_lock(&mutex_new);

		while (!is_start) {

			pthread_cond_wait(&cond_new, &mutex_new);

		}

		is_start = false;
		time_ar = time_src_ar;
		src.copyTo(src_copy);

		//imshow("src_copy",src_copy);

		pthread_mutex_unlock(&mutex_new);
		float lin[4];
		int mode_temp;
		int lin_is_get;
		lin_is_get = port.get_Mode1(mode_temp, lin[0], lin[1], lin[2], lin[3]);
		//printf("mode:%x\n",mode_temp);
		if (mode_temp == 0x21)
		{
			RotatedRect mubiao_get = shibie.getTargetAera(src_copy, 0, 0);

			pthread_mutex_lock(&mutex_ka);
			send_data.ROT = mubiao_get;
			send_data.Armor_type = shibie.isSamllArmor();
			send_data.a[0] = lin[0];
			send_data.a[1] = lin[1];
			send_data.a[2] = lin[2];
			send_data.a[3] = lin[3];
			src_copy.copyTo(ka_src_get); //1
			send_data.mode = mode_temp;
			send_data.is_get = lin_is_get;

			time_ar_ka = time_ar;

			is_ka = true;
			pthread_cond_signal(&cond_ka);
			pthread_mutex_unlock(&mutex_ka);
		}
		else if (mode_temp == 0x22)
		{
			ka_src_get.copyTo(quan_src);
			quan_ab_pitch = lin[0];
			quan_ab_yaw = lin[1];
			quan_ab_roll = lin[2];
			quan_speed = lin[3];
			printf("quan_pitch:%f",quan_ab_pitch);
			printf("quan_yaw:%f",quan_ab_yaw);

			if (E_predicter.energy_detect(src)) //
			{
				//E_predicter.energy_predict_aim();
				pthread_mutex_lock(&mutex_ka);
				send_data.a[0] = E_predicter.E_yaw/*- send_data.a[0]*/;
				send_data.a[1] = E_predicter.E_pitch/*- send_data.a[1]*/;
				send_data.mode = mode_temp;
				send_data.da_is_get = 0x31;
				is_ka = true;
				pthread_cond_signal(&cond_ka);
				pthread_mutex_unlock(&mutex_ka);
			}
			else
			{
				pthread_mutex_lock(&mutex_ka);
				send_data.da_is_get = 0x32;
				is_ka = true;
				pthread_cond_signal(&cond_ka);
				pthread_mutex_unlock(&mutex_ka);
			}

			//VisionData vdata;
			//if (E_predicter.energy_detect(src))
			//{
			//	vdata = { -E_predicter.E_pitch, -E_predicter.E_yaw, 0x31 };
			//	port.TransformData(vdata);
			//	port.send();
			//}
			//else
			//{
			//	vdata = { -E_predicter.E_pitch, -E_predicter.E_yaw, 0x32 };
			//	port.TransformData(vdata);
			//	port.send();
			//}
		}
	}
	is_ka = true;
	pthread_cond_signal(&cond_ka);
}

void* Kal_predict(void* PARAM)
{
	VisionData vdata;
	robot_state robo_init;
	int Armor_type = 1;
	KAL ka;
	kal_filter kf;
	int mode_temp;
	double time_count = 0;
	//SerialPort port("/dev/ttyUSB"); //d
	//port.initSerialPort(); //d
	kf = ka.init();
	form get_data;
	sleep(3);
	printf("kal_open\n");
	int is_get;
	int mode;
	int is_send;
	RotatedRect mubiao;

	while (is_continue)
	{
		pthread_mutex_lock(&mutex_ka);

		while (!is_ka) {

			pthread_cond_wait(&cond_ka, &mutex_ka);
		}
		is_ka = false;

		ka_src_get.copyTo(ka._src);
		ka_src_get.copyTo(quan_src);
		ka.ab_pitch = send_data.a[0];
		ka.ab_yaw = send_data.a[1];
		ka.ab_roll = send_data.a[2];
		ka.SPEED = send_data.a[3];
		is_get=send_data.is_get;
		mode = send_data.mode;
		is_send = send_data.da_is_get;
		get_data.Armor_type = send_data.Armor_type;
		get_data.ROT = send_data.ROT;

		time_count = time_ar_ka;

		pthread_mutex_unlock(&mutex_ka);

		mubiao=get_data.ROT;
		ka.type = (get_data.Armor_type) ? 1 : 2;
		if(is_get)
		{
			if (mode == 0x21)
			{
				if (ka.predict(mubiao, kf, time_count))
				{
					vdata = { -ka.send.pitch, -ka.send.yaw, 0x31 };
					printf("yaw:%f\npitch:%f\n", -ka.send.yaw, -ka.send.pitch);
					port.TransformData(vdata);
					port.send();
				}
				else
				{
					kf.reset();
					vdata = { -ka.send.pitch, -ka.send.yaw, 0x32 };
					printf("yaw:%f\npitch:%f\n", -ka.send.yaw, -ka.send.pitch);
					port.TransformData(vdata);
					port.send();
				}
			}
			else if (mode == 0x22)
			{
				vdata = { -ka.ab_yaw, -ka.ab_pitch, is_send };
				if(is_send == 0x31)
					printf("dafu_yaw:%f\ndafu_pitch:%f\n", -ka.ab_pitch, -ka.ab_yaw);
				port.TransformData(vdata);
				port.send();
			}
		}
	}
}