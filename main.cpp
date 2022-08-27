#include "camera.h"
#include "ArmorDetector.hpp"
#include "KAl.h"
#include <opencv2/core/cvstd.hpp>
#include "CRC_Check.h"
#include"serialport.h"

//#define DETECT
#define PREDICT
#define SPEED 28.0

using namespace cv;

int main()
{
	
	SerialPort port("/dev/ttyUSB0");
	port.initSerialPort();
	auto camera_warper = new Camera;
	
	Mat src;
	ArmorDetector shibie = ArmorDetector();////Ä¬ÈÏ²ÎÊý
	shibie.enemy_color = RED;
	KAL ka;
	
	kal_filter kf;//ŽŽœš¿š¶ûÂüÂË²š
	
	
	Mat threse;
	
	kf = ka.init();//³õÊŒ»¯¿š¶ûÂüÂË²šµÄ²ÎÊý
	
	VisionData vdata;
	int mode_temp;
	//float pitch_real;
	//float yaw_real;
	ml.SVM_
	
	if (camera_warper->init())
	{
		//³õÊŒ»¯Ö¡ÍŒÏñºÍÏà»ú¶ÔÏó
		while (1)
		{
			double time_count = (double)getTickCount();
			
			camera_warper->read_frame_rgb(src); //lu xiang
			camera_warper->record_start();
			camera_warper->camera_record();
			
			ka._src = src;
			
			
			RotatedRect mubiao;
			
			double yaw = 0, pitch = 0, dis = 0;
			
			mubiao = shibie.getTargetAera(src, 0, 0); //µÚ¶þžöµÚÈýžö·Ö±ðŽú±íÉÚ±øºÍµõÉä»ùµØÄ£Êœ
			
			//int len = MIN(mubiao.size.height, mubiao.size.width);
			int type = (shibie.isSamllArmor()) ? 1 : 2;
			
			ka.type = type;
#ifdef DETECT
			Point2f vertices[4];
			mubiao.points(vertices);
			if (!mubiao.size.empty())
			{
				Eigen::Vector3d aim = ka.pnp_get_pc(vertices,0.130,0.055);


				double del_ta = pow(SPEED,4) + 2*9.8*aim(1,0)*SPEED*SPEED - 9.8*9.8*aim(2,0)*aim(2,0);

				double t_2 = (9.8*aim(1,0) + SPEED*SPEED - sqrt(del_ta))/(0.5*9.8*9.8);
				double height = 0.5*9.8*t_2;

				ka.send.yaw = atan2(aim(0,0),aim(2,0))/PI*180.0;
				ka.send.pitch = atan2(aim(1,0) + 0.075 - height*0.000001,aim(2,0))/PI*180.0;
				printf("yaw:%f	\npitch:%f\n", -ka.send.yaw, -ka.send.pitch);
				printf("dis:%f  \n",aim(2,0));

				for (int i = 0; i < 4; i++)
				{
					line(src, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 0, 0));
				}
				vdata = {,-ka.send.pitch - 0.1, -ka.send.yaw - 0.18};
				port.TransformData(vdata);
				port.send();
			}

			else
			{
				vdata = {-ka.send.pitch,-ka.send.yaw};
				port.TransformData(vdata);
				port.send();
			}
#endif

#ifdef PREDICT
			if (port.get_Mode1(mode_temp, ka.ab_pitch, ka.ab_yaw))
			{
//				printf("ab_pitch:%f\nab_yaw:%f\n",ka.ab_pitch,ka.ab_yaw);
				if (ka.predict(mubiao, kf, time_count)) {
					
					printf("yaw:%f	\npitch:%f\n", -ka.send.yaw, -ka.send.pitch);
					printf("type:%d\n", type);
					Point2f vertices[4];
					mubiao.points(vertices);
					for (int i = 0; i < 4; i++) {
						line(src, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 0, 0));
					}
					
					vdata = { -ka.send.pitch + 0.000555, -ka.send.yaw + 0.0711, 0x31};
					port.TransformData(vdata);
					port.send();
				}
				else
				{
					kf.reset();
					vdata = {-ka.send.pitch, -ka.send.yaw, 0x32};
					port.TransformData(vdata);
					port.send();
				}
			}
#endif
			
			double fps = 1 / (((double)getTickCount() - time_count) / getTickFrequency());
			char strN[10];//×ª»»ºóµÄ×Ö·ûŽ®
			sprintf(strN, "%.4f", fps);
			putText(src, strN, Point(30, 30), 5, 1.5, Scalar(0, 0, 255), 2);
			imshow("src", src);
			
			
			if (waitKey(10) == 27)
			{
				camera_warper->~Camera();
				break;
			}
		}
		
		
		
		return 0;
		
	}
	else
	{
		printf("No camera!!");
		return 0;
	}
	
}
