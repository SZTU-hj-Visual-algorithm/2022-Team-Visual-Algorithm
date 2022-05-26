#include "camera.h"
#include "ArmorDetector.hpp"
#include "KAl.h"
#include <opencv2/core/cvstd.hpp>
#include "CRC_Check.h"
#include"serialport.h"
#include"Thread.hpp"

//#define DETECT
#define PREDICT
#define SPEED 28.0

using namespace cv;

int main()
{
	SerialPort port("/dev/ttyUSB");
	Thread_Total Thread_class(port);
	std::thread t1(&Thread_Total::Build_Src, std::ref(Thread_class));
	std::thread t2(&Thread_Total::Armor_Kal, std::ref(Thread_class));
	std::thread t3(&Thread_Total::Kal_predict, std::ref(Thread_class));
	t1.join();
	t2.join();
	t3.join();

}
