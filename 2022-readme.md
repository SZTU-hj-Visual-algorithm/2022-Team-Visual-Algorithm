```
.
|──build                                    //存放cmake编译好的文件
|──include                                  //存放各种头文件
|     |───camera                            //存放相机调用需要的头文件
|     |       |──camera.h                   //存放我们调用相机实现功能所需的算法代码文件
|     |       |──CameraApi.h                //在此定义相机使用需要的api（由官方封装好的）
|     |       |──CameraDefine.h             //相机底层相关数据类型的定义
|     |       └──CameraStatus.h             //相机底层相关状态的定义
|     |——ArmorDetector.hpp                  //装甲板识别的头文件
|     |——CRC_Check.h                        //CRC8校验码相关
|     |──energy_predict.h                   //打符预测的头文件
|     |──energy_state.h                     //打符识别和获取大符各种状态的头文件
|     |──KAL.h                              //卡尔曼滤波预测的文件
|     |──kal_filter.hpp                     //卡尔曼滤波的算法实现
|     |──robot_state.h                      //电控发送的机器人当前状态数据
|     |──serialport.h                       //串口通讯头文件
|     └──Thread.hpp                         //多线程的头文件
|──other                                    //存放一些别的东西
|──src                                      //存放各种源码文件
|   |───ArmorDetector.cpp                   //装甲板识别的源码文件
|   |───camera.cpp                          //相机调用的源码文件
|   |───CRC_Check.cpp                       //CRC8校验的源码文件
|   |───energy_get.cpp                      //打符最终实现的源码文件
|   |───energy_predict.cpp                  //打符实现预测的源码文件
|   |───energy_state.cpp                    //打符实现识别和获取大符各类状态数据的源码文件
|   |───KAL.cpp                             //卡尔曼滤波预测的源码文件
|   |───serialport.cpp                      //实现串口通讯的源码文件
|   └──Thread.cpp                           //多线程实现的源码文件
|──tools                                    //存放自启动相关脚本文件
|──CMakeLists.txt                           //cmakelist
|──main.cpp                                 //主函数文件，只有创建线程
```
