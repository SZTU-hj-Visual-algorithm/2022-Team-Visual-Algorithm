//
// Created by 蓬蒿浪人 on 2022/8/23.
//
#include "Armordetector.hpp"

/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and
to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

//#include "ArmorDetector.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <queue>
#include <ArmorDetector.hpp>
#include "opencv2/dnn/dnn.hpp"


//#define SHOW_DEBUG_IMG
//#define COUT_LOG
//#define CLASSIFICATION


using namespace cv;
using namespace std;

void ArmorDetector::setImage(const cv::Mat & src){
    _size = src.size();  // 一直是1280*720的
    // 注意这个_res_last是一个旋转矩形
    const cv::Point & last_result = _res_last.center;

    // 如果上一次的目标没了，源图就是输入的图
    // 并且搜索的ROI矩形（_dect_rect）就是整个图像
    if (last_result.x == 0 || last_result.y == 0) {
        _src = src;
        _dect_rect = Rect(0, 0, src.cols, src.rows);
    }
    else{
        // 如果上一次的目标没有丢失的话，用直立矩形包围上一次的旋转矩形
        Rect rect = _res_last.boundingRect();

        // _para.max_light_delta_h 是左右灯柱在水平位置上的最大差值，像素单位
        int max_half_w = _para.max_light_delta_h * 1.3;
        int max_half_h = 300;
        ////这里是不是多余了

        // 截图的区域大小。太大的话会把45度识别进去
        double scale_w = 2;
        double scale_h = 2;

        int w = int(rect.width * scale_w);
        int h = int(rect.height * scale_h);
        Point center = last_result;
        int x = std::max(center.x - w, 0);      ////这里为什么w不是取一半
        int y = std::max(center.y - h, 0);
        Point lu = Point(x, y);  /* point left up */
        x = std::min(center.x + w, src.cols);
        y = std::min(center.y + h, src.rows);
        Point rd = Point(x, y);  /* point right down */

        ////得到左上和右下两个角点，判断max和min是为了防止越界。

        // 构造出矩形找到了搜索的ROI区域
        _dect_rect = Rect(lu, rd);
        // 为false说明矩形是空的，所以继续搜索全局像素
        // 感觉这里会有点bug
        // 矩形是空的则返回false
        if (makeRectSafe(_dect_rect, src.size()) == false){
            _res_last = cv::RotatedRect();
            _dect_rect = Rect(0, 0, src.cols, src.rows);
            _src = src;
        }
        else
            // 如果ROI矩形合法的话就用此ROI
            src(_dect_rect).copyTo(_src);
    }

    //==========================上面已经设置好了真正处理的原图============================

    // 下面是在敌方通道上二值化进行阈值分割
    // _max_color是红蓝通道相减之后的二值图像
    ///////////////////////////// begin /////////////////////////////////////////////
    /**
	 * 预处理其实是最重要的一步，这里有HSV和RGB两种预处理的思路，其实大致都差不多
	 * 只不过在特定场合可以选择特定的预处理方式
	 * 例如HSV的话可以完全过滤掉日光灯的干扰，但是耗时较大
	 */


    int thres_max_color_red = 46;
    int thres_max_color_blue = 146;

    _max_color = cv::Mat(_src.size(), CV_8UC1, cv::Scalar(0));


    Mat element = getStructuringElement(MORPH_CROSS, Size(3, 3));
    // 敌方颜色为红色时
    if (enermy_color == RED){
        //cout<<"in"<<endl;
        //Mat thres_whole;

        //cvtColor(_src,thres_whole,CV_BGR2GRAY);

        /*
		if (sentry_mode)
			threshold(thres_whole,thres_whole,33,255,THRESH_BINARY);
		else
        threshold(thres_whole,thres_whole,60,255,THRESH_BINARY);*////这里初步猜测是用来排除日光灯的影响的
        ////但经过实测，如果降低相机的对焦清晰度，或是用其他的方法把原图的清晰度降低也可以排除日光灯的干扰
        ////但降低对焦清晰度不知道会不会影响远距离的识别

        //imshow("thresh_whole", thres_whole);////测试亮度

        //不是这里的问题
        int srcdata = _src.rows * _src.cols;
        uchar* Imgdata = (uchar*)_src.data;
        uchar* Imgdata_binary = (uchar*)_max_color.data;

        for (size_t i = 0; i < srcdata; i++)
        {
            if (*(Imgdata + 2) - *Imgdata > thres_max_color_red)
                *Imgdata_binary = 255;
            Imgdata += 3;
            Imgdata_binary++;
        }
        /*Mat color;
		subtract(splited[2], splited[0], color);
		threshold(color, color, thres_max_color_red, 255, THRESH_BINARY);// red*/

        //imshow("color", color);   ////查看颜色筛选过后的效果

        //_max_color = _max_color & thres_whole;  // _max_color获得了清晰的二值图
        dilate(_max_color, _max_color, element);
        imshow("max_color", _max_color);
    }
    // 敌方颜色是蓝色时
    else {
        //Mat thres_whole;
        //vector<Mat> splited;
        //split(_src, splited);
        //cvtColor(_src,thres_whole,CV_BGR2GRAY);
        //if (sentry_mode)
        //threshold(thres_whole,thres_whole,60,255,THRESH_BINARY);
        //else
        //threshold(thres_whole,thres_whole,128,255,THRESH_BINARY);

        //imshow("thres_whole", thres_whole);

        int srcdata = _src.rows * _src.cols;

        uchar* Imgdata = (uchar*)_src.data;
        uchar* Imgdata_binary = (uchar*)_max_color.data;
        for (size_t i = 0; i < srcdata; i++)
        {
            if (*Imgdata - *(Imgdata + 2) > thres_max_color_blue)
                *Imgdata_binary = 255;
            Imgdata += 3;
            Imgdata_binary++;
        }

        /*subtract(splited[0], splited[2], _max_color);
		threshold(_max_color, _max_color, thres_max_color_blue, 255, THRESH_BINARY);// blue*/

        //imshow("color", _max_color);
        //_max_color = _max_color & thres_whole;  // _max_color获得了清晰的二值图
        dilate(_max_color, _max_color, element);
        imshow("max_color", _max_color);
    }

    ////////////////////////////// end /////////////////////////////////////////
#ifdef SHOW_DEBUG_IMG
cv::imshow("_max_color", _max_color);
#endif

}

vector<matched_rect> ArmorDetector::findTarget() {
    vector<matched_rect> match_rects;

    // find contour in sub image of blue and red
    vector<vector<Point2i>> contours_max;
    // for debug use
    //Mat FirstResult = _src.clone();


    // br是直接在_max_color上寻找的轮廓////只检测最外围，max_color是setimage里的找到两灯条的二值化图片
    //findContours(_max_color, contours_max, hierarchy, CV_RETR_EXTERNAL , CV_CHAIN_APPROX_SIMPLE);
    findContours(_max_color, contours_max, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    // 用直线拟合轮廓，找出符合斜率范围的轮廓
    vector<RotatedRect> RectFirstResult;
    for (size_t i = 0; i < contours_max.size(); ++i){
        // fit the lamp contour as a eclipse
        RotatedRect rrect = minAreaRect(contours_max[i]);
        double max_rrect_len = MAX(rrect.size.width, rrect.size.height);
        double min_rrect_len = MIN(rrect.size.width, rrect.size.height);

        /////////////////////////////// 单根灯条的条件 //////////////////////////////////////
        // 角度要根据实际进行略微改动
        bool if1 = (fabs(rrect.angle) < 30.0 && rrect.size.height > rrect.size.width); // 往左,指现在偏向左
        bool if2 = (fabs(rrect.angle) > 60.0 && rrect.size.width > rrect.size.height); // 往右，指现在偏向右
        bool if3 = max_rrect_len > _para.min_light_height; // 灯条的最小长度
        bool if4;
        if (!base_mode) // 吊射基地时条件不同
            if4 = (max_rrect_len / min_rrect_len >= 1.1) && (max_rrect_len / min_rrect_len < 15); // 灯条的长宽比
            else
                if4 = (max_rrect_len / min_rrect_len >= 9.9) && (max_rrect_len / min_rrect_len < 30); // 灯条的长宽比
                // 筛除横着的以及太小的旋转矩形 (本来是45的，后来加成60)
                if ((if1 || if2) && if3 && if4)
                {
                    RectFirstResult.push_back(rrect);
                }
    }

    // 少于两根灯条就认为没有匹配到
    if (RectFirstResult.size() >= 2) {

        // 将旋转矩形从左到右排序
        sort(RectFirstResult.begin(), RectFirstResult.end(),
             [](RotatedRect& a1, RotatedRect& a2) {
            return a1.center.x < a2.center.x; });

        Point2f _pt[4], pt[4];
        auto ptangle = [](const Point2f& p1, const Point2f& p2) {
            return fabs(atan2(p2.y - p1.y, p2.x - p1.x) * 180.0 / CV_PI);
        };////传入两个点，算出两点线段的角度。

        ///////////////////////////////////// 匹配灯条的条件 //////////////////////////////////////////////////////
        // 两两比较，有符合条件的就组成一个目标旋转矩形
        for (size_t i = 0; i < RectFirstResult.size() - 1; ++i) {
            const RotatedRect& rect_i = RectFirstResult[i];
            const Point& center_i = rect_i.center;
            float xi = center_i.x;
            float yi = center_i.y;
            float leni = MAX(rect_i.size.width, rect_i.size.height);
            float anglei = fabs(rect_i.angle);
            rect_i.points(_pt);
            /*pt
				* 0 2
				* 1 3
				* */
            // 往右斜的长灯条
            // rRect.points有顺序的，y最小的点是0,顺时针1 2 3
            if (anglei > 45.0) {
                pt[0] = _pt[2];
                pt[1] = _pt[3];
            }
            // 往左斜的
            else {
                pt[0] = _pt[1];
                pt[1] = _pt[2];
            }

            for (size_t j = i + 1; j < RectFirstResult.size(); j++) {
                const RotatedRect& rect_j = RectFirstResult[j];
                const Point& center_j = rect_j.center;
                float xj = center_j.x;
                float yj = center_j.y;
                float lenj = MAX(rect_j.size.width, rect_j.size.height);
                float anglej = fabs(rect_j.angle);
                float delta_h = xj - xi;////两灯条距离
                float lr_rate = leni > lenj ? leni / lenj : lenj / leni;////高度比
                float angleabs;


                rect_j.points(_pt);
                if (anglej > 45.0)
                {
                    pt[2] = _pt[1];
                    pt[3] = _pt[0];
                }
                else {
                    pt[2] = _pt[0];
                    pt[3] = _pt[3];
                }
                double maxangle = MAX(ptangle(pt[0], pt[2]), ptangle(pt[1], pt[3]));  ////宽，横着的那个的角度
                //std::cout<<"angle:"<<maxangle<<std::endl;
                // maxangle = 0;如果摄像头歪了呢
                if (anglei > 45.0 && anglej < 45.0) { // 八字 / \   //
                    angleabs = 90.0 - anglei + anglej;    ////可能是越小越好，两个小方框的差值
                }
                else if (anglei <= 45.0 && anglej >= 45.0) { // 倒八字 \ /
                    angleabs = 90.0 - anglej + anglei;
                }
                else {
                    if (anglei > anglej) angleabs = anglei - anglej; // 在同一边
                    else angleabs = anglej - anglei;
                }


                // if rectangle is m atch condition, put it in candidate vector
                // lr_rate1.3有点小了，调大点可以对付破掉的3号车
                bool condition1 = delta_h > _para.min_light_delta_h && delta_h < _para.max_light_delta_h;
                bool condition2 = MAX(leni, lenj) >= 113 ? abs(yi - yj) < 166\
                && abs(yi - yj) < 1.66 * MAX(leni, lenj) :
                        abs(yi - yj) < _para.max_light_delta_v\
                        && abs(yi - yj) < 1.2 * MAX(leni, lenj); // && abs(yi - yj) < MIN(leni, lenj)
                                bool condition3 = lr_rate < _para.max_lr_rate;
                //                bool condition4 = angleabs < 15 ; // 给大点防止运动时掉帧
                bool condition4;
                if (!base_mode)
                    condition4 = sentry_mode ? angleabs < 25 : angleabs < 15 - 5;
                else
                    condition4 = angleabs > 25 && angleabs < 55;
                //                bool condition5 = sentry_mode ? true : /*maxangle < 20*/true;

                //            bool condition4 = delta_angle < _para.max_light_delta_angle;

                Point text_center = Point((xi + xj) / 2, (yi + yj) / 2);


                if (condition1 && condition2 && condition3 && condition4)
                {
                    RotatedRect obj_rect = boundingRRect(rect_i, rect_j);
                    double w = obj_rect.size.width;
                    double h = obj_rect.size.height;
                    double wh_ratio = w / h;
                    // 长宽比不符

                    // 基地模式不受长宽比的限制
                    if (!base_mode) {
                        if (wh_ratio > _para.max_wh_ratio || wh_ratio < _para.min_wh_ratio)
                            continue;
                    }


                    // 将初步匹配到的结构体信息push进入vector向量
                    match_rects.push_back(matched_rect{ obj_rect, lr_rate, angleabs });
                    // for debug use
                    /*Point2f vertice[4];
					obj_rect.points(vertice);
					for (int i = 0; i < 4; i++)
						line(FirstResult, vertice[i], vertice[(i + 1) % 4], Scalar(255, 255, 255), 2);*/

                }
            }
        }
    }
    return match_rects;
}



cv::RotatedRect ArmorDetector::chooseTarget(const std::vector<matched_rect> & match_rects, const cv::Mat & src) {
    // 如果没有两条矩形围成一个目标装甲板就返回一个空的旋转矩形
    if (match_rects.size() < 1){
        _is_lost = true;
        return RotatedRect();
    }


    // 初始化参数
    int ret_idx = -1;
    bool is_small = false;
    double weight = 0;
    vector<candidate_target> candidate;
    // 二分类判断真假装甲板初始化
    Mat input_sample;
    vector<Mat> channels;

#define SafeRect(rect, max_size) {if (makeRectSafe(rect, max_size) == false) continue;}

    ///////////////////////// 匹配灯条 ////////////////////////////////////////////////
    //======================= 开始循环 ====================================

    for (size_t i = 0; i < match_rects.size(); i++){
        const RotatedRect & rect = match_rects[i].rect;

        // 长宽比不符,担心角度出问题，可以侧着车身验证一下（上面一个函数好像写过这个条件了）
        double w = rect.size.width;
        double h = rect.size.height;
        double wh_ratio = w / h;

        // 如果矩形的w和h之比小于阈值的话就是小装甲，否则是大装甲(初步判断)
        if (wh_ratio < _para.small_armor_wh_threshold)
            is_small = true;
        else
            is_small = false;

        //        cout << "wh_ratio: " << wh_ratio << endl;

        // 用均值和方差去除中间太亮的图片（例如窗外的灯光等）
        RotatedRect screen_rect = RotatedRect(rect.center, Size2f(rect.size.width * 0.88, rect.size.height), rect.angle);
        ////为什么要缩小宽度,是为了将装甲板两边的灯条去除
        Size size = Point(src.cols, src.rows);
        Point p1, p2;
        int x = screen_rect.center.x - screen_rect.size.width / 2 + _dect_rect.x;
        int y = screen_rect.center.y - screen_rect.size.height / 2 + _dect_rect.y;
        ///答：坐标轴的转换，前面是使用截取坐标（_src）后面用的是原图坐标
        p1 = Point(x, y);
        x = screen_rect.center.x + screen_rect.size.width / 2 + _dect_rect.x;
        y = screen_rect.center.y + screen_rect.size.height / 2 + _dect_rect.y;
        p2 = Point(x, y);
        Rect roi_rect = Rect(p1, p2);
        Mat roi;
        if(makeRectSafe(roi_rect, size))
        {
            roi = src(roi_rect).clone();
            Mat mean, stdDev;
            double avg, stddev;
            meanStdDev(roi, mean, stdDev);////mean、stdDev为R3向量，三个维度分别表示bgr三通道的均值和标准差
            avg = mean.ptr<double>(0)[0];
            stddev = stdDev.ptr<double>(0)[0];
#ifdef SHOW_DEBUG_IMG
            cout << "                                            " << avg << endl;
            cout << "                                            " << stddev << endl << endl;
            //            putText(roi, to_string(int(avg)), rects[i].center, CV_FONT_NORMAL, 1, Scalar(0, 255, 255), 2);
            imshow("2.jpg", roi);
#endif
            // 阈值可通过实际测量修改
            if (avg > 57.00)
                continue;
            if (stddev < 15.8)
                continue;
        }




        // 现在这个旋转矩形的角度
        float cur_angle = match_rects[i].rect.size.width > match_rects[i].rect.size.height ? \
        abs(match_rects[i].rect.angle) : 90 - abs(match_rects[i].rect.angle);
                // 现在这个旋转矩形的高度（用来判断远近）
                        int cur_height = MIN(w, h);
        // 最终目标如果太倾斜的话就筛除
        if (cur_angle > 33 - 5) continue; // (其实可以降到26)
        // 把矩形的特征信息push到一个候选vector中
        candidate.push_back(candidate_target{cur_height, cur_angle, static_cast<int>(i), is_small, match_rects[i].lr_rate, match_rects[i].angle_abs});
        ret_idx = 1;
    }
    //================================ 到这里才结束循环 =======================================
    int final_index = 0;
    if (candidate.size() > 1){
        // 将候选矩形按照高度大小排序，选出最大的（距离最近）
        sort(candidate.begin(), candidate.end(),
             [](candidate_target & target1, candidate_target & target2){
            return target1.armor_height > target2.armor_height;
        });
        // 做比较，在最近的几个矩形中（包括45度）选出斜率最小的目标
        /**
		 * 下面的几个temp值可以筛选出最终要击打的装甲板，我只用了一两个效果已经挺好的了
		 * 只是偶尔还会有误识别的情况，可以将这几个temp值都组合起来进行最终的判断
		 * */

        float temp_angle = candidate[0].armor_angle;
        float temp_lr_rate = candidate[0].bar_lr_rate;
        float temp_angle_abs = candidate[0].bar_angle_abs;
        float temp_weight = temp_angle + temp_lr_rate;

        for (int i = 1; i < candidate.size(); i++){
            double angle = match_rects[candidate[i].index].rect.angle;
            if ( candidate[0].armor_height / candidate[i].armor_height < 1.1){
                if (candidate[i].armor_angle < temp_angle
                /*&& (candidate[i].bar_lr_rate */){
                    temp_angle = candidate[i].armor_angle;
                    if (candidate[i].bar_lr_rate < 1.66) final_index = i;
                }
            }
        }
    }

#ifdef SHOW_DEBUG_IMG
    Mat rect_show;
    _src.copyTo(rect_show);
    //     候选区域
    Point2f vertices[4];
    match_rects[final_index].rect.points(vertices);
    //    putText(rect_show, to_string(int(match_rects[final_index].rect.angle)), match_rects[final_index].rect.center, CV_FONT_NORMAL, 1, Scalar(0, 255, 0), 2);
    for (int i = 0; i < 4; i++)
        line(rect_show, vertices[i], vertices[(i + 1) % 4], CV_RGB(255, 0, 0));
    imshow("final_rect", rect_show);
#endif

    // ret_idx为 -1 就说明没有目标
    if (ret_idx == -1){
        _is_lost = true;
        return RotatedRect();
    }
    // 否则就证明找到了目标
    _is_lost = false;
    _is_small_armor = candidate[final_index].is_small_armor;
    RotatedRect ret_rect = match_rects[candidate[final_index].index].rect;
    return ret_rect;
}


//将之前的各个函数都包含在一个函数中，直接用这一个
aim_information ArmorDetector::getTargetAera(const cv::Mat & src, const int & sb_mode, const int & jd_mode){
    // 传入参数为哨兵模式和吊射基地模式
    sentry_mode = sb_mode;
    base_mode = jd_mode;
    setImage(src);  // function called
    vector<matched_rect> match_rects = findTarget(); // function called
    RotatedRect final_rect = chooseTarget(match_rects, src);  // function called

    if(final_rect.size.width != 0){
        // 最后才加上了偏移量，前面那些的坐标都是不对的，所以不能用前面那些函数解角度
        final_rect.center.x += _dect_rect.x;
        final_rect.center.y += _dect_rect.y;
        _res_last = final_rect;
        _lost_cnt = 0;
    }
    else{
        _find_cnt = 0;
        ++_lost_cnt;

        // 逐次加大搜索范围（根据相机帧率调整参数）
        if (_lost_cnt < 8)
            _res_last.size = Size2f(_res_last.size.width, _res_last.size.height);
        else if(_lost_cnt == 9)
            _res_last.size =Size2f(_res_last.size.width * 1.2, _res_last.size.height * 1.2);
        else if(_lost_cnt == 12)
            _res_last.size = Size2f(_res_last.size.width * 1.5, _res_last.size.height * 1.5);
        else if(_lost_cnt == 15)
            _res_last.size = Size2f(_res_last.size.width * 1.2, _res_last.size.height * 1.2);
        else if (_lost_cnt == 18)
            _res_last.size = Size2f(_res_last.size.width * 1.2, _res_last.size.height * 1.2);
        else if (_lost_cnt > 33 )
            _res_last = RotatedRect();
    }
    int class_num = num_detect(final_rect);
    aim_information aim_inf;
    aim_inf.final_rect = final_rect;
    aim_inf.class_id = class_num;
    return aim_inf;
}


cv::RotatedRect ArmorDetector::boundingRRect(const cv::RotatedRect & left, const cv::RotatedRect & right){////这里可以改进
    // 这个函数是用来将左右边的灯条拟合成一个目标旋转矩形，没有考虑角度
    const Point & pl = left.center, & pr = right.center;
    Point2f center = (pl + pr) / 2.0;
    //    cv::Size2f wh_l = left.size;
    //    cv::Size2f wh_r = right.size;
    // 这里的目标矩形的height是之前灯柱的width
    double width_l = MIN(left.size.width, left.size.height);
    double width_r = MIN(right.size.width, right.size.height);
    double height_l = MAX(left.size.width, left.size.height);
    double height_r = MAX(right.size.width, right.size.height);
    float width = POINT_DIST(pl, pr) - (width_l + width_r) / 2.0;
    float height = std::max(height_l, height_r);
    //float height = (wh_l.height + wh_r.height) / 2.0;
    float angle = std::atan2(right.center.y - left.center.y, right.center.x - left.center.x);
    return RotatedRect(center, Size2f(width, height), angle * 180 / CV_PI);
    ////这里需要测试角度
}

int ArmorDetector::num_detect(cv::RotatedRect &f_rect)
{
    int classid = 0;
    Point2f pp[4];
    f_rect.points(pp);

    Mat num_show;
    Mat dst;
    _src.copyTo(num_show);

    Point2f src_p[4];
    src_p[0].x = pp[1].x + (pp[0].x - pp[1].x)*1.6;
    src_p[0].y = pp[1].y + (pp[0].y - pp[1].y)*1.6;

    src_p[1].x = pp[0].x + (pp[1].x - pp[0].x)*1.6;
    src_p[1].y = pp[0].y + (pp[1].y - pp[0].y)*1.6;

    src_p[2].x = pp[3].x + (pp[2].x - pp[3].x)*1.6;
    src_p[2].y = pp[3].y + (pp[2].y - pp[3].y)*1.6;

    src_p[3].x = pp[2].x + (pp[3].x - pp[2].x)*1.6;
    src_p[3].y = pp[2].y + (pp[3].y - pp[2].y)*1.6;


    //    line(num_show,src_p[0],src_p[1],Scalar(0,0,255),2);
    //    line(num_show,src_p[1],src_p[2],Scalar(255,0,0),2);
    //    line(num_show,src_p[2],src_p[3],Scalar(0,255,0),2);
    //    line(num_show,src_p[3],src_p[0],Scalar(255,255,0),2);

    Mat matrix_per = getPerspectiveTransform(src_p,dst_p);
    warpPerspective(num_show,dst,matrix_per,Size(30,60));

    //    imshow("dst",dst);
    Mat bin_dst,gray_dst;
    cvtColor(dst,gray_dst,COLOR_BGR2GRAY);

    threshold(gray_dst,bin_dst,0,255,THRESH_OTSU);

    vector<vector<Point>> cnts;
    findContours(bin_dst,cnts,RETR_EXTERNAL,CHAIN_APPROX_SIMPLE);
    sort(cnts.begin(),cnts.end(), area_sort);
    Rect dst_roi = boundingRect(cnts[0]);

    Mat num_roi;
    bin_dst(dst_roi).copyTo(num_roi);
    imshow("num",num_roi);



    double minVal,maxVal;
    Point minLoc,maxLoc;
    int max_index = -1;
    double max_val = 0;
    for (int i=0;i<6;i++)
    {
        Mat result;

        matchTemplate(num_roi,temps[i],result,TM_CCORR_NORMED);
        minMaxLoc(result,&minVal, &maxVal, &minLoc, &maxLoc);
        printf("max_val:%lf\n",maxVal);
        if((maxVal > max_num)&&(maxVal > max_val))
        {
            max_index = i;
            max_val = maxVal;
        }
    }

    if (max_index == 0)
    {
        classid = 1;
    }
    else if (max_index == 1)
    {
        classid = 3;
    }
    else if (max_index == 2)
    {
        classid = 4;
    }
    else
    {
        classid = 0;
    }
    return classid;
}




