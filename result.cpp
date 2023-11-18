#include <iostream>
#include <cstdio>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

/*************************************/
    /**********矫正灯条***********/
/*************************************/
cv::RotatedRect& adjustRec(cv::RotatedRect& rec)
{
	using std::swap;

	float& width = rec.size.width;
	float& height = rec.size.height;
	float& angle = rec.angle;

 		if(angle >= 45.0)
		{
			swap(width, height);
			angle -= 90.0;
		}
		else 
        if(angle < -45.0)
		{
			swap(width, height);
			angle += 90.0;
		}

	while(angle >= 90.0) angle -= 180.0;
	while(angle < -90.0) angle += 180.0;

	return rec;
}
/*************************************/
    /**********分类灯条***********/
/*************************************/
void _sortlr(cv::RotatedRect& recl,cv::RotatedRect& recr)
{
    if(recl.center.x>recr.center.x)
    {
        cv::RotatedRect sw;
        sw = recl;
        recl = recr;
        recr = sw;
    }
}
/*************************************/
    /**********矫正点集***********/
/*************************************/
void pointArrange(Point2f point[4])
{
    if(point[1].y>point[3].y)
    {
        Point2f tran[4];
        tran[0] = point[1];
        tran[1] = point[2];
        tran[2] = point[3];
        tran[3] = point[0];
        point[0] = tran[0];
        point[1] = tran[1];
        point[2] = tran[2];
        point[3] = tran[3];
    }
}
// bool SWAP(RotatedRect num1,RotatedRect num2)
// {
//     return num1.center.x>num2.center.x;
// }
int main()
{   
/*************************************/
    /*******卡尔曼滤波器初始化*******/
/*************************************/
    cv::Mat P(4, 4, CV_32F);
    
    P.setTo(Scalar::all(1e-6));

    int stateNum = 4;
    
    int measureNum = 2;
    
    KalmanFilter KF(stateNum, measureNum, 0);
    
    //Mat processNoise(stateNum, 1, CV_32F);
    Mat measurement = Mat::zeros(measureNum, 1, CV_32F);
    
    KF.transitionMatrix = (Mat_<float>(stateNum, stateNum) << 1, 0, 1, 0,//A 状态转移矩阵
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1);
    //这里没有设置控制矩阵B，默认为零
    
    setIdentity(KF.measurementMatrix);//H=[1,0,0,0;0,1,0,0] 测量矩阵
    
    setIdentity(KF.processNoiseCov, Scalar::all(1e-5));//Q高斯白噪声，单位阵
    
    setIdentity(KF.measurementNoiseCov, Scalar::all(1e-1));//R高斯白噪声，单位阵
    
    setIdentity(KF.errorCovPost, Scalar::all(1));//P后验误差估计协方差矩阵，初始化为单位阵
    
    randn(KF.statePost, Scalar::all(0), Scalar::all(0.1));//初始化状态为随机值

/*************************************/
    /**********读取视频***********/
/*************************************/
    cv::VideoCapture cap("car.webm"); 
    
    cv::Mat frame;
    cv::Mat midImage;
    std::vector<Mat>Channels;

    while (true)
    {
        cap.read(frame); 

/*************************************/
    /**********显示帧数***********/
/*************************************/

        double fps = cap.get(cv::CAP_PROP_FPS);

        putText(frame,std::to_string(fps),Point(20,50),FONT_HERSHEY_PLAIN,2,Scalar(0,255,0),1,8,false); 

/*************************************/
    /*********根据颜色筛选灯条********/
/*************************************/

        // cv::Mat hsv;
        // cv::cvtColor(frame,hsv,  cv::COLOR_BGR2HSV);
       
        // cv::Mat mask; 
        // cv::inRange(frame,   cv::Scalar(0, 50,50),   cv::Scalar(30, 255, 255), mask);
        
        // cv::Mat binary;          
        // cv::threshold(mask, binary, 0, 255, cv::THRESH_BINARY + THRESH_OTSU);
  
        cv::GaussianBlur(frame, frame, cv::Size(5, 5), 0, 0);

        Mat binary;
        split(frame,Channels);
        midImage = Channels.at(2)-Channels.at(0);
       
        threshold(midImage,binary,150,255,THRESH_BINARY); 

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        cv::Mat dilate;
        // cv::dilate(binary,dilate,kernel);

        cv::dilate(binary,dilate,Mat());
        // cv::dilate(dilate,dilate,Mat());

        // cv::Mat edges;  	    
        // cv::Canny(dilate, edges, 50, 150);

/*************************************/
    /**********寻找轮廓***********/
/*************************************/
        
        std::vector<vector<Point>>contours;
        std::vector<RotatedRect>lights;
        std::vector<RotatedRect>lightsneed;
        cv::RotatedRect leftlight;
        cv::RotatedRect rightlight;
           
        cv::findContours(dilate,contours,cv::RETR_EXTERNAL,  cv:: CHAIN_APPROX_SIMPLE);
    for(const auto& contour:contours)
    {
        float contourarea = cv::contourArea(contour);
        if(contourarea < 10)
        continue;
        cv::RotatedRect lightRec = cv::minAreaRect(contour);
        adjustRec(lightRec);
        if(lightRec.size.width / lightRec.size.height > 1 ||contourarea / lightRec.size.area() < 1/5  )
        continue;
        lightRec.size.width*=6/5;
        lightRec.size.height*=6/5;
        lights.push_back(lightRec);
    }
    // for(size_t i=0; i<lights.size();i++)
    // {
    //     for(size_t j=i+1; j<lights.size();j++)
    //     {
    //         sort(lights.begin(),lights.end(),SWAP(lights[i],lights[j]));
    //     }
    // } 

/*************************************/
    /******筛选并配对灯条*******/
/*************************************/
            for(size_t i=0; i<lights.size();i++)
            {
                for(size_t j=i+1; j<lights.size();j++)
                {
                     leftlight = lights[i];
                     rightlight = lights[j];
                     float anglediff = abs(leftlight.angle-rightlight.angle);
                     if(anglediff>10)
                     continue;
                     float ydiff = abs(leftlight.center.y-rightlight.center.y);
                     if(ydiff>12)
                     continue;
                     float xdiff = abs(leftlight.center.x-rightlight.center.x);
                     if(xdiff<10)
                     continue;
                     if(xdiff>120)
                     continue;
                     if(leftlight.size.height<10||leftlight.size.height>150)
                     continue;
                     if(leftlight.size.height/rightlight.size.height>5/2)
                     continue;
                     if(rightlight.size.height/leftlight.size.height>5/2)
                     continue;
                     if(rightlight.size.area()/leftlight.size.area()>5)
                     continue;
                     if(leftlight.size.area()/rightlight.size.area()>5)
                     continue;
                     float ratio = xdiff/((leftlight.size.height+rightlight.size.height)/2);
                     if(ratio<1/2)
                     continue;
                     lightsneed.push_back(leftlight);
                     lightsneed.push_back(rightlight);
                     _sortlr(leftlight,rightlight);
                     break;
                }
            }

/*************************************/
    /**********绘制轮廓***********/
/*************************************/
        RotatedRect roi;
        Point2f roipoints[4];
        Point2f lvertice[4];
        Point2f rvertice[4];

    if(lightsneed.size()>=2)
    {
        lightsneed[0].points(lvertice);
        lightsneed[1].points(rvertice);



        pointArrange(lvertice);
        pointArrange(rvertice);
        
        roi.points(roipoints);
        roipoints[0] = lvertice[3];
        roipoints[1] = lvertice[2];
        roipoints[2] = rvertice[1];
        roipoints[3] = rvertice[0];

        pointArrange(roipoints);
            
        for(int i=0; i<4; i++)
        {
            cv::line(frame, lvertice[i], lvertice[(i + 1) % 4], cv::Scalar(0, 255, 0), 3);
            cv::line(frame, rvertice[i], rvertice[(i + 1) % 4], cv::Scalar(0, 255, 0), 3);
            cv::line(frame, roipoints[i], roipoints[(i + 1) % 4], cv::Scalar(0, 255, 0), 3);
        }

    }

/*************************************/
    /********卡尔曼滤波预测*********/
/*************************************/
        cv::Point2f cen((lvertice[2].x+rvertice[0].x)/2,(lvertice[2].y+rvertice[0].y)/2);

            measurement.at<float>(0) = (float)cen.x;
            measurement.at<float>(1) = (float)cen.y;
            KF.correct(measurement);

            Mat prediction;
            Point predict_pt;

            if(!lightsneed.empty())
            {
             prediction = KF.predict();
             predict_pt = Point((int)prediction.at<float>(0), (int)prediction.at<float>(1));
            //  cen.x = (int)prediction.at<float>(0);
            //  cen.y = (int)prediction.at<float>(1);
            }
            else
            {
             const Point predict_pt;
            }

            Point2f humanPredicted((int)cen.x*2-(int)predict_pt.x,(int)cen.y*2-(int)predict_pt.y);

            Point2f result((cen.x+humanPredicted.x)/2,(cen.y+humanPredicted.y)/2);

            //circle(frame, humanPredicted, 6, Scalar(255, 255, 0), 2);
            //circle(frame, predict_pt, 6, Scalar(34, 255, 255), -1);
            circle(frame,cen,6,Scalar(255, 0, 255), -1);
            circle(frame, result, 6, Scalar(255, 255, 0), 2);

            cen.x = (int)result.x;
            cen.y = (int)result.y;

/*************************************/
    /*********solvepnp解算********/
/*************************************/

    std::vector<cv::Point2f>image_points;
    std::vector<cv::Point3f>world_points;
    
    image_points.push_back(cen);
    
    image_points.push_back(lvertice[2]);
    
    image_points.push_back(lvertice[3]);
    
    image_points.push_back(rvertice[0]);

    world_points.push_back(Point3f(0,0,0));
    
    world_points.push_back(Point3f(-67,-27,0));
    
    world_points.push_back(Point3f(-67,27,0));
    
    world_points.push_back(Point3f(67,27,0)); 

    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	
    cameraMatrix.at<double>(0,0) = frame.size().width/(2*tan(CV_PI/6));
	
    cameraMatrix.at<double>(0,1) = 0;
	
    cameraMatrix.at<double>(0,2) = frame.size().width/2;
	
    cameraMatrix.at<double>(1,0) = 0;
	
    cameraMatrix.at<double>(1,1) = frame.size().height/(2*tan(CV_PI/6));
    
    cameraMatrix.at<double>(1,2) = frame.size().height/2;
    
    cameraMatrix.at<double>(2,0) = 0;
    
    cameraMatrix.at<double>(2,1) = 0;    
    
    cameraMatrix.at<double>(2,2) = 1;

	Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
	
    distCoeffs.at<double>(0,0) = 0;
	
    distCoeffs.at<double>(1,0) = 0;
	
    distCoeffs.at<double>(2,0) = 0;
	
    distCoeffs.at<double>(3,0) = 0;
	
    distCoeffs.at<double>(4,0) = 0;

    double fx = cameraMatrix.at<double>(0,0);
    double fy = cameraMatrix.at<double>(1,1);
            
        cv::Mat rvec = cv::Mat::zeros(3,1,CV_64FC1);
        cv::Mat tvec = cv::Mat::zeros(3,1,CV_64FC1);
            
    solvePnP(world_points,image_points,cameraMatrix,distCoeffs,rvec,tvec,false,SOLVEPNP_ITERATIVE);
            
            //cout << "tvec"<<tvec<<endl;
            //cout << "rvec"<<rvec<<endl;

/*************************************/
    /*******计算距离与角度********/
/*************************************/
    if(!lightsneed.empty())
    {
        double distance = -1;
        double errorx = (int)tvec.at<double>(0,0);
        double errory = (int)tvec.at<double>(1,0);
        double errorz = (int)tvec.at<double>(2,0);
 
        distance = sqrt(errorx*errorx+errory*errory+errorz*errorz);

        int fire = 1;
        double yaw;
        double pitch;

        yaw = atan(errorx/errorz)/CV_PI*180;
        pitch = atan(errory/errorz)/CV_PI*180;

        cout<< "yaw " << yaw<< endl;
        cout<< "pitch " << pitch << endl;
        cout<< "距离是 " << distance/1000 << endl;

        // cout<< "yaw " << errorx<< endl;
        // cout<< "pitch " << errory << endl;
        // cout<< "距离是 " << errorz << endl;


    }
    else
    {
    cout << "loss" << endl;
    }

    //  void compensateGravity();
    // {
	// const auto& theta_p_prime = _yErr / 180 * CV_PI;
	// const auto& d_prime = _euclideanDistance;
	// const auto& v = _bullet_speed;
	// const auto theta_p_prime2 = atan((sin(theta_p_prime) - 0.5*9.8*d_prime / pow(v, 2)) / cos(theta_p_prime));
	// _   yErr = theta_p_prime2 / CV_PI * 180;
    // }    

    cv::imshow("edges",dilate);
    cv::imshow("frame",frame);
    int  key = cv::waitKey(30);        
    if (key ==  int('q')) 
    {
        break;
    }
    }
    cap.release(); 
    cv::destroyAllWindows();        
    return 0;
}


