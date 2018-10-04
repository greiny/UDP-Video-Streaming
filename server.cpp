#include "opencv2/opencv.hpp"
#include <sl/Camera.hpp>
#include <iostream>
#include <sys/socket.h> 
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <unistd.h> 
#include <string.h>

using namespace cv;
using namespace std;
using namespace sl;

void *display(void *);

cv::Mat slMat2cvMat(sl::Mat &);

int INPUT_WIDTH = 672;
int INPUT_HEIGHT = 376;
cv::Mat dst = cv::Mat::zeros(Size(INPUT_WIDTH , INPUT_HEIGHT/2), CV_8UC3);

int main()
{   
	Camera zed;
	InitParameters init_params;
	init_params.camera_resolution = RESOLUTION_VGA;
	init_params.camera_fps = 60;
	init_params.coordinate_units = UNIT_CENTIMETER;
	init_params.depth_minimum_distance = 30;

    RuntimeParameters params;
	params.sensing_mode = SENSING_MODE_STANDARD; //SENSING_MODE_FILL;

    // Open the camera
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        cout << errorCode2str(err) << endl;
        zed.close();
    }
	Resolution image_size = zed.getResolution();
	INPUT_WIDTH = image_size.width;
	INPUT_HEIGHT = image_size.height;

	CalibrationParameters intrin = zed.getCameraInformation().calibration_parameters;
	float fx = intrin.left_cam.fx;

	sl::Mat zed_frame(INPUT_WIDTH, INPUT_HEIGHT, MAT_TYPE_8U_C4);
	sl::Mat zed_frame_depth(INPUT_WIDTH,INPUT_HEIGHT, MAT_TYPE_8U_C4);
	sl::Mat depth_zed(INPUT_WIDTH,INPUT_HEIGHT, MAT_TYPE_32F_C1);
	cv::Mat frame2_ocv = slMat2cvMat(zed_frame);
	cv::Mat frame_ocv = slMat2cvMat(zed_frame_depth);
	cv::Mat data_ocv = slMat2cvMat(depth_zed);

	char file_name1[255]; char file_name2[255];
	int file_no = 0;
	sprintf(file_name1,"log/capture_rgb(%d).avi",file_no);
	sprintf(file_name2,"log/capture_depth(%d).avi",file_no);
	for ( int i=0 ; i < 100 ; i++ )
	{
		std::ifstream test(file_name1);
		if (!test) break;
		file_no++;
		sprintf(file_name1,"log/capture_rgb(%d).avi",file_no);
		sprintf(file_name2,"log/capture_depth(%d).avi",file_no);
	}
	VideoWriter video1(file_name1,CV_FOURCC('M','J','P','G'),20, Size(INPUT_WIDTH,INPUT_HEIGHT/2),true);
	VideoWriter video2(file_name2,CV_FOURCC('M','J','P','G'),20, Size(INPUT_WIDTH,INPUT_HEIGHT/2),true);

    //--------------------------------------------------------
    //networking stuff: socket, bind, listen
    //--------------------------------------------------------
    int                 localSocket,
                        remoteSocket,
                        port = 4097;                               

    struct  sockaddr_in localAddr,
                        remoteAddr;
    pthread_t thread_id;

    int addrLen = sizeof(struct sockaddr_in);
    localSocket = socket(AF_INET , SOCK_STREAM , 0);
    if (localSocket == -1){
         perror("socket() call failed!!");
    }    

    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = INADDR_ANY;
    localAddr.sin_port = htons( port );

    if( bind(localSocket,(struct sockaddr *)&localAddr , sizeof(localAddr)) < 0) {
         perror("Can't bind() socket");
         exit(1);
    }
    
    //Listening
    listen(localSocket , 3);
    
    std::cout <<  "Waiting for connections...\n"
              <<  "Server Port:" << port << std::endl;

    remoteSocket = accept(localSocket, (struct sockaddr *)&remoteAddr, (socklen_t*)&addrLen);  
    if (remoteSocket < 0) {
	perror("accept failed!");
	exit(1);
    } 
    std::cout << "Connection accepted" << std::endl;
    pthread_create(&thread_id,NULL,display,&remoteSocket);

    bool flag=1;
    while(1){
	if (zed.grab(params) == SUCCESS){
	if (flag == 1){
		zed.retrieveImage(zed_frame, VIEW_LEFT, MEM_CPU, INPUT_WIDTH, INPUT_HEIGHT);
		zed.retrieveMeasure(depth_zed, MEASURE_DEPTH, MEM_CPU, INPUT_WIDTH, INPUT_HEIGHT);
		flag = 0;
	}
	else
	{
		cv::Mat frame2, frame;
		cv::Mat data = data_ocv.clone();

		vector<cv::Mat> channels1(4);
		split(frame2_ocv, channels1); 
		channels1.pop_back();
		merge(channels1, frame2);;
		channels1.clear();

		data.convertTo(frame,CV_8UC1); 
		cvtColor(frame, frame, COLOR_GRAY2RGB);
		cv::Mat test1 = (data > 30) & (data < 50);
		for(int dy=0; dy<data.rows; dy++){
			for(int dx=0; dx<data.cols; dx++){
				float buf = data.at<float>(dy,dx);
				if(isnan(buf)) data.at<float>(dy,dx) = 0;
				else if (buf<50) data.at<float>(dy,dx) = 0;
				else if (buf>250) data.at<float>(dy,dx) = 0;
			}
		}
		cv::Mat test2; data.convertTo(test2,CV_8UC1);
		cv::Mat kernel1 = getStructuringElement(MORPH_RECT, Size(10,10));
		morphologyEx(test2,test2,CV_MOP_OPEN,kernel1);
		cvtColor(test2, test2, COLOR_GRAY2RGB);

		cout << "working!!" << endl;
		video1 << frame2;
		video2 << test2;

		resize(frame2,frame2,Size(),0.5,0.5);
		resize(test2,test2,Size(),0.5,0.5);
		hconcat(frame2,test2,dst);
		flag = 1;
	    }
	}

    }

    pthread_join(thread_id,NULL);
    close(remoteSocket);
    zed.close();
    return 0;
}

void *display(void *ptr){
    int socket = *(int *)ptr;
    cv::Mat img = cv::Mat::zeros(Size(INPUT_WIDTH , INPUT_HEIGHT/2), CV_8UC3);   
    if (!img.isContinuous()) img = img.clone();
    int imgSize = img.total() * img.elemSize();
    int bytes = 0;
    int key;
    std::cout << "Image Size:" << imgSize << std::endl;

    while(1) {
	dst.copyTo(img);
        //send processed image
        if ((bytes = send(socket, img.data, imgSize, 0)) < 0){
             std::cerr << "bytes = " << bytes << std::endl;
             break;
        } 
    }
}

cv::Mat slMat2cvMat(sl::Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
      	case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }
 return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}
