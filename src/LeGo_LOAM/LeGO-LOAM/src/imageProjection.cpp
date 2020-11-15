// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following papers:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#include "utility.h"
#include "jsk_recognition_msgs/BoundingBoxArray.h"
#include "autoware_msgs/DetectedObjectArray.h"
#include "autoware_msgs/DetectedObject.h"

class ImageProjection{
private:

    ros::NodeHandle nh;

    ros::Subscriber subLaserCloud;
    //추가
    ros::Subscriber subBoundingBoxInfo;
    // ros::Subscriber subBoundingBoxInfo2;

    ros::Publisher pubFullCloud;
    ros::Publisher pubFullInfoCloud;

    ros::Publisher pubGroundCloud;
    ros::Publisher pubSegmentedCloud;
    ros::Publisher pubSegmentedCloudPure;
    ros::Publisher pubSegmentedCloudInfo;
    ros::Publisher pubOutlierCloud;
    //추가
    ros::Publisher pubGroundCloudIntensity;
    // std::vector<float> Box;
    jsk_recognition_msgs::BoundingBox Box;

    //추가
    pcl::PointCloud<PointType>::Ptr groundCloudIntensity;
    pcl::PointCloud<PointType>::Ptr laserCloudIn1;
    pcl::PointCloud<PointType>::Ptr intensityCloud;

    pcl::PointCloud<PointType>::Ptr laserCloudIn;
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudInRing;

    pcl::PointCloud<PointType>::Ptr fullCloud; // projected velodyne raw cloud, but saved in the form of 1-D matrix
    pcl::PointCloud<PointType>::Ptr fullInfoCloud; // same as fullCloud, but with intensity - range

    pcl::PointCloud<PointType>::Ptr groundCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloud;
    pcl::PointCloud<PointType>::Ptr segmentedCloudPure;
    pcl::PointCloud<PointType>::Ptr outlierCloud;

    PointType nanPoint; // fill in fullCloud at each iteration

    cv::Mat rangeMat; // range matrix for range image
    cv::Mat labelMat; // label matrix for segmentaiton marking
    cv::Mat groundMat; // ground matrix for ground cloud marking
    int labelCount;

    float startOrientation;
    float endOrientation;
    
    //추가 
    float h, w, l;
    float tx, ty, tz;
    float *XM;
    float *Xm;
    float *YM;
    float *Ym;
    float *ZM;
    float *Zm;
    int que;

    size_t boxSize;

    cloud_msgs::cloud_info segMsg; // info of segmented cloud
    std_msgs::Header cloudHeader;//uint32 seq, time stamp, string frame_id

    //// neighbor iterator for segmentaiton process
    std::vector<std::pair<int8_t, int8_t> > neighborIterator; 

    uint16_t *allPushedIndX; // array for tracking points of a segmented object
    uint16_t *allPushedIndY;

    uint16_t *queueIndX; // array for breadth-first search process of segmentation, for speed
    uint16_t *queueIndY;

public:
    ImageProjection():
        nh("~"){
        //구독 ("토픽명", queue size, callback 함수)
        //추가
        que=5;
        subBoundingBoxInfo = nh.subscribe<jsk_recognition_msgs::BoundingBoxArray>("/pillar_marker_array", que, &ImageProjection::boxCallback, this);
        boxSize = 0;

        // subBoundingBoxInfo1 = nh.subscribe<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects", que, &ImageProjection::boxCallback, this);
        // subBoundingBoxInfo2 = nh.subscribe<autoware_msgs::DetectedObjectArray>("/detection/lidar_detector/objects_front", que, &ImageProjection::boxCallback1, this);

        //extern const string pointCloudTopic = "/velodyne_points"; in utility.h
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, que, &ImageProjection::cloudHandler, this);

        //발행 ("토픽명", queue size)
        pubFullCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_projected", que);
        pubFullInfoCloud = nh.advertise<sensor_msgs::PointCloud2> ("/full_cloud_info", que);

        pubGroundCloud = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud", que);
        pubSegmentedCloud = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud", que);
        pubSegmentedCloudPure = nh.advertise<sensor_msgs::PointCloud2> ("/segmented_cloud_pure", que);
        pubSegmentedCloudInfo = nh.advertise<cloud_msgs::cloud_info> ("/segmented_cloud_info", que);
        pubOutlierCloud = nh.advertise<sensor_msgs::PointCloud2> ("/outlier_cloud", que);
        //추가
        pubGroundCloudIntensity = nh.advertise<sensor_msgs::PointCloud2> ("/ground_cloud_intensity", que);

        //nan(Not a number) is point types
        nanPoint.x = std::numeric_limits<float>::quiet_NaN();
        nanPoint.y = std::numeric_limits<float>::quiet_NaN();
        nanPoint.z = std::numeric_limits<float>::quiet_NaN();
        nanPoint.intensity = -1;
        
        allocateMemory();
        resetParameters();
    }

    void allocateMemory(){
        //추가
        groundCloudIntensity.reset(new pcl::PointCloud<PointType>());
        laserCloudIn1.reset(new pcl::PointCloud<PointType>());
        XM = new float[20];
        Xm = new float[20];
        YM = new float[20];
        Ym = new float[20];
        ZM = new float[20];
        Zm = new float[20];
        intensityCloud.reset(new pcl::PointCloud<PointType>());
        intensityCloud->points.resize(N_SCAN*Horizon_SCAN);

        laserCloudIn.reset(new pcl::PointCloud<PointType>());
        laserCloudInRing.reset(new pcl::PointCloud<PointXYZIR>());

        fullCloud.reset(new pcl::PointCloud<PointType>());
        fullInfoCloud.reset(new pcl::PointCloud<PointType>());

        groundCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloud.reset(new pcl::PointCloud<PointType>());
        segmentedCloudPure.reset(new pcl::PointCloud<PointType>());
        outlierCloud.reset(new pcl::PointCloud<PointType>());
        
        //N_SCAN = 16(채널 수), Horizon_SCAN = 1800(MeasurementsPerRotation)
        fullCloud->points.resize(N_SCAN*Horizon_SCAN);//28800개
        fullInfoCloud->points.resize(N_SCAN*Horizon_SCAN);

        //assign(원소의 개수, 가ㅄ);
        segMsg.startRingIndex.assign(N_SCAN, 0);
        segMsg.endRingIndex.assign(N_SCAN, 0);

        //N_SCAN*Horizon_SCAN = 16*1800 array
        segMsg.segmentedCloudGroundFlag.assign(N_SCAN*Horizon_SCAN, false);
        segMsg.segmentedCloudColInd.assign(N_SCAN*Horizon_SCAN, 0);
        segMsg.segmentedCloudRange.assign(N_SCAN*Horizon_SCAN, 0);

        //segmentation process를 위해 구역 나누는듯?
        std::pair<int8_t, int8_t> neighbor;
        neighbor.first = -1; neighbor.second =  0; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second =  1; neighborIterator.push_back(neighbor);
        neighbor.first =  0; neighbor.second = -1; neighborIterator.push_back(neighbor);
        neighbor.first =  1; neighbor.second =  0; neighborIterator.push_back(neighbor);

        // array for tracking points of a segmented object
        allPushedIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        allPushedIndY = new uint16_t[N_SCAN*Horizon_SCAN];

        // array for breadth(폭)-first search process of segmentation, for speed
        queueIndX = new uint16_t[N_SCAN*Horizon_SCAN];
        queueIndY = new uint16_t[N_SCAN*Horizon_SCAN];
    }

    void resetParameters(){
        //cloud point reset
        laserCloudIn->clear();
        groundCloud->clear();
        segmentedCloud->clear();
        segmentedCloudPure->clear();
        outlierCloud->clear();
        //추가
        groundCloudIntensity->clear();
        laserCloudIn1->clear();

        //Matrix초기화
        //CV_32F(32bit floating-point number)
        //CV_8S(8bit signed integer)
        //cv::scalar::all(0) 모든 matrix 0으로 초기화
        //FLT_MAX floating 최대 가ㅄ
        rangeMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32F, cv::Scalar::all(FLT_MAX));
        groundMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_8S, cv::Scalar::all(0));
        labelMat = cv::Mat(N_SCAN, Horizon_SCAN, CV_32S, cv::Scalar::all(0));
        labelCount = 1;
        //matrix 확인
        //cout << "rangeMat = " << endl << " " << groundMat << endl << endl;

        //fullCloud, fullInfoCloud의 포인트들을 nan으로 다 채운다.
        std::fill(fullCloud->points.begin(), fullCloud->points.end(), nanPoint);
        std::fill(fullInfoCloud->points.begin(), fullInfoCloud->points.end(), nanPoint);
        std::fill(intensityCloud->points.begin(), intensityCloud->points.end(), nanPoint);

    }
    //reset 끝

    ~ImageProjection(){}

    void boxCallback(const jsk_recognition_msgs::BoundingBoxArray::ConstPtr& BoundingBox){
        float temp = 1.2;
        //ROS_INFO("boxCallback start %d", BoundingBox->header.seq);
        boxSize = BoundingBox->boxes.size();
        // double ro, pi, ya, ya_deg;
        // float x, y, z, w;
        //ROS_INFO("box size %d", boxSize);

        for(int i=0 ; i < boxSize ; i++){
            Box = BoundingBox->boxes[i];

            l = Box.dimensions.x;
            w = Box.dimensions.y;
            h = Box.dimensions.z;

            tx = Box.pose.position.x;
            ty = Box.pose.position.y;
            tz = (Box.pose.position.z)*2;

            // tf::Quaternion orientation;
            // tf::quaternionMsgToTF(Box.pose.orientation, orientation);
            // tf::Matrix3x3(orientation).getRPY(ro, pi, ya);

            // ya_deg = ya * 180.0 / M_PI;
            // ROS_INFO("%lf", ya_deg);

            XM[i] = tx + temp*l;
            Xm[i] = tx - temp*l;
            
            YM[i] = ty + temp*w;
            Ym[i] = ty - temp*w;

            ZM[i] = tz + temp*h;
            Zm[i] = tz - 10*h;
        }
        //ROS_INFO("boxCallback end %d", BoundingBox->header.seq);

    }
    
    
    //velodyne point들을 받아서 함수 실행
    void copyPointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){

        cloudHeader = laserCloudMsg->header;
        // //cloudHeader.stamp = ros::Time::now(); // Ouster lidar users may need to uncomment this line
        //lasercloudmsg(sensor msgs를) -> lasercloudin(pcl msgs로), 즉, data 택배 포장지 뜯기
        pcl::fromROSMsg(*laserCloudMsg, *laserCloudIn1);
        //추가
        pcl::fromROSMsg(*laserCloudMsg, *groundCloudIntensity);
        // Remove Nan points
        //pcl::removeNaNFromPointCloud(input point cloud, output point cloud, vector)
        //revomes points with x,y,z equal to NaN.
        //ROS_INFO("111 %zu", laserCloudIn1->points.size());
        //pcl::removeNaNFromPointCloud(*laserCloudIn1, *laserCloudIn, indices);
        //추가
        size_t cloudSize; 
        cloudSize = laserCloudIn1->points.size();
        PointType thisPoint;
        //ROS_INFO("middle %d", laserCloudIn1->header.seq);
        //ROS_INFO("boxSize %zu", boxSize);
        for (size_t i = 0; i < cloudSize; ++i){
            thisPoint.x = laserCloudIn1->points[i].x;
            thisPoint.y = laserCloudIn1->points[i].y;
            thisPoint.z = laserCloudIn1->points[i].z;
            for(int j=0 ; j < boxSize ; j++){
                if(Xm[j] < thisPoint.x && thisPoint.x < XM[j]){
                    if(Ym[j] < thisPoint.y && thisPoint.y < YM[j]){
                        laserCloudIn1->points[i] = nanPoint;
                        }
                }
            }
        }

        std::vector<int> indices;
        pcl::removeNaNFromPointCloud(*laserCloudIn1, *laserCloudIn, indices);

        //추가
        std::vector<int> indices1;
        pcl::removeNaNFromPointCloud(*groundCloudIntensity, *groundCloudIntensity, indices1);
        
        // have "ring" channel in the cloud
        //extern const bool useCloudRing = true;
        if (useCloudRing == true){
            //velodyne point들을 laserCloudInRing data로 택배 포장 뜯기
            pcl::fromROSMsg(*laserCloudMsg, *laserCloudInRing);
            if (laserCloudInRing->is_dense == false) {//없으면 error 띄우기
                ROS_ERROR("Point cloud is not in dense format, please remove NaN points first!");
                ros::shutdown();
            }  
        }
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg){//velodyne point들 받기
        //ROS_INFO("cloudHander start %d", laserCloudMsg->header.seq);
        // 1. Convert ros message to pcl point cloud
        copyPointCloud(laserCloudMsg);
        // 2. Start and end angle of a scan
        findStartEndAngle();
        // 3. Range image projection
        projectPointCloud();
        // 4. Mark ground points
        groundRemoval();
        // 5. Point cloud segmentation
        cloudSegmentation();
        // 6. Publish all clouds
        publishCloud();
        // 7. Reset parameters for next iteration
        resetParameters();
        //ROS_INFO("cloudHander end %d", laserCloudMsg->header.seq);
    }

    void findStartEndAngle(){
        // start and end orientation of this cloud
        //cloud_msgs::cloud_info segMsg; // info of segmented cloud
        segMsg.startOrientation = -atan2(laserCloudIn->points[0].y, laserCloudIn->points[0].x);
        segMsg.endOrientation   = -atan2(laserCloudIn->points[laserCloudIn->points.size() - 1].y,
                                                     laserCloudIn->points[laserCloudIn->points.size() - 1].x) + 2 * M_PI;//M_PI is pi
        
        if (segMsg.endOrientation - segMsg.startOrientation > 3 * M_PI) {
            segMsg.endOrientation -= 2 * M_PI;
        } else if (segMsg.endOrientation - segMsg.startOrientation < M_PI)
            segMsg.endOrientation += 2 * M_PI;
        // printf("2. startOrientation=%f, endOrientation=%f\n", segMsg.startOrientation* 180.0 / M_PI, segMsg.endOrientation* 180.0 / M_PI);
        segMsg.orientationDiff = segMsg.endOrientation - segMsg.startOrientation;
        // printf("orientationDiff=%f \n\n", (segMsg.endOrientation-segMsg.startOrientation)* 180.0 / M_PI);
        //segMsg.orientationDiff는 6.58~6.59(377도)로 일정, if문에서 맞춰줌(?)
        
    }

    void projectPointCloud(){
        // range image projection
        // intensity를 구하고 XYZI 정보를 fullcloud에 넣는 함수 + rangeMat에 포인트별 거리 넣는 함수
        float verticalAngle, horizonAngle, range;
        size_t rowIdn, columnIdn, index, cloudSize; 
        PointType thisPoint, thisPoint2;//pcl::pointXYZI

        cloudSize = laserCloudIn->points.size();
        
        for (size_t i = 0; i < cloudSize; ++i){
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            // groundCloudIntensity->points[i].intensity = (groundCloudIntensity->points[i].intensity)*255;
            // find the row and column index in the iamge for this point
            if (useCloudRing == true){
                rowIdn = laserCloudInRing->points[i].ring;
            }
            else{
                //radian이라 *180/pi 해주는 것 결국 degree로 변환
                verticalAngle = atan2(thisPoint.z, sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y)) * 180 / M_PI;
                //rowIdn 구하기, 채널의 index 구하는듯.
                //extern const float ang_res_x = 0.2;
                //extern const float ang_res_y = 2.0;
                //extern const float ang_bottom = 15.0+0.1;
                rowIdn = (verticalAngle + ang_bottom) / ang_res_y;
            }
            //printf("rowIdn = %zu ", rowIdn);
            if (rowIdn < 0 || rowIdn >= N_SCAN)//채널 안에 들어오지 않으면 for문 끝으로
                continue;
         
            //채널 안에 들어오면 horizonangle 구하기
            horizonAngle = atan2(thisPoint.x, thisPoint.y) * 180 / M_PI;
            //columnIdn 구하기, sample의 index 뜻하는 듯
            columnIdn = -round((horizonAngle-90.0)/ang_res_x) + Horizon_SCAN/2;//round 반올림
            
            if (columnIdn >= Horizon_SCAN)//0 ~ 1800대 숫자 맞춰주는 작업
                columnIdn -= Horizon_SCAN;
            //printf("columnIdn = %zu    ", columnIdn);

            if (columnIdn < 0 || columnIdn >= Horizon_SCAN)// 샘플 1800개 범위 밖을 벗어나면 for문 끝으로
                continue;

            //원점으로부터 거리 계산
            range = sqrt(thisPoint.x * thisPoint.x + thisPoint.y * thisPoint.y + thisPoint.z * thisPoint.z);
            //printf("range = %f\n", range);

            //extern const float sensorMinimumRange = 1.0;
            //1보다 작으면 Matrix에 넣지 않음
            if (range < sensorMinimumRange)
                continue;
            //해당되는 row 와 column에 range 거리가 대입, 대입되지 않는 곳은 168번 줄에 넣었던 FLT_MAX 가ㅄ.
            rangeMat.at<float>(rowIdn, columnIdn) = range;
            
            //intensity로 00.0000을 표현하는데 정수 부분은 row, 소수 부분은 column 으로 표현
            //thispoint의 intensity자리에 대입
            thisPoint.intensity = (float)rowIdn + (float)columnIdn / 10000.0;

            index = columnIdn  + rowIdn * Horizon_SCAN;
            
            fullCloud->points[index] = thisPoint;//size = 28800개 의 index에 thispoint(x,y,z,i) 대입
            fullInfoCloud->points[index] = thisPoint;//위와 동일
            fullInfoCloud->points[index].intensity = range; // the corresponding range of a point is saved as "intensity"
                                                            // fullInfocloud의 intensity에는 range 대입
            intensityCloud->points[index] = thisPoint;
            intensityCloud->points[index].intensity = laserCloudIn->points[i].intensity;
        }
        //cout << "rangeMat = " << endl << " " << rangeMat << endl << endl;
    }

    //segmentation(ground extraction)
    void groundRemoval(){
        size_t lowerInd, upperInd;//size_t는 부호없는 64비트 정수, index가 매우 커질 때 사용
        float diffX, diffY, diffZ, angle;
        // groundMat
        // -1, no valid info to check if ground of not
        //  0, initial value, after validation, means not ground
        //  1, ground
        for (size_t j = 0; j < Horizon_SCAN; ++j){//1800동안 반복
            for (size_t i = 0; i < groundScanInd; ++i){//7동안 반복

                //7개 층들의 동일한 위, 아래 index
                lowerInd = j + ( i )*Horizon_SCAN;
                upperInd = j + (i+1)*Horizon_SCAN;

                //아래서부터 7개의 ring의 1800개 샘플을 검사하는데 intensity가 -1이면
                //ground 인지 아닌지 불명확
                if (fullCloud->points[lowerInd].intensity == -1 ||
                    fullCloud->points[upperInd].intensity == -1){
                    // no info to check, invalid points
                    groundMat.at<int8_t>(i,j) = -1;
                    continue;
                }
                //위 아래 point들의 x, y, z의 차를 구함, 0부터 7층까지 한 sample의 index 순으로 구함.
                diffX = fullCloud->points[upperInd].x - fullCloud->points[lowerInd].x;
                diffY = fullCloud->points[upperInd].y - fullCloud->points[lowerInd].y;
                diffZ = fullCloud->points[upperInd].z - fullCloud->points[lowerInd].z;
                //printf("%d층의 %zu 번째 sample\n", i, j);
                
                //diffx, diffy, diffz로 베타를 구함 z가 높을 수록 angle이 높아져 ground로 판정하지 않음
                //diffz가 높다는 것은 위층 아래층의 Z축 차이가 크다는 것, 즉 ground가 아니라 어떤 물체일 가능성
                angle = atan2(diffZ, sqrt(diffX*diffX + diffY*diffY) ) * 180 / M_PI;
                //printf("angle = %f\n\n", angle);

                //extern const float sensorMountAngle = 0.0; in utility.h
                //보통 -1~1사이 가ㅄ, 10이하는 groundMatrix에 X 
                if (abs(angle - sensorMountAngle) <= 2){//원래 10
                    groundMat.at<int8_t>(i,j) = 1;
                    groundMat.at<int8_t>(i+1,j) = 1;
                }
            }
        }
        // extract ground cloud (groundMat == 1)
        // mark entry that doesn't need to label (ground and invalid point) for segmentation
        // note that ground remove is from 0~N_SCAN-1, need rangeMat for mark label matrix for the 16th scan
        for (size_t i = 0; i < N_SCAN; ++i){//16개의 채널
            for (size_t j = 0; j < Horizon_SCAN; ++j){//1800개의 샘플 수 
                //ground matrix가 ground로 판정 나거나, range matrix에 range가ㅄ이 들어오지 않았을때 
                if (groundMat.at<int8_t>(i,j) == 1 || rangeMat.at<float>(i,j) == FLT_MAX){
                    labelMat.at<int>(i,j) = -1;//label matrix에 -1대입, reset때 모두 0으로 초기화 했었음
                }
            }
        }
        
        //pubGroundCloud.getNumSubscribers() 이 pub에 연결된 sub의 수가 0이 아니면 if문 진입
        if (pubGroundCloud.getNumSubscribers() != 0){
            for (size_t i = 0; i <= groundScanInd; ++i){
                for (size_t j = 0; j < Horizon_SCAN; ++j){
                    if (groundMat.at<int8_t>(i,j) == 1){
                        groundCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                    }
                }
            }
        }

        for (size_t i = 0; i <= groundScanInd; ++i){
            for (size_t j = 0; j < Horizon_SCAN; ++j){
                if (groundMat.at<int8_t>(i,j) == 1){
                    groundCloudIntensity->push_back(intensityCloud->points[j + i*Horizon_SCAN]);
                }
            }
        }
    }

    void cloudSegmentation(){
        // segmentation process
        for (size_t i = 0; i < N_SCAN; ++i)//16번
            for (size_t j = 0; j < Horizon_SCAN; ++j)//1800번
                if (labelMat.at<int>(i,j) == 0)//groundMatrix는 1, rangeMatrix는 FLT_MAX, 제외 포인트 들은 다 0일 것
                    labelComponents(i, j);//해당좌표의 가ㅄ이 0인 point들을 labelComponents함수로
        
        int sizeOfSegCloud = 0;
        //segmentedCloudprintf("sizeOfSegCloud = %d\n", sizeOfSegCloud);
        // extract segmented cloud for lidar odometry
        for (size_t i = 0; i < N_SCAN; ++i) {

            segMsg.startRingIndex[i] = sizeOfSegCloud-1 + 5;//cloud_msgs::cloud_info segMsg, startRingIndex는 16개
            //printf("startRingIndex[%zu] = %d\n", i, segMsg.startRingIndex[i]);
            for (size_t j = 20; j < 880; ++j) {//전방 outliercloud FOV 범위
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){//range and FLT_MAX || ground 
                    // outliers that will not be used for optimization (always continue)
                    if (labelMat.at<int>(i,j) == 999999 && groundMat.at<int8_t>(i,j) != 1){//FLT_MAX일 때
                        if (i > groundScanInd && j % 5 == 0){//ring이 7보다 위 채널이고, sample이 5의 배수 라면 //마지막 && 조건 추가
                            //j + i*Horizon_SCAN 인덱스에 들어있는 fullCloud가ㅄ을 outlierCloud에 대입
                            outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                            continue;
                        }else{//range 있을 때
                            continue;
                        }
                    }
                    // majority of ground points are skipped
                    if (groundMat.at<int8_t>(i,j) == 1){//확실한 ground Matrix
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)//중에 sample이 5이상인 5의 배수이고 1795보다 작을 때 
                            continue;
                    }
                    //if문 추가 groundmat seg에서 제외
                    if (groundMat.at<int8_t>(i,j) != 1){
                        //여기를 지나는 matrix는 range가 있는 matrix들
                        // mark ground points so they will not be considered as edge features later
                        segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                        // mark the points' column index for marking occlusion later
                        //index (i+1)-i = segment된 point 수
                        segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                        // save range info
                        segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                        // save seg cloud
                        segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        // size of seg cloud
                        ++sizeOfSegCloud;//16개층에 걸친 segmented cloud 수(?)
                    }
                }
            }
            for (size_t j = 920; j < 1780; ++j) {
                if (labelMat.at<int>(i,j) > 0 || groundMat.at<int8_t>(i,j) == 1){//range and FLT_MAX || ground 
                    // outliers that will not be used for optimization (always continue)
                    if (labelMat.at<int>(i,j) == 999999 && groundMat.at<int8_t>(i,j) != 1){//FLT_MAX일 때
                        if (i > groundScanInd && j % 5 == 0){//ring이 7보다 위 채널이고, sample이 5의 배수 라면 //마지막 && 조건 추가
                            //j + i*Horizon_SCAN 인덱스에 들어있는 fullCloud가ㅄ을 outlierCloud에 대입
                            //outlierCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                            continue;
                        }else{//range 있을 때
                            continue;
                        }
                    }
                    // majority of ground points are skipped
                    if (groundMat.at<int8_t>(i,j) == 1){//확실한 ground Matrix
                        if (j%5!=0 && j>5 && j<Horizon_SCAN-5)//중에 sample이 5이상인 5의 배수이고 1795보다 작을 때 
                            continue;
                    }
                    //if문 추가 groundmat seg에서 제외
                    if (groundMat.at<int8_t>(i,j) != 1){
                        //여기를 지나는 matrix는 range가 있는 matrix들
                        // mark ground points so they will not be considered as edge features later
                        segMsg.segmentedCloudGroundFlag[sizeOfSegCloud] = (groundMat.at<int8_t>(i,j) == 1);
                        // mark the points' column index for marking occlusion later
                        //index (i+1)-i = segment된 point 수
                        segMsg.segmentedCloudColInd[sizeOfSegCloud] = j;
                        // save range info
                        segMsg.segmentedCloudRange[sizeOfSegCloud]  = rangeMat.at<float>(i,j);
                        // save seg cloud
                        segmentedCloud->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        // size of seg cloud
                        ++sizeOfSegCloud;//16개층에 걸친 segmented cloud 수(?)
                    }
                }
            }
            //printf("sizeOfSegCloud=%d\n", sizeOfSegCloud);
            segMsg.endRingIndex[i] = sizeOfSegCloud-1 - 5;
            //printf("endRingIndex[%zu] =%d \n\n", i, segMsg.endRingIndex[i]);
        }
        
        // extract segmented cloud for visualization
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){//이 pub에 연결된 sub 수가 0 이 아니면 들어오라
            for (size_t i = 0; i < N_SCAN; ++i){//16번 반복
                for (size_t j = 0; j < Horizon_SCAN; ++j){//1800번 반복
                    if (labelMat.at<int>(i,j) > 0 && labelMat.at<int>(i,j) != 999999){//ground matrix 해당
                        segmentedCloudPure->push_back(fullCloud->points[j + i*Horizon_SCAN]);
                        segmentedCloudPure->points.back().intensity = labelMat.at<int>(i,j);
                    }
                }
            }
        }
    }

    void labelComponents(int row, int col){
        // use std::queue std::vector std::deque will slow the program down greatly
        float d1, d2, alpha, angle;
        int fromIndX, fromIndY, thisIndX, thisIndY; 
        bool lineCountFlag[N_SCAN] = {false};

        //labelMat에서 ground로 판정되지 않은 포인트 x, y를 받아서 x=row, y=col으로
        queueIndX[0] = row;
        queueIndY[0] = col;
        int queueSize = 1;
        int queueStartInd = 0;
        int queueEndInd = 1;

        allPushedIndX[0] = row;
        allPushedIndY[0] = col;
        int allPushedIndSize = 1;
        
        while(queueSize > 0){
            // Pop point
            //queueStartInd=0부터 시작이니 들어온 x,y좌표를 대입
            fromIndX = queueIndX[queueStartInd];
            fromIndY = queueIndY[queueStartInd];
            --queueSize;
            ++queueStartInd;
            // Mark popped point
            //printf("for문 들어오기 전 fromIndX=%d, fromIndY=%d\n", fromIndX, fromIndY);

            labelMat.at<int>(fromIndX, fromIndY) = labelCount;//초기세팅 labelCount=1, 즉 labelMat해당 좌표가 0->1로 바뀜
            // Loop through all the neighboring grids of popped grid
            for (auto iter = neighborIterator.begin(); iter != neighborIterator.end(); ++iter){
                //neighborIterator(-1,0) -> (0,1) -> (0, -1) -> (1, 0)순
                // new index
                thisIndX = fromIndX + (*iter).first; //들어온 x좌표 + iter(-1 -> 0 -> 0 -> 1 순)
                thisIndY = fromIndY + (*iter).second; //들어온 y좌표 + iter(0 -> 1 -> -1 -> 0 순)
                //printf("%d", (*iter).first);
                //printf(" 째 thisIndX=%d, thisIndY=%d\n", thisIndX, thisIndY);
                // index should be within the boundary
                if (thisIndX < 0 || thisIndX >= N_SCAN)
                    continue;//들어온 x좌표가 0보다 작거나 16보다 이상이면 다음 반복 시작
                // at range image margin (left or right side)
                if (thisIndY < 0)
                    thisIndY = Horizon_SCAN - 1;
                if (thisIndY >= Horizon_SCAN)
                    thisIndY = 0;
                // prevent infinite loop (caused by put already examined point back)
                if (labelMat.at<int>(thisIndX, thisIndY) != 0)
                    continue;//다음 반복 시작

                //여기까지 한점의 주변 점 4개 뽑았고 거를것 거름
                //input : fromIndX, fromIndY -> output : thisIndX, thisIndY 
                //보통 Y 1증가 or X 1증가
                //포인트의 거리가 담겨있던 rangeMat의 index비교해서 거리가 젤 먼 max, 젤 가까운 min
                d1 = std::max(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                d2 = std::min(rangeMat.at<float>(fromIndX, fromIndY), 
                              rangeMat.at<float>(thisIndX, thisIndY));
                //printf("거른 후 thisIndX=%d, thisIndY=%d\n", thisIndX, thisIndY);
                if ((*iter).first == 0)
                    alpha = segmentAlphaX;//ang_res_x(0.2)의 radian
                else
                    alpha = segmentAlphaY;//ang_res_y(2.0)의 radian

                //두 포인트로 얻어진 angle구하기 (논문의 베타)
                angle = atan2(d2*sin(alpha), (d1 -d2*cos(alpha)));
                //angle이 60도 보다 커야 물체로 인정
                //segmentTheta는 60도의 라디안
                //if문 킵 , 뭔지 모르게음
                if (angle > segmentTheta){//segmentTheta=1.04666
                    queueIndX[queueEndInd] = thisIndX;//thisIndX : 이웃점 X
                    queueIndY[queueEndInd] = thisIndY;//thisIndY : 이웃점 Y
                    ++queueSize;//while문 돌 수 있게 해주는 변수
                    ++queueEndInd;
                    
                    labelMat.at<int>(thisIndX, thisIndY) = labelCount;//뭉치는 이웃점끼리 같은 labelCound 가짐
                    //탐색한 주변 점들 중에서 angle이 큰 즉, segment된 물체들의 구분점이 된 ring의 인덱스
                    lineCountFlag[thisIndX] = true;

                    allPushedIndX[allPushedIndSize] = thisIndX;
                    allPushedIndY[allPushedIndSize] = thisIndY;
                    ++allPushedIndSize;
                                        
                }
            }
        }

        // check if this segment is valid
        bool feasibleSegment = false;
        
        if (allPushedIndSize >= 30)//30보다 적은 객체는 segment 삭제, 실제 가ㅄ)allPushedIndSize =50, 40, 31, 134, 256, 30, 503
            feasibleSegment = true;
        else if (allPushedIndSize >= segmentValidPointNum){//5<= allPushedIndSize <=30
            int lineCount = 0;
            for (size_t i = 0; i < N_SCAN; ++i)// 16번 반복
                if (lineCountFlag[i] == true)//angle(논문에서 베타)이 60도 이상이었던 ring이라면
                    ++lineCount;
            if (lineCount >= segmentValidLineNum)//angle(논문에서 베타)이 60도 이상인 ring이 3개 이상이라면
                feasibleSegment = true;            
        }
        // segment is valid, mark these points
        if (feasibleSegment == true){//segment가 됐다면
            ++labelCount;
        }else{ // segment is invalid, mark these points
            for (size_t i = 0; i < allPushedIndSize; ++i){//segment 포인트 갯수만큼 다시 FLT_MAX로 설정
                labelMat.at<int>(allPushedIndX[i], allPushedIndY[i]) = 999999;
            }
        }
    }

    
    void publishCloud(){
        // 1. Publish Seg Cloud Info
        segMsg.header = cloudHeader;
        pubSegmentedCloudInfo.publish(segMsg);
        // 2. Publish clouds
        sensor_msgs::PointCloud2 laserCloudTemp;

        pcl::toROSMsg(*outlierCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubOutlierCloud.publish(laserCloudTemp);
        // segmented cloud with ground
        pcl::toROSMsg(*segmentedCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubSegmentedCloud.publish(laserCloudTemp);
        // projected full cloud
        if (pubFullCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullCloud.publish(laserCloudTemp);
        }
        // original dense ground cloud
        if (pubGroundCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*groundCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "velo_link";
            pubGroundCloud.publish(laserCloudTemp);
        }
        //추가
        pcl::toROSMsg(*groundCloudIntensity, laserCloudTemp);
        laserCloudTemp.header.stamp = cloudHeader.stamp;
        laserCloudTemp.header.frame_id = "base_link";
        pubGroundCloudIntensity.publish(laserCloudTemp);
        
        // segmented cloud without ground
        if (pubSegmentedCloudPure.getNumSubscribers() != 0){
            pcl::toROSMsg(*segmentedCloudPure, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubSegmentedCloudPure.publish(laserCloudTemp);
        }
        // projected full cloud info
        if (pubFullInfoCloud.getNumSubscribers() != 0){
            pcl::toROSMsg(*fullInfoCloud, laserCloudTemp);
            laserCloudTemp.header.stamp = cloudHeader.stamp;
            laserCloudTemp.header.frame_id = "base_link";
            pubFullInfoCloud.publish(laserCloudTemp);
        }
    }
};




int main(int argc, char** argv){

    ros::init(argc, argv, "test_lego_loam");
    
    ImageProjection IP;
    
    ROS_INFO("\033[1;32m---->\033[0m Image Projection Started.");
    
    ros::spin();
    
    return 0;
}
