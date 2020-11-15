#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import sys
import rospy
import numpy as np
import copy
import tf
import xml.etree.ElementTree as ET

from std_msgs.msg import String
from std_msgs.msg import UInt32
from std_msgs.msg import Header 
from std_msgs.msg import ColorRGBA
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import TwistStamped

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point
from tf import transformations # rotation_matrix(), concatenate_matrices()
from tf import TransformListener

from jsk_recognition_msgs.msg import BoundingBox        #sudo apt-get install ros-melodic-jsk-recognition-msgs
from jsk_recognition_msgs.msg import BoundingBoxArray
from jsk_rviz_plugins.msg import Pictogram              #sudo apt-get install ros-melodic-jsk-rviz-plugins
from jsk_rviz_plugins.msg import PictogramArray
from jsk_rviz_plugins.msg import OverlayText 

from visualization_msgs.msg import Marker, MarkerArray
from model import AB3DMOT
import std_msgs
import os.path 

import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

from kalman_filter import KalmanBoxTracker
import message_filters


TRACKLET_PATH = '/home/user/rosbag/tracklet_labels.xml'
CAR_DAE_PATH = 'file:///home/user/catkin_ws/src/kf_tracker/dae/car.dae'


#Global Variables
detectionBoxes = None
prior_trk_xyz = {}      # 속도계산을 위해 이전 frame들의 xyz를 저장하는 데이터구조
current_id_list = []
prior_path_xyz = {}     # 이동경로를 표현하기 위해 이전 frame들의 xyz를 저장하는 데이터구조
d_obj = {}


prior_history = []      # Record All Detection Info

MIN_WARNING_DIST = 15     # _m 
# oxtLinear = None

 
class MoDetect_N_Track:
    def __init__(self):
        # Subscriber
        # self.pc_sub = rospy.Subscriber("/kitti/oxts/gps/vel",TwistStamped, self.IMUcallback, queue_size=1)
        # self.imu_sub = rospy.Subscriber("/kitti/oxts/gps/vel",TwistStamped, self.IMUcallback, queue_size=1)
        self.imu_sub = message_filters.Subscriber("/kitti/oxts/gps/vel",TwistStamped)
        self.det_front_sub = message_filters.Subscriber("/detection/lidar_detector/objects_markers_front",MarkerArray)
        self.det_back_sub = message_filters.Subscriber("/detection/lidar_detector/objects_markers",MarkerArray)

        # Publisher
        self.pub_det_markerarray = rospy.Publisher('pillar_marker_array', BoundingBoxArray, queue_size=1)

        self.pub_frame_seq = rospy.Publisher('kitti_frame_seq', OverlayText, queue_size=1)
        self.pub_boxes = rospy.Publisher('kitti_box_track', BoundingBoxArray, queue_size=1)
        self.pub_pictograms = rospy.Publisher('kitti_box_pictogram_track', PictogramArray, queue_size=1)
        self.pub_selfvelo_text = rospy.Publisher('kitti_selfvelo_text_track', Marker, queue_size=1)
        self.pub_selfveloDirection = rospy.Publisher('kitti_selfvelo_direction_track', Marker, queue_size=1)
        self.pub_objs_ori = rospy.Publisher('kitti_objs_ori_track', MarkerArray, queue_size=3)
        self.pub_objs_velo = rospy.Publisher('kitti_objs_velo_track', MarkerArray, queue_size=1)
        self.pub_path = rospy.Publisher('kitti_objs_path_track', MarkerArray, queue_size=1)
        self.pub_warning_lines = rospy.Publisher('kitti_warning_lines_track', MarkerArray, queue_size=1)
        self.pub_ego_outCircle = rospy.Publisher('kitti_ego_outCircle_track', Marker, queue_size=1)
        self.pub_ego_innerCircle = rospy.Publisher('kitti_ego_innerCircle_track', Marker, queue_size=1)
        self.pub_ego_car = rospy.Publisher('kitti_ego_car_track', Marker, queue_size=1)


        # Publisher & Subscriber Wrapper
        pub_list = [self.det_front_sub, self.det_back_sub, self.imu_sub]
        # pub_list = [self.det_front_sub, self.det_back_sub]
        # pub_list = [self.det_front_sub]

        self.ts = message_filters.ApproximateTimeSynchronizer(pub_list, 1, 0.1, allow_headerless=True)
        # self.ts = message_filters.TimeSynchronizer(pub_list, 10)
        # self.ts = message_filters.Cache(self.det_front_sub, 10, allow_headerless=False)
        # self.ts = message_filters.Cache(self.det_back_sub, 10, allow_headerless=False)
        self.ts.registerCallback(self.callback)
        self.tf2 = TransformListener()              # for callback function

        # Multi-Objects tracking instance
        self.mot_tracker = AB3DMOT() 


    # def IMUcallback(self, TwistStamped):
    #     # 자기 속도 Publishing Logic    
    #     headerImu = TwistStamped.header     
    #     headerImu.frame_id="/map"

    #     # odom_mat = get_odom(self.tf2, "velo_link", "map")
    #     # origin_velo_xyz = np.array([[0.0, 0.0, 0.0]]).reshape(1,-1)
    #     # origin_points = np.array((0,3), float)

    #     # if odom_mat is not None:
    #     #     origin_points = get_transformation(odom_mat, origin_velo_xyz)
        

    #     oxtLinear = TwistStamped.twist.linear
    #     selfvelo = np.sqrt(oxtLinear.x ** 2 + oxtLinear.y ** 2 + oxtLinear.z ** 2)
    #     selfvelo = np.round_(selfvelo,1)    # m/s
    #     selfvelo = selfvelo * 3.6           # km/h
        
    #     oxtAngular = TwistStamped.twist.angular
    #     q = tf.transformations.quaternion_from_euler(oxtAngular.x, oxtAngular.y, oxtAngular.z)

    #     text_marker = Marker(
    #             type=Marker.TEXT_VIEW_FACING,
    #             id=0,
    #             lifetime=rospy.Duration(0.5),
    #             pose=Pose(Point(-7.0, 0.0, 0.0), Quaternion(0, 0, 0, 1)),
    #             scale=Vector3(1.5, 1.5, 1.5),
    #             header=headerImu,
    #             color=ColorRGBA(1.0, 1.0, 1.0, 1.0),
    #             text="{}km/h".format(selfvelo))

    #     selfvelo_scale = convert_velo2scale(selfvelo)
    #     arrow_marker = Marker(
    #             type=Marker.ARROW,
    #             id=0,
    #             lifetime=rospy.Duration(0.1),
    #             pose=Pose(Point(0.0, 0.0, 0.0), Quaternion(*q)),
    #             scale=Vector3(selfvelo_scale, 0.5, 0.5),
    #             header=headerImu,
    #             color=ColorRGBA(1.0, 0.0, 0.0, 0.8))

    #     self.pub_selfvelo_text.publish(text_marker)

        

    def callback(self, front_MarkerArray, back_MarkerArray, TwistStamped):
        # print("front",len(front_MarkerArray.markers)/4)
        # print("back",len(back_MarkerArray.markers)/4)
        # #  Concat front and back MarkerArray Messages
        add_MarkerArray = copy.deepcopy(front_MarkerArray)
        for i in range(len(back_MarkerArray.markers)):
            add_MarkerArray.markers.append(back_MarkerArray.markers[i])
        # print("add",len(add_MarkerArray.markers)/4)
        # print("done")

        if len(add_MarkerArray.markers) == 0:
            return


        header = add_MarkerArray.markers[0].header
        frame = header.seq

        boxes = BoundingBoxArray() #3D Boxes with JSK
        boxes.header = header     

        texts = PictogramArray() #Labels with JSK
        texts.header = header

        obj_ori_arrows = MarkerArray() #arrow with visualization_msgs 

        velocity_markers = MarkerArray() #text with visualization_msgs 

        obj_path_markers = MarkerArray() # passed path

        warning_line_markers = MarkerArray()

        dets = np.zeros((0,9))     # (None, 9) : 9는 사용할 3d bbox의 파라미터 개수

        obj_box_info = np.empty((0,7))
        obj_label_info = np.empty((0,2))


        # frame을 rviz에 출력
        overlayTxt = OverlayText()
        overlayTxt.left = 10
        overlayTxt.top = 10
        overlayTxt.width = 1200
        overlayTxt.height = 1200
        overlayTxt.fg_color.a = 1.0
        overlayTxt.fg_color.r = 1.0
        overlayTxt.fg_color.g = 1.0
        overlayTxt.fg_color.b = 1.0
        overlayTxt.text_size = 12
        overlayTxt.text = "Frame_seq : {}".format(frame)


        det_boxes = BoundingBoxArray() #3D Boxes with JSK
        det_boxes.header = header


        # Receive each objects info in this frame
        for object_info in add_MarkerArray.markers:
            #extract info  [ frame,type(label),tx,ty,tz,h,w,l,ry ]
            if object_info.ns == "/detection/lidar_detector/box_markers":
                tx = object_info.pose.position.x
                ty = object_info.pose.position.y
                tz = object_info.pose.position.z
                l = object_info.scale.x
                w = object_info.scale.y
                h = object_info.scale.z 
                quaternion_xyzw = [object_info.pose.orientation.x, object_info.pose.orientation.y, \
                        object_info.pose.orientation.z, object_info.pose.orientation.w]
                rz =tf.transformations.euler_from_quaternion(quaternion_xyzw)[2]
                obj_box_info = np.append(obj_box_info, [[-ty, -tz, tx-0.27, h, w, l, -rz + np.pi/2 ]], axis=0)
                

                size_det = Vector3(l, w, h) 
                det_box = BoundingBox()
                det_box.header = header
                det_box.pose.position = Point(tx, ty, tz)
                q_det_box = tf.transformations.quaternion_from_euler(0.0, 0.0, rz)     # 어쩔 수 없이 끝단에서만 90도 돌림
                det_box.pose.orientation = Quaternion(*q_det_box)
                det_box.dimensions = size_det
                det_boxes.boxes.append(det_box)

            elif object_info.ns == "/detection/lidar_detector/label_markers":
                label = object_info.text.strip()
                if label == '':
                    label='None'
                obj_label_info = np.append(obj_label_info, [[frame, label]], axis=0)

        dets = np.concatenate((obj_label_info, obj_box_info), axis=1)
        self.pub_det_markerarray.publish(det_boxes)


        del current_id_list[:]


        # All Detection Info in one Frame
        bboxinfo = dets[dets[:,0]==str(frame),2:9]              # [ tx, ty, tz, h, w, l, rz ]
        additional_info = dets[dets[:,0]==str(frame), 0:2]      # frame, labe
        reorder = [3,4,5,0,1,2,6]           # [tx,ty,tz,h,w,l,ry] -> [h,w,l,tx,ty,tz,theta]
        reorder_back = [3,4,5,0,1,2,6]      # [h,w,l,tx,ty,tz,theta] -> [tx,ty,tz,h,w,l,ry]
        reorder2velo = [2,0,1,3,4,5,6]
        bboxinfo = bboxinfo[:,reorder]      # reorder bboxinfo parameter [h,w,l,x,y,z,theta]
        bboxinfo = bboxinfo.astype(np.float64)    
        dets_all = {'dets': bboxinfo, 'info': additional_info}
        

        # ObjectTracking from Detection
        trackers = self.mot_tracker.update(dets_all)        # h,w,l,x,y,z,theta
        trackers_bbox = trackers[:,0:7]
        trackers_info = trackers[:,7:10]                    # id, frame, label
        trackers_bbox = trackers_bbox[:,reorder_back]       # reorder_back bboxinfo parameter [tx,ty,tz,h,w,l,ry]
        trackers_bbox = trackers_bbox[:,reorder2velo]       # reorder coordinate system cam to velo 
        trackers_bbox = trackers_bbox.astype(np.float64)
        trackers_bbox[:,0] = trackers_bbox[:,0]
        trackers_bbox[:,1] = trackers_bbox[:,1]*-1 
        trackers_bbox[:,2] = trackers_bbox[:,2]*-1
        trackers_bbox[:,6] = trackers_bbox[:,6]*-1

        
        # for문을 통해 각 objects들의 정보를 추출하여 사용
        for b, info in zip(trackers_bbox, trackers_info):   
            bbox = BoundingBox()
            bbox.header = header


            # parameter 뽑기     [tx,ty,tz,h,w,l,rz]
            tx_trk, ty_trk, tz_trk = float(b[0]), float(b[1]), float(b[2])  
            rz_trk = float(b[6])
            size_trk = Vector3(float(b[5]), float(b[4]), float(b[3]) )       
            obj_id = info[0]
            label_trk = info[2]
            bbox_color = colorCategory20(int(obj_id))

            odom_mat = get_odom(self.tf2, "velo_link", "map")
            xyz = np.array(b[:3]).reshape(1,-1)
            points = np.array((0,3), float)

            if odom_mat is not None:
                points = get_transformation(odom_mat,xyz)

                # 이전 x frame 까지 지나온 points들을 저장하여 반환하는 함수
                # obj_id와 bbox.label은 단지 type차이만 날뿐 같은 데이터
                # path_points_list = points_path(tx_trk, ty_trk, tz_trk, obj_id)
                path_points_list = points_path(points[0,0], points[0,1], points[0,2], obj_id)
                map_header = copy.deepcopy(header)
                map_header.frame_id="/map"
                bbox_color = colorCategory20(int(obj_id))
                path_marker = Marker(
                        type=Marker.LINE_STRIP,
                        id=int(obj_id),
                        lifetime=rospy.Duration(0.5),
                        # pose=Pose(Point(0,0,0), Quaternion(0, 0, 0, 1)),        # origin point position
                        scale=Vector3(0.1, 0.0, 0.0),                           # line width
                        header=map_header,
                        color=bbox_color)
                path_marker.points = path_points_list
                obj_path_markers.markers.append(path_marker)


            # Tracker들의 BoundingBoxArray 설정
            bbox.pose.position = Point(tx_trk, ty_trk, tz_trk/2.0)
            q_box = tf.transformations.quaternion_from_euler(0.0, 0.0, rz_trk + np.pi/2)     # 어쩔 수 없이 끝단에서만 90도 돌림
            bbox.pose.orientation = Quaternion(*q_box)
            bbox.dimensions = size_trk
            bbox.label = int(obj_id)
            boxes.boxes.append(bbox)


            picto_text = Pictogram()
            picto_text.header = header
            picto_text.mode = Pictogram.STRING_MODE
            picto_text.pose.position = Point(tx_trk, ty_trk, -tz_trk)
            # q = tf.transformations.quaternion_from_euler(0.7, 0.0, -0.7)
            picto_text.pose.orientation = Quaternion(0.0, -0.5, 0.0, 0.5)
            picto_text.size = 4
            picto_text.color = std_msgs.msg.ColorRGBA(1, 1, 1, 1)
            picto_text.character = label_trk + ' ' + str(bbox.label)
            texts.pictograms.append(picto_text)



            # GPS sensor values
            oxtLinear = TwistStamped.twist.linear



            # oxtLinear = TwistStamped.twist.linear
            # Tracker들의 속도 추정
            obj_velo,dx_t,dy_t,dz_t = obj_velocity([tx_trk,ty_trk,tz_trk], bbox.label, oxtLinear)
            if obj_velo != None:    
                obj_velo = np.round_(obj_velo,1)    # m/s
                obj_velo = obj_velo * 3.6           # km/h
            obj_velo_scale = convert_velo2scale(obj_velo)



            # # Tracker들의 Orientation
            q_ori = tf.transformations.quaternion_from_euler(0.0, 0.0, rz_trk + np.pi/2)     # 어쩔 수 없이 끝단에서만 90도 돌림
            obj_ori_arrow = Marker(
                type=Marker.ARROW,
                id=bbox.label,
                lifetime=rospy.Duration(0.2),
                pose=Pose(Point(tx_trk, ty_trk, tz_trk/2.0), Quaternion(*q_ori)),      
                scale=Vector3(obj_velo_scale, 0.5, 0.5),
                header=header,
                # color=ColorRGBA(0.0, 1.0, 0.0, 0.8))
                color=bbox_color)
            obj_ori_arrows.markers.append(obj_ori_arrow)


            obj_velo_marker = Marker(
                type=Marker.TEXT_VIEW_FACING,
                id=bbox.label,
                lifetime=rospy.Duration(0.5),
                pose=Pose(Point(tx_trk, ty_trk, tz_trk), Quaternion(0.0, -0.5, 0.0, 0.5)),
                scale=Vector3(1.5, 1.5, 1.5),
                header=header,
                color=ColorRGBA(1.0, 1.0, 1.0, 1.0),
                text="{}km/h".format(obj_velo))
            velocity_markers.markers.append(obj_velo_marker)
            current_id_list.append(bbox.label)
        


            # Warning object line
            warning_line = Marker(
                    type=Marker.LINE_LIST,
                    id=int(obj_id),
                    lifetime=rospy.Duration(0.2),
                    pose=Pose(Point(0,0,0), Quaternion(0, 0, 0, 1)),        # origin point position
                    scale=Vector3(0.2, 0.0, 0.0),                           # line width
                    header=header,
                    color=ColorRGBA(1.0, 0.0, 0.0, 1.0))

            d = dist_from_objBbox(tx_trk,ty_trk,tz_trk,size_trk.x, size_trk.y, size_trk.z)
            if d < MIN_WARNING_DIST:
                warning_line.points = Point(tx_trk,ty_trk,tz_trk), Point(0.0, 0.0, 0.0)
                warning_line_markers.markers.append(warning_line)



            # Change Outer Circle Color
            outer_circle_color = ColorRGBA(1.0*25/255, 1.0, 0.0, 1.0)
            if len(warning_line_markers.markers) > 0 :
                outer_circle_color = ColorRGBA(1.0*255/255, 1.0*0/255, 1.0*0/255, 1.0)


            # ego_vehicle's warning boundary
            outer_circle = Marker(
                type=Marker.CYLINDER,
                id=int(obj_id),
                lifetime=rospy.Duration(0.5),
                pose=Pose(Point(0.0,0.0,-2.0), Quaternion(0, 0, 0, 1)),
                scale=Vector3(8.0, 8.0, 0.1),                           # line width
                header=header,
                color=outer_circle_color
            )


            inner_circle = Marker(
                type=Marker.CYLINDER,
                id=int(obj_id),
                lifetime=rospy.Duration(0.5),
                pose=Pose(Point(0.0,0.0,-1.8), Quaternion(0, 0, 0, 1)),
                scale=Vector3(7.0, 7.0, 0.2),                           # line width
                header=header,
                color=ColorRGBA(0.22, 0.22, 0.22, 1.0)
            )



        # ego-vehicle velocity
        selfvelo = np.sqrt(oxtLinear.x ** 2 + oxtLinear.y ** 2 + oxtLinear.z ** 2)
        selfvelo = np.round_(selfvelo,1)    # m/s
        selfvelo = selfvelo * 3.6           # km/h
        oxtAngular = TwistStamped.twist.angular
        q_gps = tf.transformations.quaternion_from_euler(oxtAngular.x, oxtAngular.y, oxtAngular.z)


        # # ego-vehicle 사진 출력
        ego_car = Marker(
            type=Marker.MESH_RESOURCE,
            id=0,
            lifetime=rospy.Duration(0.5),
            pose=Pose(Point(0.0, 0.0, -1.8), Quaternion(0,0,0,1)),
            scale=Vector3(1.5, 1.5, 1.5),
            header=header,
            action=Marker.ADD,
            mesh_resource=CAR_DAE_PATH,
            color=ColorRGBA(1.0, 1.0, 1.0, 1.0)
        )


        # Self ego Velocity
        text_marker = Marker(
            type=Marker.TEXT_VIEW_FACING,
            id=0,
            lifetime=rospy.Duration(0.5),
            pose=Pose(Point(-7.0, 0.0, 0.0), Quaternion(0, 0, 0, 1)),
            scale=Vector3(1.5, 1.5, 1.5),
            header=header,
            color=ColorRGBA(1.0, 1.0, 1.0, 1.0),
            text="{}km/h".format(selfvelo))  


        
        for i in prior_trk_xyz.keys():
            if i not in current_id_list:
                prior_trk_xyz.pop(i)


        self.pub_frame_seq.publish(overlayTxt)
        self.pub_boxes.publish(boxes)
        self.pub_pictograms.publish(texts)
        self.pub_selfvelo_text.publish(text_marker)
        # self.pub_selfveloDirection.publish(arrow_marker)
        self.pub_objs_ori.publish(obj_ori_arrows)
        self.pub_objs_velo.publish(velocity_markers)
        self.pub_path.publish(obj_path_markers)
        self.pub_warning_lines.publish(warning_line_markers)
        self.pub_ego_outCircle.publish(outer_circle)
        self.pub_ego_innerCircle.publish(inner_circle)
        self.pub_ego_car.publish(ego_car)





def get_odom(tf2,frame, dest_frame):
    try:
        t = tf2.getLatestCommonTime(dest_frame, frame)
        position, quaternion = tf2.lookupTransform(dest_frame, frame, t)

        trans_mat = tf.transformations.translation_matrix(position)
        rot_mat = tf.transformations.quaternion_matrix(quaternion)
        # create a 4x4 matrix
        mat = np.dot(trans_mat, rot_mat)
    except(tf.Exception, tf.ConnectivityException, tf.LookupException):
        print("No odometry Exept")
        return
    return mat
  


def get_transformation(odom_mat,points):
    odom_mat = odom_mat[:3, :]
    num_pts = points.transpose().shape[1]
    points = np.vstack((points.transpose(), np.ones((1, num_pts))))
    points = np.matmul(odom_mat, points)
    points = points.transpose()
    return points



def convert_velo2scale(velo):
    '''
    입력속도 : km/h
    입력받은 속도를 화살표 마커의 길이로 변환
    최소길이 : 0.0 (0km/h)/ 최대길이 : 10.0 (100km/h)
    ret : km/h
    '''
    scale_len = None

    if velo != None:
        if velo > 100:
            scale_len = 15.0
        elif velo < 0:
            scale_len = 0.0
        else:
            scale_len = 15 * velo/100
    # else:
    #     scale_len = 0

    return scale_len



def points_path(tx,ty,tz,trk_id):
    '''
    input : x,y,z좌표, tracker_boundingbox id
    output : 이전 _frame에서의 x,y,z 좌표를 전역변수에 저장하여 리스트로 출력
    전역변수 prior_path_xyz : tracker_boundingbox id를 key로 가지는 Dictionary
    '''
    if prior_path_xyz.has_key(trk_id):
        prior_path_xyz[trk_id].append(Point(tx,ty,tz))

        if len(prior_path_xyz[trk_id]) > 10:
            prior_path_xyz[trk_id].pop(0)
        # ret = prior_path_xyz[trk_id]
    else:
        prior_path_xyz[trk_id] = [Point(tx,ty,tz)]
        
    return prior_path_xyz[trk_id]



def obj_velocity(trk_xyz_list, trk_id, oxtLinear):

    obj_velo = None
    dx_t,dy_t,dz_t = None,None,None
    
    if prior_trk_xyz.has_key(trk_id):
        # 계산
        tx,ty,tz = trk_xyz_list                  # 현재 좌표
        x,y,z = prior_trk_xyz[trk_id]            # 이전 좌표
        dx,dy,dz = x - tx, y - ty, z - tz
        dx_t = dx/0.1
        dy_t = dy/0.1
        dz_t = dz/0.1
        vx = oxtLinear.x - dx/0.1
        vy = oxtLinear.y - dy/0.1
        vz = oxtLinear.z - dz/0.1
        obj_velo = np.sqrt(vx ** 2 + vy ** 2)
        prior_trk_xyz[trk_id] = trk_xyz_list
    else:
        prior_trk_xyz[trk_id] = trk_xyz_list

    return obj_velo, dx_t, dy_t, dz_t
    


# Rviz에서 자동으로 색깔을 지정하는 함수를 직접 구현
def colorCategory20(obj_id):
    c = ColorRGBA()
    c.a = 1.0

    if (obj_id % 20)==0:
        c.r = 0.121569
        c.g = 0.466667
        c.b = 0.705882
    elif (obj_id % 20)==1:
        c.r = 0.682353
        c.g = 0.780392
        c.b = 0.909804  
    elif (obj_id % 20)==2:
        c.r = 1.000000
        c.g = 0.498039
        c.b = 0.054902  
    elif (obj_id % 20)==3:
        c.r = 1.000000
        c.g = 0.733333
        c.b = 0.470588  
    elif (obj_id % 20)==4:
        c.r = 0.172549
        c.g = 0.627451
        c.b = 0.172549
    elif (obj_id % 20)==5: 
        c.r = 0.596078
        c.g = 0.874510
        c.b = 0.541176 
    elif (obj_id % 20)==6:
        c.r = 0.839216
        c.g = 0.152941
        c.b = 0.156863  
    elif (obj_id % 20)==7:  
        c.r = 1.000000
        c.g = 0.596078
        c.b = 0.588235  
    elif (obj_id % 20)==8:  
        c.r = 0.580392
        c.g = 0.403922
        c.b = 0.741176
    elif (obj_id % 20)==9:  
        c.r = 0.772549
        c.g = 0.690196
        c.b = 0.835294
    elif (obj_id % 20)==10: 
        c.r = 0.549020
        c.g = 0.337255
        c.b = 0.294118 
    elif (obj_id % 20)==11: 
        c.r = 0.768627
        c.g = 0.611765
        c.b = 0.580392  
    elif (obj_id % 20)==12:  
        c.r = 0.890196
        c.g = 0.466667
        c.b = 0.760784 
    elif (obj_id % 20)==13: 
        c.r = 0.968627
        c.g = 0.713725
        c.b = 0.823529 
    elif (obj_id % 20)==14: 
        c.r = 0.498039
        c.g = 0.498039
        c.b = 0.498039
    elif (obj_id % 20)==15: 
        c.r = 0.780392
        c.g = 0.780392
        c.b = 0.780392
    elif (obj_id % 20)==16:  
        c.r = 0.737255
        c.g = 0.741176
        c.b = 0.133333
    elif (obj_id % 20)==17: 
        c.r = 0.858824
        c.g = 0.858824
        c.b = 0.552941  
    elif (obj_id % 20)==18:   
        c.r = 0.090196
        c.g = 0.745098
        c.b = 0.811765
    elif (obj_id % 20)==19:
        c.r = 0.619608
        c.g = 0.854902
        c.b = 0.898039

    return c



# 원점과 / object들의 BoundingBox의 꼭지점을 둘러싸는 원과의 거리를 반환
def dist_from_objBbox(tx, ty, tz, h, w, l):
    dist2orgin = np.sqrt(tx**2 + ty**2 + tz**2)
    dist2edge = np.sqrt((h/2)**2 + (w/2)**2 + (w/2)**2)
    dist2orgin - dist2edge

    return dist2orgin - dist2edge



# Read Detection info from tracklet.xml
def readXML(file):
    '''
    tracklet.xml 파일로 부터 Object들의 Bounding Box정보들을 읽어와 
    np.array 타입으로 리턴한다.
    - det_all의 index순서 :  frame,type(label),tx,ty,tz,h,w,l,rz
    '''
    tree = ET.parse(file)
    root = tree.getroot()
    
    item = root.findall('./tracklets/item')

    det_all = np.zeros((0,9))     # (None, 9) : 9는 사용할 3d bbox의 파라미터 개수

    for i, v in enumerate(item):
        h = float(v.find('h').text)
        w = float(v.find('w').text)
        l = float(v.find('l').text)
        frame = int(v.find('first_frame').text)
        label = v.find('objectType').text
        pose = v.findall('./poses/item')
        for j, p in enumerate(pose):
            tx = float(p.find('tx').text)
            ty = float(p.find('ty').text)
            tz = float(p.find('tz').text)
            rz = float(p.find('rz').text)
            
            # frame,type(label),h,w,l,tx,ty,tz,rz  9개 파라미터 저장
            # convert coordinate from velodyne to left-cam
            # det_all = np.append(det_all,[[frame+j,label,tx,ty,tz,h,w,l,rz]], axis=0)
            # det_all = np.append(det_all,[[frame+j,label,-ty,-tz,tx-0.27,h,w,l,-rz - (np.pi/2)]], axis=0) 
            det_all = np.append(det_all,[[frame+j,label,-ty,-tz,tx-0.27,h,w,l,-rz + np.pi/2]], axis=0) 
    return det_all     



def main(args):
    global detectionBoxes

    #Initializes and cleanup ros node with node name
    rospy.init_node('kf_tracker_node', anonymous=True)

    pcl_obj = MoDetect_N_Track() 

    print("\nkf_tracker_node start")

    # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"

if __name__ == '__main__':
    main(sys.argv)
