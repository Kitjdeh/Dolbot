import rclpy
import numpy as np
from rclpy.node import Node
import os
from ssafy_msgs.msg import TurtlebotStatus
from geometry_msgs.msg import Pose, PoseStamped
import cv2
import math
from sensor_msgs.msg import CompressedImage, LaserScan
from std_msgs.msg import Float32MultiArray
from ssafy_msgs.msg import BBox
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from . import utils as utils

# lidar 변환 파트
params_lidar = {
    "Range" : 90, #min & max range of lidar azimuths
    "CHANNEL" : int(1), #verticla channel of a lidar
    "localIP": "127.0.0.1",
    "localPort": 2368,
    "Block_SIZE": int(1206),
    "X": 0, # meter
    "Y": 0,
    "Z": 0.4+0.1,
    "YAW": 0, # deg
    "PITCH": 0,
    "ROLL": 0
}


params_cam = {
    "WIDTH": 320, # image width
    "HEIGHT": 240, # image height
    "FOV": 60, # Field of view
    "localIP": "127.0.0.1",
    "localPort": 1232,
    "Block_SIZE": int(65000),
    "X": 0., # meter
    "Y": 0,
    "Z":  0.9,
    "YAW": 0, # deg
    "PITCH": 0.0,
    "ROLL": 0
}


params_map = {
    "MAP_RESOLUTION": 0.05,
    "OCCUPANCY_UP": 0.02,
    "OCCUPANCY_DOWN": 0.01,
    "MAP_CENTER": (-7.0, 9.0),
    "MAP_SIZE": (17.5, 17.5),
    "MAP_FILENAME": 'test.png',
    "MAPVIS_RESIZE_SCALE": 2.0
}

def rotationMtx(yaw, pitch, roll):
    
    R_x = np.array([[1,         0,              0,                0],
                    [0,         math.cos(roll), -math.sin(roll) , 0],
                    [0,         math.sin(roll), math.cos(roll)  , 0],
                    [0,         0,              0,               1],
                    ])
                     
    R_y = np.array([[math.cos(pitch),    0,      math.sin(pitch) , 0],
                    [0,                  1,      0               , 0],
                    [-math.sin(pitch),   0,      math.cos(pitch) , 0],
                    [0,         0,              0,               1],
                    ])
    
    R_z = np.array([[math.cos(yaw),    -math.sin(yaw),    0,    0],
                    [math.sin(yaw),    math.cos(yaw),     0,    0],
                    [0,                0,                 1,    0],
                    [0,         0,              0,               1],
                    ])
                     
    R = np.matmul(R_x, np.matmul(R_y, R_z))
 
    return R

def translationMtx(x, y, z):
     
    M = np.array([[1,         0,              0,               x],
                  [0,         1,              0,               y],
                  [0,         0,              1,               z],
                  [0,         0,              0,               1],
                  ])
    
    return M



def transformMTX_lidar2cam(params_lidar, params_cam):

    lidar_yaw, lidar_pitch, lidar_roll = params_lidar['YAW'], params_lidar['PITCH'], params_lidar['ROLL']
    cam_yaw, cam_pitch, cam_roll = params_cam['YAW'], params_cam['PITCH'], params_cam['ROLL']

    lidar_pos = np.array([params_lidar["X"],params_lidar["Y"],params_lidar["Z"],1])
    cam_pos = np.array([params_cam["X"],params_cam["Y"],params_cam["Z"],1])

    RT = np.matmul(rotationMtx(lidar_yaw, lidar_pitch, lidar_roll).T, translationMtx(lidar_pos[0] - cam_pos[0],lidar_pos[1] - cam_pos[1], lidar_pos[2] - cam_pos[2]).T)
    RT = np.matmul(RT,rotationMtx(cam_yaw, cam_pitch,cam_roll))
    RT = np.matmul(RT,rotationMtx(np.deg2rad(-90),0,0))

    RT = np.matmul(RT, rotationMtx(0,0,np.deg2rad(-80)))
 
    RT = RT.T

    return RT


def project2img_mtx(params_cam):

    #로직 1. params에서 카메라의 width, height, fov를 가져와서 focal length를 계산.
    fov_radians = np.deg2rad(params_cam["FOV"])
    fc_x = params_cam["WIDTH"] / (2 * np.tan(fov_radians / 2))
    fc_y = params_cam["HEIGHT"] / (2 * np.tan(fov_radians / 2))
    

    
    #로직 2. 카메라의 파라메터로 이미지 프레임 센터를 계산.
    cx = params_cam["WIDTH"] / 2
    cy = params_cam["HEIGHT"] / 2


    #로직 3. Projection 행렬을 계산.
    R_f = np.array([[fc_x, 0, cx],
                    [0, fc_y, cy]])

    return R_f


def draw_pts_img(img, xi, yi):

    point_np = img
    xi =xi.astype(int)
    yi = yi.astype(int)

    #Left Lane
    for ctr in zip(xi, yi):
        point_np = cv2.circle(point_np, ctr, 2, (255,0,0),-1)

    return point_np

class LIDAR2CAMTransform:
    def __init__(self, params_cam, params_lidar):

        
        # 로직 1. Params에서 필요한 파라메터들과 RT 행렬, projection 행렬 등을 정의
        self.width = params_cam["WIDTH"]
        self.height = params_cam["HEIGHT"]

        self.n = float(params_cam["WIDTH"])
        self.m = float(params_cam["HEIGHT"])

        self.RT = transformMTX_lidar2cam(params_lidar, params_cam)

        self.proj_mtx = project2img_mtx(params_cam)
        self.idx = []

    def transform_lidar2cam(self, xyz_p):
        
        xyz_c = xyz_p
        
        xyz_c = np.dot(self.RT[:3,:3], xyz_c.T).T + self.RT[:3,3]
        
        
        return xyz_c

    def project_pts2img(self, xyz_c, crop=True):

        xyi=np.zeros((xyz_c.shape[0], 2))

        # Calculate normalizing plane coordinates

        xn = xyz_c[:, 0] / xyz_c[:, 2]
        yn = xyz_c[:, 1] / xyz_c[:, 2]

        # Apply projection matrix to get pixel coordinates
        xyi[:, 0] = np.round(self.proj_mtx[0, 0] * xn + self.proj_mtx[0, 1] * yn + self.proj_mtx[0, 2])
        xyi[:, 1] = np.round(self.proj_mtx[1, 0] * xn + self.proj_mtx[1, 1] * yn + self.proj_mtx[1, 2])

        # Crop points outside the image frame
        if crop:
            xyi = np.round(xyi)
            valid_indices = np.where((xyi[:, 0] >= 0) & (xyi[:, 0] < self.width) & (xyi[:, 1] >= 0) & (xyi[:, 1] < self.height))[0]

            self.idx = valid_indices

            xyi = xyi[valid_indices]

        return xyi


    def crop_pts(self, xyi):

        xyi = xyi[np.logical_and(xyi[:, 0]>=0, xyi[:, 0]<self.width), :]
        xyi = xyi[np.logical_and(xyi[:, 1]>=0, xyi[:, 1]<self.height), :]

        return xyi



# human detector 부분
#------------------------------------------------
#------------------------------------------------

def non_maximum_supression(bboxes, threshold=0.5):
   
    # 로직 1 : bounding box 크기 역순으로 sort   
    bboxes = sorted(bboxes, key=lambda detections: detections[3],
            reverse=True)
    new_bboxes=[]
    
    # 로직 2 : new_bboxes 리스트 정의 후 첫 bbox save
    new_bboxes.append(bboxes[0])
    
    # 로직 3 : 기존 bbox 리스트에 첫 bbox delete
    bboxes.pop(0)

    for _, bbox in enumerate(bboxes):

        for new_bbox in new_bboxes:

            x1_tl = bbox[0]
            x2_tl = new_bbox[0]
            x1_br = bbox[0] + bbox[2]
            x2_br = new_bbox[0] + new_bbox[2]
            y1_tl = bbox[1]
            y2_tl = new_bbox[1]
            y1_br = bbox[1] + bbox[3]
            y2_br = new_bbox[1] + new_bbox[3]
            
            """
            # 로직 4 : 두 bbox의 겹치는 영역을 구해서, 영역이 안 겹칠때 new_bbox로 save
            """
            x_overlap = max(0, min(x1_br, x2_br) - max(x1_tl, x2_tl))
            y_overlap = max(0, min(y1_br, y2_br) - max(y1_tl, y2_tl))
            overlap_area = x_overlap * y_overlap
            
            area_1 = bbox[2] * bbox[3]
            area_2 = new_bbox[2] * new_bbox[3]
            
            total_area = area_1 + area_2 - overlap_area
            overlap_area = overlap_area / float(total_area)

            if overlap_area < threshold:
                new_bboxes.append(bbox)

            

            

    return new_bboxes

    




class following(Node) :

    def __init__(self) :
        super().__init__('following')

        # 터틀봇 위치 부분 초기 세팅
        self.status_sub = self.create_subscription(TurtlebotStatus, '/turtlebot_status', self.status_callback, 10)
        self.status_msg = TurtlebotStatus()
        self.map_sub = self.create_subscription(
            OccupancyGrid, 'map', self.map_callback, 1)
        
        self.is_map = False
        self.is_grid_update = False
        self.map_msg = OccupancyGrid()
        
        # 사람 감지 파트 초기 세팅

        
        self.subs_img = self.create_subscription(
            CompressedImage,
            '/image_jpeg/compressed',
            self.img_callback,
        1)
        self.subs_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback, 10)

        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose2', 1)

        self.img_bgr = None
        
        self.timer_period = 0.03

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.bbox_pub_ = self.create_publisher(BBox, '/bbox', 1)

        self.pedes_detector = cv2.HOGDescriptor()                              
        self.pedes_detector.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.able_to_pub = True

        # 라이다 변환 부분 
        self.l2c_trans = LIDAR2CAMTransform(params_cam, params_lidar)

        self.timer_period = 0.1

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.xyz, self.R, self.intens = None, None, None
        self.x, self.y, self.h, self.w = None, None, None, None 
        self.T_r_l = np.array([[0,-1,0],[1,0,0],[0,0,1]])

        self.map_resolution = params_map["MAP_RESOLUTION"]
        self.map_size = np.array(params_map["MAP_SIZE"]) / self.map_resolution
        self.map_center = params_map["MAP_CENTER"]
        self.map = np.ones((self.map_size[0].astype(np.int), self.map_size[1].astype(np.int)))*0.5
        self.goalIdx = None

    def map_callback(self,msg):
        self.is_map = True
        self.map_msg = msg

    def grid_update(self):
        self.is_grid_update = True

        # 로직 3. 맵 데이터 행렬로 바꾸기
        print(self.map_msg.data)
        map_to_grid = np.array(self.map_msg.data)
        self.grid = np.reshape(map_to_grid, (350, 350))

    def pose_to_grid_cell(self, x, y):
        map_point_x = 0
        map_point_y = 0
        map_point_x = int((x+15.75) * 20)
        map_point_y = int((y-0.25) * 20)
        return [map_point_x, map_point_y]

    # 터틀봇 초기 위치 세팅
    def status_callback(self, msg):
        self.is_status = True
        self.status_msg = msg
        self.finalPoint = []

        self.start = [self.status_msg.twist.angular.x,self.status_msg.twist.angular.y]


    #사람 감지 파트 초기
    def img_callback(self, msg):

        np_arr = np.frombuffer(msg.data, np.uint8)

        self.img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


    def detect_human(self, img_bgr):
    
        self.bbox_msg = BBox()
    
        # 로직 2 : image grayscale conversion
        img_pre = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2GRAY)

        # 로직 3 : human detection 실행 후 bounding box 출력
        (rects_temp, _) = self.pedes_detector.detectMultiScale(img_pre, winStride=(2, 2), padding=(8, 8), scale=2)

        if len(rects_temp) != 0:

            # 로직 4 : non maximum supression으로 bounding box 정리
            rects = non_maximum_supression(rects_temp)

            for (x,y,w,h) in rects:
                # print(x,y,w,h)
                self.x = x + w//2
                self.y = y + h//2
                self.w = w
                self.h = h
                self.xStart = x
                self.xEnd = x+ w
        

            for (x,y,w,h) in rects:

                cv2.rectangle(img_bgr, (x,y),(x+w,y+h),(0,255,255), 2)


        cv2.imshow("detection result", img_bgr)        
        cv2.waitKey(1)

    def transform_point_cam2lidar(self, point_cam, params_cam, params_lidar):
        # Convert point from pixel coordinates to normalized image coordinates
        x_norm = (point_cam[0] - params_cam['WIDTH']/2) / (params_cam['WIDTH']/2)
        y_norm = (point_cam[1] - params_cam['HEIGHT']/2) / (params_cam['HEIGHT']/2)

        # Project point onto image plane
        fx = params_cam['WIDTH'] / (2 * np.tan(np.deg2rad(params_cam['FOV']) / 2))
        fy = params_cam['HEIGHT'] / (2 * np.tan(np.deg2rad(params_cam['FOV']) / 2))
        K = np.array([[fx, 0, params_cam['WIDTH']/2], [0, fy, params_cam['HEIGHT']/2], [0, 0, 1]])
        point_cam = np.array([x_norm, y_norm, 1,1])
        point_cam = np.matmul(np.linalg.inv(self.l2c_trans.RT), point_cam)

        return point_cam

    def timer_callback(self):

        if self.xyz is not None and self.img_bgr is not None:

            self.detect_human(self.img_bgr)
            xyz_p = self.xyz[np.where(self.xyz[:,0]> 0)[0]] 
            xyz_c = self.l2c_trans.transform_lidar2cam(xyz_p)
            xy_i = self.l2c_trans.project_pts2img(xyz_c)   
            self.idxList = self.l2c_trans.idx
    
            if self.x is not None:
              
                self.stt = -1
                self.end = -1
                isStart = 0
                for i in range(len(xy_i[:,0])) :
             

                    if self.xStart <= int(xy_i[i,0]) <= self.xEnd :
                        if not isStart :
                            isStart = 1
                            self.stt = i

                    elif isStart :
                        self.end = i
                        break 
                try :    
                    self.goalIdx = self.idxList[(self.stt + self.end)//2]
                    minLen = 22134567890
                    minIdx = -1
                    for i in range(self.stt, self.end) :
                        if (minLen > self.R[self.idxList[i]]) :
                            minLen = self.R[self.idxList[i]]
                            minIdx = self.idxList[i]

                        
            
                    self.finalPoint = [self.start[0] + (minLen-0.2)* np.cos(np.deg2rad(self.status_msg.twist.linear.z + minIdx)),self.start[1] + (minLen-0.2)* np.sin(np.deg2rad(self.status_msg.twist.linear.z+minIdx))]
                    goal_x = (self.finalPoint[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) /self.map_resolution
                    goal_y = (self.finalPoint[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) /self.map_resolution
                except :
                    print("벗어난 범위")
                    
                self.x = None

                return_p = PoseStamped()
                return_p.header.frame_id = 'map'
                return_p.pose.position.x = self.finalPoint[0]
                return_p.pose.position.y = self.finalPoint[1]
                return_p.pose.orientation.w =self.status_msg.twist.linear.z
                if self.is_map == True:
                    if self.is_grid_update == False:
                        self.grid_update()
                    now_cell = self.pose_to_grid_cell(self.finalPoint[0],self.finalPoint[1])
                    if self.grid[now_cell[1]][now_cell[0]] < 50:
                        self.goal_pub.publish(return_p)
 
            # 로직 8 : bbox msg 송신s
            # self.bbox_pub_.publish(self.bbox_msg)

        else:
            print("waiting for msg")
            pass


    def scan_callback(self, msg):

        pose_x = msg.range_min
        pose_y = msg.scan_time
        heading = msg.time_increment

        self.start = [pose_x, pose_y]
        self.pose = np.array([[pose_x],[pose_y],[heading]])
        self.R = np.array(msg.ranges)
        self.angles =  np.linspace(0.0, 2*np.pi,len(self.R) )

        y = self.R * np.sin(self.angles)
        x = self.R * np.cos(self.angles)
        self.laser = np.vstack((x.reshape((1,-1)), y.reshape((1,-1))))
        z = np.zeros(len(self.R))

        self.xyz = np.concatenate([
            x.reshape([-1, 1]),
            y.reshape([-1, 1]),
            z.reshape([-1, 1])
        ], axis=1)  


def main(args=None):

    rclpy.init(args=args)

    follow = following()

    rclpy.spin(follow)

if __name__ == '__main__':

    main()

