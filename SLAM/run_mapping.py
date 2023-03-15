import rclpy
from rclpy.node import Node
import ros2pkg
from geometry_msgs.msg import Twist,PoseStamped,Pose,TransformStamped
from ssafy_msgs.msg import TurtlebotStatus
from sensor_msgs.msg import Imu,LaserScan
from std_msgs.msg import Float32
from squaternion import Quaternion
from nav_msgs.msg import Odometry,Path,OccupancyGrid,MapMetaData
from math import pi,cos,sin,sqrt
import tf2_ros
import os
# import utils_map
import numpy as np
import cv2
import time

# mapping node의 전체 로직 순서
# 1. publisher, subscriber, msg 생성
# 2. mapping 클래스 생성
# 3. 맵의 resolution, 중심좌표, occupancy에 대한 threshold 등의 설정 받기
# 4. laser scan 메시지 안의 ground truth pose 받기
# 5. lidar scan 결과 수신
# 6. map 업데이트 시작
# 7. pose 값을 받아서 좌표변환 행렬로 정의
# 8. laser scan 데이터 좌표 변환
# 9. pose와 laser의 grid map index 변환
# 10. laser scan 공간을 맵에 표시
# 11. 업데이트 중인 map publish
# 12. 맵 저장

params_map = {
    "MAP_RESOLUTION": 0.05,
    "OCCUPANCY_UP": 0.02,
    "OCCUPANCY_DOWN": 0.01,
    # "MAP_CENTER": (-8.0, -4.0),
    "MAP_CENTER": (-7, 9),
    "MAP_SIZE": (17.5, 17.5),
    # "MAP_SIZE": (50, 50),
    "MAP_FILENAME": 'test.png',
    "MAPVIS_RESIZE_SCALE": 2.0
}

def xyh2mat2D(vec):
    euler_rad = vec[2] * np.pi / 180
    rot = np.array([np.cos(euler_rad), -np.sin(euler_rad), np.sin(euler_rad), np.cos(euler_rad)])
    trans = vec[0:2]

    T = np.identity(3)
    T[0:2, 0:2] = rot.reshape(2, 2)
    T[0:2, 2] = trans.reshape(-1)

    # print('vec', vec)
    # print('T', T)

    return T

def createLineIterator(P1, P2, img):
    """
    Produces and array that consists of the coordinates and intensities of each pixel in a line between two points

    Parameters:
        -P1: a numpy array that consists of the coordinate of the first point (x,y)
        -P2: a numpy array that consists of the coordinate of the second point (x,y)
        -img: the image being processed

    Returns:
        -it: a numpy array that consists of the coordinates and intensities of each pixel in the radii (shape: [numPixels, 3], row = [x,y,intensity])
    """
    # 로직 순서
    # 1. 두 점을 있는 백터의 x, y 값과 크기 계산
    # 2. 직선을 그릴 grid map의 픽셀 좌표를 넣을 numpy array 를 predifine
    # 3. 직선 방향 체크
    # 4. 수직선의 픽셀 좌표 계산
    # 5. 수평선의 픽셀 좌표 계산
    # 6. 대각선의 픽셀 좌표 계산
    # 7. 맵 바깥 픽셀 좌표 삭제

   #define local variables for readability
    imageH = img.shape[0] #height
    imageW = img.shape[1] #width
    P1Y = P1[1] #시작점 y 픽셀 좌표
    P1X = P1[0] #시작점 x 픽셀 좌표
    P2X = P2[0] #끝점 y 픽셀 좌표
    P2Y = P2[1] #끝점 x 픽셀 좌표

    #difference and absolute difference between points
    #used to calculate slope and relative location between points
    dX = P2X - P1X
    dY = P2Y - P1Y
    dXa = np.abs(dX)
    dYa = np.abs(dY)

    #predefine numpy array for output based on distance between points
    itbuffer = np.empty(shape=(np.maximum(dYa,dXa),3),dtype=np.float32)
    itbuffer.fill(np.nan)

    #Obtain coordinates along the line using a form of Bresenham's algorithm
    negY = P1Y > P2Y
    negX = P1X > P2X
    if P1X == P2X: #vertical line segment
        itbuffer[:,0] = P1X
        if negY:
            itbuffer[:,1] = np.arange(P1Y - 1,P1Y - dYa - 1,-1)
        else:
            itbuffer[:,1] = np.arange(P1Y+1,P1Y+dYa+1)
    elif P1Y == P2Y: #horizontal line segment
        itbuffer[:,1] = P1Y
        if negX:
            itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
        else:
            itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)
    else: #diagonal line segment
        steepSlope = dYa > dXa
        if steepSlope:
            slope = dX.astype(np.float32)/dY.astype(np.float32)
            if negY:
                itbuffer[:,1] = np.arange(P1Y-1,P1Y-dYa-1,-1)
            else:
                itbuffer[:,1] = np.arange(P1Y+1,P1Y+dYa+1)
            itbuffer[:,0] = (slope*(itbuffer[:,1]-P1Y)).astype(np.int) + P1X
        else:
            slope = dY.astype(np.float32)/dX.astype(np.float32)
            if negX:
                itbuffer[:,0] = np.arange(P1X-1,P1X-dXa-1,-1)
            else:
                itbuffer[:,0] = np.arange(P1X+1,P1X+dXa+1)
            itbuffer[:,1] = (slope*(itbuffer[:,0]-P1X)).astype(np.int) + P1Y

    #Remove points outside of image
    colX = itbuffer[:,0]
    colY = itbuffer[:,1]
    itbuffer = itbuffer[(colX >= 0) & (colY >=0) & (colX<imageW) & (colY<imageH)]

    #Get intensities from img ndarray
    itbuffer[:,2] = img[itbuffer[:,1].astype(np.uint),itbuffer[:,0].astype(np.uint)]

    return itbuffer

class Mapping:
    """
    Mapping Class
    """
    def __init__(self, params_map):
        self.map_resolution = params_map["MAP_RESOLUTION"]
        self.map_size = np.array(params_map["MAP_SIZE"]) / self.map_resolution
        self.map_center = params_map["MAP_CENTER"]
        # 17*17 array를 0.5로 가득 채움
        self.map = np.ones((self.map_size[0].astype(np.int), self.map_size[1].astype(np.int)))*0.5
        self.occu_up = params_map["OCCUPANCY_UP"]
        self.occu_down = params_map["OCCUPANCY_DOWN"]

        self.map_filename = params_map["MAP_FILENAME"]
        self.map_vis_resize_scale = params_map["MAPVIS_RESIZE_SCALE"]

        self.T_r_l = np.array([[0,-1,0],[1,0,0],[0,0,1]])

    def update(self, pose, laser):
        n_points = laser.shape[1]
        pose_mat = xyh2mat2D(pose)  ## 여기까지,, 
        
        pose_mat = np.matmul(pose_mat,self.T_r_l) # matmul : 행렬곱
        laser_mat = np.ones((3, n_points))
        laser_mat[:2, :] = laser

        laser_global = np.matmul(pose_mat, laser_mat) # 행렬곱을 해주는 이유?

        pose_x = (pose[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        pose_y = (pose[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution
        laser_global_x = (laser_global[0, :] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        laser_global_y =  (laser_global[1, :] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        ### Original Plot
        for i in range(laser_global.shape[1]):
            p1 = np.array([pose_x, pose_y]).reshape(-1).astype(np.int)
            p2 = np.array([laser_global_x[i], laser_global_y[i]]).astype(np.int)
        
            line_iter = createLineIterator(p1, p2, self.map)
        
            if (line_iter.shape[0] is 0):
                continue
        
            avail_x = line_iter[:, 0].astype(np.int)
            avail_y = line_iter[:, 1].astype(np.int)
        
            # Empty
            self.map[avail_y[:-1], avail_x[:-1]] = self.map[avail_y[:-1], avail_x[:-1]] + self.occu_down
        
            # Occupied
            self.map[avail_y[-1], avail_x[-1]] = self.map[avail_y[-1], avail_x[-1]] - self.occu_up

        self.show_pose_and_points(pose, laser_global)        

    def __del__(self):
        self.save_map(())
        
    def save_map(self):
        map_clone = self.map.copy()
        cv2.imwrite(self.map_filename, map_clone*255)

    def show_pose_and_points(self, pose, laser_global):
        tmp_map = self.map.astype(np.float32)
        map_bgr = cv2.cvtColor(tmp_map, cv2.COLOR_GRAY2BGR)

        pose_x = (pose[0] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        pose_y = (pose[1] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        laser_global_x = (laser_global[0, :] - self.map_center[0] + (self.map_size[0]*self.map_resolution)/2) / self.map_resolution
        laser_global_y =  (laser_global[1, :] - self.map_center[1] + (self.map_size[1]*self.map_resolution)/2) / self.map_resolution

        for i in range(laser_global.shape[1]):
            (l_x, l_y) = np.array([laser_global_x[i], laser_global_y[i]]).astype(np.int)
            center = (l_x, l_y)
            cv2.circle(map_bgr, center, 1, (0,255,0), -1)

        center = (pose_x.astype(np.int32)[0], pose_y.astype(np.int32)[0])
        
        cv2.circle(map_bgr, center, 2, (0,0,255), -1)

        map_bgr = cv2.resize(map_bgr, dsize=(0, 0), fx=self.map_vis_resize_scale, fy=self.map_vis_resize_scale)
        cv2.imshow('Sample Map', map_bgr)
        cv2.waitKey(1)



        
class Mapper(Node):

    def __init__(self):
        super().__init__('Mapper')
        
        # 로직 1 : publisher, subscriber, msg 생성
        # 첫번째 인자: 메세지타입, 두번째 인자: 토픽 이름, 세번째 인자: 콜백함수, 네번째 인자: QoS(Queue of Size) 
        # http://wiki.ros.org/nav_msgs
        
        self.subscription = self.create_subscription(LaserScan,
        '/scan',self.scan_callback,10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        
        # OccupancyGrid => (Header header, MapMetaData info, int8[] data)
        # http://docs.ros.org/en/api/std_msgs/html/msg/Header.html
        # Header => (uint32 seq, time stamp, string frame_id) => seq:연속적으로 증가하는 ID, frame_id: 데이터와 연결되는 프레임
        self.map_msg=OccupancyGrid()
        self.map_msg.header.frame_id="map"

        # params_map : 위에서 정의한 맵 정보
        # map_size : 실제 맵의 크기 = W/해상도 * H/해상도 => MAP_RESOLUTION: 해상도(한픽셀당 차지하는 크기)
        self.map_size=int(params_map["MAP_SIZE"][0]\
            /params_map["MAP_RESOLUTION"]*params_map["MAP_SIZE"][1]/params_map["MAP_RESOLUTION"])

        # MapMetaData => (time map_load_time, float32 resolution, uint32 width, uint32 height, geometry_msgs/Pose origin)
        # http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html
        m = MapMetaData()
        m.resolution = params_map["MAP_RESOLUTION"]
        m.width = int(params_map["MAP_SIZE"][0]/params_map["MAP_RESOLUTION"])
        m.height = int(params_map["MAP_SIZE"][1]/params_map["MAP_RESOLUTION"])
        
        quat = np.array([0, 0, 0, 1])  # 왜 있는거지? 아무곳에서도 안씀 
        m.origin = Pose() # Pose : 위치(position)와 방향(orientation)으로 구성된 free space에서의 pose 표현
        
        # 각 항에 -8.75를 뺀이유는 ros에서 occupancygrid의 offset이라는 데이터는 맵의 중앙에서 기준좌표계까지 거리가 아니라 맵의 우측하단에서 부터 기준좌표계까지의 거리를 의미
        m.origin.position.x = params_map["MAP_CENTER"][0]-8.75
        m.origin.position.y = params_map["MAP_CENTER"][1]-8.75
        self.map_meta_data = m

        self.map_msg.info=self.map_meta_data # pub할 OccupancyGrid.MapMetaData에 적용

        # 로직 2 : mapping 클래스 생성
        self.mapping = Mapping(params_map)


    def scan_callback(self,msg):
        
        # http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
        # LaserScan.msg : 평면 라이다에서 단일 스캔
        # pose_x=msg.range_min # 최소 범위 값
        # pose_y=msg.scan_time # 스캔 사이의 시간
        pose_x=msg.range_min # 최소 범위 값
        pose_y=msg.scan_time # 스캔 사이의 시간
        heading=msg.time_increment # 측정 사이의 시간
        Distance=np.array(msg.ranges) # 범위 데이터

        # numpy.linspace(start, stop, num=50, endpoint=True, retstep=False, dtype=None, axis=0)
        # linespace : 지정된 간격 동안 균일한 간격의 숫자 반환 
        x = Distance * np.cos(np.linspace(0, 2 * np.pi, 360))  # 2*np.pi=360
        y = Distance * np.sin(np.linspace(0, 2 * np.pi, 360))

        # reshape: 2차원 배열로 만들어라 => -1: 남은 길이로 부터 추정해서 알아서 해라 => 1차원 배열로 만든것과 동일
        # vstack: 수직으로 행렬 결합 => 0번째 행에는 x, 1번째 행에는 y
        laser = np.vstack((x.reshape((1, -1)), y.reshape((1, -1))))

        pose = np.array([[pose_x],[pose_y],[heading]])
        self.mapping.update(pose, laser)

        np_map_data=self.mapping.map.reshape(1,self.map_size) 
        list_map_data=np_map_data.tolist()
        for i in range(self.map_size):
            list_map_data[0][i]=100-int(list_map_data[0][i]*100)
            if list_map_data[0][i] >100 :
                list_map_data[0][i]=100
 
            if list_map_data[0][i] <0 :
                list_map_data[0][i]=0
  
        self.map_msg.header.stamp =rclpy.clock.Clock().now().to_msg()
        self.map_msg.data=list_map_data[0]
        self.map_pub.publish(self.map_msg)

def save_map(node,file_path):
    pkg_path =os.getcwd()
    back_folder='..'
    folder_name='map'
    file_name=file_path
    full_path=os.path.join(pkg_path,back_folder,folder_name,file_name)
    print(full_path)
    f=open(full_path,'w')
    data=''
    for pixel in node.map_msg.data :

        data+='{0} '.format(pixel)
    f.write(data) 
    f.close()

        
def main(args=None):    
    rclpy.init(args=args)
    
    try :    
        run_mapping = Mapper()
        rclpy.spin(run_mapping)
        run_mapping.destroy_node()
        rclpy.shutdown()

    except :
        save_map(run_mapping,'map.txt')
# 


if __name__ == '__main__':
    main()