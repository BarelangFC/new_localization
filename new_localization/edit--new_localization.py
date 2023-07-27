import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes
# from geometry_msgs.msg import Point
import cv2 as cv
import numpy as np

class Localization(Node):
    def __init__(self):
        super().__init__('localization')
        # self.publisher = self.create_publisher(Point, 'camera_position', 1)
        self.subscriber_darknet = self.create_subscription(BoundingBoxes, 'darknet_ros/bounding_boxes', self.callbackBoundingBox, 1)
        self.subscriber_ballDis = self.create_subscription(Float32, 'ball_distance', self.callbackBallDistance, 1)
        self.publish_poseRobot = self.create_publisher(Odometry, "camera_odom", 1)
        self.timer = self.create_timer(0.1, self.timer_callback)

        
        self.Ball_X = None
        self.Ball_Y = None
        self.ballDis = 0
        self.bX = [0, 0]
        self.bY = [0, 0]
        self.bw = [0, 0]
        self.bH = [0, 0]

        self.tX = [0, 0]
        self.tY = [0, 0]
        self.tw = [0, 0]
        self.tH = [0, 0]
        self.A = 0 
        self.B = 0
        self.R_BX = self.R_BY = self.L_BX = self.L_BY = self.R_TX = self.R_TY = self.L_TX = self.L_TY = self.R_CX = self.R_CY = self.L_CX = self.L_CY = -1
        self.last_R_CY = self.last_L_CY = 0
        self.last_L_BX = self.last_R_BX = 0
        self.update = False
        self.check_update = 0
        
        # self.Ball_X = self.Ball_Y = self.Pinalty_X = self.Pinalty_Y = self.Left_X_Cross_X = self.Left_X_Cross_Y = self.Right_X_Cross_X = self.Right_X_Cross_Y = -1
        # self.Left_T_Cross_X = self.Left_T_Cross_Y = self.Right_T_Cross_X = self.Right_T_Cross_Y = self.Left_Corner_X = self.Left_Corner_Y = -1
        # self.Pinalty_X = self.Pinalty_Y = self.Right_Corner_X = self.Right_Corner_Y = self.Robot_X = self.Robot_Y = self.goal_L_pole_X = self.goal_L_pole_Y = -1
        # self.goal_R_pole_X = self.goal_R_pole_Y = -1
        #self.Bottom_Pole_X = self.Bottom_Pole_Y = self.Top_Pole_X = self.Top_Pole_Y = self.Pinalty_X = self.Pinalty_Y = self.X_Cross_X =self.X_Cross_Y = -1
        npzfile = np.load('/home/barelang5/bfc_ros2/src/new_localization/new_localization/camera_parameters.npz')

        self.mtx = npzfile['mtx']
        self.dist = npzfile['dist']

    def callbackBoundingBox(self, msg):
        # self.Ball_X = self.Ball_Y = self.Pinalty_X = self.Pinalty_Y = self.Left_X_Cross_X = self.Left_X_Cross_Y = self.Right_X_Cross_X = self.Right_X_Cross_Y = -1
        # self.Left_T_Cross_X = self.Left_T_Cross_Y = self.Right_T_Cross_X = self.Right_T_Cross_Y = self.Left_Corner_X = self.Left_Corner_Y = -1
        # self.Pinalty_X = self.Pinalty_Y = self.Right_Corner_X = self.Right_Corner_Y = self.Robot_X = self.Robot_Y = self.goal_L_pole_X = self.goal_L_pole_Y = -1
        # self.goal_R_pole_X = self.goal_R_pole_Y = -1
        #self.Bottom_Pole_X = self.Bottom_Pole_Y = self.Top_Pole_X = self.Top_Pole_Y = self.Pinalty_X = self.Pinalty_Y = self.X_Cross_X =self.X_Cross_Y = -1
        
        for bbox in msg.bounding_boxes:
            id_class = bbox.id
            x_center = (bbox.xmin + bbox.xmax) / 2
            y_center = (bbox.ymin + bbox.ymax) / 2
          
            if id_class == 2:
                if self.A == 0:
                    self.bX[0] = x_center
                    self.bY[0] = y_center
                    # print("Bx0 = \n",self.bX[0])
                    # print("By0 = \n",self.bY[0])
                    self.A = self.A + 1
                else :
                    self.bX[1] = x_center
                    self.bY[1] = y_center
                    # print("Bx1 = \n",self.bX[1])
                    # print("By1 = \n",self.bY[1])
                continue
           
            elif id_class == 6:
                if self.B == 0:
                    self.tX[0] = x_center
                    self.tY[0] = y_center
                    # print("Tx0 = \n",self.tX[0])
                    # print("Ty0 = \n",self.tY[0])
                    self.B = self.B + 1
                else :
                    self.tX[1] = x_center
                    self.tY[1] = y_center
                    # print("Tx1 = \n",self.tX[1])
                    # print("Ty1 = \n",self.tY[1])
                continue

            # add additional conditions for each class
            else:
                pass # ignore any other classes

            if self.bX[0] > self.bX[1]:
                self.R_BX = self.bX[0]
                self.R_BY = self.bY[0]
                self.L_BX = self.bX[1]
                self.L_BY = self.bY[1]
            elif self.bX[1] > self.bX[0]:
                self.R_BX = self.bX[1]
                self.R_BY = self.bY[1]
                self.L_BX = self.bX[0]
                self.L_BY = self.bY[0]
            else :
                self.R_BX = self.R_BY = self.L_BX = self.L_BY = -1
        
            # if self.tX[0] > self.tX[1]:
            #     self.R_TX = self.tX[0]
            #     self.R_TY = self.tY[0]
            #     self.L_TX = self.tX[1]
            #     self.L_TY = self.tY[1]
            # elif self.tX[1] > self.tX[0]:
            #     self.R_TX = self.tX[1]
            #     self.R_TY = self.tY[1]
            #     self.L_TX = self.tX[0]
            #     self.L_TY = self.tY[0]
            # else :
            #     self.R_TX = self.R_TY = self.L_TX = self.L_TY = -1
        

            # print("R_BX =  ",self.R_BX,"R_BY =  ",self.R_BY,"L_BX =  ",self.L_BX,"L_BY =  ",self.L_BY)
            # print("R_TX =  ",self.R_TX,"R_TY =  ",self.R_TY,"L_TX =  ",self.L_TX,"L_TY =  ",self.L_TY)

            self.M_CX = (self.R_BX + self.L_BX) / 2
            self.M_CY = (self.R_BY + self.L_BY) / 2

            self._1_BX = self.R_BX
            self._2_BX = self.L_BX
            self._3_BX = (self.R_BX + self.M_CX) / 2
            self._4_BX = (self.L_BX + self.M_CX) / 2
            self._5_BX = (self._3_BX + self.M_CX) / 2
            self._6_BX = (self._4_BX + self.M_CX) / 2

            self._1_BY = self._2_BY = self._3_BY = self._4_BY = self._5_BY = self._6_BY = self.M_CY

            # print("R_CX =  ",self.R_CX,"R_CY =  ",self.R_CY,"L_CX =  ",self.L_CX,"L_CY =  ",self.L_CY)
            

    def callbackBallDistance(self, msg):
        self.ballDis = msg.data

    # def CallbackCameraPose(self, msg):
    #     self.pose_X = msg.pose.pose.position.x
    #     self.pose_Y = msg.pose.pose.position.y

    def timer_callback(self):
        # print("Bx0 = \n",self.bX[0])
        # print("By0 = \n",self.bY[0])
        # print("Bx1 = \n",self.bX[1])
        # print("By1 = \n",self.bY[1])
        # print("Tx0 = \n",self.tX[0])
        # print("Ty0 = \n",self.tY[0])
        # print("Tx1 = \n",self.tX[1])
        # print("Ty1 = \n",self.tY[1])
        msg_robot_pose = Odometry()
        if(self.R_BX != -1 and self.R_BY != -1 and self.L_BX != -1 and self.L_BY != -1):
            if (self.check_update > 2):
                self.update = True
            else:
                self.check_update = self.check_update + 1
        else:
            self.update = False
            self.check_update = 0
        if self.update:    
            object_points = np.array([[1300, 850, 4500],
                                      [-1300, 850, 4500],
                                      [650, 850, 4500],
                                      [-650, 850, 4500],
                                      [325, 850, 4500],
                                      [-325, 850, 4500]], dtype=np.float32)
            image_points = np.array([[self._1_BX,self.M_CY],
                                    [self._2_BX,self.M_CY],
                                    [self._3_BX,self.M_CY],
                                    [self._4_BX,self.M_CY],
                                    [self._5_BX,self.M_CY],
                                    [self._6_BX,self.M_CY]], dtype=np.float32)
            _, rvecs, tvecs = cv.solvePnP(object_points, image_points, self.mtx, self.dist)

            # Convert rotation vector to rotation matrix
            rotation_matrix, _ = cv.Rodrigues(rvecs)

            # Inverse rotation matrix and translation vector to get camera pose in world coordinate
            camera_pose_world = -np.dot(rotation_matrix.T, tvecs)

            # Print camera pose in world coordinate
            # print("Camera Pose (World Coordinate):")
            # print(camera_pose_world.flatten())
            # mm to cm
            xx = camera_pose_world.flatten()[2] / 10
            yy = camera_pose_world.flatten()[0] / 10
            zz = camera_pose_world.flatten()[1] / 10    
            print("x =", xx, "y = ", yy, "z =", zz)
            msg_robot_pose.pose.pose.position.x = float(xx)
            msg_robot_pose.pose.pose.position.y = float(yy)
            msg_robot_pose.pose.pose.position.z = 0.0
            # print("update")
             # Publish the point
            # point = Point()
            # point.x = xx
            # point.y = yy
            # point.z = zz
            # self.publisher_.publish(point)
            self.publish_poseRobot.publish(msg_robot_pose)
        # else :
        #     msg_robot_pose.pose.pose.position.x = -1.0
        #     msg_robot_pose.pose.pose.position.y = -1.0
        #     msg_robot_pose.pose.pose.position.z = 0.0
        
        self.R_BX = self.R_BY = self.L_BX = self.L_BY = self.R_TX = self.R_TY = self.L_TX = self.L_TY = self.R_CX = self.R_CY = self.L_CX = self.L_CY = -1
        self.bX[0] = self.bX[1] = self.tX[0] = self.tX[1] = self.bY[0] = self.bY[1] = self.tY[0] = self.tY[1]  = -1
        self.A = 0
        self.B = 0
        # Koordinat landmark yang terdeteksi (x,y,z) dalam world coordinate
        # world_points = np.array([[0, 0, 0], [0, 0, 1], [0, 1, 0], [1, 0, 0], [1, 1, 0]])

        # # Koordinat landmark yang terdeteksi di frame kamera (x,y)
        # img_points = np.array([[427, 305], [414, 199], [512, 297], [624, 226], [537, 156]])

        # homogen_matrix, _ = cv2.findHomography(world_points, img_points)

        # # # Camera matrix hasil kalibrasi
        # # camera_matrix = np.array([[1406.08415449821, 0, 0],
        # #                         [2.20679787308599, 1417.99930662800, 0],
        # #                         [1014.13643417416, 566.347754321696, 1]])

        # # Menentukan matriks rotasi dan transalasi
        # _, rvec, tvec = cv2.solvePnP(world_points, img_points, self.mtx, self.dist)

        # # Menampilkan hasil
        # print("Homogen matrix:")
        # print(homogen_matrix)

        # # Estimasi arah hadap kamera di lapangan
        # print("Rotation matrix:")
        # print(cv2.Rodrigues(rvec)[0])

        # # Estimasi posisi kamera di lapangan
        # print("Translation vector:")
        # print(tvec)

def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
