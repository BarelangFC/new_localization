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
        self.subscriber_darknet = self.create_subscription(BoundingBoxes, 'darknet_ros/bounding_boxes', self.callback_bounding_box, 1)
        self.publish_poseRobot = self.create_publisher(Odometry, "camera_odom", 1)
        self.timer = self.create_timer(0.1, self.timer_callback)

        
        self.Ball_X = None
        self.Ball_Y = None
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
        self.R_XX = self.R_XY = self.L_XX = self.L_XY = self.M_BX = self.M_BY = self.M_XCRX = self.M_XCRY = -1
        self.R_BX = self.R_BY = self.L_BX = self.L_BY = self.R_TX = self.R_TY = self.L_TX = self.L_TY = self.R_CX = self.R_CY = self.L_CX = self.L_CY = -1
        self.last_R_CY = self.last_L_CY = 0
        self.last_L_BX = self.last_R_BX = 0
        self.update = False
        self.check_update = 0
        
        self.M_BX = self.M_BY = self.M_R_BX = self.M_R_BY = self.M_L_BX = self.M_L_BY = self.Ball_X = self.Ball_Y = self.Pinalty_X = self.Pinalty_Y = self.X_Cross_X = self.X_Cross_Y = -1
        # self.Left_T_Cross_X = self.Left_T_Cross_Y = self.Right_T_Cross_X = self.Right_T_Cross_Y = self.Left_Corner_X = self.Left_Corner_Y = -1
        # self.Pinalty_X = self.Pinalty_Y = self.Right_Corner_X = self.Right_Corner_Y = self.Robot_X = self.Robot_Y = self.goal_L_pole_X = self.goal_L_pole_Y = -1
        # self.goal_R_pole_X = self.goal_R_pole_Y = -1
        #self.Bottom_Pole_X = self.Bottom_Pole_Y = self.Top_Pole_X = self.Top_Pole_Y = self.Pinalty_X = self.Pinalty_Y = self.X_Cross_X =self.X_Cross_Y = -1
        npzfile = np.load('/home/tegra/bfc_ros2/src/new_localization/new_localization/camera_parameters.npz')

        self.mtx = npzfile['mtx']
        self.dist = npzfile['dist']

    def callback_bounding_box(self, msg):
        x_values_b = []
        y_values_b = []
        x_values_t = []
        y_values_t = []
        x_values_x = []
        y_values_x = []
        id_classes_b = []
        id_classes_t = []
        id_classes_x = []
        b_pole_1_x = b_pole_1_y = b_pole_2_x = b_pole_2_y = t_pole_1_x = t_pole_1_y = t_pole_2_x = t_pole_2_y = x_cross_1_x = x_cross_1_y = x_cross_2_x = x_cross_2_y = -1
        self.M_BX = self.M_BY = self.M_R_BX = self.M_R_BY = self.M_L_BX = self.M_L_BY = self.L_XX = self.L_XY = self.R_XX = self.R_XY = -1
        for bbox in msg.bounding_boxes:
            id_class = bbox.id
            x_center = (bbox.xmin + bbox.xmax) / 2
            y_center = (bbox.ymin + bbox.ymax) / 2

            if id_class == 0:
                Ball_X = x_center
                Ball_Y = y_center
            elif id_class == 1:
                x_values_b.append(x_center)
                y_values_b.append(y_center)
                id_classes_b.append(id_class)
            elif id_class == 2:
                Robot_X = x_center
                Robot_Y = y_center
            elif id_class == 3:
                self.Pinalty_X = x_center
                self.Pinalty_Y = y_center
            elif id_class == 4:
                x_values_x.append(x_center)
                y_values_x.append(y_center)
                id_classes_x.append(id_class)
            elif id_class == 5:
                x_values_t.append(x_center)
                y_values_t.append(y_center)
                id_classes_t.append(id_class)

        if len(id_classes_b) == 2:
            for i in range(2):
                B_pole_X = x_values_b[i]
                B_pole_Y = y_values_b[i]
                if id_classes_b[i] == 1:
                    if i == 0:
                        b_pole_1_x = x_values_b[i]
                        b_pole_1_y = y_values_b[i]
                    else:
                        b_pole_2_x = x_values_b[i]
                        b_pole_2_y = y_values_b[i]

        if len(id_classes_t) == 2:
            for i in range(2):
                T_pole_X = x_values_t[i]
                T_pole_Y = y_values_t[i]
                if id_classes_t[i] == 5:
                    if i == 0:
                        t_pole_1_x = x_values_t[i]
                        t_pole_1_y = y_values_t[i]
                    else:
                        t_pole_2_x = x_values_t[i]
                        t_pole_2_y = y_values_t[i]

        if len(id_classes_x) == 2:
            for i in range(2):
                X_Cross_X = x_values_x[i]
                X_Cross_Y = y_values_x[i]
                if id_classes_x[i] == 4:
                    if i == 0:
                        x_cross_1_x = x_values_x[i]
                        x_cross_1_y = y_values_x[i]
                    else:
                        x_cross_2_x = x_values_x[i]
                        x_cross_2_y = y_values_x[i]

        # print("Detected B_poles: (%f, %f) and (%f, %f)" % (b_pole_1_x, b_pole_1_y, b_pole_2_x, b_pole_2_y))
        # print("Detected T_poles: (%f, %f) and (%f, %f)" % (t_pole_1_x, t_pole_1_y, t_pole_2_x, t_pole_2_y))

        if t_pole_1_x > t_pole_2_x:
            self.R_TX = t_pole_1_x
            self.R_TY = t_pole_1_y
            self.L_TX = t_pole_2_x
            self.L_TY = t_pole_2_y
        elif t_pole_2_x > t_pole_1_x:
            self.R_TX = t_pole_2_x
            self.R_TY = t_pole_2_y
            self.L_TX = t_pole_1_x
            self.L_TY = t_pole_1_y       

        if b_pole_1_x > b_pole_2_x:
            self.R_BX = b_pole_1_x
            self.R_BY = b_pole_1_y
            self.L_BX = b_pole_2_x
            self.L_BY = b_pole_2_y
        elif b_pole_2_x > b_pole_1_x:
            self.R_BX = b_pole_2_x
            self.R_BY = b_pole_2_y
            self.L_BX = b_pole_1_x
            self.L_BY = b_pole_1_y     

        if x_cross_1_x > x_cross_2_x:
            self.R_XX = x_cross_1_x
            self.R_XY = x_cross_1_y
            self.L_XX = x_cross_2_x
            self.L_XY = x_cross_2_y
        elif x_cross_2_x > x_cross_1_x:
            self.R_XX = x_cross_2_x
            self.R_XY = x_cross_2_y
            self.L_XX = x_cross_1_x
            self.L_XY = x_cross_1_y

        #print(self.L_BX, self.L_BY, self.R_BX, self.R_BY, self.L_XX, self.L_XY, self.R_XX, self.R_XY)

       

        # triangle
        

        
        
        #print(self.M_BX, self.M_BY, self.M_R_BX, self.M_R_BY, self.M_L_BX, self.M_L_BY)
    	
        msg_robot_pose = Odometry()
        if(self.L_BX != -1 and self.L_BY != -1 and self.R_BX != -1 and self.R_BY != -1 and self.L_XX != -1 and self.L_XY != -1 and self.R_XX != -1 and self.R_XY != -1):
            print("trapezoid_priority")
            #print(self.M_BX, self.M_BY, self.M_XCRX, self.M_XCRY)
            object_points = np.array([[-700, 850, 0],
                                      [700, 850, 0],
                                      [0, 850, 0],
                                      [-1400,850,4500],
                                      [1400,850,4500],
                                      [0, 850, 4500]], dtype=np.float32)
            # trapezoid
            self.M_BX = (self.R_BX + self.L_BX) / 2
            self.M_BY = (self.R_BY + self.L_BY) / 2
            self.M_XCRX = (self.R_XX + self.L_XX) / 2
            self.M_XCRY = (self.R_XY + self.L_XY) / 2
            print("M_BX = ", self.M_BX)
            print("M_BY = ", self.M_BY)
            print("M_XCRX = ", self.M_XCRX)
            print("M_XCRY = ", self.M_XCRY)
            #print(self.M_BX, self.M_BY, self.M_R_BX, self.M_R_BY, self.M_L_BX, self.M_L_BY)
            image_points = np.array([[self.L_XX,self.L_XY],
                                    [self.R_XX,self.R_XY],
                                    [self.M_XCRX,self.M_XCRY],
                                    [self.L_BX,self.L_BY],
                                    [self.R_BX,self.R_BY],
                                    [self.M_BX,self.M_BY]], dtype=np.float32)
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
            if abs(xx) <= 450 and abs(yy) <= 300:
                print("x =", xx, "y = ", yy, "z =", zz)
                msg_robot_pose.pose.pose.position.x = float(xx)
                msg_robot_pose.pose.pose.position.y = float(yy)
                msg_robot_pose.pose.pose.position.z = 0.0
            else :
                msg_robot_pose.pose.pose.position.x = -1.0
                msg_robot_pose.pose.pose.position.y = -1.0
                msg_robot_pose.pose.pose.position.z = 0.0
        elif(b_pole_1_x != -1 and b_pole_1_y != -1 and b_pole_2_x != -1 and b_pole_2_y != -1 and t_pole_1_x != -1 and t_pole_1_y != -1 and t_pole_2_x != -1 and t_pole_2_y != -1):
            print("goal_priority")
            #print(self.L_CX, self.L_CY, self.R_CX, self.R_CY)
            object_points = np.array([[-1400, -850, 4500],
                                      [1400, -850, 4500],
                                      [-1400, 0, 4500],
                                      [1400, 0, 4500],
                                      [-1400, 850, 4500],
                                      [1400, 850, 4500]], dtype=np.float32)
             # goal_priority
            self.L_CX = (self.L_TX + self.L_BX) / 2
            self.L_CY = (self.L_TY + self.L_BY) / 2
            self.R_CX = (self.R_TX + self.R_BX) / 2
            self.R_CY = (self.R_TY + self.R_BY) / 2
            print("L_CX = ", self.L_CX)
            print("L_CY = ", self.L_CY)
            print("R_CX = ", self.R_CX)
            print("R_CY = ", self.R_CY)
            image_points = np.array([[self.L_TX,self.L_TY],
                                    [self.R_TX,self.R_TY],
                                    [self.L_CX,self.L_CY],
                                    [self.R_CX,self.R_CY],
                                    [self.L_BX,self.L_BY],
                                    [self.R_BX,self.R_BY]], dtype=np.float32)
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
            if abs(xx) <= 450 and abs(yy) <= 300:    
                print("x =", xx, "y = ", yy, "z =", zz)
                msg_robot_pose.pose.pose.position.x = float(xx)
                msg_robot_pose.pose.pose.position.y = float(yy)
                msg_robot_pose.pose.pose.position.z = 0.0
            else :
                msg_robot_pose.pose.pose.position.x = -1.0
                msg_robot_pose.pose.pose.position.y = -1.0
                msg_robot_pose.pose.pose.position.z = 0.0
        
        elif(self.Pinalty_X != -1 and self.Pinalty_Y != -1 and self.L_BX != -1 and self.L_BY != -1 and self.R_BX != -1 and self.R_BY != -1):
            print("triangle_priority")
            #print(self.M_BX, self.M_BY, self.M_R_BX, self.M_R_BY, self.M_L_BX, self.M_L_BY)
            object_points = np.array([[-1400, 850, 4500],
                                      [1400, 850, 4500],
                                      [0, 850, 4500],
                                      [0, 850, 2400],
                                      [-700, 850, 3450],
                                      [700, 850, 3450]], dtype=np.float32)
            self.M_BX = (self.R_BX + self.L_BX) / 2
            self.M_BY = (self.R_BY + self.L_BY) / 2
            self.M_R_BX = (self.R_BX + self.Pinalty_X)/2
            self.M_R_BY = (self.R_BY + self.Pinalty_Y)/2
            self.M_L_BX = (self.L_BX + self.Pinalty_X)/2
            self.M_L_BY = (self.L_BY + self.Pinalty_Y)/2
            
            print("m_bx = ", self.M_BX)
            print("m_by = ", self.M_BY)
            print("M_R_BX = ", self.M_R_BX)
            print("M_R_BY = ", self.M_R_BY)
            print("M_L_BX = ", self.M_L_BX)
            print("M_L_BY = ", self.M_L_BY)
            image_points = np.array([[self.L_BX,self.L_BY],
                                    [self.R_BX,self.R_BY],
                                    [self.M_BX,self.M_BY],
                                    [self.Pinalty_X,self.Pinalty_Y],
                                    [self.M_L_BX,self.M_L_BY],
                                    [self.M_R_BX,self.M_R_BY]], dtype=np.float32)
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
            if abs(xx) <= 450 and abs(yy) <= 300:
                print("x =", xx, "y = ", yy, "z =", zz)
                msg_robot_pose.pose.pose.position.x = float(xx)
                msg_robot_pose.pose.pose.position.y = float(yy)
                msg_robot_pose.pose.pose.position.z = 0.0
            else :
                msg_robot_pose.pose.pose.position.x = -1.0
                msg_robot_pose.pose.pose.position.y = -1.0
                msg_robot_pose.pose.pose.position.z = 0.0
        
        else:
            msg_robot_pose.pose.pose.position.x = -1.0
            msg_robot_pose.pose.pose.position.y = -1.0
            msg_robot_pose.pose.pose.position.z = 0.0

        self.publish_poseRobot.publish(msg_robot_pose)
        

    def timer_callback(self):
        self.R_BX = self.R_BY = self.L_BX = self.L_BY = self.R_TX = self.R_TY = self.L_TX = self.L_TY = self.R_CX = self.R_CY = self.L_CX = self.L_CY = -1
        self.bX[0] = self.bX[1] = self.tX[0] = self.tX[1] = self.bY[0] = self.bY[1] = self.tY[0] = self.tY[1]  = -1
        self.A = 0
        self.B = 0

def main(args=None):
    rclpy.init(args=args)
    node = Localization()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == '__main__':
    main()
