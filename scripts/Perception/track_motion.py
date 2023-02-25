#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from  math import *
from time import *
import numpy as np
import cv2
import time
x=0
y=0
yaw=0
ball=False

def poseCallBack(pose_message):
    global x,y,yaw
    x=pose_message.x
    y=pose_message.y
    yaw=pose_message.theta



def move(velocity_publisher,speed,distance,is_forward):
    velocity_message=Twist()
    global x,y
    x0=x
    y0=y
    distance_moved =0.0
    loop_rate=rospy.Rate(10)
    if is_forward:
        velocity_message.linear.x=abs(speed)
    else:
        velocity_message.linear.x=-abs(speed)
    
    while True:
        rospy.loginfo("Moving forwards")
        velocity_publisher.publish(velocity_message)

        loop_rate.sleep()
        distance_moved =abs(sqrt(((x-x0) ** 2)+((y-y0)**2)))
        print (distance_moved)
        if distance_moved>distance:
            rospy.loginfo('Reached')
            break
    velocity_message.linear.x=0
    velocity_publisher.publish(velocity_message)



def rotate(velocity_publisher,angular_speed_degree,relative_angle_degree, clockwise):
    
    velocity_message=Twist()
    angular_speed=radians(abs(angular_speed_degree))

    if(clockwise):
        velocity_message.angular.z=-abs(angular_speed)
    else:
        velocity_message.angular.z=abs(angular_speed)
    loop_rate=rospy.Rate(10)
    t0=rospy.Time.now().to_sec()

    while True:
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1=rospy.Time.now().to_sec()
        current_angle_degree=(t1-t0)*angular_speed_degree
        loop_rate.sleep()

        if current_angle_degree > relative_angle_degree:
            rospy.loginfo("REACHED")
            break
    velocity_message.angular.z=0
    velocity_publisher.publish(velocity_message)



def go_to_goal(velocity_publisher,x_goal,y_goal):
    global x
    global y,yaw


    velocity_message=Twist()
    loop_rate=rospy.Rate(100)
    while(True):
        K_linear=0.5
        distance=abs(sqrt(((x_goal-x)**2)+((y_goal-y)**2)))

        linear_speed = distance * K_linear  #use an proportional controller


        K_angular=4.0
        desired_angle_goal = atan2(y_goal-y,x_goal-x)
        angular_speed = (desired_angle_goal-yaw) * K_angular 

        velocity_message.linear.x=linear_speed
        velocity_message.angular.z=angular_speed
        velocity_publisher.publish(velocity_message)
        print(f'x = {x}, y = {y}, distance to goal = {distance}')

        if(distance < 0.01):
            break
    
    velocity_message.linear.x=0
    velocity_message.angular.z=0
    velocity_publisher.publish(velocity_message)


def setDesiredOrientation(publisher,speed_in_degree,desired_angle_degree):
    global yaw
    relative_angle_radians = radians(desired_angle_degree) - yaw
    clockwise=0
    if relative_angle_radians < 0:
        clockwise=1
    else:
        clockwise=0
    print("relative_angle_radians:", degrees(relative_angle_radians))
    print("desired_angle_radians:", desired_angle_degree)
    rotate(publisher,speed_in_degree,degrees(abs(relative_angle_radians)),clockwise)


def spiral(velocity_publisher,rk,wk):
    
    vel_msg=Twist()
    loop_rate=rospy.Rate(1)

    while ((x<10.5) and (y <10.5)):
        rk=rk+0.1
        vel_msg.linear.x=rk
        vel_msg.linear.y=0
        vel_msg.linear.z=0
        vel_msg.angular.x=0
        vel_msg.angular.y=0
        vel_msg.angular.z=wk
        velocity_publisher.publish(vel_msg)
        loop_rate.sleep()
    vel_msg.linear.x=0
    vel_msg.angular.z=0
    velocity_publisher.publish(vel_msg)



def gridClean(publisher):
    desired_pose=Pose()
    desired_pose.x=1
    desired_pose.y=1
    desired_pose.theta=0

    go_to_goal(publisher,1,1)

    setDesiredOrientation(publisher,30,radians(desired_pose.theta))

    for i in range(5):
        move(publisher,2.0,1.0,True)
        rotate(publisher,20,90,False)
        move(publisher,2.0,9.0,True)
        rotate(publisher,20,90,True)
        move(publisher,2.0,1.0,True)
        rotate(publisher,20,90,True)
        move(publisher,2.0,9.0,True)
        rotate(publisher,20,90,False)
    pass












def read_rgb_image(image_name, show):
    rgb_image = cv2.imread(image_name)
    if show: 
        cv2.imshow("RGB Image",rgb_image)
    return rgb_image

def filter_color(rgb_image, lower_bound_color, upper_bound_color):
    #convert the image into the HSV color space
    hsv_image = cv2.cvtColor(rgb_image, cv2.COLOR_BGR2HSV)
    cv2.imshow("hsv image",hsv_image)

    #define a mask using the lower and upper bounds of the yellow color 
    mask = cv2.inRange(hsv_image, lower_bound_color, upper_bound_color)

    return mask


    

def getContours(binary_image):     
    # _, contours, hierarchy = cv2.findContours(binary_image, 
    #                                          cv2.RETR_TREE, 
    #                                           cv2.CHAIN_APPROX_SIMPLE)
    contours, hierarchy = cv2.findContours(binary_image.copy(), 
                                            cv2.RETR_EXTERNAL,
	                                        cv2.CHAIN_APPROX_SIMPLE)
    return contours


def draw_ball_contour(binary_image, rgb_image, contours):
    black_image = np.zeros([binary_image.shape[0], binary_image.shape[1],3],'uint8')
    
    for c in contours:
        area = cv2.contourArea(c)
        perimeter= cv2.arcLength(c, True)
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        if (area>3000):
            cv2.drawContours(rgb_image, [c], -1, (150,250,150), 1)
            cv2.drawContours(black_image, [c], -1, (150,250,150), 1)
            cx, cy = get_contour_center(c)
            cv2.circle(rgb_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),(int)(radius),(0,0,255),1)
            cv2.circle(black_image, (cx,cy),5,(150,150,255),-1)
            #print ("Area: {}, Perimeter: {}".format(area, perimeter))
    #print ("number of contours: {}".format(len(contours)))
    cv2.imshow("RGB Image Contours",rgb_image)
    cv2.imshow("Black Image Contours",black_image)

def get_contour_center(contour):
    M = cv2.moments(contour)
    cx=-1
    cy=-1
    if (M['m00']!=0):
        cx= int(M['m10']/M['m00'])
        cy= int(M['m01']/M['m00'])
    return cx, cy

def detect_ball_in_a_frame(image_frame):
    yellowLower =(30, 100, 50)
    yellowUpper = (60, 255, 255)
    rgb_image = image_frame
    binary_image_mask = filter_color(rgb_image, yellowLower, yellowUpper)
    contours = getContours(binary_image_mask)
    draw_ball_contour(binary_image_mask, rgb_image,contours)
    return True


def main():
    global ball
    #video_capture = cv2.VideoCapture(0)
    video_capture = cv2.VideoCapture('/home/sergiu/catkin_ws/src/ros_tutorial/scripts/Perception/video/tennis-ball-video.mp4')
  
    #video_capture = cv2.VideoCapture('/home/sergiu/catkin_ws/src/ros_tutorial/scripts/Perception/video/ros.mp4')
    while(True):
        ret, frame = video_capture.read()
        ball=detect_ball_in_a_frame(frame)
        if ball:
            rotate(velocity_publisher,30,90,True)
            # font = cv2.FONT_HERSHEY_SIMPLEX
            # cv2.putText(video_capture,'i am happy!',(10,350), font, 1,(255,255,255),2,cv2.LINE_AA)
        time.sleep(0.033)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
   
    try:
        
        rospy.init_node('turtlesim_motion_pose',anonymous=True)
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher=rospy.Publisher(cmd_vel_topic,Twist,queue_size=10)

        position_topic='/turtle1/pose'
        pose_subscriber=rospy.Subscriber(position_topic,Pose,poseCallBack)
        sleep(2)
        main()
        # if ball:
        #     rotate(velocity_publisher,30,90,True)
        # move(velocity_publisher,1.0,4.0,True)
        #rotate(velocity_publisher,30,90,True)
        #go_to_goal(velocity_publisher,9,9)
        #setDesiredOrientation(velocity_publisher,30,180)
        #spiral(velocity_publisher, 0,2)
        #gridClean(velocity_publisher)
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")


cv2.waitKey(0)
cv2.destroyAllWindows()