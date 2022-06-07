#!/usr/bin/env python
from helpers.robot_actions import RobotActions
import helpers.constants as env_constants
import rospy
import math
import time
from geometry_msgs.msg import Twist, Point

robot_actions = RobotActions()

SLIT_1 = 0
SLIT_2 = 0

CYLINDER_POS = Point()
CYLINDER_POS.x = 0
CYLINDER_POS.y = -4.4
CYLINDER_POS.z = 0

AREA = 0
# -------------------------------------
# AREA 0 : Wall following (INSIDE maze)
# AREA 1 : Start of slit wall to WALL_POS1 (Section of Door 1, OUTSIDE maze )
# AREA 2 : WALL_POS1 to WALL_POS2 (Section of Door 2)
# AREA 3 : Go to cylinder
# AREA 4 : Rotate cylinder
# AREA 5 : Complete
# -------------------------------------


def main():
    global AREA, SLIT_1, SLIT_2
    rospy.init_node('maze_navigation')
    command = Twist()
    command.angular.z = 0.0
    command.linear.x = 0.0
    rate = rospy.Rate(7)
    time.sleep(1)
    command.angular.z = -0.5
    command.linear.x = 0.1
    print("Looking for wall")
    robot_actions.cmd_vel.publish(command)
    time.sleep(2)

    while not rospy.is_shutdown():
        if(AREA == 1):
            if(robot_actions.progress_state == 0):
                robot_actions.correct_yaw(env_constants.WALL_POS1)
            elif(robot_actions.progress_state == 1):
                robot_actions.move_directly(env_constants.WALL_POS1)
            else:  # progress_state is 2 (Done with current area)
                AREA = 2
                robot_actions.change_progress_state(0)
                if(SLIT_1 > 1):
                    SLIT_1 = 1
                print("AREA 01 complete, Doors found :"+str(SLIT_1))
            if(math.isinf(robot_actions.robot_left) and math.isinf(robot_actions.robot_right)):
                SLIT_1 = SLIT_1 + 1
        elif(AREA == 2):
            if(robot_actions.progress_state == 0):
                robot_actions.correct_yaw(env_constants.WALL_POS2)
            elif(robot_actions.progress_state == 1):
                robot_actions.move_directly(env_constants.WALL_POS2)
            else:  # progress_state is 2 (Done with current area)
                AREA = 3
                robot_actions.change_progress_state(0)
                if(SLIT_2 > 1):
                    SLIT_2 = 1
                print("AREA 02 complete, Doors found :"+str(SLIT_2))
                robot_actions.correct_yaw(env_constants.WALL_POS2)
                CYLINDER_POS.x = robot_actions.robot_front + \
                    robot_actions.robot_position.x - env_constants.ROBOT_POS_X_MARGIN
            if (math.isinf(robot_actions.robot_left) and math.isinf(robot_actions.robot_right)):
                SLIT_2 = SLIT_2 + 1
        elif(AREA == 3):
            print("Area 3: Moving towards cylinder")
            if(robot_actions.progress_state == 0):
                robot_actions.correct_yaw(CYLINDER_POS)
            elif(robot_actions.progress_state == 1):
                robot_actions.move_directly(CYLINDER_POS)
            else:  # progress_state is 2 (Done with current area)
                AREA = 4
                robot_actions.change_progress_state(0)
        elif(AREA == 4):
            print("Area 4: Cylinder rotation")
            open_slits = SLIT_1 + SLIT_2
            print("Total slit count : "+str(open_slits))
            robot_actions.correct_yaw(CYLINDER_POS)

            rotation_direction = 1 if (open_slits == 1) else -1
            command.angular.z = 0.5236 * rotation_direction
            command.linear.x = 0
            robot_actions.cmd_vel.publish(command)
            time.sleep(2.9)

            start_x_pos = robot_actions.robot_position.x

            command.angular.z = -0.18 * rotation_direction
            command.linear.x = 0.26
            robot_actions.cmd_vel.publish(command)
            time.sleep(10)

            while(True):
                if(robot_actions.robot_position.x <= start_x_pos + env_constants.ROBOT_POS_X_PRECISION):
                    break
            AREA = 5
        elif(AREA == 5):
            robot_actions.stop()
        else:
            robot_actions.follow_wall(env_constants.DISTANCE, command)
        if(math.isinf(robot_actions.robot_front) and math.isinf(robot_actions.robot_front_left) and not math.isinf(robot_actions.robot_right)):
            if(not AREA == 2 and not AREA == 3 and not AREA == 4 and not AREA == 5):
                AREA = 1
        rate.sleep()


if __name__ == '__main__':
    main()