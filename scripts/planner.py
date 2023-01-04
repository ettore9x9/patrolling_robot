#! /usr/bin/env python
"""
.. module:: planner
  :platform: Unix 
  :synopsis: Python module for the node that plans the robot's trajectory
.. moduleauthor:: Ettore Sani 5322242@studenti.unige.it

This module simulates the planner node of the architecture. It implements a service that, provided the initial position, plans a variable number of waypoints to reach the goal position.

Service:
    /motion/planner

"""

### IMPORTS ###
import random
import rospy
# Import constant name defined to structure the architecture.
from surveillance_robot import architecture_name_mapper as anm
# Import the ActionServer implementation used.
from actionlib import SimpleActionServer
# Import custom message, actions and services.
from surveillance_robot.msg import Point, PlanFeedback, PlanResult, PlanAction
from patrolling_robot.srv import AskPosition, AskPositionRequest

### GLOBAL ###
LOG_TAG = anm.NODE_PLANNER   # Tag for identifying logs producer.

### CLASSES ###
class PlanningAction(object):
    """This class implements an action server to simulate motion planning.
    Given an initial and a target position, it returns a plan as a set of via points.

    """
    def __init__(self):

        # Instantiate and start the action server based on the `SimpleActionServer` class.
        self._as = SimpleActionServer(anm.ACTION_PLANNER, 
                                      PlanAction, 
                                      execute_cb=self.execute_callback, 
                                      auto_start=False)
        self._as.start()
        rospy.wait_for_service('/ask_position')

        try:
            self.client = rospy.ServiceProxy("/ask_position", AskPosition)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
      
    def execute_callback(self, goal):
        """Callback function invoked when a client set a goal to the `planner` server.
        This function will return a list of random points (i.e., the plan) when the fist point
        is the current robot position, while the last point is the `goal` position. The plan will contain 
        a random number of other points, which spans in the range 
        [`self._random_plan_points[0]`, `self._random_plan_points[1]`). To simulate computation,
        each point is added to the plan with a random delay spanning in the range 
        [`self._random_plan_time[0]`, `self._random_plan_time[1]`).

        """
        req = AskPositionRequest()
        req.what = "rosbot"
        start_point  = self.client(req)
        req.what = goal.target
        target_point = self.client(req)

        # Check if the start and target positions are correct. If not, this service will be aborted.
        if  start_point is None or target_point is None:
            log_msg = 'Cannot have `None` target_point. This service will be aborted!.'
            rospy.logerr(anm.tag_log(log_msg, LOG_TAG))
            # Close service by returning an `ABORT` state to the client.
            self._as.set_aborted()
            return
        
        # Initialise the `feedback` with the starting point of the plan.
        feedback = PlanFeedback()
        feedback.via_points = []
        feedback.via_points.append(start_point)
        # Publish the feedback and wait to simulate computation.
        self._as.publish_feedback(feedback)

        # Append the target point to the plan as the last point.
        feedback.via_points.append(target_point)

        # Publish the results to the client.        
        result = PlanResult()
        result.via_points = feedback.via_points
        self._as.set_succeeded(result)
        log_msg = 'Motion plan succeeded with plan: '
        log_msg += ''.join('(' + str(point.x) + ', ' + str(point.y) + '), ' for point in result.via_points)
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

def main():
    """This function initializes the planner ros node.

    """
    rospy.init_node(anm.NODE_PLANNER, log_level=rospy.INFO)
    server = PlanningAction()
    rospy.spin()

### MAIN ###
if __name__ == '__main__':
    main()   