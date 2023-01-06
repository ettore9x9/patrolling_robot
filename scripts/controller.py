#! /usr/bin/env python
"""
.. module:: controller 
    :platform: Unix 
    :synopsis: Node for calling the move_base action service.

.. moduleauthor:: Ettore Sani <ettoresani0@gmail.com> 

ROS node for calling the move_base action service. It implements a service that, provided with
a list of waypoints, asks the move_base planning, mapping and controlling algorithm to bring the
robot sequencially through all of them. When a waypoint is reached, the robot turns the camera to
look around, then starts again reaching the next waypoint, if any.

Client of: 
    * /move_base

Publishes on:
    * /camera/camera_command

"""

### LIBRARIES ###
import rospy
import actionlib

### MESSAGES ###
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionServer
from surveillance_robot.msg import ControlAction, ControlFeedback, ControlResult
from surveillance_robot import architecture_name_mapper as anm
from patrolling_robot.msg import MoveCamera

LOG_TAG = anm.NODE_CONTROLLER   # Tag for identifying logs producer.

### CODE ###
class ControllingAction(object):
    """Class representing an action for controlling the robot using an action server.

    """

    def __init__(self):
        """Initializes the action server, the move base client, and the move camera publisher.

        """
        self.goalCounter = 0
        self.feedbackCounter = 0
        self.isActive = False
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.moveCamPub = rospy.Publisher('/camera/camera_command', MoveCamera, queue_size=1)
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,ControlAction ,execute_cb=self.ExecuteCb, auto_start=False)
        self._as.start()

    def ExecuteCb(self, goal):
        """Callback function invoked when a client sets a goal to the `controller` server.
        This function requires a list of via points (i.e., the plan).

        :param goal: The goal for the `controller` server.
        :type goal: ControlGoal

        """
        # Check if the provided plan is processable. If not, this service will be aborted.
        if goal is None or goal.via_points is None or len(goal.via_points) == 0:
            rospy.logerr(anm.tag_log('No via points provided! This service will be aborted!', LOG_TAG))
            self._as.set_aborted()
            return

        # Construct the feedback and loop for each via point.
        feedback = ControlFeedback()
        rospy.loginfo(anm.tag_log('Server is controlling...', LOG_TAG))
        for point in goal.via_points[1:]:
            # Check that the client did not cancel this service.
            if self._as.is_preempt_requested():
                rospy.loginfo(anm.tag_log('Service has been cancelled by the client!', LOG_TAG))
                # Actually cancel this service.
                self._as.set_preempted()
                return
            # Wait before to reach the following via point.
            self.reach_goal(point.x, point.y)

            # Publish a feedback to the client to simulate that a via point has been reached. 
            feedback.reached_point = point
            self._as.publish_feedback(feedback)
            r = rospy.Rate(10)
            while self.isActive is True and not rospy.is_shutdown():
                r.sleep()
            log_msg = f'Reaching point ({point.x}, {point.y}).'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(anm.tag_log('Motion control successes.', LOG_TAG))
        self._as.set_succeeded(result)
        return  # succeeded


    def ActiveCb(self):
        """Function executed when the communication starts.

        """
        self.goalCounter += 1   # Increments the goal counter.
        rospy.loginfo("Communication started, goal " + str(self.goalCounter))

    def FeedbackCb(self, feedback):
        """Function executed when a feedback is received. 
        This function increments the feedback counter and logs a message every 10 feedbacks.

        :param feedback: The feedback from the action server.
        :type feedback: object

        """
        self.feedbackCounter += 1   # Increments the feedback counter.
        if self.feedbackCounter % 10 == 0:
            rospy.loginfo("feedback "+str(self.feedbackCounter)+" received")

    def DoneCb(self, status, result):
        """Function executed when the communication ends.

        :param status: The status code returned by the action server communication.
        :type status: int
        :param result: The result returned by the action server communication.
        :type result: object

        """

        # Prints on the info window the status returned by the action server communication.
        if status == 2:
            rospy.loginfo("Goal n "+str(self.goalCounter)+" received a cancel request.")
            self.isActive = False   # The action client communication is not active.
            return

        if status == 3:
            rospy.loginfo("Goal n "+str(self.goalCounter)+" reached.")
            moveCam = MoveCamera()
            moveCam.omega = 2.0
            self.moveCamPub.publish(moveCam)
            rospy.sleep(4)
            moveCam.omega = 0.0
            self.moveCamPub.publish(moveCam)
            self.isActive = False   # The action client communication is not active.
            return

        if status == 4:
            rospy.loginfo("Goal n "+str(self.goalCounter)+" was aborted.")
            self.isActive = False   # The action client communication is not active.
            return

        if status == 5:
            rospy.loginfo("Goal n "+str(self.goalCounter)+" has been rejected.")
            self.isActive = False   # The action client communication is not active.
            return

        if status == 8:
            rospy.loginfo("Goal n "+str(self.goalCounter)+" received a cancel request.")
            self.isActive = False   # The action client communication is not active.
            return

    def ReachGoal(self, x, y):
        """This function sends a goal to the move base node, starting the action client communication.

        :param x: The x-coordinate of the goal.
        :type x: float
        :param y: The y-coordinate of the goal.
        :type y: float

        """

        self.isActive = True   # Processing the goal.

        # Waits until the action server has started.
        self.client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = MoveBaseGoal()

        # Fills the elements of the goal.
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.orientation.w = 1
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
   
        # Sends the goal to the action server.
        self.client.send_goal(goal, self.DoneCb, self.ActiveCb, self.FeedbackCb)

    def CancelGoal(self):
        """This function sends a cancel request to the move_base server.

        """
        self.isActive = False
        self.client.cancel_goal()

def main():
    """This function initializes the controller ros node.

    """
    rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()

### MAIN ###
if __name__ == '__main__':
    main()   