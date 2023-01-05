#! /usr/bin/env python

# Run with: roslaunch final_assignment assignment.launch

### LIBRARIES ###
import rospy
import actionlib

### MESSAGES ###
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionServer
from surveillance_robot.msg import ControlAction, ControlFeedback, ControlResult
from surveillance_robot import architecture_name_mapper as anm

LOG_TAG = anm.NODE_CONTROLLER   # Tag for identifying logs producer.

### CODE ###
class ControllingAction(object):

    def __init__(self):
        self.goal_counter = 0
        self.feedback_counter = 0
        self.is_active = False
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self._as = SimpleActionServer(anm.ACTION_CONTROLLER,ControlAction ,execute_cb=self.execute_callback, auto_start=False)
        self._as.start()

    def execute_callback(self, goal):
        """Callback function invoked when a client set a goal to the `controller` server.
        This function requires a list of via points (i.e., the plan)

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
            while self.is_active is True and not rospy.is_shutdown():
                r.sleep()
            log_msg = f'Reaching point ({point.x}, {point.y}).'
            rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

        # Publish the results to the client.
        result = ControlResult()
        result.reached_point = feedback.reached_point
        rospy.loginfo(anm.tag_log('Motion control successes.', LOG_TAG))
        self._as.set_succeeded(result)
        return  # succeeded


    def active_cb(self):
        # Function executed when the communication starts.
        self.goal_counter += 1   # Increments the goal counter.
        rospy.loginfo("Communication started, goal " + str(self.goal_counter))

    def feedback_cb(self, feedback):
        # Function executed when a feedback is received.
        self.feedback_counter += 1   # Increments the feedback counter.
        if self.feedback_counter % 10 == 0:
            rospy.loginfo("feedback "+str(self.feedback_counter)+" received")

    def done_cb(self, status, result):
        # Function executed when the communication ends.
        
        self.is_active = False   # The action client communication is not active.

        # Prints on the info window the status returned by the action server communication.
        if status == 2:
            rospy.loginfo("Goal n "+str(self.goal_counter)+" received a cancel request.")
            return

        if status == 3:
            rospy.loginfo("Goal n "+str(self.goal_counter)+" reached.")
            return

        if status == 4:
            rospy.loginfo("Goal n "+str(self.goal_counter)+" was aborted.")
            return

        if status == 5:
            rospy.loginfo("Goal n "+str(self.goal_counter)+" has been rejected.")
            return

        if status == 8:
            rospy.loginfo("Goal n "+str(self.goal_counter)+" received a cancel request.")
            return

    def reach_goal(self, x, y):
        # This function sends a goal to the move base node, starting the action client communication.

        self.is_active = True   # Processing the goal.

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
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)

    def cancel_goal(self):
        # This function sends a cancel request to the move_base server.
        self.is_active = False
        self.client.cancel_goal()

def main():
    """This function initializes the planner ros node.

    """
    rospy.init_node(anm.NODE_CONTROLLER, log_level=rospy.INFO)
    server = ControllingAction()
    rospy.spin()

### MAIN ###
if __name__ == '__main__':
    main()   