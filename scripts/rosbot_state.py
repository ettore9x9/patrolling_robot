#! /usr/bin/env python

### LIBRARIES ###
import rospy
from patrolling_robot.srv import SetRoomPosition, AskPosition, SetRoomPositionResponse, AskPositionResponse
from nav_msgs.msg import Odometry

class RoomPositionManager:

	def __init__(self):
		rospy.Service("/set_room_position", SetRoomPosition, self.add_new_room)
		rospy.Service("/ask_position", AskPosition, self.find_position)
		rospy.Subscriber("/odom", Odometry, self.update_robot_pose)

		self.robot_pose_x = None
		self.robot_pose_y = None
		self.dictionary = {}

	def add_new_room(self, request):
		self.dictionary[request.room] = [request.x,request.y]
		response = SetRoomPositionResponse()
		response.data = True;
		rospy.loginfo("location %s registered", request.room)
		return response

	def find_position(self, request):
		response = AskPositionResponse()
		if request.what == "rosbot":
			response.x = self.robot_pose_x
			response.y = self.robot_pose_y
		else:
			response.x = self.dictionary[request.what][0]
			response.y = self.dictionary[request.what][1]
		return response

	def update_robot_pose(self, data):
		self.robot_pose_x = data.pose.pose.position.x
		self.robot_pose_y = data.pose.pose.position.y

def main():
    """This function initializes the battery_manager ros node.

    """
    rospy.init_node("rosbot_state", log_level=rospy.INFO)
    RoomPositionManager()
    rospy.spin()


### MAIN ###
if __name__ == "__main__":
    main()