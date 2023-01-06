#! /usr/bin/env python
"""
.. module:: rosbot_state 
	:platform: Unix 
	:synopsis: Node for managing the robot's and rooms' positions.

.. moduleauthor:: Ettore Sani <ettoresani0@gmail.com> 

ROS node for managing the positions of rooms and the robot in a robotics system. 
It provides services for adding new rooms and asking for the positions of rooms and the robot, 
and it listens for updates to the robot's pose and stores the current pose of the robot.

Subscribes to: 
	* /odom topic where the simulator publishes the robot position

Services: 
	* /info/ask_position it replies with the robot's or room's position
	* /info/set_room_position used for setting a new room's position

"""

### LIBRARIES ###
import rospy
from patrolling_robot.srv import SetRoomPosition, AskPosition, SetRoomPositionResponse, AskPositionResponse
from nav_msgs.msg import Odometry

class RoomPositionManager:
	"""Class for managing the robot's and rooms' positions.

    This class provides services for adding new rooms and asking for the positions
    of rooms and the robot. It also listens for updates to the robot's pose and
    stores the current pose of the robot.

    Attributes:
        robotPoseX: The x-coordinate of the robot's current pose.
        robotPoseY: The y-coordinate of the robot's current pose.
        dictionary: A dictionary mapping room names to their positions.

    """

	def __init__(self):
		"""Initialize the room position manager.

        This sets up the service and subscriber for the class.

        """
		rospy.Service("/info/set_room_position", SetRoomPosition, self.AddNewRoom)
		rospy.Service("/info/ask_position", AskPosition, self.FindPosition)
		rospy.Subscriber("/odom", Odometry, self.UpdateRobotPose)

		self.robotPoseY = None
		self.robotPoseX = None
		self.dictionary = {}

	def AddNewRoom(self, request):
		"""Add a new room to the dictionary of positions.

        Args:
            request: A SetRoomPosition request containing the name and position of 
            the new room.

        Returns:
            A SetRoomPositionResponse with a boolean data field indicating whether
            the room was successfully added to the dictionary.

        """
		self.dictionary[request.room] = [request.x,request.y]
		response = SetRoomPositionResponse()
		response.data = True;
		rospy.loginfo("@rosbot_state: Location %s registered", request.room)
		return response

	def FindPosition(self, request):
		"""Find the position of a room or the robot.

        Args:
            request: An AskPosition request containing the name of the room or "rosbot"
            to indicate the position of the robot.

        Returns:
            An AskPositionResponse with the x and y coordinates of the requested
            room or robot.

        """
		response = AskPositionResponse()
		if request.what == "rosbot":
			response.x = self.robotPoseX
			response.y = self.robotPoseY
		else:
			response.x = self.dictionary[request.what][0]
			response.y = self.dictionary[request.what][1]
		return response

	def UpdateRobotPose(self, data):
		"""Update the stored pose of the robot.

        Args:
            data: An Odometry message containing the current pose of the robot.

        """
		self.robotPoseX = data.pose.pose.position.x
		self.robotPoseY = data.pose.pose.position.y

def main():
    """Main function for the room position manager node.

    This function sets up the ROS node and creates an instance of the
    RoomPositionManager class.

    """
    rospy.init_node("rosbot_state", log_level=rospy.INFO)
    RoomPositionManager()
    rospy.spin()


### MAIN ###
if __name__ == "__main__":
    main()