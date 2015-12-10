#!/usr/bin/env python

import rospy

from moveit_commander import RobotCommander, PlanningSceneInterface
from moveit_commander import roscpp_initialize, roscpp_shutdown

from actionlib import SimpleActionClient, GoalStatus

from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion
from moveit_msgs.msg import PickupAction, PickupGoal
from moveit_msgs.msg import PlaceAction, PlaceGoal
from moveit_msgs.msg import PlaceLocation
from moveit_msgs.msg import MoveItErrorCodes
from moveit_simple_grasps.msg import GenerateGraspsAction, GenerateGraspsGoal, GraspGeneratorOptions

from tf.transformations import quaternion_from_euler

import sys
import copy
import numpy

# Create dict with human readable MoveIt! error codes:
moveit_error_dict = {}
for name in MoveItErrorCodes.__dict__.keys():
    if not name[:1] == '_':
        code = MoveItErrorCodes.__dict__[name]
        moveit_error_dict[code] = name

#Create de class
class PigeonPickAndPlace:
    #Class constructor (parecido com java, inicializa o que foi instanciado)
    def __init__(self):
      
        # Retrieve params:
        
        self._grasp_object_name = rospy.get_param('~grasp_object_name', 'lego_block')
        
        self._grasp_object_width = rospy.get_param('~grasp_object_width', 0.016)

        self._arm_group     = rospy.get_param('~arm', 'arm_move_group')
        self._gripper_group = rospy.get_param('~gripper', 'gripper')

        self._approach_retreat_desired_dist = rospy.get_param('~approach_retreat_desired_dist', 0.01)
        self._approach_retreat_min_dist = rospy.get_param('~approach_retreat_min_dist', 0.4)

        # Create (debugging) publishers:
        self._grasps_pub = rospy.Publisher('grasps', PoseArray, queue_size=1, latch=True)
        
        # Create planning scene and robot commander:
        self._scene = PlanningSceneInterface()
        self._robot = RobotCommander()

        rospy.sleep(1.0)

        # Clean the scene (remove the old objects:
        self._scene.remove_world_object(self._grasp_object_name)

        # Add table and Coke can objects to the planning scene:
        # Posso fazer um objeto com a TF do objeto detectado =D
        self._pose_object_grasp = self._add_object_grasp(self._grasp_object_name)

        rospy.sleep(1.0)

        # Retrieve groups (arm and gripper):
        self._arm     = self._robot.get_group(self._arm_group)
        self._gripper = self._robot.get_group(self._gripper_group)

        # Create grasp generator 'generate' action client:
        self._grasps_ac = SimpleActionClient('/moveit_simple_grasps_server/generate', GenerateGraspsAction)
        if not self._grasps_ac.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr('Grasp generator action client not available!')
            rospy.signal_shutdown('Grasp generator action client not available!')
            return

        # Create move group 'pickup' action client:
        self._pickup_ac = SimpleActionClient('/pickup', PickupAction)
        if not self._pickup_ac.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr('Pick up action client not available!')
            rospy.signal_shutdown('Pick up action client not available!')
            return

        # Pick object:
        while not self._pickup(self._arm_group, self._grasp_object_name, self._grasp_object_width):
            rospy.logwarn('Pick up failed! Retrying ...')
            rospy.sleep(1.0)

        rospy.loginfo('Pick up successfully')
        
    def __del__(self):
        # Clean the scene
        self._scene.remove_world_object(self._grasp_object_name)

    def _add_object_grasp(self, name):

        p = PoseStamped()
        
        p.header.frame_id = self._robot.get_planning_frame() #pose
        p.header.stamp = rospy.Time.now()
	rospy.loginfo(self._robot.get_planning_frame())
        p.pose.position.x = 0.11 # 0.26599 # antigo = 0.75 - 0.01
        p.pose.position.y = -0.31 # -0.030892 #antigo = 0.25 - 0.01
        p.pose.position.z = 0.41339 #antigo = 1.00 + (0.3 + 0.03) / 2.0
        #p.pose.orientation.x = 0.0028925
        #p.pose.orientation.y = -0.0019311
        #p.pose.orientation.z = -0.71058
        #p.pose.orientation.w = 0.70361

        q = quaternion_from_euler(0.0, 0.0, 0.0)
        p.pose.orientation = Quaternion(*q)

        # Coke can size from ~/.gazebo/models/coke_can/meshes/,
        # using the measure tape tool from meshlab.
        # The box is the bounding box of the lego cylinder.
        # The values are taken from the cylinder base diameter and height.
        # the size is length (x),thickness(y),height(z)
        # I don't know if the detector return this values of object
        self._scene.add_box(name, p, (0.032, 0.016, 0.020))

        return p.pose

    def _generate_grasps(self, pose, width):
        """
        Generate grasps by using the grasp generator generate action; based on
        server_test.py example on moveit_simple_grasps pkg.
        """

        # Create goal:
        goal = GenerateGraspsGoal()

        goal.pose  = pose
        goal.width = width

        options = GraspGeneratorOptions()
        # simple_graps.cpp doesn't implement GRASP_AXIS_Z!
        #options.grasp_axis      = GraspGeneratorOptions.GRASP_AXIS_Z
        options.grasp_direction = GraspGeneratorOptions.GRASP_DIRECTION_UP
        options.grasp_rotation  = GraspGeneratorOptions.GRASP_ROTATION_FULL

        # @todo disabled because it works better with the default options
        #goal.options.append(options)

        # Send goal and wait for result:
        state = self._grasps_ac.send_goal_and_wait(goal)
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr('Grasp goal failed!: %s' % self._grasps_ac.get_goal_status_text())
            return None

        grasps = self._grasps_ac.get_result().grasps
        #rospy.logerr('Generated: %f grasps:' % grasps.size)

        # Publish grasps (for debugging/visualization purposes):
        self._publish_grasps(grasps)

        return grasps

    def _create_pickup_goal(self, group, target, grasps):
        """
        Create a MoveIt! PickupGoal
        """

        # Create goal:
        goal = PickupGoal()

        goal.group_name  = group
        goal.target_name = target

        goal.possible_grasps.extend(grasps)

        goal.allowed_touch_objects.append(target)

        #goal.support_surface_name = self._table_object_name

        # Configure goal planning options:
        goal.allowed_planning_time = 5.0

        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 10

        return goal

   
    def _pickup(self, group, target, width):
        """
        Pick up a target using the planning group
        """

        # Obtain possible grasps from the grasp generator server:
        grasps = self._generate_grasps(self._pose_object_grasp, 0.016)

        # Create and send Pickup goal:
        goal = self._create_pickup_goal(group, target, grasps)

        state = self._pickup_ac.send_goal_and_wait(goal)
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr('Pick up goal failed!: %s' % self._pickup_ac.get_goal_status_text())
            return None

        result = self._pickup_ac.get_result()

        # Check for error:
        err = result.error_code.val
        if err != MoveItErrorCodes.SUCCESS:
            rospy.logwarn('Group %s cannot pick up target %s!: %s' % (group, target, str(moveit_error_dict[err])))

            return False

        return True

    def _publish_grasps(self, grasps):
        """
        Publish grasps as poses, using a PoseArray message
        """

        if self._grasps_pub.get_num_connections() > 0:
            msg = PoseArray()
            msg.header.frame_id = self._robot.get_planning_frame()
            msg.header.stamp = rospy.Time.now()

            for grasp in grasps:
                p = grasp.grasp_pose.pose

                msg.poses.append(Pose(p.position, p.orientation))

            self._grasps_pub.publish(msg)

def main():
    p = PigeonPickAndPlace()

    rospy.spin()

if __name__ == '__main__':
    roscpp_initialize(sys.argv)
    rospy.init_node('pick_lego')

    main()

    roscpp_shutdown()

