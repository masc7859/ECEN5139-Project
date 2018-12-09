import rospy
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header
from sensor_msgs.msg import JointState
from intera_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from std_msgs.msg import String
from intera_motion_interface import (
    MotionTrajectory,
    MotionWaypoint,
    MotionWaypointOptions
)
from intera_motion_msgs.msg import TrajectoryOptions
from geometry_msgs.msg import PoseStamped
import PyKDL
from tf_conversions import posemath
from intera_interface import Limb

class CurrentJointStates:
    def __init__(self):
        rospy.init_node('kinematics', anonymous=True)
        self.lock = threading.Lock()
        self.thread0 = threading.Thread(target=self.CurrentJointsListener)
        self.thread1 = threading.Thread(target=self.GoalEndPoseListener)
        self.ikRequestFlag = False
        self.currentJointPositions = []
        self.currentJointsName = ""
        self.endPoseName = ""
        self.endPosePosition = ""
        self.endOrientPosition = ""
        self.endJointPositions = []
        self.ikResults = {}

    def GetCurrentJoints(self, msg):
        self.lock.acquire()
        self.currentJointsName = msg.name
        self.currentJointPositions = msg.position
        self.lock.release()

    def GetEndPose(self, msg):
        self.lock.acquire()
        self.ikRequestFlag = True
        self.endPoseName = msg.name
        self.endPosePosition = msg.position
        self.endOrientPosition = msg.orientation
        self.lock.release()
        self.ikResults = self.GetEndJoints(self)
        self.goToPose(self)

    def CurrentJointsListener(self):
        rospy.Subscriber("/robot/joint_states", JointState, self.GetCurrentJoints)
        rospy.spin()

    def GoalEndPoseListener(self):
        rospy.Subscriber("/robot/goal_end_pose", Pose, self.GetEndPose)
        rospy.spin()

    def GetEndJoints(self):
        if not self.currentJointPositions:
            rospy.loginfo("Cant get current joint poses")
            return {}

        limb = self.endPoseName
        #limb = "right"
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        ikreq = SolvePositionIKRequest()
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        poses = {
            'right': PoseStamped(
                header=hdr,
                pose=Pose(
                    position=Point(
                        x=self.endPosePosition.x,
                        y=self.endPosePosition.y,
                        z=self.endPosePosition.z,
                    ),
                    orientation=Quaternion(
                        x=self.endOrientPosition.x,
                        y=self.endOrientPosition.y,
                        z=self.endOrientPosition.z,
                        w=self.endOrientPosition.w,
                    ),
                ),
            ),
        }
        ikreq.pose_stamp.append(poses[limb])
        ikreq.tip_names.append('right_hand')

        rospy.loginfo("Running Advanced IK Service Client example.")

        ikreq.seed_mode = ikreq.SEED_CURRENT
        seed = JointState()
        seed.name = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
                     'right_j4', 'right_j5', 'right_j6']
        seed.position = self.currentJointPositions[0:6]
        ikreq.seed_angles.append(seed)

        ikreq.use_nullspace_goal.append(True)
        goal = JointState()
        goal.name = ['right_j1', 'right_j2', 'right_j3']
        goal.position = [0.1, 0.1, 0.1]
        ikreq.nullspace_goal.append(goal)
        ikreq.nullspace_gain.append(0.1)

        try:
            rospy.wait_for_service(ns, 5.0)
            resp = iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return {}

        if (resp.result_type[0] > 0):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp.result_type[0], 'None')

            limbJoints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            rospy.loginfo("\nIK Joint Solution:\n%s", limbJoints)
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            rospy.logerr("Result Error %d", resp.result_type[0])
            return {}

        return limbJoints

    def goToPose(self):
        limb = Limb()

        traj_options = TrajectoryOptions()
        traj_options.interpolation_type = TrajectoryOptions.CARTESIAN
        traj = MotionTrajectory(trajectory_options = traj_options, limb = limb)

        wpt_opts = MotionWaypointOptions(max_linear_speed=0.1,
                                         max_linear_accel=0.1,
                                         max_rotational_speed=0.1,
                                         max_rotational_accel=0.1,
                                         max_joint_speed_ratio=1.0)
        waypoint = MotionWaypoint(options = wpt_opts.to_msg(), limb = limb)

        joint_names = limb.joint_names()

        waypoint.set_joint_angles(self.ikResults.values(), "left_hand", joint_names)

        traj.append_waypoint(waypoint.to_msg())
        result = traj.send_trajectory(timeout=30.0)
        if result is None:
            rospy.logerr('Trajectory Failed')
            return

        if result.result:
            rospy.loginfo('Trajectory Success')
        else:
            rospy.logerr('Trajectory failed with error %s',
                        result.errorId)


if __name__ == '__main__':
    currJointStates = CurrentJointStates()
    rospy.spin()
