import math
import rclpy
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose as Nav2NavigateToPose
from action_msgs.msg import GoalStatus

from modules.base_bt_nodes import BTNodeList, Status, Node, Sequence, Fallback, ReactiveSequence, ReactiveFallback
from modules.base_bt_nodes_ros import ConditionWithROSTopics, ActionWithROSAction
# --------------------------------------------------------
# 1. 노드 등록
# --------------------------------------------------------
CUSTOM_ACTION_NODES = [
    'NavigateToPoseNode', 
    'WallAvoid'           
]

CUSTOM_CONDITION_NODES = [
    'CheckWallSafe'       
]

BTNodeList.ACTION_NODES.extend(CUSTOM_ACTION_NODES)
BTNodeList.CONDITION_NODES.extend(CUSTOM_CONDITION_NODES)


# --------------------------------------------------------
# 2. 커스텀 노드 구현
# --------------------------------------------------------

class CheckWallSafe(ConditionWithROSTopics):
    """
    [Condition] 벽 상태 확인
    """
    def __init__(self, tag_name, agent, name=None):
        # name 인자 충돌 방지 로직 적용
        actual_name = name if name else tag_name
        
        super().__init__(actual_name, agent, [
            (String, '/wall_state', 'wall_state_msg')
        ])

    def _predicate(self, agent, blackboard):
        cache = self._cache
        if 'wall_state_msg' not in cache:
            return False

        msg = cache['wall_state_msg']
        return (msg.data == "NORMAL")


class NavigateToPoseNode(ActionWithROSAction):
    """
    [Action] Nav2 이동
    """
    def __init__(self, tag_name, agent, name=None, goal_x=0.0, goal_y=0.0):
        actual_name = name if name else tag_name
        
        super().__init__(actual_name, agent, (Nav2NavigateToPose, 'navigate_to_pose'))
        
        self.goal_x = float(goal_x)
        self.goal_y = float(goal_y)

    def _build_goal(self, agent, blackboard):
        goal = Nav2NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.ros.node.get_clock().now().to_msg()
        ps.pose.position.x = self.goal_x
        ps.pose.position.y = self.goal_y
        ps.pose.orientation.w = 1.0 
        goal.pose = ps
        return goal

    def _interpret_result(self, result, agent, blackboard, status_code=None):
        if status_code == GoalStatus.STATUS_SUCCEEDED:
            return Status.SUCCESS
        return Status.FAILURE


class WallAvoid(Node):
    """
    [Action] 단순 회피
    """
    def __init__(self, tag_name, agent, name=None, duration=2.0):
        actual_name = name if name else tag_name
        super().__init__(actual_name)
        
        self.agent = agent
        self.duration = float(duration)
        self.start_time = None
        self.pub = None
        self.ros_node = agent.ros_bridge.node

    def setup(self, **kwargs):
        self.pub = self.ros_node.create_publisher(Twist, '/cmd_vel_bt', 10)

    def initialise(self):
        self.start_time = self.ros_node.get_clock().now().nanoseconds / 1e9
        self.ros_node.get_logger().info(f"[{self.name}] Start Avoiding...")

    def update(self):
        if self.pub is None:
            return Status.FAILURE

        current_time = self.ros_node.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.start_time

        if elapsed > self.duration:
            stop_msg = Twist()
            self.pub.publish(stop_msg)
            return Status.SUCCESS

        twist = Twist()
        twist.linear.x = -0.15
        twist.angular.z = 0.5
        self.pub.publish(twist)
        return Status.RUNNING

    def terminate(self, new_status):
        if self.pub:
            self.pub.publish(Twist())