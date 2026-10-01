# SPDX-License-Identifier: GPL-3.0
"""Talk to the robot the same way the real RoboParty tools do.

- Services on inference_node: /init_motors, /reset_joints, /start_inference, ...
- Topic /cmd_vel: how fast to walk and turn.
- Topic /joy: a "virtual gamepad". Some features (switch to /cmd_vel mode,
  dances) are only on gamepad buttons, so we press buttons in software.
- Topic /rosout: every node's log messages. We read inference_node's log to
  see the result of a button press.
"""
import collections
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Joy
from std_srvs.srv import SetBool, Trigger

# Gamepad buttons, as used in roboparty_inference ros_interface.cpp.
BUTTON_A = 0   # reset joints
BUTTON_B = 1   # start / pause inference
BUTTON_X = 2   # init / deinit motors
BUTTON_Y = 3   # switch gamepad <-> /cmd_vel control
BUTTON_LB = 4  # motion policy on / off (dance)
BUTTON_RB = 5  # select next motion

# Safe command range, from clip_cmd in the policy config.
MAX_FORWARD, MAX_BACKWARD = 0.6, -0.4
MAX_SIDE = 0.4
MAX_TURN = 0.8

FALLEN_HEIGHT = 0.45


class RobotLink(Node):

    def __init__(self, name='robot_link'):
        super().__init__(name)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.joy_pub = self.create_publisher(Joy, '/joy', 10)
        self._service_clients = {}
        self._log_lock = threading.Condition()
        self._logs = collections.deque(maxlen=200)
        self._log_count = 0
        rosout_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(Log, '/rosout', self._on_log, rosout_qos)
        self.base_pose = None
        self.create_subscription(PoseStamped, '/sim/base_pose', self._on_pose, 10)

    # ----- logs -----------------------------------------------------------

    def _on_log(self, msg):
        if msg.name != 'inference_node':
            return
        with self._log_lock:
            self._log_count += 1
            self._logs.append((self._log_count, msg.msg))
            self._log_lock.notify_all()

    def log_mark(self):
        """Number of log lines so far. Pass it to wait_log() to see only newer lines."""
        with self._log_lock:
            return self._log_count

    def wait_log(self, mark, prefixes, timeout=2.0):
        """Wait for an inference_node log line after mark that starts with a prefix."""
        end = time.monotonic() + timeout
        with self._log_lock:
            while True:
                for number, line in self._logs:
                    if number > mark and line.startswith(tuple(prefixes)):
                        return line
                remaining = end - time.monotonic()
                if remaining <= 0:
                    return None
                self._log_lock.wait(remaining)

    def _on_pose(self, msg):
        self.base_pose = msg.pose

    # ----- services -------------------------------------------------------

    def call(self, name, srv_type=Trigger, request=None, timeout=5.0):
        """Call a service. Returns (success, message)."""
        client = self._service_clients.get(name)
        if client is None:
            client = self.create_client(srv_type, name)
            self._service_clients[name] = client
        if not client.wait_for_service(timeout_sec=timeout):
            return False, f'service {name} is not available (is the robot running?)'
        future = client.call_async(request or srv_type.Request())
        end = time.monotonic() + timeout
        while not future.done() and time.monotonic() < end:
            time.sleep(0.01)
        if not future.done():
            return False, f'service {name} did not answer'
        result = future.result()
        return result.success, result.message

    def hold(self, on):
        return self.call('/sim/hold', SetBool, SetBool.Request(data=on))

    # ----- topics ---------------------------------------------------------

    def send_velocity(self, forward=0.0, side=0.0, turn=0.0):
        msg = Twist()
        msg.linear.x = max(MAX_BACKWARD, min(MAX_FORWARD, float(forward)))
        msg.linear.y = max(-MAX_SIDE, min(MAX_SIDE, float(side)))
        msg.angular.z = max(-MAX_TURN, min(MAX_TURN, float(turn)))
        self.cmd_pub.publish(msg)
        return msg

    def press(self, button):
        """Press and release one virtual gamepad button."""
        for value in (1, 0):
            msg = Joy()
            msg.axes = [0.0] * 8
            msg.buttons = [0] * 12
            msg.buttons[button] = value
            for _ in range(3):  # /joy is "best effort": send a few copies
                msg.header.stamp = self.get_clock().now().to_msg()
                self.joy_pub.publish(msg)
                time.sleep(0.03)

    def press_and_read(self, button, prefixes, timeout=2.0):
        mark = self.log_mark()
        self.press(button)
        return self.wait_log(mark, prefixes, timeout)

    def ensure_cmd_vel_mode(self, attempts=4):
        """Switch inference_node from gamepad control to /cmd_vel control."""
        for _ in range(attempts):
            line = self.press_and_read(BUTTON_Y, ['Controlled by'])
            if line == 'Controlled by /cmd_vel':
                return True
        return False

    def is_fallen(self):
        return self.base_pose is not None and self.base_pose.position.z < FALLEN_HEIGHT


def start_spinning(node):
    """Spin a node in a background thread so callbacks run while we wait."""
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return Spinner(executor, thread)


class Spinner:

    def __init__(self, executor, thread):
        self.executor = executor
        self.thread = thread

    def stop(self, node):
        """Stop spinning, then clean up. The order matters: a C++ thread that
        is still waiting at exit aborts the program."""
        rclpy.try_shutdown()
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        node.destroy_node()
