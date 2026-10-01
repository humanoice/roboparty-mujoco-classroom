# SPDX-License-Identifier: GPL-3.0
"""Put inference_node in /cmd_vel mode each time it starts.

inference_node starts in gamepad mode and ignores /cmd_vel. On the real robot
you press Y on the gamepad. Here we press a virtual Y button.
"""
import time

import rclpy

from roboparty_sim.robot_link import RobotLink, start_spinning


def inference_node_running(link):
    # inference_node is the only publisher of /action.
    return link.count_publishers('/action') > 0


def main():
    rclpy.init()
    link = RobotLink('cmd_vel_mode')
    spinner = start_spinning(link)
    log = link.get_logger()
    try:
        while rclpy.ok():
            while rclpy.ok() and not inference_node_running(link):
                time.sleep(0.5)
            time.sleep(1.0)  # let its subscriptions connect
            if not link.ensure_cmd_vel_mode():
                log.warn('Could not switch inference_node to /cmd_vel mode, will try again')
                continue
            log.info('inference_node now listens to /cmd_vel')
            # If inference_node restarts, it is in gamepad mode again.
            while rclpy.ok() and inference_node_running(link):
                time.sleep(0.5)
            log.info('inference_node stopped, waiting for it to restart')
    except KeyboardInterrupt:
        pass
    finally:
        spinner.stop(link)


if __name__ == '__main__':
    main()
