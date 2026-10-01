# SPDX-License-Identifier: GPL-3.0
"""party: simple commands for the simulated RoboParty robot.

Each command is a few ROS calls. Read this file to see which ones.
"""
import argparse
import os
import sys
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory

from roboparty_sim.robot_link import (BUTTON_LB, BUTTON_RB, MAX_TURN, RobotLink,
                                      start_spinning)

MOTIONS = ['wave', 'dance0', 'dance1']
POLICY_FPS = 50.0  # inference runs every dt * decimation = 0.004 * 5 = 0.02 s


def say(text):
    print(text, flush=True)


def check(ok_message, result, ok_if=()):
    success, message = result
    if success or any(text in message for text in ok_if):
        say(f'  ✔ {ok_message}')
        return True
    say(f'  ✘ {message}')
    return False


# ----- commands --------------------------------------------------------------

def cmd_start(link, args):
    """Wake up the robot: the same steps as on the real robot."""
    say('Starting the robot:')
    if not check('robot is on the stand', link.hold(True)):
        return 1
    if not check('motors on            (/init_motors)', link.call('/init_motors'),
                 ok_if=['already initialized']):
        return 1
    if not check('moving to start pose (/reset_joints)', link.call('/reset_joints')):
        return 1
    for left in range(4, 0, -1):
        say(f'    ... {left}')
        time.sleep(1.0)
    time.sleep(0.6)
    if not check('brain on             (/start_inference)', link.call('/start_inference'),
                 ok_if=['already running']):
        return 1
    time.sleep(0.5)
    check('off the stand: the robot stands on its own!', link.hold(False))
    say('Try:  party walk   party turn left   party wave   party dance')
    return 0


def move(link, forward, side, turn, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        link.send_velocity(forward, side, turn)
        time.sleep(0.1)
        if link.is_fallen():
            link.send_velocity()
            say('Oh no, the robot fell! Run:  party reset')
            return 1
    link.send_velocity()
    return 0


def cmd_walk(link, args):
    say(f'Walking at {args.speed} m/s for {args.seconds} s (/cmd_vel linear.x)')
    return move(link, args.speed, 0.0, 0.0, args.seconds)


def cmd_back(link, args):
    say(f'Walking back at {args.speed} m/s for {args.seconds} s (/cmd_vel linear.x < 0)')
    return move(link, -abs(args.speed), 0.0, 0.0, args.seconds)


def cmd_turn(link, args):
    turn = args.speed if args.direction == 'left' else -args.speed
    say(f'Turning {args.direction} for {args.seconds} s (/cmd_vel angular.z = {turn:.2f})')
    return move(link, 0.0, 0.0, turn, args.seconds)


def cmd_side(link, args):
    side = args.speed if args.direction == 'left' else -args.speed
    say(f'Side step {args.direction} for {args.seconds} s (/cmd_vel linear.y = {side:.2f})')
    return move(link, 0.0, side, 0.0, args.seconds)


def cmd_stop(link, args):
    link.send_velocity()
    say('Stop (/cmd_vel all zero)')
    return 0


def cmd_push(link, args):
    return 0 if check('push! (/sim/push)', link.call('/sim/push')) else 1


def cmd_reset(link, args):
    # No /stop_inference here: the brain must still run when "party start"
    # calls /reset_joints, because only then does it clear its memory.
    say('Putting the robot back at the start:')
    link.send_velocity()
    if not check('robot is back on the stand (/sim/reset)', link.call('/sim/reset')):
        return 1
    return cmd_start(link, args)


def cmd_sleep(link, args):
    say('Putting the robot to sleep:')
    link.send_velocity()
    check('robot is on the stand', link.hold(True))
    # /deinit_motors also stops the brain and clears its memory.
    check('brain and motors off (/deinit_motors)', link.call('/deinit_motors'),
          ok_if=['already deinitialized'])
    return 0


def motion_seconds(name):
    path = os.path.join(get_package_share_directory('roboparty_inference'),
                        'robots', 'rpo', 'motions', f'{name}.npz')
    with np.load(path) as motion:
        frames = max(np.shape(motion[key])[0] for key in motion.files)
    return frames / POLICY_FPS


def cmd_motion(link, name):
    """Play one motion with the beyondmimic brain, then go back to walking."""
    want = f'policy_{name}.onnx'
    link.send_velocity()
    say(f'Selecting {name} (gamepad button RB)')
    for _ in range(len(MOTIONS) + 1):
        line = link.press_and_read(BUTTON_RB, ['Selected policy:', 'Cannot switch'])
        if line is None:
            say('  ✘ No answer. Start the robot with the beyondmimic policy and run:  party start')
            return 1
        if line.startswith('Cannot switch'):
            link.press_and_read(BUTTON_LB, ['Policy enabled:'])  # leave the old motion first
            continue
        if line.endswith(want):
            break
    else:
        say(f'  ✘ Could not select {name}')
        return 1
    seconds = motion_seconds(name)
    line = link.press_and_read(BUTTON_LB, ['Policy enabled:'])
    say(f'  ✔ {line} (gamepad button LB) - {seconds:.1f} s')
    end = time.monotonic() + seconds + 0.5
    while time.monotonic() < end:
        if link.is_fallen():
            say('Oh no, the robot fell! Run:  party reset')
            return 1
        time.sleep(0.1)
    line = link.press_and_read(BUTTON_LB, ['Policy enabled:'])
    say(f'  ✔ back to walking: {line} (gamepad button LB)')
    return 0


def cmd_wave(link, args):
    return cmd_motion(link, 'wave')


def cmd_dance(link, args):
    return cmd_motion(link, f'dance{args.number}')


def cmd_status(link, args):
    end = time.monotonic() + 3.0
    while link.base_pose is None and time.monotonic() < end:
        time.sleep(0.1)
    pose = link.base_pose
    if pose is None:
        say('No robot. Is the simulator running?')
        return 1
    p = pose.position
    say(f'Robot body at x={p.x:.2f} m, y={p.y:.2f} m, height={p.z:.2f} m')
    say('The robot has FALLEN. Run:  party reset' if link.is_fallen() else 'The robot is up.')
    return 0


def build_parser():
    parser = argparse.ArgumentParser(prog='party', description='Control the simulated RoboParty robot.')
    sub = parser.add_subparsers(dest='command', required=True)

    def add(name, func, help_text):
        p = sub.add_parser(name, help=help_text, description=help_text)
        p.set_defaults(func=func)
        return p

    add('start', cmd_start, 'motors on, start pose, brain on, off the stand')
    p = add('walk', cmd_walk, 'walk forward')
    p.add_argument('speed', nargs='?', type=float, default=0.4, help='m/s, max 0.6 (default 0.4)')
    p.add_argument('seconds', nargs='?', type=float, default=3.0, help='default 3')
    p = add('back', cmd_back, 'walk backward')
    p.add_argument('speed', nargs='?', type=float, default=0.3, help='m/s, max 0.4 (default 0.3)')
    p.add_argument('seconds', nargs='?', type=float, default=2.0, help='default 2')
    p = add('turn', cmd_turn, 'turn on the spot')
    p.add_argument('direction', choices=['left', 'right'])
    p.add_argument('seconds', nargs='?', type=float, default=2.0, help='default 2')
    p.add_argument('--speed', type=float, default=MAX_TURN * 0.75, help='rad/s, max 0.8')
    p = add('side', cmd_side, 'step sideways')
    p.add_argument('direction', choices=['left', 'right'])
    p.add_argument('seconds', nargs='?', type=float, default=2.0, help='default 2')
    p.add_argument('--speed', type=float, default=0.3, help='m/s, max 0.4')
    add('stop', cmd_stop, 'stand still')
    add('wave', cmd_wave, 'wave hello')
    p = add('dance', cmd_dance, 'dance!')
    p.add_argument('number', nargs='?', type=int, choices=[0, 1], default=0)
    add('push', cmd_push, 'give the robot a push')
    add('reset', cmd_reset, 'put the robot back at the start and start again')
    add('sleep', cmd_sleep, 'brain off, motors off')
    add('status', cmd_status, 'where is the robot? did it fall?')
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    rclpy.init()
    link = RobotLink('party')
    spinner = start_spinning(link)
    time.sleep(0.3)  # let topics connect
    try:
        code = args.func(link, args)
    except KeyboardInterrupt:
        link.send_velocity()
        code = 130
    finally:
        time.sleep(0.2)
        spinner.stop(link)
    sys.exit(code)


if __name__ == '__main__':
    main()
