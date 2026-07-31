#!/usr/bin/env python3

import os
import signal
import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class KeyMissionLauncher(Node):
    def __init__(self):
        super().__init__('key_mission_launcher')
        self.declare_parameter('vision_start_delay', 15.0)
        self.declare_parameter('offboard_start_delay',30.0)
        self.vision_start_delay = float(self.get_parameter('vision_start_delay').value)
        self.offboard_start_delay = float(self.get_parameter('offboard_start_delay').value)
        self.triggered = False
        self.selected_task = None
        self.processes = []
        self.vision_timer = None
        self.offboard_timer = None
        self.create_subscription(Bool, '/gpio_keys/key1', self._key1_callback, 10)
        self.create_subscription(Bool, '/gpio_keys/key2', self._key2_callback, 10)
        self.get_logger().info('Waiting for /gpio_keys/key1 or /gpio_keys/key2 to become true')

    def _key1_callback(self, message):
        if message.data:
            self._trigger('offboard_task1_node', '/gpio_keys/key1')

    def _key2_callback(self, message):
        if message.data:
            self._trigger('offboard_task2_node', '/gpio_keys/key2')

    def _trigger(self, executable, topic):
        if self.triggered:
            return
        self.triggered = True
        self.selected_task = executable
        self.get_logger().info(
            f'{topic}=true; starting vision in {self.vision_start_delay:.1f} s')
        self.vision_timer = self.create_timer(self.vision_start_delay, self._start_vision)

    def _start_vision(self):
        self.vision_timer.cancel()
        self.get_logger().info('Starting: ros2 launch track2vision tracked2vision.launch.py')
        self._start_process(['ros2', 'launch', 'track2vision', 'tracked2vision.launch.py'])
        self.get_logger().info(
            f'Starting {self.selected_task} in {self.offboard_start_delay:.1f} s')
        self.offboard_timer = self.create_timer(self.offboard_start_delay, self._start_offboard)

    def _start_offboard(self):
        self.offboard_timer.cancel()
        self.get_logger().info(f'Starting: ros2 run offboard_control {self.selected_task}')
        self._start_process(['ros2', 'run', 'offboard_control', self.selected_task])

    def _start_process(self, command):
        try:
            self.processes.append(subprocess.Popen(command, start_new_session=True))
        except OSError as error:
            self.get_logger().error(f'Failed to start {" ".join(command)}: {error}')
            rclpy.shutdown()

    def stop_processes(self):
        for process in reversed(self.processes):
            if process.poll() is None:
                try:
                    os.killpg(process.pid, signal.SIGINT)
                    process.wait(timeout=5.0)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGTERM)


def main(args=None):
    rclpy.init(args=args)
    node = KeyMissionLauncher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_processes()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
