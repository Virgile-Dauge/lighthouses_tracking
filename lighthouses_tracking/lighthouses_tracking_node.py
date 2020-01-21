#!/usr/bin/env python3
import os
import sys
import time
import signal
import subprocess

import tf2_ros

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from std_srvs.srv import Trigger
from geometry_msgs.msg import TransformStamped

import pose_openvr_wrapper
from pose_transform import Transform

class steamvr_process:
    def __init__(self):
        self.steam_runtime = os.path.expanduser('~/.steam/steam/ubuntu12_32/steam-runtime/run.sh')
        self.vr_monitor = os.path.expanduser('~/.steam/steam/steamapps/common/SteamVR/bin/vrmonitor.sh')
        self.proc = None
        signal.signal(signal.SIGINT, self.sigint_handler)
        self.is_running = False

    def sigint_handler(self, signal_received, frame):
        # Handle any cleanup here
        self.kill()
        print('SIGINT or CTRL-C detected. Exiting gracefully')
        sys.exit(0)

    def start(self, waiting_time=0.5):
        self.proc = subprocess.Popen([self.steam_runtime, self.vr_monitor],
                                     stderr=subprocess.DEVNULL,
                                     stdout=subprocess.DEVNULL)
        time.sleep(waiting_time)
        self.is_running = True

    def kill(self):
        self.is_running = False
        self.proc.kill()

    def restart(self):
        self.kill()
        time.sleep(0.1)
        self.start()

class pose_wrapper(Node):

    def __init__(self, frequency=250):
        super().__init__('pose_wrapper')
        self.up_and_running = False
        self.vr_process = steamvr_process()
        self.vr_process.start()
        self.broadcaster = tf2_ros.TransformBroadcaster(self)
        self.reset_srv = self.create_service(Trigger, "/restart_steamvr",
                                             self.restart_handler)
        self.poser = None
        #self.tmr = None
        self.tmr = self.create_timer(1.0/frequency, self.timer_callback)

    def restart_handler(self, request, response):
        self.stop()
        self.vr_process.restart()
        time.sleep(2)
        self.start()
        self.tmr.reset()
        response.success = True
        response.message = 'check steamvr window for further info'
        return response

    def stop(self):
        self.tmr.cancel()
        #self.tmr.destroy()
        self.up_and_running = False
        if self.poser is not None:
            self.poser.shutdown()

    def start(self, frequency=100):
        #self.tmr = self.create_timer(1.0/frequency, self.timer_callback)
        self.reset_lighthouse_db()
        #rate = self.create_rate(10)  # 10hz
        steamvr_launch_ok = False
        while not steamvr_launch_ok:
            print('noooooooooo')
            try:
                self.poser = pose_openvr_wrapper.OpenvrWrapper(
                    './config.json')
                steamvr_launch_ok = True
            except RuntimeError:
                #print('steamVR_not_running')
                pass
        #time.sleep(0.5) #rate.sleep()
        self.up_and_running = True
        self.get_logger().info('poser up and running')


    def timer_callback(self):
        matrices = self.poser.get_all_transformation_matrices(
            samples_count=1)
        for device, matrix in matrices.items():
            self.send_transform(matrix, device)

    def send_transform(self, matrix, device):
        transform = Transform(matrix)
        quaternion = transform.quaternion()
        position = transform.position()
        t = TransformStamped()
        ti = self.get_clock().now()
        print(ti, type(ti))
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "local"
        t.child_frame_id = device
        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]
        t.transform.rotation.x = quaternion.x
        t.transform.rotation.y = quaternion.y
        t.transform.rotation.z = quaternion.z
        t.transform.rotation.w = quaternion.w
        self.broadcaster.sendTransform(t)

    def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self)
        # try:
        #     rclpy.spin(self)
        # except KeyboardInterrupt:
        #     pass

        # rate = self.create_rate(frequency)  # 10hz
        # while rclpy.ok():
        #     if self.up_and_running:
        #         matrices = self.poser.get_all_transformation_matrices(
        #             samples_count=1)
        #         for device, matrix in matrices.items():
        #             transform = Transform(matrix)
        #             quaternion = transform.quaternion()
        #             position = transform.position()
        #             self.broadcaster.sendTransform(
        #                 position,
        #                 (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
        #                 rclpy.time.now(),
        #                 device,
        #                 "local")
        #     rate.sleep()

    def reset_lighthouse_db(self):
        lighthouse_db = os.path.expanduser('~/.steam/debian-installation/config/lighthouse/lighthousedb.json')
        if os.path.exists(lighthouse_db):
            subprocess.run(['rm', lighthouse_db])

def main(args=None):
    rclpy.init(args=args)

    poser = pose_wrapper()
    poser.start(250)
    poser.spin()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    poser.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
