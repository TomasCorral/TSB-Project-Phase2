#!/usr/bin/env python3

import time
import math
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry


def euler_from_quaternion(x, y, z, w): #https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


class Plotter(Node):

    def __init__(self):
        super().__init__('plotter')
        self.subscription_state = self.create_subscription(
            Odometry,
            'topic3',
            self.odometry_callback,
            10)
        self.subscription_pid = self.create_subscription(
            Float32MultiArray,
            'topic2',
            self.pid_callback,
            10)
        self.subscription_reference = self.create_subscription(
            Float32MultiArray,
            'topic1',
            self.reference_callback,
            10)
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.update_plot)
        
        self.get_logger().info('Plotter node started')

        self.start_time = time.time()

        self.x = []
        self.y = []
        self.psi = []
        self.u = []
        self.v = []
        self.r = []
        self.state_time = []
        self.tau_u = []
        self.tau_r = []
        self.tau_time = []
        self.ref_u = []
        self.ref_psi = []
        self.ref_time = []

        self.init_plot()

    def odometry_callback(self, msg):
        q = msg.pose.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion(q.x, q.y, q.z, q.w)
        yaw = yaw * 180 / math.pi # Convert to degrees

        self.x.append(msg.pose.pose.position.x)
        self.y.append(msg.pose.pose.position.y)
        self.psi.append(yaw) 
        self.u.append(msg.twist.twist.linear.x)
        self.v.append(msg.twist.twist.linear.y)
        self.r.append(msg.twist.twist.angular.z)
        
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        self.state_time.append(elapsed_time)

        self.get_logger().info('Odometry: x="%s", y="%s", psi="%s"' % (msg.pose.pose.position.x, msg.pose.pose.position.y, yaw))

    
    def pid_callback(self, msg):
        self.tau_u.append(msg.data[0])
        self.tau_r.append(msg.data[1])
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        self.tau_time.append(elapsed_time)

        self.get_logger().info('PID: tau_u:"%s" tau_r:"%s"' % (msg.data[0], msg.data[1]))


    def reference_callback(self, msg):
        self.ref_u.append(msg.data[0])
        psi_normalized = ((msg.data[1] + 180) % 360) - 180
        self.ref_psi.append(psi_normalized)

        current_time = time.time()
        elapsed_time = current_time - self.start_time
        self.ref_time.append(elapsed_time)

        self.get_logger().info('Reference: u:"%s" psi:"%s"' % (msg.data[0], msg.data[1]))


    def init_plot(self):
        self.fig, (self.ax1, self.ax3) = plt.subplots(2, 1, figsize=(8, 8))

        # Plot reference and current state
        self.line_ref_u, = self.ax1.plot([], [], color='blue', label='Reference u')
        self.line_true_u, = self.ax1.plot([], [], color='cyan', label='True u')
        self.ax1.set_xlabel('Time (s)')
        self.ax1.set_ylabel('Speed (m/s)')
        
        self.ax2 = self.ax1.twinx()  # Create second y-axis
        self.line_ref_psi, = self.ax2.plot([], [], color='red', label='Reference psi')
        self.line_true_psi, = self.ax2.plot([], [], color='orange', label='True psi')
        self.ax2.set_ylabel('Heading (rad)')

        self.ax1.legend(loc='upper left')
        self.ax2.legend(loc='lower left')

        # Plot Tau
        self.line_tau_u, = self.ax3.plot([], [], label='Tau u')
        self.line_tau_r, = self.ax3.plot([], [], label='Tau r')
        self.ax3.set_xlabel('Time (s)')
        self.ax3.set_ylabel('Force output')
        self.ax3.set_title('PID Output')
        self.ax3.legend()

        plt.tight_layout()
        plt.ion()  # Turn on interactive mode for real-time plotting
        plt.show()

    def update_plot(self):
        #print(len(self.ref_time), len(self.ref_u))
        #print(len(self.state_time), len(self.u))
        #print(len(self.ref_time), len(self.ref_psi))
        #print(len(self.state_time), len(self.psi))
        #return

        # Update PID plot
        self.line_ref_u.set_data(self.ref_time, self.ref_u)
        self.line_true_u.set_data(self.state_time, self.u)
        self.ax1.relim()
        self.ax1.autoscale_view()

        self.line_ref_psi.set_data(self.ref_time, self.ref_psi)
        self.line_true_psi.set_data(self.state_time, self.psi)
        self.ax2.relim()
        self.ax2.autoscale_view()

        # Update Tau plot
        self.line_tau_u.set_data(self.tau_time, self.tau_u)
        self.line_tau_r.set_data(self.tau_time, self.tau_r)
        self.ax3.relim()  # Recalculate limits
        self.ax3.autoscale_view()  # Rescale view

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


        

def main(args=None):
    rclpy.init(args=args)
    plotter= Plotter()
    rclpy.spin(plotter)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()