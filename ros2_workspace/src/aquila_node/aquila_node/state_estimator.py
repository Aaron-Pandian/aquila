#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

# Mathematical Imports
import gtsam
from gtsam import Values, NonlinearFactorGraph
import numpy as np
import math

class GTSAMStateEstimator(Node):
    def __init__(self):
        super().__init__('factor_graph_estimator')
        
        # --- 1. GTSAM Setup (iSAM2 Incremental Optimizer) ---
        params = gtsam.ISAM2Params()
        params.setRelinearizeThreshold(0.1)
        params.setRelinearizeSkip(1)
        self.isam = gtsam.ISAM2(params)
        
        # Noise Models (Tuning is key for Factor Graphs)
        # We trust the IMU gyroscope a lot, but accelerometer less
        self.imu_noise = gtsam.noiseModel.Isotropic.Sigma(6, 0.1) 
        # We trust GPS position to about 0.5 meters
        self.gps_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.5) 
        
        # State Storage
        self.current_pose_key = 1
        self.graph = NonlinearFactorGraph()
        self.initial_values = Values()
        
        # Preintegration (The magic that handles high-speed IMU data)
        # Using standard Gravity (9.81) and zero bias initially
        imu_params = gtsam.PreintegrationParams.MakeSharedU(9.81)
        self.imu_integrator = gtsam.PreintegratedImuMeasurements(imu_params)
        self.last_imu_time = None
        self.prev_state = gtsam.NavState() # Holds Pose, Velocity
        self.prev_bias = gtsam.imuBias.ConstantBias()

        # GPS Origin (Simple Lat/Lon linearization)
        self.origin_lat = None
        self.origin_lon = None

        # --- 2. ROS Interface ---
        # Subscribe to the topics remapped in the bridge script
        self.create_subscription(Imu, '/aquila/imu', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/aquila/gps', self.gps_callback, 10)
        
        self.odom_pub = self.create_publisher(Odometry, '/aquila/state_estimate', 10)
        
        self.get_logger().info("GTSAM State Estimator Online. Waiting for IMU/GPS...")

    def imu_callback(self, msg):
        """
        Runs at ~250Hz. Accumulates IMU data into the pre-integrator.
        """
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Extract vectors
        acc = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        gyr = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        
        if self.last_imu_time is None:
            self.last_imu_time = timestamp
            return

        dt = timestamp - self.last_imu_time
        self.last_imu_time = timestamp
        
        # 1. Integrate Measurement
        self.imu_integrator.integrateMeasurement(acc, gyr, dt)

    def gps_callback(self, msg):
        """
        Runs at ~5-10Hz. Adds factors to the graph and optimizes.
        """
        # Set Origin on first fix
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f"GPS Origin Set: {self.origin_lat}, {self.origin_lon}")
            
            # Initialize Graph with Prior
            prior_noise = gtsam.noiseModel.Diagonal.Sigmas(
                np.array([0.1]*6 + [0.1]*3 + [0.1]*6)) # Pose(6), Vel(3), Bias(6)
            
            self.graph.add(gtsam.PriorFactorPose3(0, gtsam.Pose3(), gtsam.noiseModel.Isotropic.Sigma(6, 0.1)))
            self.graph.add(gtsam.PriorFactorVelocity3(0, np.array([0,0,0]), gtsam.noiseModel.Isotropic.Sigma(3, 0.1)))
            self.graph.add(gtsam.PriorFactorConstantBias(0, self.prev_bias, gtsam.noiseModel.Isotropic.Sigma(6, 1.0)))
            
            self.initial_values.insert(0, gtsam.Pose3())
            self.initial_values.insert(gtsam.symbol('v', 0), np.array([0,0,0]))
            self.initial_values.insert(gtsam.symbol('b', 0), self.prev_bias)
            return

        # 2. Linearize GPS (Lat/Lon -> Local X/Y meters)
        # 1 deg lat ~= 111,000m. 1 deg lon ~= 111,000 * cos(lat)
        d_lat = msg.latitude - self.origin_lat
        d_lon = msg.longitude - self.origin_lon
        
        local_x = d_lat * 111132.92
        local_y = d_lon * 111412.84 * math.cos(math.radians(self.origin_lat))
        local_z = msg.altitude 
        
        gps_point = gtsam.Point3(local_x, local_y, local_z)

        # 3. Add IMU Factor (Connecting prev state to current state)
        imu_factor = gtsam.ImuFactor(
            self.current_pose_key - 1, 
            gtsam.symbol('v', self.current_pose_key - 1),
            self.current_pose_key, 
            gtsam.symbol('v', self.current_pose_key),
            gtsam.symbol('b', self.current_pose_key - 1),
            self.imu_integrator
        )
        self.graph.add(imu_factor)

        # 4. Add GPS Factor (Unary factor on Position)
        gps_factor = gtsam.GPSFactor(
            self.current_pose_key,
            gps_point,
            self.gps_noise
        )
        self.graph.add(gps_factor)
        
        # 5. Add Initial Guesses (Predict using IMU only)
        # This gives the optimizer a starting point
        predicted_state = self.imu_integrator.predict(self.prev_state, self.prev_bias)
        self.initial_values.insert(self.current_pose_key, predicted_state.pose())
        self.initial_values.insert(gtsam.symbol('v', self.current_pose_key), predicted_state.velocity())
        self.initial_values.insert(gtsam.symbol('b', self.current_pose_key), self.prev_bias) # Bias random walk not implemented for simplicity

        # 6. Optimize!
        self.isam.update(self.graph, self.initial_values)
        self.isam.update() # Double update for stability
        
        # 7. Extract Results
        result = self.isam.calculateEstimate()
        current_pose = result.atPose3(self.current_pose_key)
        
        # Update previous state for next loop
        self.prev_state = gtsam.NavState(current_pose, result.atVector(gtsam.symbol('v', self.current_pose_key)))
        
        # 8. Publish Odometry
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.pose.pose.position.x = current_pose.x()
        odom_msg.pose.pose.position.y = current_pose.y()
        odom_msg.pose.pose.position.z = current_pose.z()
        # (Add orientation/quaternion here if needed)
        
        self.odom_pub.publish(odom_msg)
        
        # Reset graph containers for next iteration
        self.graph.resize(0)
        self.initial_values.clear()
        self.imu_integrator.resetIntegration()
        self.current_pose_key += 1

def main(args=None):
    rclpy.init(args=args)
    node = GTSAMStateEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()