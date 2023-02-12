#pragma once

#include "adrc_controller.h"
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include "state.h"

class AdrcProcess
{
    public:
        void init(const ros::NodeHandle & nh_)
        {
            status_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("eso_status", 100);

            imu_sub_ = nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                            100,
                                            boost::bind(&imu_preprocess, &imu_data_, _1),
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().tcpNoDelay());

            debug_sub_ = nh.subscribe<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl",
                                            10
                                            boost::bind(&debug_preprocess, &debug_data_, _1),
                                            ros::VoidConstPtr(),
                                            ros::TransportHints().tcpNoDelay());
        }

        void imu_preprocess(sensor_msgs::ImuConstPtr imu_msg)
        {
            ros::Time now = ros::Time::now();
            sensor_msgs::Imu msg;

            msg = *imu_msg;
            rcv_stamp = now;

            odom_.w(0) = msg.angular_velocity.x;
            odom_.w(1) = msg.angular_velocity.y;
            odom_.w(2) = msg.angular_velocity.z;

            odom_.a(0) = msg.linear_acceleration.x;
            odom_.a(1) = msg.linear_acceleration.y;
            odom_.a(2) = msg.linear_acceleration.z;

            odom_.q.x() = msg.orientation.x;
            odom_.q.y() = msg.orientation.y;
            odom_.q.z() = msg.orientation.z;
            odom_.q.w() = msg.orientation.w;

            // check the frequency
            static int one_min_count = 9999;
            static ros::Time last_clear_count_time = ros::Time(0.0);
            if ( (now - last_clear_count_time).toSec() > 1.0 )
            {
                if ( one_min_count < 100 )
                {
                    ROS_WARN("IMU frequency seems lower than 100Hz, which is too low!");
                }
                one_min_count = 0;
                last_clear_count_time = now;
            }
            one_min_count ++;
        }

        void debug_preprocess(quadrotor_msgs::Px4ctrlDebugConstPtr debug_msg)
        {
            ros::Time now = ros::Time::now();
            quadrotor_msgs::Px4ctrlDebug msg;
            
            msg = *debug_msg;
            rcv_stamp = now;

            controller_output_.torque[0] = msg.roll_torque;
            controller_output_.torque[1] = msg.pitch_torque;
            controller_output_.torque[2] = msg.yaw_torque;

            // check the frequency
            static int one_min_count = 9999;
            static ros::Time last_clear_count_time = ros::Time(0.0);
            if ( (now - last_clear_count_time).toSec() > 1.0 )
            {
                if ( one_min_count < 100 )
                {
                    ROS_WARN("IMU frequency seems lower than 100Hz, which is too low!");
                }
                one_min_count = 0;
                last_clear_count_time = now;
            }
            one_min_count ++;
        }

        void eso_process()
        {
            adrc_controller_rx.eso_linear_2d(odom_.w(0), controller_output_.torque(0));

            adrc_controller_ry.eso_linear_2d(odom_.w(1), controller_output_.torque(1));

            adrc_controller_rz.eso_linear_2d(odom_.w(2), controller_output_.torque(2));
            publish_eso_status();
        }

        void publish_eso_status()
        {
            std_msgs::Float32MultiArray eso_output;
            eso_output.data.push_back(adrc_controller_rx.get_z1());
            eso_output.data.push_back(adrc_controller_rx.get_z2());

            eso_output.data.push_back(adrc_controller_ry.get_z1());
            eso_output.data.push_back(adrc_controller_ry.get_z2());

            eso_output.data.push_back(adrc_controller_rz.get_z1());
            eso_output.data.push_back(adrc_controller_rz.get_z2());
            status_pub_.publish(eso_output);
        }

    private:
        AdrcController adrc_controller_rx = AdrcController(0.0025, 1);
        AdrcController adrc_controller_ry = AdrcController(0.0025, 1);
        AdrcController adrc_controller_rz = AdrcController(0.0025, 1);

        quadrotor_msgs::Px4ctrlDebug debug_data_;
        sensor_msgs::Imu imu_data_;

        State odom_;
        ControllerOutput controller_output_;


        // subscriber
        ros::Subscriber imu_sub_, debug_sub_;
        // publisher
        ros::Publisher status_pub_;
}
