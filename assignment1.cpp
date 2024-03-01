/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <thread>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace std::this_thread;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node 
{
	public:
		OffboardControl() : Node("offboard_control")
		{
			offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode",10);
			vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command",10);
			
			offboard_setpoint_counter_ = 0;
			
			auto timer_callback = [this]() -> void {
				if (offboard_setpoint_counter_ == 10) {
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE,1,6);
				}
				publish_offboard_control_mode();
				if (offboard_setpoint_counter_ < 11) {
					offboard_setpoint_counter_++;
				}
			};
			timer_ = this->create_wall_timer(100ms,timer_callback);
		}	
	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
		rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
		
		std::atomic<uint64_t> timestamp_;
		uint64_t offboard_setpoint_counter_;
		
		void publish_offboard_control_mode();
		void publish_vehicle_command(uint16_t command, float param1=0.0, float param2=0.0);
};

class TrajControl : public rclcpp::Node
{
	public: TrajControl() : Node("traj_control")
		{
			trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint",10);
			auto timer_callback = [this]() -> void {
				rclcpp::sleep_for(seconds(15));
				publish_trajectory_setpoint1();
				rclcpp::sleep_for(seconds(10));
				publish_trajectory_setpoint2();
				rclcpp::sleep_for(seconds(10));
				publish_trajectory_setpoint3();
				rclcpp::sleep_for(seconds(10));
				publish_trajectory_setpoint4();
				rclcpp::sleep_for(seconds(10));
				publish_trajectory_setpoint1();
				rclcpp::sleep_for(seconds(10));
			};
			timer_ = this->create_wall_timer(100ms,timer_callback);
		}	
	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
		
		std::atomic<uint64_t> timestamp_;
		
		void publish_trajectory_setpoint1();
		void publish_trajectory_setpoint2();
		void publish_trajectory_setpoint3();
		void publish_trajectory_setpoint4();
};

void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void TrajControl::publish_trajectory_setpoint1()
{
	TrajectorySetpoint msg{};
	msg.position = {0.0, 0.0, -10.0};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void TrajControl::publish_trajectory_setpoint2()
{
	TrajectorySetpoint msg{};
	msg.position = {10, 0.0, -10.0};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);	
}

void TrajControl::publish_trajectory_setpoint3()
{
	TrajectorySetpoint msg{};
	msg.position = {10, 0.0, -25.0};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void TrajControl::publish_trajectory_setpoint4()
{
	TrajectorySetpoint msg{};
	msg.position = {10, -5, -25.0};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);	
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_->publish(msg);
}

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	auto offboardnode = std::make_shared<OffboardControl>();
	auto trajnode = std::make_shared<TrajControl>();
	executor.add_node(offboardnode);
	executor.add_node(trajnode);
	executor.spin();
	rclcpp::shutdown();
	return 0;
}
