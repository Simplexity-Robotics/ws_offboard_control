#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class CompRoutine : public rclcpp::Node {
	private:
		rclcpp::TimerBase::SharedPtr timer_;

		rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
		rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher_;
		rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

		std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

		uint64_t offboard_setpoint_counter_ = 0;   //!< counter for the number of setpoints sent

		void publish_offboard_control_mode();
		void publish_attitude_setpoint(double x, double y, double z, double yaw, double pitch, double roll);
		void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);

		void move(double x, double y, double z, double angle, double seconds);

		enum State {
			ARM,
			SEARCH,
			ALIGN,
			GATE,
			DISARM,
		};

		State currentState = State::ARM;

		bool foundTarget = false;

	public:
		CompRoutine() : Node("comp_routine") { // constructor
			offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
			attitude_setpoint_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
			vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

			timer_ =this->create_wall_timer(std::chrono::milliseconds(100), [this] {loop(); });
		}

		void arm() {
			publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

			RCLCPP_INFO(this->get_logger(), "Arm command send");
		}

		void disarm() {
			publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
		//    	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,0);
			RCLCPP_INFO(this->get_logger(), "Disarm command send");
		}

		void loop() {
			publish_offboard_control_mode();
			switch (currentState) {
				case ARM:
					publish_attitude_setpoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

					if (offboard_setpoint_counter_ == 10) {
						// Change to Offboard mode after 10 setpoints
						this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

						// Arm the vehicle
						this->arm();

						currentState = State::SEARCH;
					}

					offboard_setpoint_counter_++;
					break;
				case SEARCH:
					move(0.3, 0.0, 0.0, 4.27606, 10.0);
					std::cout << "SEARCH" << std::endl;
					break;
				case ALIGN:
					std::cout << "FOUND" << std::endl;
					break;
				case GATE:
					std::cout << "GATE" << std::endl;
					break;
				case DISARM:
					std::cout << "DISARM" << std::endl;
					break;
			}
		}
};

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void CompRoutine::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = false;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = true;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void CompRoutine::publish_attitude_setpoint(double x, double y, double z, double yaw, double pitch, double roll) {
	VehicleAttitudeSetpoint msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.thrust_body[0] = x; // x
	msg.thrust_body[1] = y; // y
	msg.thrust_body[2] = z; // z
	msg.yaw_body = yaw;
	msg.pitch_body = pitch;
	msg.roll_body = roll;

	attitude_setpoint_publisher_->publish(msg);
}

void CompRoutine::move(double x, double y, double z, double angle, double seconds) {
	auto start = std::chrono::steady_clock::now(); // Get the start time
    auto end = start + std::chrono::duration<double>(seconds); // Calculate the end time
    
    while (std::chrono::steady_clock::now() < end) {
       publish_attitude_setpoint(x, y, z, angle, 0.0, 0.0);
    }
}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void CompRoutine::publish_vehicle_command(uint16_t command, float param1, float param2)
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
	std::cout << "Starting comp routine..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CompRoutine>());

	rclcpp::shutdown();
	return 0;
}
