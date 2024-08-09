#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_attitude_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <stdint.h>

#include <chrono>
#include <iostream>
#include <unistd.h>

using namespace std;

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class CompRoutine : public rclcpp::Node {
	private:
		rclcpp::TimerBase::SharedPtr timer_;

		rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
		rclcpp::Publisher<VehicleAttitudeSetpoint>::SharedPtr attitude_setpoint_publisher_;
		rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
		rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscriber_;
		rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr buoy_detector_subscriber_;

		std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

		uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

		void publish_offboard_control_mode();
		void publish_attitude_setpoint(double x, double y, double z, double yaw, double pitch, double roll);
		void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
		
		double toRadians(double degree);
		double toDegrees(double radians);

		double searchDirection = 0.0;

		void move(double x, double y, double z, double angle, double seconds);
		void wait(double seconds);

		enum State {
			INIT,
			WAIT,
			DESCEND,
			SEARCH_FOR_BUOY,
			BUOY,
			GATE,
			STYLE_YAW,
			STYLE_ROLL,
			DISARM,
			IDLE
		};

		State currentState = State::INIT;

		bool armed_ = false;
		bool shutdown_ = false;

		void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg) {
			// look for arming state
			if (msg->arming_state == 2) {
				armed_ = true;
			}
		}

		double downPower = 0.15;
		double descendPower = 0.25;
		double gateTime = 16.0;
		double waitTime = 10.0;
		double armWaitTime = 20.0;

		bool foundBuoy = false;
		double yPixelError = 0.0;
		double zPixelError = 0.0;

		double forwardHeading = 0.0;
		double currentHeading_ = 0.0;
		int search_counter = 0;
		int min_search_angle = 0; // whatever you put here must be divisble by the search_incrementor
		int max_search_angle = 0;
		int search_incrementor = 10;

		double bestHeading = 0.0;
		double bestHeadingYError = 0.0;

		double calculatedBuoyHeading = 0.0;
		double styleHeading = currentHeading_;

		int styleCount = 0;

		void buoy_detector_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
			yPixelError = (640/2) - msg->x;
			zPixelError = (480/2) - msg->y;
			
			if ((currentState == SEARCH_FOR_BUOY && foundBuoy != true) || abs(yPixelError) < abs(bestHeadingYError)) {
				foundBuoy = true;

				bestHeading = currentHeading_;
				bestHeadingYError = abs(yPixelError);
			}

			// if (abs(yPixelError) < 50) {
			if (yPixelError > 0) { // positive
				calculatedBuoyHeading = currentHeading_ - 1;
			} else {
				calculatedBuoyHeading = currentHeading_ + 1;
			}
			// }
			
			if (currentState == BUOY) {
				std::cout << "calculatedBuoyHeading (deg): " << calculatedBuoyHeading << std::endl;
			}
		}

	public:
		CompRoutine() : Node("comp_routine") { // constructor
			rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
			qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
			qos_profile.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
			qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_LAST);
			qos_profile.keep_last(1);

			offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
			attitude_setpoint_publisher_ = this->create_publisher<VehicleAttitudeSetpoint>("/fmu/in/vehicle_attitude_setpoint", 10);
			vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);
			vehicle_status_subscriber_ = this->create_subscription<VehicleStatus>("/fmu/out/vehicle_status", qos_profile, std::bind(&CompRoutine::vehicle_status_callback, this, std::placeholders::_1));
			buoy_detector_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>("/contour_center", 10, std::bind(&CompRoutine::buoy_detector_callback, this, std::placeholders::_1));

			offboard_setpoint_counter_ = 0;

			this->declare_parameter<double>("forwardHeading", 0.0);
			this->declare_parameter<double>("downPower", 0.2);
			this->declare_parameter<double>("descendPower", 0.15);
			this->declare_parameter<double>("gateTime", 16.0);
			this->declare_parameter<double>("waitTime", 10.0);
			this->declare_parameter<double>("armWaitTime", 20.0);

			forwardHeading = this->get_parameter("forwardHeading").as_double();
			downPower = this->get_parameter("downPower").as_double();
			descendPower = this->get_parameter("descendPower").as_double();
			gateTime = this->get_parameter("gateTime").as_double();
			waitTime = this->get_parameter("waitTime").as_double();
			armWaitTime = this->get_parameter("armWaitTime").as_double();

			currentHeading_ = forwardHeading;
			min_search_angle = forwardHeading - 40; // whatever you put here must be divisble by the search_incrementor
			max_search_angle = forwardHeading + 40;
			searchDirection = min_search_angle;

			auto timer_callback = [this]() -> void {
				// After 10 iterations (1 second) switch to offboard and ARM.  Keep sending the ARM command until the vehicle is ARMed
				// If the vehicle is slightly moving it will not ARM so we will keep trying

				if ((offboard_setpoint_counter_ >= armWaitTime*10) && (!armed_)) {
					RCLCPP_INFO(this->get_logger(), "Offboard command send");
					// Change to Offboard mode after 10 setpoints
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);

					// Arm the vehicle
					this->arm();
				}

				// offboard_control_mode needs and to be constantly published and you cannot switch to offboard mode until is being sent
				if (!shutdown_) {
					publish_offboard_control_mode();
				}

				if (!armed_) {
					publish_attitude_setpoint(0.0, 0.0, 0.0, forwardHeading, 0.0, 0.0);
			
					if (offboard_setpoint_counter_ > (armWaitTime*10 + 200)) {
						shutdown_ = true;
					} 
				} else {
					run_state_machine();
				}

				if (shutdown_) {
					this->disarm();
					rclcpp::shutdown();
				}

				offboard_setpoint_counter_++;
			};

			timer_ = this->create_wall_timer(100ms, timer_callback); // TODO: fix and exit callback and use while true
			
		}

		~CompRoutine(){
			RCLCPP_INFO(this->get_logger(), "DESTRUCTOR");

			publish_attitude_setpoint(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
			shutdown_ = true;
			this->disarm();
			sleep(10);
			RCLCPP_INFO(this->get_logger(), "DONE SLEEPING");
		};

		void arm() {
			publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
			RCLCPP_INFO(this->get_logger(), "ARM command send");
		}

		void disarm() {
			// publish_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0);
			// publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_LAND_START, 0.0);
			publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 21196);
			RCLCPP_INFO(this->get_logger(), "DISARM command send");
		}

		std::chrono::steady_clock::time_point start;
		std::chrono::steady_clock::time_point start_of_style;

		void run_state_machine() {
			switch (currentState) {
				case INIT:
					start = std::chrono::steady_clock::now(); // Get the start time
					std::cout << "GOING INTO WAIT" << std::endl;
					currentState = State::WAIT;
					break;
				case WAIT:
					publish_attitude_setpoint(0.0, 0.0, 0.0, forwardHeading, 0.0, 0.0);

					if (std::chrono::steady_clock::now() > start + std::chrono::duration<double>(waitTime)) { // check to see if we have waited 10 secs
						start = std::chrono::steady_clock::now(); // Reset the start time
						std::cout << "GOING INTO DESCEND" << std::endl;
						currentState = State::DESCEND;
					}
					break;
				case DESCEND:
				std::cout << "descendPower: " << descendPower << std::endl;
					publish_attitude_setpoint(0.0, 0.0, descendPower, forwardHeading, 0.0, 0.0);
					if (std::chrono::steady_clock::now() > start + std::chrono::duration<double>(5)) { // check to see if we have waited 15 secs
						start = std::chrono::steady_clock::now(); // Reset the start time
						std::cout << "GOING INTO GATE" << std::endl;
						currentState = State::GATE; // TODO: change to GATE
					}
					break;
				case GATE:
					publish_attitude_setpoint(0.3, 0.0, downPower, forwardHeading, 0.0, 0.0);
					if (std::chrono::steady_clock::now() > start + std::chrono::duration<double>(gateTime)) { // check to see if we have waited 15 secs
						start = std::chrono::steady_clock::now(); // Reset the start time
						std::cout << "GOING INTO STYLE_YAW" << std::endl;

						start_of_style = std::chrono::steady_clock::now();
						styleHeading = currentHeading_;
						currentState = State::STYLE_YAW;
					}
					break;
				case STYLE_YAW:
					if (std::chrono::steady_clock::now() > start_of_style + std::chrono::duration<double>(0.5)) { 
						start_of_style = std::chrono::steady_clock::now();
						styleHeading += 50;
					}

					publish_attitude_setpoint(0.0, 0.0, downPower, styleHeading, 0.0, 0.0);

					if (std::chrono::steady_clock::now() > start + std::chrono::duration<double>(9)) { 
						start = std::chrono::steady_clock::now(); // Reset the start time
						styleHeading = 0.0;
						start_of_style = std::chrono::steady_clock::now();
						std::cout << "GOING INTO SEARCH FOR BUOY" << std::endl;

						currentState = State::STYLE_ROLL;
					}
					break;
				case STYLE_ROLL:
					if (std::chrono::steady_clock::now() > start_of_style + std::chrono::duration<double>(0.5)) { 
						start_of_style = std::chrono::steady_clock::now();
						styleHeading += 50;
						std::cout << "styleHeading: " << styleHeading << std::endl;
					}

					publish_attitude_setpoint(0.0, 0.0, downPower, 0.0, 0.0, styleHeading);

					if (std::chrono::steady_clock::now() > start + std::chrono::duration<double>(13.5)) { 
						start = std::chrono::steady_clock::now(); // Reset the start time
						std::cout << "GOING INTO SEARCH FOR BUOY" << std::endl;
						currentState = State::SEARCH_FOR_BUOY;
					}	
					break;
				case SEARCH_FOR_BUOY:
					publish_attitude_setpoint(0.0, 0.0, downPower, searchDirection, 0.0, 0.0);

					if (std::chrono::steady_clock::now() > start + std::chrono::duration<double>(5)) { 
						start = std::chrono::steady_clock::now(); // Reset the start time
						searchDirection += search_incrementor;
						if (searchDirection >= max_search_angle) { // we've finished the search pattern
							searchDirection = min_search_angle;
							if (foundBuoy) {
								start = std::chrono::steady_clock::now(); // Reset the start time
								std::cout << "GOING INTO BUOY" << std::endl;
								publish_attitude_setpoint(0.0, 0.0, downPower, bestHeading, 0.0, 0.0);
								calculatedBuoyHeading = bestHeading;
								currentState = State::BUOY;
							}
						}
					}
					break;
				case BUOY:
					publish_attitude_setpoint(0.2, 0.0, downPower, calculatedBuoyHeading, 0.0, 0.0);

					if (std::chrono::steady_clock::now() > start + std::chrono::duration<double>(60)) {
						start = std::chrono::steady_clock::now(); // Reset the start time
						std::cout << "GOING INTO IDLE" << std::endl;
						currentState = State::IDLE;
					}
					break;
				case IDLE:
					std::cout << "IDLE" << std::endl;
					publish_attitude_setpoint(0.0, 0.0, 0.0, forwardHeading, 0.0, 0.0);

					if (std::chrono::steady_clock::now() > start + std::chrono::duration<double>(2)) { 
						start = std::chrono::steady_clock::now(); // Reset the start time
						currentState = State::DISARM;
					}
					break;
				case DISARM:
					std::cout << "DISARM" << std::endl;
					shutdown_ = true;
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

double lastYaw = -1000;
void CompRoutine::publish_attitude_setpoint(double x, double y, double z, double yaw, double pitch, double roll) {
	VehicleAttitudeSetpoint msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.thrust_body[0] = x; // x
	msg.thrust_body[1] = y; // y
	msg.thrust_body[2] = z; // z
	msg.yaw_body = toRadians(yaw);
	msg.pitch_body = toRadians(pitch);
	msg.roll_body = toRadians(roll);

	currentHeading_ = yaw;
	attitude_setpoint_publisher_->publish(msg);

	if (lastYaw != yaw) {
		std::cout << "currentTargetHeading (deg): " << yaw << std::endl;
		lastYaw = yaw;
	}
}


void CompRoutine::move(double x, double y, double z, double angle, double seconds) {
	auto start = std::chrono::steady_clock::now(); // Get the start time
    auto end = start + std::chrono::duration<double>(seconds); // Calculate the end time
	currentHeading_ = angle;
    
    while (std::chrono::steady_clock::now() < end) {
		std::cout << "in moving. Angle is: " << angle << std::endl;
		publish_offboard_control_mode();
		publish_attitude_setpoint(x, y, z, angle, 0.0, 0.0);
    }
}

void CompRoutine::wait(double seconds) {
	auto start = std::chrono::steady_clock::now(); // Get the start time
    auto end = start + std::chrono::duration<double>(seconds); // Calculate the end time
    
    while (std::chrono::steady_clock::now() < end) {
		std::cout << "waiting" << std::endl;
		publish_offboard_control_mode();
    }
}

double CompRoutine::toRadians(double degrees) {
	double pi = 3.14159265359;
	return degrees * (pi / 180);
}

double CompRoutine::toDegrees(double radians) {
	double pi = 3.14159265359;
	return radians * (180 / pi);
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

// void mySigintHandler(int sig) {
// 	std::cout << "Sig Recieved: " << sig << std::endl;
// 	rclcpp::shutdown();
// }

double inputHeading = 0;
int main(int argc, char *argv[]) {
	std::cout << "Starting comp routine..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	// signal(SIGINT, mySigintHandler);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<CompRoutine>());

	rclcpp::shutdown();
	return 0;
}
