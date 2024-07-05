#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>

using std::placeholders::_1;

class BagRec : public rclcpp::Node
{
public:
	BagRec() : Node("bag_rec")
	{
		// Node Parameters
		this->declare_parameter<std::string>("bags_dir", "/fred/ros2_demo_ws/bags/");
		this->get_parameter("bags_dir", bags_dir_);

		// String formatting for timestamped filename
		std::ostringstream oss;
		auto now = std::chrono::system_clock::now();
		std::time_t now_time_t = std::chrono::system_clock::to_time_t(now);
		std::tm now_tm = *std::localtime(&now_time_t);
		oss << bags_dir_ << "bag_" << std::put_time(&now_tm, "%Y%m%d_%H%M%S");
		std::string bag_fn_ = oss.str();

		const rosbag2_cpp::StorageOptions storage_options({bag_fn_, "sqlite3"});
		const rosbag2_cpp::ConverterOptions converter_options({rmw_get_serialization_format(),rmw_get_serialization_format()});
		writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
		writer_->open(storage_options, converter_options);

		writer_->create_topic({"/fmu/out/vehicle_local_position", "px4_msgs/msg/VehicleLocalPosition", rmw_get_serialization_format(), ""});
		subscription_ = create_subscription<std_msgs::msg::String>("/fmu/out/vehicle_local_position", 10, std::bind(&BagRec::topic_callback, this, _1));
	}

private:
	void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
	{
		auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

		bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
				new rcutils_uint8_array_t,
				[this] (rcutils_uint8_array_t *msg) {
					auto fini_return = rcutils_uint8_array_fini(msg);
					delete msg;
					if (fini_return != RCUTILS_RET_OK) {
						RCLCPP_ERROR(get_logger(), "Failed to destroy serialized message %s", rcutils_get_error_string().str);
					}
				});
		*bag_message->serialized_data = msg->release_rcl_serialized_message();

		bag_message->topic_name = "/fmu/out/vehicle_local_position";
		if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK) {
			RCLCPP_ERROR(get_logger(), "Error getting current time: %s", rcutils_get_error_string().str);
		}

		writer_->write(bag_message);
	}

	rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr subscription_;
	std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
	std::string bags_dir_;
};

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<BagRec>());
	rclcpp::shutdown();
	return 0;
}

