// Copyright 2023 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>

int main(int argc, char const *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("receiver");
  double time_between_publishes = node->declare_parameter("time_between_publishes", 3.0);
  auto callback = [node, &time_between_publishes](
    const std_msgs::msg::Header & msg,
    const rclcpp::MessageInfo & message_info
  ) {
    auto now = rclcpp::Clock().now();
    auto time_received = rclcpp::Time(message_info.get_rmw_message_info().received_timestamp);
    auto time_sent = rclcpp::Time(msg.stamp, RCL_SYSTEM_TIME);
    auto transit_time = time_received - time_sent;
    auto time_waited_for_processing = now - time_received;
    RCLCPP_INFO_STREAM(
      node->get_logger(),
      "Received '" << msg.frame_id << "' msg" <<
      " at '" << std::fixed << time_received.seconds() << "'s" <<
      " - sent at '" << time_sent.seconds() << "'s" <<
      " = transit time of '" << transit_time.seconds() << "'s"
      ", and processed at '" << now.seconds() <<
      "', wait time: " << time_waited_for_processing.seconds());
    if (msg.frame_id == "Hello World: 7") {
      //std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    if (time_waited_for_processing.seconds() > (time_between_publishes * 0.75)) {
      RCLCPP_ERROR(node->get_logger(), "Took too long to process message.");
      rclcpp::shutdown();
    }
  };
  rclcpp::SubscriptionOptions so;
  //so.event_callbacks.matched_callback = [node](const rclcpp::MatchedInfo & matched_info) {
  //  RCLCPP_INFO_STREAM(
  //    node->get_logger(),
  //    "Matched publisher event, " <<
  //    "total_count: '" << matched_info.total_count << "', " <<
  //    "total_count_change: '" << matched_info.total_count_change << "', " <<
  //    "current_count: '" << matched_info.current_count << "', " <<
  //    "current_count_change: '" << matched_info.current_count_change << "'");
  //};
  auto subscription = node->create_subscription<std_msgs::msg::Header>("data", 10, callback, so);

  rclcpp::spin(node);

  return 0;
}
