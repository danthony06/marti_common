// *****************************************************************************
//
// Copyright (c) 2018, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <swri_roscpp/msg/test_topic_service_request.hpp>
#include <swri_roscpp/msg/test_topic_service_response.hpp>

#include <swri_roscpp/topic_service_client.h>
#include <swri_roscpp/topic_service_server.h>

#include <thread>


namespace swri_roscpp
{
  /*
   * This class would normally be auto-generated by the add_topic_service_files
   * CMake macro, but we can't use that in swri_roscpp's own tests because the
   * .cmake file will not have been generated and installed at the time the
   * tests are run.
   */
  namespace msg
  {
    class TestTopicService
    {
    public:
      typedef TestTopicServiceResponse Response;
      typedef TestTopicServiceRequest Request;

      Request request;
      Response response;
    };
  }

  static const std::string topic_name = "/test_topic_service";

  static const size_t value_count = 5;
  static const int test_values[] = {5, 10, 100, 10000, 50000};

  /*
   * Class used by the server to respond to service requests
   */
  class TopicServiceHandler
  {
  public:
    TopicServiceHandler(rclcpp::Node::SharedPtr node) :
      node_(node),
      call_count_(0),
      error_(false),
      is_running_(true)
    {}

    bool handleTopicServiceRequest(const swri_roscpp::msg::TestTopicServiceRequest& req,
                                   swri_roscpp::msg::TestTopicServiceResponse& resp)
    {
      RCLCPP_INFO(node_->get_logger(), "TopicServiceHandler::handleTopicServiceRequest");
      resp.response_value = req.request_value;

      if (call_count_ >= value_count || (test_values[call_count_] != req.request_value))
      {
        error_ = true;
        return false;
      }

      if (req.request_value == test_values[value_count-1])
      {
        is_running_ = false;
      }

      call_count_++;

      return is_running_;
    }

    rclcpp::Node::SharedPtr node_;
    int call_count_;
    bool error_;
    bool is_running_;
  };
}

class TopicServiceServerTests : public rclcpp::Node
{
public:
  TopicServiceServerTests() :
    rclcpp::Node("topic_service_server_test")
  {}

  void DoStuff()
  {
    swri_roscpp::TopicServiceHandler handler(this->shared_from_this());

    swri::TopicServiceServer server;
    server.initialize(
        this->shared_from_this(),
        swri_roscpp::topic_name,
        &swri_roscpp::TopicServiceHandler::handleTopicServiceRequest,
        &handler);

    RCLCPP_INFO(this->get_logger(), "Initializing server.");

    rclcpp::Rate rate(50);
    rclcpp::Time start = this->now();
    // Wait up to 20s for the client to complete; it should be much faster than that
    while (handler.is_running_ && (this->now() - start) < std::chrono::seconds(20))
    {
      // If the server encounters any errors, it will set error_ to true
      //ASSERT_FALSE(handler.error_);
      rclcpp::spin_some(this->shared_from_this());
      rate.sleep();
    }

    RCLCPP_INFO(this->get_logger(), "Server is exiting.");
  }
};

class TopicServiceClientTests : public rclcpp::Node
{
public:
  TopicServiceClientTests() :
      rclcpp::Node("topic_service_client_test")
  {}

  void DoStuff()
  {
    swri::TopicServiceClient<swri_roscpp::msg::TestTopicService> client;
    client.initialize(this->shared_from_this(), swri_roscpp::topic_name, "test_client");

    client.wait_for_service(std::chrono::seconds(1));
    ASSERT_TRUE(client.exists());

    swri_roscpp::msg::TestTopicService srv;

    // Iterate through our tests values and test submitting all of them
    for (size_t i = 0; i < swri_roscpp::value_count; i++)
    {
      srv.request.request_value = swri_roscpp::test_values[i];
      bool result = client.call(srv);

      if (i + 1 < swri_roscpp::value_count)
      {
        ASSERT_TRUE(result);
      }
      else
      {
        // The very last value should cause the server to return false
        ASSERT_FALSE(result);
      }
      ASSERT_EQ(swri_roscpp::test_values[i], srv.response.response_value);
    }
  }
};

TEST(SwriRoscppTests, TopicServiceClient)
{
  auto client = std::shared_ptr<TopicServiceClientTests>(new TopicServiceClientTests);

  client->DoStuff();
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);

  rclcpp::init(argc, argv);

  std::shared_ptr<TopicServiceServerTests> server(new TopicServiceServerTests);

  std::thread server_thread([&]()
  {
    server->DoStuff();
  });

  int res = RUN_ALL_TESTS();

  server_thread.join();

  return res;
}
