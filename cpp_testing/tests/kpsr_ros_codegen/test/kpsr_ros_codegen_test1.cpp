/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2031  Klepsydra Technologies AG
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies AG and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies AG and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies AG.
*
****************************************************************************/

#include <string>

#include "std_msgs/String.h"
#include <gtest/gtest.h>

#include <klepsydra/core/cache_listener.h>
#include <klepsydra/core/event_emitter_middleware_provider.h>

#include <kpsr_ros_serialization/primitive_type_ros_mapper.h>

#include <kpsr_ros_core/from_ros_middleware_provider.h>
#include <kpsr_ros_core/to_ros_middleware_provider.h>

#include "gps_ros_mapper.h"
#include "header_ros_mapper.h"
#include "imu_ros_mapper.h"
#include "pose_stamped_ros_mapper.h"
#include "quaternion_ros_mapper.h"
#include "vector3_ros_mapper.h"

TEST(KpsrRosCodegeTest, headerMapperTest)
{
    int argc = 0;
    char **argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Header> basicProvider(nullptr,
                                                                               "test",
                                                                               0,
                                                                               nullptr,
                                                                               nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider
        .registerToTopic<kpsr::geometry::Header, std_msgs::Header>("kpsr_ros_codegen_test_topicA",
                                                                   10,
                                                                   basicProvider.getPublisher());

    ros::Publisher stringPublisher =
        nodeHandle.advertise<std_msgs::Header>("kpsr_ros_codegen_test_topicA", 10, true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::geometry::Header> *kpsrPublisher =
        toRosProvider.getToMiddlewareChannel<kpsr::geometry::Header, std_msgs::Header>(
            "kpsr_ros_codegen_test_topicA", 10, nullptr, stringPublisher);

    kpsr::mem::CacheListener<kpsr::geometry::Header> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener",
                                                    cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    kpsr::geometry::Header event;

    int maxNumAttempts = 10;
    int numAttempts = 0;
    while ((numAttempts < maxNumAttempts) && (0 == stringPublisher.getNumSubscribers())) {
        numAttempts++;
        rate.sleep();
    }
    rate.sleep();
    EXPECT_LE(numAttempts, maxNumAttempts);
    // Test publishing event with non-default values.
    event.seq = 1141;
    event.frame_id = "hola.1141";

    for (int i = 0; i < 10; i++) {
        kpsrPublisher->publish(event);
        ros::spinOnce();
        rate.sleep();
    }

    numAttempts = 0;
    while (ros::ok() && (numAttempts++ < maxNumAttempts)) {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_LE(cacheListener.counter, 10);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->seq, event.seq);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->frame_id, event.frame_id);
}

TEST(KpsrRosCodegeTest, gpsMapperTest)
{
    int argc = 0;
    char **argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Gps> basicProvider(nullptr,
                                                                            "test",
                                                                            0,
                                                                            nullptr,
                                                                            nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<kpsr::geometry::Gps, sensor_msgs::NavSatFix>(
        "kpsr_ros_codegen_test_topicB", 10, basicProvider.getPublisher());

    ros::Publisher stringPublisher =
        nodeHandle.advertise<sensor_msgs::NavSatFix>("kpsr_ros_codegen_test_topicB", 10, true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::geometry::Gps> *kpsrPublisher =
        toRosProvider.getToMiddlewareChannel<kpsr::geometry::Gps, sensor_msgs::NavSatFix>(
            "kpsr_ros_codegen_test_topicB", 10, nullptr, stringPublisher);

    kpsr::mem::CacheListener<kpsr::geometry::Gps> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener",
                                                    cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    kpsr::geometry::Gps event;
    int maxNumAttempts = 10;
    int numAttempts = 0;
    while ((numAttempts < maxNumAttempts) && (0 == stringPublisher.getNumSubscribers())) {
        numAttempts++;
        rate.sleep();
    }
    rate.sleep();
    EXPECT_LE(numAttempts, maxNumAttempts);

    event.seq++;
    event.altitude = 4.0;
    event.latitude = 4.1;
    event.longitude = 4.2;
    for (int i = 0; i < 10; i++) {
        kpsrPublisher->publish(event);
        ros::spinOnce();
        rate.sleep();
    }

    numAttempts = 0;
    while (ros::ok() && (numAttempts++ < maxNumAttempts)) {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_LE(cacheListener.counter, 10);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->altitude, event.altitude);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->latitude, event.latitude);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->longitude, event.longitude);
}

TEST(KpsrRosCodegeTest, vector3MapperTest)
{
    int argc = 0;
    char **argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Vector3> basicProvider(nullptr,
                                                                                "test",
                                                                                0,
                                                                                nullptr,
                                                                                nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<kpsr::geometry::Vector3, geometry_msgs::Vector3>(
        "kpsr_ros_codegen_test_topicC", 10, basicProvider.getPublisher());

    ros::Publisher stringPublisher =
        nodeHandle.advertise<geometry_msgs::Vector3>("kpsr_ros_codegen_test_topicC", 10, true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::geometry::Vector3> *kpsrPublisher =
        toRosProvider.getToMiddlewareChannel<kpsr::geometry::Vector3, geometry_msgs::Vector3>(
            "kpsr_ros_codegen_test_topicC", 10, nullptr, stringPublisher);

    kpsr::mem::CacheListener<kpsr::geometry::Vector3> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener",
                                                    cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    kpsr::geometry::Vector3 event;
    int maxNumAttempts = 10;
    int numAttempts = 0;
    while ((numAttempts < maxNumAttempts) && (0 == stringPublisher.getNumSubscribers())) {
        numAttempts++;
        rate.sleep();
    }
    rate.sleep();
    EXPECT_LE(numAttempts, maxNumAttempts);

    event.seq = 114;
    event.x = 4.0;
    event.y = 4.1;
    event.z = 4.2;

    for (int i = 0; i < 10; i++) {
        kpsrPublisher->publish(event);
        ros::spinOnce();
        rate.sleep();
    }

    numAttempts = 0;
    while (ros::ok() && (numAttempts++ < maxNumAttempts)) {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_LE(cacheListener.counter, 10);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->x, event.x);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->y, event.y);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->z, event.z);
}

TEST(KpsrRosCodegeTest, quaternionMapperTest)
{
    int argc = 0;
    char **argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Quaternion> basicProvider(nullptr,
                                                                                   "test",
                                                                                   0,
                                                                                   nullptr,
                                                                                   nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<kpsr::geometry::Quaternion, geometry_msgs::Quaternion>(
        "kpsr_ros_codegen_test_topicD", 10, basicProvider.getPublisher());

    ros::Publisher stringPublisher =
        nodeHandle.advertise<geometry_msgs::Quaternion>("kpsr_ros_codegen_test_topicD", 10, true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::geometry::Quaternion> *kpsrPublisher =
        toRosProvider.getToMiddlewareChannel<kpsr::geometry::Quaternion, geometry_msgs::Quaternion>(
            "kpsr_ros_codegen_test_topicD", 10, nullptr, stringPublisher);

    kpsr::mem::CacheListener<kpsr::geometry::Quaternion> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener",
                                                    cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    kpsr::geometry::Quaternion event;
    int maxNumAttempts = 10;
    int numAttempts = 0;
    while ((numAttempts < maxNumAttempts) && (0 == stringPublisher.getNumSubscribers())) {
        numAttempts++;
        rate.sleep();
    }
    rate.sleep();
    EXPECT_LE(numAttempts, maxNumAttempts);

    event.seq = 15;
    event.x = 4.0;
    event.y = 4.1;
    event.z = 4.2;
    event.w = 4.3;

    for (int i = 0; i < 10; i++) {
        kpsrPublisher->publish(event);
        ros::spinOnce();
        rate.sleep();
    }

    numAttempts = 0;
    while (ros::ok() && (numAttempts++ < maxNumAttempts)) {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_LE(cacheListener.counter, 10);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->x, event.x);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->y, event.y);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->z, event.z);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->w, event.w);
}

TEST(KpsrRosCodegeTest, imuMapperTest)
{
    int argc = 0;
    char **argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Imu> basicProvider(nullptr,
                                                                            "test",
                                                                            0,
                                                                            nullptr,
                                                                            nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider
        .registerToTopic<kpsr::geometry::Imu, sensor_msgs::Imu>("kpsr_ros_codegen_test_topicE",
                                                                10,
                                                                basicProvider.getPublisher());
    rate.sleep();

    ros::Publisher stringPublisher =
        nodeHandle.advertise<sensor_msgs::Imu>("kpsr_ros_codegen_test_topicE", 10, true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::geometry::Imu> *kpsrPublisher =
        toRosProvider.getToMiddlewareChannel<kpsr::geometry::Imu, sensor_msgs::Imu>(
            "kpsr_ros_codegen_test_topicE", 10, nullptr, stringPublisher);

    kpsr::mem::CacheListener<kpsr::geometry::Imu> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener",
                                                    cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    int maxNumAttempts = 10;
    int numAttempts = 0;
    while ((numAttempts < maxNumAttempts) && (0 == stringPublisher.getNumSubscribers())) {
        numAttempts++;
        rate.sleep();
    }
    rate.sleep();
    EXPECT_LE(numAttempts, maxNumAttempts);
    unsigned int seq = 0;
    ros::spinOnce();
    rate.sleep();

    kpsr::geometry::Quaternion orientation(seq++, 40.1, 40.2, 40.3, 40.4);
    std::array<double, 9> orientation_covariance{
        {41.1, 41.2, 41.3, 41.4, 41.5, 41.6, 41.7, 41.8, 41.9}};
    kpsr::geometry::Vector3 angular_velocity(seq++, 30.5, 30.6, 30.7);
    std::array<double, 9> angular_velocity_covariance{
        {42.1, 42.2, 42.3, 42.4, 42.5, 42.6, 42.7, 42.8, 42.9}};
    kpsr::geometry::Vector3 linear_acceleration(seq++, 40.8, 40.9, 41.0);
    std::array<double, 9> linear_acceleration_covariance{
        {43.1, 43.2, 43.3, 43.4, 43.5, 43.6, 43.7, 43.8, 43.9}};

    kpsr::geometry::Imu event(seq++,
                              orientation,
                              orientation_covariance,
                              angular_velocity,
                              angular_velocity_covariance,
                              linear_acceleration,
                              linear_acceleration_covariance);

    for (int i = 0; i < 10; i++) {
        kpsrPublisher->publish(event);
        ros::spinOnce();
        rate.sleep();
    }

    numAttempts = 0;
    while (ros::ok() && (numAttempts++ < maxNumAttempts)) {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_LE(cacheListener.counter, 10);
    ASSERT_EQ(event.orientation.x, cacheListener.getLastReceivedEvent()->orientation.x);
    ASSERT_EQ(event.orientation.y, cacheListener.getLastReceivedEvent()->orientation.y);
    ASSERT_EQ(event.orientation.z, cacheListener.getLastReceivedEvent()->orientation.z);
    ASSERT_EQ(event.orientation.w, cacheListener.getLastReceivedEvent()->orientation.w);

    ASSERT_EQ(event.angular_velocity.x, cacheListener.getLastReceivedEvent()->angular_velocity.x);
    ASSERT_EQ(event.angular_velocity.y, cacheListener.getLastReceivedEvent()->angular_velocity.y);
    ASSERT_EQ(event.angular_velocity.z, cacheListener.getLastReceivedEvent()->angular_velocity.z);

    for (int i = 0; i < 9; i++) {
        ASSERT_EQ(event.orientation_covariance[i],
                  cacheListener.getLastReceivedEvent()->orientation_covariance[i]);
        ASSERT_EQ(event.angular_velocity_covariance[i],
                  cacheListener.getLastReceivedEvent()->angular_velocity_covariance[i]);
        ASSERT_EQ(event.linear_acceleration_covariance[i],
                  cacheListener.getLastReceivedEvent()->linear_acceleration_covariance[i]);
    }

    ASSERT_EQ(event.linear_acceleration.x,
              cacheListener.getLastReceivedEvent()->linear_acceleration.x);
    ASSERT_EQ(event.linear_acceleration.y,
              cacheListener.getLastReceivedEvent()->linear_acceleration.y);
    ASSERT_EQ(event.linear_acceleration.z,
              cacheListener.getLastReceivedEvent()->linear_acceleration.z);
}

TEST(KpsrRosCodegeTest, poseStampedTest)
{
    int argc = 0;
    char **argv = nullptr;

    ros::init(argc, argv, "kpsr_ros_codegen_test");
    ros::NodeHandle nodeHandle;
    ros::Rate rate(100);

    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::PoseStamped> basicProvider(nullptr,
                                                                                    "test",
                                                                                    0,
                                                                                    nullptr,
                                                                                    nullptr);

    kpsr::ros_mdlw::FromRosMiddlewareProvider fromRosProvider(nodeHandle);
    fromRosProvider.registerToTopic<kpsr::geometry::PoseStamped, geometry_msgs::PoseStamped>(
        "kpsr_ros_codegen_test_topicE", 10, basicProvider.getPublisher());
    rate.sleep();

    ros::Publisher stringPublisher =
        nodeHandle.advertise<geometry_msgs::PoseStamped>("kpsr_ros_codegen_test_topicE", 10, true);

    kpsr::ros_mdlw::ToRosMiddlewareProvider toRosProvider(nullptr);

    kpsr::Publisher<kpsr::geometry::PoseStamped> *kpsrPublisher =
        toRosProvider.getToMiddlewareChannel<kpsr::geometry::PoseStamped, geometry_msgs::PoseStamped>(
            "kpsr_ros_codegen_test_topicE", 1, nullptr, stringPublisher);

    kpsr::mem::CacheListener<kpsr::geometry::PoseStamped> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener",
                                                    cacheListener.cacheListenerFunction);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_EQ(cacheListener.counter, 0);

    int maxNumAttempts = 10;
    int numAttempts = 0;
    while ((numAttempts < maxNumAttempts) && (0 == stringPublisher.getNumSubscribers())) {
        numAttempts++;
        rate.sleep();
    }
    rate.sleep();
    EXPECT_LE(numAttempts, maxNumAttempts);
    int num_events = 5;
    for (int i = 0; i < num_events; i++) {
        kpsr::geometry::PoseStamped event;

        event.header.seq = 2432;
        event.header.frame_id = "hola." + std::to_string(i);
        event.pose.position.x = 1;
        event.pose.position.y = 2;
        event.pose.position.z = 3;
        event.pose.orientation.seq = 1;
        event.pose.orientation.x = 0.0;
        event.pose.orientation.y = 0.1;
        event.pose.orientation.z = 0.2;
        event.pose.orientation.w = 0.3;
        kpsrPublisher->publish(event);
        ros::spinOnce();
        rate.sleep();
    }

    numAttempts = 0;
    while (ros::ok() && (numAttempts++ < maxNumAttempts)) {
        ros::spinOnce();
        rate.sleep();
    }

    ASSERT_LE(cacheListener.counter, num_events);

    auto lastEvent = cacheListener.getLastReceivedEvent();
    EXPECT_EQ(lastEvent->header.seq,
              num_events - 1); // seq number is automatically filled by ROS for pose stamped
    EXPECT_EQ(lastEvent->header.frame_id, "hola." + std::to_string(num_events - 1));
}
