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

#include <gtest/gtest.h>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

#include <zmq.hpp>

#include <klepsydra/serialization/json_cereal_mapper.h>
#include <klepsydra/serialization/binary_cereal_mapper.h>

#include <klepsydra/core/event_emitter_middleware_provider.h>
#include <klepsydra/core/cache_listener.h>

#include <klepsydra/zmq_core/to_zmq_middleware_provider.h>
#include <klepsydra/zmq_core/from_zmq_middleware_provider.h>

#include <klepsydra/codegen/cereal/header_serializer.h>
#include <klepsydra/codegen/cereal/gps_serializer.h>
#include <klepsydra/codegen/cereal/vector3_serializer.h>
#include <klepsydra/codegen/cereal/quaternion_serializer.h>
#include <klepsydra/codegen/cereal/imu_serializer.h>

TEST(KpsrZmqCodegeTest, headerMapperTest) {
    std::string serverUrl = "tcp://*:5556";
    std::string topic = "test";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://test.ipc");

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQProvider(nullptr, publisher);

    std::string clientUrl = "tcp://localhost:5556";

    //  Socket to talk to server
    spdlog::info("Collecting updates from zmq server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    auto fromZMQProvider =
            _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<kpsr::geometry::Header>(subscriber, 10);
    fromZMQProvider->start();

    kpsr::Publisher<kpsr::geometry::Header> * kpsrPublisher =
            toZMQProvider.getJsonToMiddlewareChannel<kpsr::geometry::Header>(topic, 0);

    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Header> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    fromZMQProvider->registerToTopic(topic, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::geometry::Header> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    kpsr::geometry::Header event;

    event.seq = 1;
    event.frame_id = "hola.1";
    while (cacheListener.counter < 1) {
        kpsrPublisher->publish(event);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    event.seq = 2;
    event.frame_id = "hola.2";
    kpsrPublisher->publish(event);

    event.seq = 3;
    event.frame_id = "hola.3";
    kpsrPublisher->publish(event);

    event.seq = 4;
    event.frame_id = "hola.4";
    kpsrPublisher->publish(event);

    event.seq = 5;
    event.frame_id = "hola.5";
    kpsrPublisher->publish(event);

    while (cacheListener.counter < 5) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    fromZMQProvider->stop();

    ASSERT_GE(cacheListener.counter, 5);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->seq, 5);
    ASSERT_EQ(cacheListener.getLastReceivedEvent()->frame_id, "hola.5");
}

TEST(KpsrZmqCodegeTest, gpsMapperTest) {
    std::string serverUrl = "tcp://*:5556";
    std::string topic = "test";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://test.ipc");

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQProvider(nullptr, publisher);

    std::string clientUrl = "tcp://localhost:5556";

    //  Socket to talk to server
    spdlog::info("Collecting updates from test server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    auto fromZMQProvider =
            _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<kpsr::geometry::Gps>(subscriber, 10);
    fromZMQProvider->start();

    kpsr::Publisher<kpsr::geometry::Gps> * kpsrPublisher =
            toZMQProvider.getJsonToMiddlewareChannel<kpsr::geometry::Gps>(topic, 0);

    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Gps> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    fromZMQProvider->registerToTopic(topic, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::geometry::Gps> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);
    ASSERT_EQ(cacheListener.counter, 0);

    kpsr::geometry::Gps event;

    event.seq = 1;
    event.altitude = 0.0;
    event.latitude = 0.1;
    event.longitude = 0.2;
    while (cacheListener.counter < 1) {
        kpsrPublisher->publish(event);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    event.seq++;
    event.altitude = 1.0;
    event.latitude = 1.1;
    event.longitude = 1.2;
    kpsrPublisher->publish(event);

    event.seq++;
    event.altitude = 2.0;
    event.latitude = 2.1;
    event.longitude = 2.2;
    kpsrPublisher->publish(event);

    event.seq++;
    event.altitude = 3.0;
    event.latitude = 3.1;
    event.longitude = 3.2;
    kpsrPublisher->publish(event);

    event.seq++;
    event.altitude = 4.0;
    event.latitude = 4.1;
    event.longitude = 4.2;
    kpsrPublisher->publish(event);

    while (cacheListener.counter < 5) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    fromZMQProvider->stop();

    ASSERT_GE(cacheListener.counter, 5);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->seq, event.seq);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->altitude, 4.0);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->latitude, 4.1);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->longitude, 4.2);
}


TEST(KpsrZmqCodegeTest, vector3MapperTest) {
    std::string serverUrl = "tcp://*:5556";
    std::string topic = "test";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://test.ipc");

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQProvider(nullptr, publisher);

    std::string clientUrl = "tcp://localhost:5556";

    //  Socket to talk to server
    spdlog::info("Collecting updates from test server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    auto fromZMQProvider =
            _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<kpsr::geometry::Vector3>(subscriber, 10);
    fromZMQProvider->start();

    kpsr::Publisher<kpsr::geometry::Vector3> * kpsrPublisher =
            toZMQProvider.getJsonToMiddlewareChannel<kpsr::geometry::Vector3>(topic, 0);

    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Vector3> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    fromZMQProvider->registerToTopic(topic, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::geometry::Vector3> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    kpsr::geometry::Vector3 event;

    event.seq = 1;
    event.x = 0.0;
    event.y = 0.1;
    event.z = 0.2;
    while (cacheListener.counter < 1) {
        kpsrPublisher->publish(event);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    event.seq++;
    event.x = 1.0;
    event.y = 1.1;
    event.z = 1.2;
    kpsrPublisher->publish(event);

    event.seq++;
    event.x = 2.0;
    event.y = 2.1;
    event.z = 2.2;
    kpsrPublisher->publish(event);

    event.seq++;
    event.x = 3.0;
    event.y = 3.1;
    event.z = 3.2;
    kpsrPublisher->publish(event);

    event.seq++;
    event.x = 4.0;
    event.y = 4.1;
    event.z = 4.2;
    kpsrPublisher->publish(event);

    while (cacheListener.counter < 5) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    fromZMQProvider->stop();

    ASSERT_GE(cacheListener.counter, 5);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->x, 4.0);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->y, 4.1);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->z, 4.2);
}

TEST(KpsrZmqCodegeTest, quaternionMapperTest) {
    std::string serverUrl = "tcp://*:5556";
    std::string topic = "test";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://test.ipc");

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQProvider(nullptr, publisher);

    std::string clientUrl = "tcp://localhost:5556";

    //  Socket to talk to server
    spdlog::info("Collecting updates from test server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    auto fromZMQProvider =
            _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<kpsr::geometry::Quaternion>(subscriber, 10);
    fromZMQProvider->start();

    kpsr::Publisher<kpsr::geometry::Quaternion> * kpsrPublisher =
            toZMQProvider.getJsonToMiddlewareChannel<kpsr::geometry::Quaternion>(topic, 0);

    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Quaternion> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    fromZMQProvider->registerToTopic(topic, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::geometry::Quaternion> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    kpsr::geometry::Quaternion event;

    event.seq = 1;
    event.x = 0.0;
    event.y = 0.1;
    event.z = 0.2;
    event.w = 0.3;
    while (cacheListener.counter < 1) {
        kpsrPublisher->publish(event);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    event.seq ++;
    event.x = 1.0;
    event.y = 1.1;
    event.z = 1.2;
    event.w = 1.3;
    kpsrPublisher->publish(event);

    event.seq ++;
    event.x = 2.0;
    event.y = 2.1;
    event.z = 2.2;
    event.w = 2.3;
    kpsrPublisher->publish(event);

    event.seq ++;
    event.x = 3.0;
    event.y = 3.1;
    event.z = 3.2;
    event.w = 3.3;
    kpsrPublisher->publish(event);

    event.seq ++;
    event.x = 4.0;
    event.y = 4.1;
    event.z = 4.2;
    event.w = 4.3;
    kpsrPublisher->publish(event);

    while (cacheListener.counter < 5) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    fromZMQProvider->stop();

    ASSERT_GE(cacheListener.counter, 5);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->x, 4.0);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->y, 4.1);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->z, 4.2);
    ASSERT_FLOAT_EQ(cacheListener.getLastReceivedEvent()->w, 4.3);
}

TEST(KpsrZmqCodegeTest, imuMapperTest) {
    std::string serverUrl = "tcp://*:5556";
    std::string topic = "test";

    //  Prepare our context and publisher
    zmq::context_t context (1);
    zmq::socket_t publisher (context, ZMQ_PUB);
    publisher.bind(serverUrl);
    publisher.bind("ipc://test.ipc");

    kpsr::zmq_mdlw::ToZMQMiddlewareProvider toZMQProvider(nullptr, publisher);

    std::string clientUrl = "tcp://localhost:5556";

    //  Socket to talk to server
    spdlog::info("Collecting updates from test server...\n");
    zmq::socket_t subscriber (context, ZMQ_SUB);

    subscriber.connect(clientUrl);
    subscriber.setsockopt(ZMQ_SUBSCRIBE, topic.c_str(), topic.size());

    //  Process 100 updates
    kpsr::zmq_mdlw::FromZmqMiddlewareProvider _fromZmqMiddlewareProvider;
    auto fromZMQProvider =
            _fromZmqMiddlewareProvider.getJsonFromMiddlewareChannel<kpsr::geometry::Imu>(subscriber, 10);
    fromZMQProvider->start();

    kpsr::Publisher<kpsr::geometry::Imu> * kpsrPublisher =
            toZMQProvider.getJsonToMiddlewareChannel<kpsr::geometry::Imu>(topic, 0);

    kpsr::EventEmitterMiddlewareProvider<kpsr::geometry::Imu> basicProvider(nullptr, "test", 0, nullptr, nullptr);

    fromZMQProvider->registerToTopic(topic, basicProvider.getPublisher());

    kpsr::mem::CacheListener<kpsr::geometry::Imu> cacheListener;
    basicProvider.getSubscriber()->registerListener("cacheListener", cacheListener.cacheListenerFunction);

    ASSERT_EQ(cacheListener.counter, 0);

    unsigned int seq = 0;
    while (cacheListener.counter < 1) {
        kpsr::geometry::Quaternion orientation(seq++, 0.1, 0.2, 0.3, 0.4);
        std::array<double, 9> orientation_covariance{{1.1, 1.2, 1.3, 1.4, 1.5, 1.6, 1.7, 1.8, 1.9}};
        kpsr::geometry::Vector3 angular_velocity(seq++, 0.5, 0.6, 0.7);
        std::array<double, 9> angular_velocity_covariance{{2.1, 2.2, 2.3, 2.4, 2.5, 2.6, 2.7, 2.8, 2.9}};
        kpsr::geometry::Vector3 linear_acceleration(seq++, 0.8, 0.9, 1.0);
        std::array<double, 9> linear_acceleration_covariance{{3.1, 3.2, 3.3, 3.4, 3.5, 3.6, 3.7, 3.8, 3.9}};

        kpsr::geometry::Imu event(seq++, orientation, orientation_covariance, angular_velocity,
                                  angular_velocity_covariance, linear_acceleration,
                                  linear_acceleration_covariance);
        kpsrPublisher->publish(event);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
        kpsr::geometry::Quaternion orientation(seq++, 10.1, 10.2, 10.3, 10.4);
        std::array<double, 9> orientation_covariance{{11.1, 11.2, 11.3, 11.4, 11.5, 11.6, 11.7, 11.8, 11.9}};
        kpsr::geometry::Vector3 angular_velocity(seq++, 10.5, 10.6, 10.7);
        std::array<double, 9> angular_velocity_covariance{{12.1, 12.2, 12.3, 12.4, 12.5, 12.6, 12.7, 12.8, 12.9}};
        kpsr::geometry::Vector3 linear_acceleration(seq++, 0.8, 0.9, 1.0);
        std::array<double, 9> linear_acceleration_covariance{{3.1, 3.2, 3.3, 3.4, 3.5, 13.6, 13.7, 13.8, 13.9}};

        kpsr::geometry::Imu event(seq++, orientation, orientation_covariance, angular_velocity,
                                  angular_velocity_covariance, linear_acceleration,
                                  linear_acceleration_covariance);
        kpsrPublisher->publish(event);
    }

    {
        kpsr::geometry::Quaternion orientation(seq++, 20.1, 20.2, 20.3, 20.4);
        std::array<double, 9> orientation_covariance{{21.1, 21.2, 21.3, 21.4, 21.5, 21.6, 21.7, 21.8, 21.9}};
        kpsr::geometry::Vector3 angular_velocity(seq++, 0.5, 0.6, 0.7);
        std::array<double, 9> angular_velocity_covariance{{22.1, 22.2, 22.3, 22.4, 22.5, 22.6, 22.7, 22.8, 22.9}};
        kpsr::geometry::Vector3 linear_acceleration(seq++, 0.8, 0.9, 1.0);
        std::array<double, 9> linear_acceleration_covariance{{23.1, 23.2, 23.3, 23.4, 23.5, 23.6, 23.7, 23.8, 23.9}};

        kpsr::geometry::Imu event(seq++, orientation, orientation_covariance, angular_velocity,
                                  angular_velocity_covariance, linear_acceleration,
                                  linear_acceleration_covariance);
        kpsrPublisher->publish(event);
    }

    {
        kpsr::geometry::Quaternion orientation(seq++, 30.1, 30.2, 30.3, 30.4);
        std::array<double, 9> orientation_covariance{{31.1, 31.2, 31.3, 31.4, 31.5, 31.6, 31.7, 31.8, 31.9}};
        kpsr::geometry::Vector3 angular_velocity(seq++, 30.5, 30.6, 30.7);
        std::array<double, 9> angular_velocity_covariance{{32.1, 32.2, 32.3, 32.4, 32.5, 32.6, 32.7, 32.8, 32.9}};
        kpsr::geometry::Vector3 linear_acceleration(seq++, 0.8, 0.9, 1.0);
        std::array<double, 9> linear_acceleration_covariance{{33.1, 33.2, 33.3, 33.4, 33.5, 33.6, 33.7, 33.8, 33.9}};

        kpsr::geometry::Imu event(seq++, orientation, orientation_covariance, angular_velocity,
                                  angular_velocity_covariance, linear_acceleration,
                                  linear_acceleration_covariance);
        kpsrPublisher->publish(event);
    }

    {
        kpsr::geometry::Quaternion orientation(seq++, 40.1, 40.2, 40.3, 40.4);
        std::array<double, 9> orientation_covariance{{41.1, 41.2, 41.3, 41.4, 41.5, 41.6, 41.7, 41.8, 41.9}};
        kpsr::geometry::Vector3 angular_velocity(seq++, 30.5, 30.6, 30.7);
        std::array<double, 9> angular_velocity_covariance{{42.1, 42.2, 42.3, 42.4, 42.5, 42.6, 42.7, 42.8, 42.9}};
        kpsr::geometry::Vector3 linear_acceleration(seq++, 40.8, 40.9, 41.0);
        std::array<double, 9> linear_acceleration_covariance{{43.1, 43.2, 43.3, 43.4, 43.5, 43.6, 43.7, 43.8, 43.9}};

        kpsr::geometry::Imu event(seq++, orientation, orientation_covariance, angular_velocity,
                                  angular_velocity_covariance, linear_acceleration,
                                  linear_acceleration_covariance);
        kpsrPublisher->publish(event);
    }

    while (cacheListener.counter < 5) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    fromZMQProvider->stop();

    kpsr::geometry::Quaternion orientation(seq++, 40.1, 40.2, 40.3, 40.4);
    std::array<double, 9> orientation_covariance{{41.1, 41.2, 41.3, 41.4, 41.5, 41.6, 41.7, 41.8, 41.9}};
    kpsr::geometry::Vector3 angular_velocity(seq++, 30.5, 30.6, 30.7);
    std::array<double, 9> angular_velocity_covariance{{42.1, 42.2, 42.3, 42.4, 42.5, 42.6, 42.7, 42.8, 42.9}};
    kpsr::geometry::Vector3 linear_acceleration(seq++, 40.8, 40.9, 41.0);
    std::array<double, 9> linear_acceleration_covariance{{43.1, 43.2, 43.3, 43.4, 43.5, 43.6, 43.7, 43.8, 43.9}};

    kpsr::geometry::Imu event(seq++, orientation, orientation_covariance, angular_velocity,
                              angular_velocity_covariance, linear_acceleration,
                              linear_acceleration_covariance);

    ASSERT_GE(cacheListener.counter, 5);
    ASSERT_EQ(event.orientation.x, cacheListener.getLastReceivedEvent()->orientation.x);
    ASSERT_EQ(event.orientation.y, cacheListener.getLastReceivedEvent()->orientation.y);
    ASSERT_EQ(event.orientation.z, cacheListener.getLastReceivedEvent()->orientation.z);
    ASSERT_EQ(event.orientation.w, cacheListener.getLastReceivedEvent()->orientation.w);

    ASSERT_EQ(event.angular_velocity.x, cacheListener.getLastReceivedEvent()->angular_velocity.x);
    ASSERT_EQ(event.angular_velocity.y, cacheListener.getLastReceivedEvent()->angular_velocity.y);
    ASSERT_EQ(event.angular_velocity.z, cacheListener.getLastReceivedEvent()->angular_velocity.z);

    for (int i = 0; i < 9; i ++) {
        ASSERT_EQ(event.orientation_covariance[i], cacheListener.getLastReceivedEvent()->orientation_covariance[i]);
        ASSERT_EQ(event.angular_velocity_covariance[i], cacheListener.getLastReceivedEvent()->angular_velocity_covariance[i]);
        ASSERT_EQ(event.linear_acceleration_covariance[i], cacheListener.getLastReceivedEvent()->linear_acceleration_covariance[i]);
    }

    ASSERT_EQ(event.linear_acceleration.x, cacheListener.getLastReceivedEvent()->linear_acceleration.x);
    ASSERT_EQ(event.linear_acceleration.y, cacheListener.getLastReceivedEvent()->linear_acceleration.y);
    ASSERT_EQ(event.linear_acceleration.z, cacheListener.getLastReceivedEvent()->linear_acceleration.z);

}
