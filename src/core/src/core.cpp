#include "core.hpp"

void Core::onConnect(
    const HeartbeatConnect::Request::SharedPtr request,
    HeartbeatConnect::Response::SharedPtr response)
{
    RCLCPP_INFO(get_logger(), "Got heartbeat connect request");

    if (active)
    {
        RCLCPP_WARN(get_logger(), "Already connected, ignoring request");
        response->accepted = false;
        return;
    }
    else
    {
        RCLCPP_INFO(get_logger(), "Accepting connection request");
        active = true;
    }

    response->accepted = true;

    lastHeartbeatTime = now();

    heartbeatInterval = std::chrono::milliseconds(request->heartbeat_interval);
    heartbeatTimeout = std::chrono::milliseconds(request->heartbeat_timeout);
    heartbeatTimeoutLimit = request->heartbeat_timeout_limit;

    heartbeatCheckTimer = create_wall_timer(
        std::chrono::milliseconds(request->heartbeat_check_interval),
        std::bind(&Core::checkHeartbeat, this));
}

void Core::onDisconnect(const HeartbeatDisconnect::SharedPtr)
{
    if (!active)
    {
        RCLCPP_WARN(get_logger(), "Received disconnect request while not connected");
        return;
    }

    RCLCPP_INFO(get_logger(), "Received disconnect request");
    active = false;
}

void Core::heartbeatSubscriberCallback(const Heartbeat::SharedPtr beat)
{
    if (!active)
    {
        RCLCPP_WARN(get_logger(), "Received heartbeat while not connected");
        return;
    }

    lastHeartbeatTime = now();
    missedHeartbeats = 0;

    if (beat->id != expectedHeartbeatId)
    {
        RCLCPP_WARN(get_logger(), "Received heartbeat with ID %d instead of %d",
                    beat->id, expectedHeartbeatId);

        if (beat->id > expectedHeartbeatId)
        {
            expectedHeartbeatId = beat->id + 1;
        }
    }
    else
    {
        expectedHeartbeatId++;
    }

    auto msg = Heartbeat();
    msg.id = beat->id;
    heartbeatPub->publish(msg);
}

void Core::checkHeartbeat()
{
    if (!active)
    {
        return;
    }

    // Determine the time elapsed since the last heartbeat should have been received,
    // accounting for the number of missed heartbeats
    auto expectedHeartbeatTime = lastHeartbeatTime + heartbeatInterval * (missedHeartbeats + 1);

    if (now() - expectedHeartbeatTime > heartbeatTimeout)
    {
        missedHeartbeats++;

        RCLCPP_WARN(get_logger(), "Missed heartbeat #%d", missedHeartbeats);

        if (missedHeartbeats >= heartbeatTimeoutLimit)
        {
            RCLCPP_ERROR(get_logger(), "Missed too many heartbeats, disconnecting");
            active = false;
        }
    }
}

Core::Core() : rclcpp::Node("core"), heartbeatInterval(0, 0), heartbeatTimeout(0, 0)
{
    using namespace std::placeholders;

    connectService = create_service<HeartbeatConnect>(
        "/heartbeat/connect",
        std::bind(&Core::onConnect, this, _1, _2));

    disconnectSub = create_subscription<HeartbeatDisconnect>(
        "/heartbeat/disconnect",
        10,
        std::bind(&Core::onDisconnect, this, _1));

    heartbeatSub = create_subscription<Heartbeat>(
        "/heartbeat/rover",
        10,
        std::bind(&Core::heartbeatSubscriberCallback, this, _1));

    heartbeatPub = create_publisher<Heartbeat>("/heartbeat/control", 10);

    declare_parameter("killswitch", false);

    RCLCPP_INFO(get_logger(), "Core ready");
}