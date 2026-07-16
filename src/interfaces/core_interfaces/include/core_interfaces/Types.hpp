#pragma once

#include "core_interfaces/msg/heartbeat.hpp"
#include "core_interfaces/msg/heartbeat_disconnect.hpp"
#include "core_interfaces/msg/killswitch.hpp"
#include "core_interfaces/srv/heartbeat_connect.hpp"

using HeartbeatMsg = core_interfaces::msg::Heartbeat;
using HeartbeatDcMsg = core_interfaces::msg::HeartbeatDisconnect;
using KillswitchMsg = core_interfaces::msg::Killswitch;

using HeartbeatCnSrv = core_interfaces::srv::HeartbeatConnect;