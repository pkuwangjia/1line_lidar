#ifndef MODULES_HMI_SERVER_INCLUDE_STATE_STATE_FLAGS_H
#define MODULES_HMI_SERVER_INCLUDE_STATE_STATE_FLAGS_H

#include "gflags/gflags.h"

namespace adu
{
namespace hmi
{

// Server address
DECLARE_string(state_server_host);
DECLARE_string(state_server_port);

DECLARE_string(state_main_state_config);
DECLARE_string(state_driving_state_config);

// Grpc profiles
DECLARE_int32(state_grpc_max_message_size);

// Channel & params
DECLARE_string(state_du_drive_chassis_channel);
DECLARE_string(state_patrol_status_channel);
DECLARE_string(state_pad_proxy_channel);
// DECLARE_string(state_routing_param);
DECLARE_string(state_global_state_param);

} // namespace hmi
} // namespace adu

#endif
