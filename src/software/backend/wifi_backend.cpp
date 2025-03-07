#include "software/backend/wifi_backend.h"

#include "shared/constants.h"
#include "shared/parameter/cpp_dynamic_parameters.h"
#include "shared/proto/robot_log_msg.pb.h"
#include "software/constants.h"
#include "software/logger/logger.h"
#include "software/proto/message_translation/defending_side.h"
#include "software/proto/message_translation/tbots_protobuf.h"
#include "software/util/design_patterns/generic_factory.h"

WifiBackend::WifiBackend(std::shared_ptr<const BackendConfig> config)
    : network_config(config->getWifiBackendConfig()->getNetworkConfig()),
      sensor_fusion_config(config->getWifiBackendConfig()->getSensorFusionConfig()),
      ssl_proto_client(boost::bind(&Backend::receiveSSLWrapperPacket, this, _1),
                       boost::bind(&Backend::receiveSSLReferee, this, _1),
                       network_config->getSslCommunicationConfig())
{
    std::string network_interface = this->network_config->getNetworkInterface()->value();
    int channel                   = this->network_config->getChannel()->value();

    network_config->getChannel()->registerCallbackFunction([this](int new_channel) {
        std::string new_network_interface =
            this->network_config->getNetworkInterface()->value();
        joinMulticastChannel(new_channel, new_network_interface);
    });

    // connect to current channel
    joinMulticastChannel(channel, network_interface);
}

void WifiBackend::onValueReceived(TbotsProto::PrimitiveSet primitives)
{
    primitive_output->sendProto(primitives);

    if (sensor_fusion_config->getOverrideGameControllerDefendingSide()->value())
    {
        defending_side_output->sendProto(
            *createDefendingSide(sensor_fusion_config->getDefendingPositiveSide()->value()
                                     ? FieldSide::POS_X
                                     : FieldSide::NEG_X));
    }
    else
    {
        defending_side_output->sendProto(*createDefendingSide(FieldSide::NEG_X));
    }
}

void WifiBackend::onValueReceived(World world)
{
    vision_output->sendProto(*createVision(world));
}

void WifiBackend::receiveRobotLogs(TbotsProto::RobotLog log)
{
    LOG(INFO) << "[ROBOT " << log.robot_id() << " " << LogLevel_Name(log.log_level())
              << "]"
              << "[" << log.file_name() << ":" << log.line_number()
              << "]: " << log.log_msg() << std::endl;
}

void WifiBackend::joinMulticastChannel(int channel, const std::string& interface)
{
    vision_output.reset(new ThreadedProtoUdpSender<TbotsProto::Vision>(
        std::string(MULTICAST_CHANNELS[channel]) + "%" + interface, VISION_PORT, true));

    primitive_output.reset(new ThreadedProtoUdpSender<TbotsProto::PrimitiveSet>(
        std::string(MULTICAST_CHANNELS[channel]) + "%" + interface, PRIMITIVE_PORT,
        true));

    robot_status_input.reset(new ThreadedProtoUdpListener<TbotsProto::RobotStatus>(
        std::string(MULTICAST_CHANNELS[channel]) + "%" + interface, ROBOT_STATUS_PORT,
        boost::bind(&Backend::receiveRobotStatus, this, _1), true));

    robot_log_input.reset(new ThreadedProtoUdpListener<TbotsProto::RobotLog>(
        std::string(MULTICAST_CHANNELS[channel]) + "%" + interface, ROBOT_LOGS_PORT,
        boost::bind(&WifiBackend::receiveRobotLogs, this, _1), true));

    defending_side_output.reset(new ThreadedProtoUdpSender<DefendingSideProto>(
        std::string(MULTICAST_CHANNELS[channel]) + "%" + interface, DEFENDING_SIDE_PORT,
        true));
}

// Register this backend in the genericFactory
static TGenericFactory<std::string, Backend, WifiBackend, BackendConfig> factory;
