//
// Created by stuka on 08.07.2026.
//

#include "App.h"

#include <SGCore/Network/ClientConnectedMessage.h>
#include <SGCore/Network/ClientDisconnectedMessage.h>
#include <SGCore/Network/Packet.h>
#include <SGCore/Render/MeshBuilder.h>
#include <SGCore/Render/Alpha/OpaqueEntityTag.h>
#include <SGCore/Render/RenderAbilities/EnableMeshPass.h>

#include "TransformMessage.h"
#include "AnyMessage.h"
#include "PlayerInfoMessage.h"

void App::onInit() noexcept
{
    if(m_startupType == StartupType::SERVER)
    {
        std::cout << "network test: running as server..." << std::endl;

        m_server = SGCore::Net::Server(boost::asio::ip::udp::v4(), 3045);

        m_server->registerDataType<TransformMessage>();
        m_server->registerDataType<SGCore::Net::ClientConnectedMessage>();
        m_server->registerDataType<SGCore::Net::ClientDisconnectedMessage>();
        m_server->registerDataType<AnyMessage>();
        m_server->registerDataType<PlayerInfoMessage>();

        m_server->runReceivePoll([this](const SGCore::Net::Packet& packet, size_t packetSize, boost::asio::ip::udp::endpoint clientEndpoint) {
            // std::cout << "got packet with size: " << packetSize << std::endl;
            m_server->propagatePacket(packet, std::move(clientEndpoint));
        });

        std::cout << "network test: server created and running" << std::endl;
    }
    else
    {
        auto ecsRegistry = SGCore::Scene::getCurrentScene()->getECSRegistry();

        std::srand(std::time(nullptr));
        m_myID = std::rand();

        std::cout << "m_myID: " << m_myID << std::endl;

        SGCore::MeshBuilder::buildBox3D(m_exampleMesh.m_base, { 4.0, 4.0, 4.0 });

        m_client.onConnected = [this]() {
            PlayerInfoMessage msg;
            msg.m_playerID = m_myID;

            m_client.send(msg);
        };

        m_client.connect("127.0.0.1", 3045);

        m_client.registerDataStream<TransformMessage>().onReceive = [ecsRegistry, this](const SGCore::Net::Packet& packet, boost::asio::ip::udp::endpoint clientEndpoint) {
            // std::cout << "got transform" << std::endl;
            const auto& msg = reinterpret_cast<const TransformMessage&>(*packet.data());

            const auto playerEntity = m_players[msg.m_playerID];

            auto& playerTransform = ecsRegistry->get<SGCore::Transform>(playerEntity);

            playerTransform.m_localTransform.m_position = msg.m_position;
            playerTransform.m_localTransform.m_rotation = msg.m_rotation;
        };

        m_client.registerDataStream<AnyMessage>().onReceive = [](const SGCore::Net::Packet& packet, boost::asio::ip::udp::endpoint clientEndpoint) {
            std::cout << packet.data() << std::endl;
        };

        m_client.registerDataStream<PlayerInfoMessage>().onReceive = [ecsRegistry, this](const SGCore::Net::Packet& packet, boost::asio::ip::udp::endpoint clientEndpoint) {
            const auto& msg = reinterpret_cast<const PlayerInfoMessage&>(*packet.data());

            const auto playerEntity = ecsRegistry->create();
            auto& playerMesh = ecsRegistry->emplace<SGCore::Mesh>(playerEntity);
            auto& playerTransform = ecsRegistry->emplace<SGCore::Transform>(playerEntity);
            ecsRegistry->emplace<SGCore::EnableMeshPass>(playerEntity);
            ecsRegistry->emplace<SGCore::OpaqueEntityTag>(playerEntity);

            playerMesh.m_base.setMeshData(m_exampleMesh.m_base.getMeshData());

            m_players[msg.m_playerID] = playerEntity;
        };

        /*m_client.registerDataStream<SGCore::Net::ClientConnectedMessage>().onReceive = [](const SGCore::Net::Packet& packet, boost::asio::ip::udp::endpoint clientEndpoint) {
            std::cout << "network test: client connected" << std::endl;


        };
        m_client.registerDataStream<SGCore::Net::ClientDisconnectedMessage>().onReceive = [](const SGCore::Net::Packet& packet, boost::asio::ip::udp::endpoint clientEndpoint) {
            std::cout << "network test: client disconnected" << std::endl;
        };*/

        m_client.runReceivePoll();
    }
}

void App::onUpdate(double dt, double fixedDt) noexcept
{
    if(m_startupType == StartupType::CLIENT)
    {
        auto ecsRegistry = SGCore::Scene::getCurrentScene()->getECSRegistry();

        auto& cameraTransform = ecsRegistry->get<SGCore::Transform>(m_cameraEntity);

        TransformMessage msg;
        msg.m_position = cameraTransform.m_localTransform.m_position;
        msg.m_rotation = cameraTransform.m_localTransform.m_rotation;
        msg.m_playerID = m_myID;

        m_client.send(msg);
    }
}

void App::onFixedUpdate(double dt, double fixedDt) noexcept
{

}