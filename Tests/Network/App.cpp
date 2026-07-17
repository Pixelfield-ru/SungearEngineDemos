//
// Created by stuka on 08.07.2026.
//

#include "App.h"

#include <SGCore/Input/PCInput.h>
#include <SGCore/Network/ClientConnectedMessage.h>
#include <SGCore/Network/ClientDisconnectedMessage.h>
#include <SGCore/Network/Packet.h>
#include <SGCore/Render/MeshBuilder.h>
#include <SGCore/Render/Alpha/OpaqueEntityTag.h>
#include <SGCore/Render/RenderAbilities/EnableMeshPass.h>

#include "TransformMessage.h"
#include "AnyMessage.h"
#include "AuthMessage.h"
#include "AuthResponseMessage.h"
#include "GetPlayersMessage.h"
#include "GetPlayersResponseMessage.h"
#include "PlayerInfoMessage.h"

#define LOG_TAG "SGNetworkTest"

void App::onInit() noexcept
{
    std::system("chcp 1251");

    if(m_startupType == StartupType::SERVER)
    {
        std::cout << "network test: running as server..." << std::endl;

        m_server = SGCore::Net::Server(3045);
        m_server->m_clientTimeout = std::chrono::seconds(10);

        auto& authResponseType = m_server->m_stream.registerDataType<AuthResponseMessage>();

        auto& authType = m_server->m_stream.registerDataType<AuthMessage>();
        authType.m_authRequired = false;
        authType.onReceive = [this](const SGCore::Net::Packet& packet, SGCore::Net::RUDPStream::endpoint_t senderEndpoint, SGCore::Net::session_id_t senderSessionID) {
            LOG_I(LOG_TAG, "got new client! his new session id is: {}", senderSessionID);

            if(m_server->m_stream.isClientRegistered(senderSessionID))
            {
                LOG_I(LOG_TAG, "this client is registered!");
                return;
            }

            LOG_I(LOG_TAG, "client is not registered! assigning {} as session ID!", m_currentMaxID);

            m_server->m_stream.registerClient(senderEndpoint, m_currentMaxID);

            AuthResponseMessage response;
            response.m_sessionID = m_currentMaxID;
            m_server->send(response, m_currentMaxID);

            SGCore::Net::ClientConnectedMessage clientConnectedMessage;
            m_server->propagate(clientConnectedMessage, m_currentMaxID);

            // dumbest way to generate session ID
            ++m_currentMaxID;
        };

        auto& getPlayersResponseType = m_server->m_stream.registerDataType<GetPlayersResponseMessage>();

        auto& getPlayersType = m_server->m_stream.registerDataType<GetPlayersMessage>();
        getPlayersType.onReceive = [this](const SGCore::Net::Packet& packet, SGCore::Net::RUDPStream::endpoint_t senderEndpoint, SGCore::Net::session_id_t senderSessionID) {
            const auto& players = m_server->m_stream.getRegisteredClients();

            GetPlayersResponseMessage response;

            std::size_t i = 0;
            for(const auto& [sessionID, endpointInfo] : players)
            {
                if(i >= response.m_players.size()) break;

                response.m_players[i] = sessionID;
                ++i;
            }

            response.m_playersCount = i;

            m_server->send(response, senderSessionID);
        };

        m_server->runReceivePoll();

        LOG_I(LOG_TAG, "network test: server created and running");
    }
    else
    {
        auto ecsRegistry = SGCore::Scene::getCurrentScene()->getECSRegistry();

        /*std::srand(std::time(nullptr));
        m_myID = std::rand();*/

        SGCore::MeshBuilder::buildBox3D(m_exampleMesh.m_base, { 4.0, 4.0, 4.0 });

        auto& authType = m_client.m_stream.registerDataType<AuthMessage>();
        auto& authResponseType = m_client.m_stream.registerDataType<AuthResponseMessage>();
        authResponseType.onReceive = [this](const SGCore::Net::Packet& packet, SGCore::Net::RUDPStream::endpoint_t senderEndpoint, SGCore::Net::session_id_t senderSessionID) {
            const auto& response = *reinterpret_cast<const AuthResponseMessage*>(packet.data());
            m_client.m_stream.m_sessionID = response.m_sessionID;

            LOG_I(LOG_TAG, "i am client and i have session id: {}. now i want to get players!", m_client.m_stream.m_sessionID.load());

            m_client.send(GetPlayersMessage{});
        };

        auto& disconnectedType = m_client.m_stream.registerDataType<SGCore::Net::ClientDisconnectedMessage>();
        disconnectedType.onReceive = [this](const SGCore::Net::Packet& packet, SGCore::Net::RUDPStream::endpoint_t senderEndpoint, SGCore::Net::session_id_t senderSessionID) {
            LOG_I(LOG_TAG, "disconnected client with session id: {}", senderSessionID);
            m_client.m_stream.removeClient(senderSessionID);
        };

        auto& getPlayersType = m_client.m_stream.registerDataType<GetPlayersMessage>();

        auto& getPlayersResponseType = m_client.m_stream.registerDataType<GetPlayersResponseMessage>();
        getPlayersResponseType.onReceive = [this](const SGCore::Net::Packet& packet, SGCore::Net::RUDPStream::endpoint_t senderEndpoint, SGCore::Net::session_id_t senderSessionID) {
            const auto& response = *reinterpret_cast<const GetPlayersResponseMessage*>(packet.data());

            for(size_t i = 0; i < response.m_players.size(); ++i)
            {
                if(i >= response.m_playersCount) break;

                const auto sessionID = response.m_players[i];

                LOG_I(LOG_TAG, "got player with session id: {}", sessionID);
                m_client.m_stream.registerClient(SGCore::Net::RUDPStream::endpoint_t{}, sessionID);
            }
        };

        auto& clientConnectedType = m_client.m_stream.registerDataType<SGCore::Net::ClientConnectedMessage>();
        clientConnectedType.m_authRequired = false;
        clientConnectedType.onReceive = [this](const SGCore::Net::Packet& packet, SGCore::Net::RUDPStream::endpoint_t senderEndpoint, SGCore::Net::session_id_t senderSessionID) {
            LOG_I(LOG_TAG, "client connected with session id: {}", senderSessionID);

            m_client.m_stream.registerClient(SGCore::Net::RUDPStream::endpoint_t{}, senderSessionID);
        };

        m_client.connect("127.0.0.1", 3045);

        /*m_client.registerDataStream<TransformMessage>().onReceive = [ecsRegistry, this](const SGCore::Net::Packet& packet, boost::asio::ip::udp::endpoint clientEndpoint) {
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
        };*/

        /*m_client.registerDataStream<SGCore::Net::ClientConnectedMessage>().onReceive = [](const SGCore::Net::Packet& packet, boost::asio::ip::udp::endpoint clientEndpoint) {
            std::cout << "network test: client connected" << std::endl;


        };
        m_client.registerDataStream<SGCore::Net::ClientDisconnectedMessage>().onReceive = [](const SGCore::Net::Packet& packet, boost::asio::ip::udp::endpoint clientEndpoint) {
            std::cout << "network test: client disconnected" << std::endl;
        };*/

        m_client.runReceivePoll();

        m_client.send(AuthMessage{});
    }
}

void App::onUpdate(double dt, double fixedDt) noexcept
{
    if(m_startupType == StartupType::CLIENT)
    {
        /*auto ecsRegistry = SGCore::Scene::getCurrentScene()->getECSRegistry();

        auto& cameraTransform = ecsRegistry->get<SGCore::Transform>(m_cameraEntity);

        TransformMessage msg;
        msg.m_position = cameraTransform.m_localTransform.m_position;
        msg.m_rotation = cameraTransform.m_localTransform.m_rotation;
        msg.m_playerID = m_myID;

        m_client.send(msg);*/

        if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_0))
        {
            std::cout << "sending auth message..." << std::endl;
            m_client.send(AuthMessage{});
        }

        if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_1))
        {
            std::cout << "sending get players message..." << std::endl;
            m_client.send(GetPlayersMessage{});
        }
    }
}

void App::onFixedUpdate(double dt, double fixedDt) noexcept
{

}