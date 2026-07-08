//
// Created by stuka on 08.07.2026.
//

#include "App.h"

#include <SGCore/Network/ClientConnectedMessage.h>
#include <SGCore/Network/ClientDisconnectedMessage.h>
#include <SGCore/Network/Packet.h>

#include "TransformMessage.h"

void App::onInit() noexcept
{
    std::cout << "hash of TransformMessage: " << SGCore::constexprHash("TransformMessage") << std::endl;

    if(m_startupType == StartupType::SERVER)
    {
        std::cout << "network test: running as server..." << std::endl;

        m_server = SGCore::Net::Server(boost::asio::ip::udp::v4(), 3045);
        m_server.registerDataType<TransformMessage>();
        m_server.registerDataType<SGCore::Net::ClientConnectedMessage>();
        m_server.registerDataType<SGCore::Net::ClientDisconnectedMessage>();
        m_server.runReceivePoll([this](const SGCore::Net::Packet& packet, size_t packetSize, boost::asio::ip::udp::endpoint clientEndpoint) {
            std::cout << "got packet with size: " << packetSize << std::endl;
            m_server.propagatePacket(packet, std::move(clientEndpoint));
        });

        std::cout << "network test: server created and running" << std::endl;
    }
    else
    {
        // todo: make
        m_client.connect("127.0.0.1", 3045);

        m_client.registerDataStream<TransformMessage>();
        m_client.registerDataStream<SGCore::Net::ClientConnectedMessage>().onReceive = [](const SGCore::Net::Packet& packet, boost::asio::ip::udp::endpoint clientEndpoint) {
            std::cout << "network test: client connected" << std::endl;
        };
        m_client.registerDataStream<SGCore::Net::ClientDisconnectedMessage>().onReceive = [](const SGCore::Net::Packet& packet, boost::asio::ip::udp::endpoint clientEndpoint) {
            std::cout << "network test: client disconnected" << std::endl;
        };

        m_client.runReceivePoll();
    }
}

void App::onUpdate(double dt, double fixedDt) noexcept
{
    if(m_startupType == StartupType::CLIENT)
    {
        TransformMessage msg;
        m_client.send(msg);
    }
}

void App::onFixedUpdate(double dt, double fixedDt) noexcept
{

}