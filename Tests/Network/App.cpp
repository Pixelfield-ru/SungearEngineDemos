//
// Created by stuka on 08.07.2026.
//

#include "App.h"

#include "TransformPacket.h"

void App::onInit() noexcept
{
    if(m_startupType == StartupType::SERVER)
    {
        std::cout << "network test: running as server..." << std::endl;

        m_server = SGCore::Net::Server(boost::asio::ip::udp::v4(), 3045);
        m_server.registerDataType<TransformPacket>();
        m_server.runReceivePoll([this](const SGCore::Net::Packet& packet, size_t packetSize, boost::asio::ip::udp::endpoint clientEndpoint) {
            m_server.propagatePacket(packet, clientEndpoint);
        });

        std::cout << "network test: server created and running" << std::endl;
    }
    else
    {
        // todo: make
        m_client.connect("127.0.0.1", 3045);
        // m_client.registerDataStream<TransformPacket>().onReceive

        m_client.runReceivePoll();
    }
}

void App::onUpdate(double dt, double fixedDt) noexcept
{

}

void App::onFixedUpdate(double dt, double fixedDt) noexcept
{

}