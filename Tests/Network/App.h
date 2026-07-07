//
// Created by stuka on 08.07.2026.
//

#pragma once

#include <SGCore/Memory/AssetRef.h>
#include <SGCore/Memory/Assets/ModelAsset.h>
#include <SGCore/Network/Server.h>
#include <SGCore/Network/Client.h>

#include "StartupType.h"
#include "SGCore/Main/BasicApp.h"

struct App final : SGCore::BasicApp
{
    StartupType m_startupType = StartupType::CLIENT;

    void onInit() noexcept override;
    void onUpdate(double dt, double fixedDt) noexcept override;
    void onFixedUpdate(double dt, double fixedDt) noexcept override;

private:
    SGCore::AssetRef<SGCore::ModelAsset> m_cubeModel;

    SGCore::Net::Server m_server;
    SGCore::Net::Client m_client;

    SGCore::ECS::entity_t m_playerEntity = entt::null;
};