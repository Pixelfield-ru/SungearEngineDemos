//
// Created by stuka on 08.07.2026.
//

#pragma once

#include <SGCore/Memory/AssetRef.h>
#include <SGCore/Memory/Assets/ModelAsset.h>
#include <SGCore/Network/Server.h>
#include <SGCore/Network/Client.h>
#include <SGCore/Render/Mesh.h>

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

    std::optional<SGCore::Net::Server> m_server;
    SGCore::Net::Client m_client;

    std::unordered_map<SGCore::Net::session_id_t, SGCore::ECS::entity_t> m_players;

    std::int64_t m_currentMaxID = 1;

    SGCore::Mesh m_exampleMesh;

    void createPlayer(SGCore::Net::session_id_t playerSessionID) noexcept;

    // SGCore::ECS::entity_t m_playerEntity = entt::null;
};