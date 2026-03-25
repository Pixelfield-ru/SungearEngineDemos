//
// Created by stuka on 25.03.2026.
//

#pragma once

#include <SGCore/Main/BasicApp.h>
#include <SGCore/Memory/AssetRef.h>
#include <SGCore/Memory/Assets/ModelAsset.h>

struct App final : SGCore::BasicApp
{
    void onInit() noexcept override;
    void onUpdate(double dt, double fixedDt) override;
    void onFixedUpdate(double dt, double fixedDt) override;

private:
    SGCore::AssetRef<SGCore::ModelAsset> m_cubeModel;
    SGCore::ECS::entity_t m_playerEntity = entt::null;
    float m_playerSpeed = 30.0f;
    float m_mouseSensitivity = 0.2f;
};
