//
// Created by stuka on 08.03.2026.
//

#pragma once

#include <SGCore/Memory/AssetRef.h>
#include <SGCore/Memory/Assets/ModelAsset.h>

#include "SGCore/Main/BasicApp.h"

struct App final : SGCore::BasicApp
{
    void onInit() noexcept override;
    void onUpdate(double dt, double fixedDt) noexcept override;
    void onFixedUpdate(double dt, double fixedDt) noexcept override;

private:
    SGCore::AssetRef<SGCore::ModelAsset> m_cubeModel;
    SGCore::AssetRef<SGCore::ModelAsset> m_sphereModel;
    SGCore::AssetRef<SGCore::ModelAsset> m_humanModel;
    SGCore::AssetRef<SGCore::Skeleton> m_humanSkeleton;

    SGCore::ECS::entity_t m_playerEntity = entt::null;
    SGCore::ECS::entity_t m_floorEntity = entt::null;
    SGCore::ECS::entity_t m_humanEntity = entt::null;
    SGCore::ECS::entity_t m_sphere0Entity = entt::null;
    SGCore::ECS::entity_t m_sphere1Entity = entt::null;
    SGCore::ECS::entity_t m_sphere2Entity = entt::null;

    void createBallAndApplyImpulse(const glm::vec3& spherePos, const glm::vec3& impulse) noexcept;
};