//
// Created by stuka on 21.02.2026.
//

#pragma once

#include <SGCore/Graphics/API/ITexture2D.h>
#include <SGCore/Main/BasicApp.h>
#include <SGCore/Memory/Assets/AudioTrackAsset.h>
#include <SGCore/Memory/Assets/ModelAsset.h>
#include <SGCore/Motion/MotionPlannerNode.h>

struct App final : SGCore::BasicApp
{
    void onInit() noexcept override;
    void onUpdate(double dt, double fixedDt) noexcept override;
    void onFixedUpdate(double dt, double fixedDt) noexcept override;

    void rebuildNavMesh(const std::vector<SGCore::ECS::entity_t>& meshedEntities) noexcept;

private:
    SGCore::Slot<void(SGCore::Window&, double xScroll, double yScroll)> onMouseScroll;

    SGCore::ECS::entity_t m_navMeshEntity {};
    SGCore::AssetRef<SGCore::ModelAsset> m_locationModel;
    SGCore::AssetRef<SGCore::ModelAsset> m_floorModel;
    SGCore::AssetRef<SGCore::ModelAsset> m_cubeModel;
};