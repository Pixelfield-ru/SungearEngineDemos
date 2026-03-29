//
// Created by stuka on 18.02.2026.
//

#pragma once

#include <SGCore/Animation/IAnimationNode.h>
#include <SGCore/Graphics/API/ITexture2D.h>
#include <SGCore/Main/BasicApp.h>
#include <SGCore/Memory/Assets/AudioTrackAsset.h>
#include <SGCore/Memory/Assets/ModelAsset.h>

struct App : SGCore::BasicApp
{
    void onInit() noexcept final;
    void onUpdate(double dt, double fixedDt) final;
    void onFixedUpdate(double dt, double fixedDt) final;

private:
    SGCore::AssetRef<SGCore::ITexture2D> m_testTexture;
    SGCore::AssetRef<SGCore::AudioTrackAsset> m_copterSound;
    SGCore::AssetRef<SGCore::ModelAsset> m_roboarmModel;
    SGCore::AssetRef<SGCore::Skeleton> m_roboarmSkeleton;
    SGCore::ECS::entity_t m_characterEntity = entt::null;
    SGCore::ECS::entity_t m_roboarmEntity = entt::null;
    std::vector<SGCore::ECS::entity_t> m_roboarmJoints;
    SGCore::Ref<SGCore::IAnimationNode> m_testIdleNode;
};
