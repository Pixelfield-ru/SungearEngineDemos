//
// Created by stuka on 18.02.2026.
//

#pragma once

#include <SGCore/Graphics/API/ITexture2D.h>
#include <SGCore/Main/BasicApp.h>
#include <SGCore/Memory/Assets/AudioTrackAsset.h>
#include <SGCore/Motion/MotionPlannerNode.h>

struct App : SGCore::BasicApp
{
    void onInit() noexcept final;
    void onUpdate(double dt, double fixedDt) final;
    void onFixedUpdate(double dt, double fixedDt) final;

private:
    SGCore::AssetRef<SGCore::ITexture2D> m_testTexture;
    SGCore::AssetRef<SGCore::AudioTrackAsset> m_copterSound;
    SGCore::ECS::entity_t m_characterEntity = entt::null;
    SGCore::Ref<SGCore::MotionPlannerNode> m_testIdleNode;
};
