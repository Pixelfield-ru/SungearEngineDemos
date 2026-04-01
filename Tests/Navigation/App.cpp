//
// Created by stuka on 21.02.2026.
//

#include "App.h"

#include <SGCore/Graphics/API/IFrameBuffer.h>
#include <SGCore/Input/PCInput.h>
#include <SGCore/Memory/Assets/ModelAsset.h>
#include <SGCore/Navigation/NavMesh/NavMesh.h>
#include <SGCore/Navigation/NavMesh/Steps/InputFilteringStep.h>
#include <SGCore/Navigation/NavMesh/Steps/RegionsPartitionStep.h>
#include <SGCore/Navigation/NavMesh/Steps/VoxelizationStep.h>
#include <SGCore/Render/DebugDraw.h>
#include <SGCore/Scene/Scene.h>
#include <SGCore/Render/Mesh.h>
#include <SGCore/Render/RenderingBase.h>
#include <SGCore/Render/RenderPipelinesManager.h>
#include <SGCore/Render/PostProcess/StandardFX/SSAO.h>
#include <SGCore/Render/PostProcess/StandardFX/Vignette.h>
#include <SGCore/Render/PostProcess/StandardFX/FilmGrain.h>
#include <SGCore/Render/PostProcess/StandardFX/SSR.h>
#include <SGCore/Render/PostProcess/StandardFX/Bloom.h>
#include <SGCore/Render/ShadowMapping/CSM/CSMTarget.h>
#include <SGCore/Render/ShadowMapping/ShadowCaster.h>
#include <SGCore/Render/Atmosphere/Atmosphere.h>

void App::rebuildNavMesh(const std::vector<SGCore::ECS::entity_t>& meshedEntities) noexcept
{
    const auto ecsRegistry = SGCore::Scene::getCurrentScene()->getECSRegistry();

    auto& navMesh = ecsRegistry->get<SGCore::Navigation::NavMesh>(m_navMeshEntity);

    std::vector<SGCore::MathPrimitivesUtils::Triangle<>> navMeshTriangles;

    for(auto e : meshedEntities)
    {
        const auto transform = ecsRegistry->get<SGCore::Transform>(e);
        const auto& mesh = ecsRegistry->tryGet<SGCore::Mesh>(e);

        navMeshTriangles.reserve(navMeshTriangles.size() + mesh->m_base.getMeshData()->m_indices.size() / 3);

        for(size_t i = 0; i < mesh->m_base.getMeshData()->m_indices.size(); i += 3)
        {
            const auto& v0 = mesh->m_base.getMeshData()->m_vertices[mesh->m_base.getMeshData()->m_indices[i + 0]];
            const auto& v1 = mesh->m_base.getMeshData()->m_vertices[mesh->m_base.getMeshData()->m_indices[i + 1]];
            const auto& v2 = mesh->m_base.getMeshData()->m_vertices[mesh->m_base.getMeshData()->m_indices[i + 2]];

            SGCore::MathPrimitivesUtils::Triangle<> tri;
            tri.m_vertices[0] = transform->m_worldTransform.m_animatedModelMatrix * glm::vec4(v0.m_position, 1.0f);
            tri.m_vertices[1] = transform->m_worldTransform.m_animatedModelMatrix * glm::vec4(v1.m_position, 1.0f);
            tri.m_vertices[2] = transform->m_worldTransform.m_animatedModelMatrix * glm::vec4(v2.m_position, 1.0f);

            tri.calculateNormal();

            navMeshTriangles.push_back(tri);
        }
    }

    navMesh.build(std::move(navMeshTriangles));
}

void App::onInit() noexcept
{
    const std::string demosPath = SG_STRINGIFY_MACRO(SUNGEAR_DEMOS_ROOT);

    auto ecsRegistry = SGCore::Scene::getCurrentScene()->getECSRegistry();

    auto assetManager = SGCore::AssetManager::getInstance();

    auto& frameReceiver = ecsRegistry->get<SGCore::LayeredFrameReceiver>(getCameraEntity());
    auto bloomLayer = frameReceiver.addLayer("BloomLayer");

    auto& csmTarget = ecsRegistry->emplace<SGCore::CSMTarget>(getCameraEntity());

    // m_locationModel = assetManager->loadAsset<SGCore::ModelAsset>(demosPath / "Tests/Navigation/Resources/location_1/ai_test.gltf");
    m_locationModel = assetManager->loadAsset<SGCore::ModelAsset>(demosPath / "Tests/AITest/Resources/location_0/scene.gltf");
    auto locationEntities = m_locationModel->m_rootNode->addOnScene(SGCore::Scene::getCurrentScene());
    for(const auto& locationEntity : locationEntities)
    {
        if(ecsRegistry->allOf<SGCore::Mesh>(locationEntity))
        {
            ecsRegistry->emplace<SGCore::ShadowCaster>(locationEntity);
        }
    }

    m_floorModel = assetManager->loadAsset<SGCore::ModelAsset>(demosPath / "Tests/Navigation/Resources/floor_2/scene.gltf");
    auto floorEntities = m_floorModel->m_rootNode->addOnScene(SGCore::Scene::getCurrentScene());
    auto floorTransform = ecsRegistry->get<SGCore::Transform>(floorEntities[0]);
    floorTransform->m_localTransform.m_position.y += 150.0f;
    floorTransform->m_localTransform.m_scale *= 0.1f;

    m_cubeModel = assetManager->loadAsset<SGCore::ModelAsset>(demosPath / "Tests/Navigation/Resources/location_1/ai_test.gltf");
    const auto cubeEntities = m_cubeModel->m_rootNode->addOnScene(SGCore::Scene::getCurrentScene());
    for(auto&& cubeEntity : cubeEntities)
    {
        if(auto* mesh = ecsRegistry->tryGet<SGCore::Mesh>(cubeEntity))
        {
            mesh->m_base.m_layeredFrameReceiversMarkup[&frameReceiver] = bloomLayer;
        }
    }

    m_navMeshEntity = ecsRegistry->create();
    auto& navMesh = ecsRegistry->emplace<SGCore::Navigation::NavMesh>(m_navMeshEntity);
    navMesh.useStandardSteps();
    navMesh.m_config.m_agentRadius = 0.5f;
    // navMesh.m_config.m_agentHeight = 100.0f;
    navMesh.m_config.m_cellHeight = 1.0f;
    navMesh.m_config.m_cellSize = 1.0f;
    navMesh.m_config.m_agentMaxSlope = 40.0f;
    navMesh.m_config.m_agentMaxClimb = 2.0f;

    {
        auto filmGrainFX = SGCore::MakeRef<SGCore::FilmGrain>();
        filmGrainFX->setIntensity(0.3);
        frameReceiver.getDefaultLayer()->addEffect(filmGrainFX);

        auto vignetteFX = SGCore::MakeRef<SGCore::Vignette>();
        vignetteFX->setRadius(0.5);
        frameReceiver.getDefaultLayer()->addEffect(vignetteFX);

        auto ssrFX = SGCore::MakeRef<SGCore::SSR>();
        frameReceiver.getDefaultLayer()->addEffect(ssrFX);
    }

    {
        auto ssaoFX = SGCore::MakeRef<SGCore::SSAO>();
        auto bloomFX = SGCore::MakeRef<SGCore::Bloom>();
        auto ssrFX = SGCore::MakeRef<SGCore::SSR>();

        bloomLayer->addEffect(ssaoFX);
        bloomLayer->addEffect(ssrFX);
        bloomLayer->addEffect(bloomFX);
    }

    // ==========================================================

    onMouseScroll = [this, ecsRegistry](SGCore::Window&, double xScroll, double yScroll) {
        ecsRegistry->get<SGCore::RenderingBase>(getCameraEntity())->m_fov -= yScroll;
    };

    SGCore::Input::PC::onMouseScroll() += onMouseScroll;

    SGCore::CoreMain::getWindow().setSwapInterval(false);
    SGCore::CoreMain::getRenderTimer().setTargetFrameRate(200.0);
}

void App::onUpdate(double dt, double fixedDt) noexcept
{
    const auto debugDraw = SGCore::RenderPipelinesManager::instance().getCurrentRenderPipeline()->getRenderPass<SGCore::DebugDraw>();

    auto ecsRegistry = SGCore::Scene::getCurrentScene()->getECSRegistry();

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_0))
    {
        std::vector<SGCore::ECS::entity_t> meshedEntities;

        auto entitiesView = ecsRegistry->view<SGCore::EntityBaseInfo>();
        entitiesView.each([&](auto e, auto&) {
            if(getAtmosphereEntity() == e) return;

            const auto* mesh = ecsRegistry->tryGet<SGCore::Mesh>(e);
            if(mesh)
            {
                meshedEntities.push_back(e);
            }
        });

        rebuildNavMesh(meshedEntities);
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_1))
    {
        if(debugDraw->m_mode == SGCore::DebugDrawMode::NO_DEBUG)
        {
            debugDraw->m_mode = SGCore::DebugDrawMode::WIREFRAME;
        }
        else
        {
            debugDraw->m_mode = SGCore::DebugDrawMode::NO_DEBUG;
        }
    }

    if(debugDraw->m_mode != SGCore::DebugDrawMode::NO_DEBUG)
    {
        auto& navMesh = ecsRegistry->get<SGCore::Navigation::NavMesh>(m_navMeshEntity);

        const auto inputFilteringStep = navMesh.getStep<SGCore::Navigation::InputFilteringStep>();
        const auto voxelizationStep = navMesh.getStep<SGCore::Navigation::VoxelizationStep>();
        const auto regionsPartitionStep = navMesh.getStep<SGCore::Navigation::RegionsPartitionStep>();

        const auto& navMeshConfig = navMesh.m_config;

        for(const auto& region : regionsPartitionStep->m_regions)
        {
            for(auto idx : region.m_contourVoxelsIndices)
            {
                const auto& voxel = voxelizationStep->m_voxels[idx];

                const glm::vec3 p = voxelizationStep->voxelToWorld(
                    voxel.m_position,
                    navMeshConfig.m_cellSize,
                    navMeshConfig.m_cellHeight);

                debugDraw->drawLine(p, p + glm::vec3 { 0.0f, 1.0f, 0.0f }, { 0, 0, 1, 1.0 });
            }
        }

        for(const auto& voxel : voxelizationStep->m_voxels)
        {
            const glm::vec3 min = voxelizationStep->voxelToWorld(
                                      voxel.m_position, navMeshConfig.m_cellSize,
                                      navMeshConfig.m_cellHeight) - glm::vec3(
                                      navMeshConfig.m_cellSize * 0.5f,
                                      navMeshConfig.m_cellHeight * 0.5f,
                                      navMeshConfig.m_cellSize * 0.5f);

            const glm::vec3 max = min + glm::vec3(navMeshConfig.m_cellSize,
                                                  navMeshConfig.m_cellHeight,
                                                  navMeshConfig.m_cellSize);

            const glm::vec3 offset { 0.01, 0.01, 0.01 };
            if(voxel.m_isWalkable)
            {
                debugDraw->drawAABB(min + offset, max - offset, { 0.47, 0.87, 0.78, 1.0 });
            }
            else
            {
                debugDraw->drawAABB(min + offset, max - offset, { 1.0, 0.0, 0.0, 1.0 });
            }
        }
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_2))
    {
        auto shaders = SGCore::AssetManager::getInstance()->getAssetsWithType<SGCore::IShader>();
        for(const auto& shader : shaders)
        {
            shader->reloadFromDisk();
        }
    }

    auto& frameReceiver = ecsRegistry->get<SGCore::LayeredFrameReceiver>(getCameraEntity());

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_3))
    {
        for(const auto& layer : frameReceiver.getLayers())
        {
            auto fx = layer->getEffect<SGCore::SSAO>();
            if(!fx) continue;

            fx->setEnabled(!fx->isEnabled());
        }
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_4))
    {
        for(const auto& layer : frameReceiver.getLayers())
        {
            auto fx = layer->getEffect<SGCore::Vignette>();
            if(!fx) continue;

            fx->setEnabled(!fx->isEnabled());
        }
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_5))
    {
        for(const auto& layer : frameReceiver.getLayers())
        {
            auto fx = layer->getEffect<SGCore::FilmGrain>();
            if(!fx) continue;

            fx->setEnabled(!fx->isEnabled());
        }
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_6))
    {
        for(const auto& layer : frameReceiver.getLayers())
        {
            auto fx = layer->getEffect<SGCore::SSR>();
            if(!fx) continue;

            fx->setEnabled(!fx->isEnabled());
        }
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_7))
    {
        for(const auto& layer : frameReceiver.getLayers())
        {
            auto fx = layer->getEffect<SGCore::Bloom>();
            if(!fx) continue;

            fx->setEnabled(!fx->isEnabled());
        }
    }

    auto& atmosphere = ecsRegistry->get<SGCore::Atmosphere>(getAtmosphereEntity());

    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_EQUAL))
    {
        atmosphere.m_sunRotation.x += 0.2;
    }

    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_MINUS))
    {
        atmosphere.m_sunRotation.x -= 0.2;
    }

    SGCore::CoreMain::getWindow().setTitle("Navigation Test. FPS: " + std::to_string(SGCore::CoreMain::getFPS()));
}

void App::onFixedUpdate(double dt, double fixedDt) noexcept
{

}
