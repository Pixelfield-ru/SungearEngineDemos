//
// Created by stuka on 18.02.2026.
//

#include "App.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/ext.hpp>

#include <SGCore/Audio/AudioSource.h>
#include <SGCore/Input/PCInput.h>
#include <SGCore/Memory/AssetManager.h>
#include <SGCore/Memory/Assets/AnimationsFile.h>
#include <SGCore/Memory/Assets/GIF.h>
#include <SGCore/Memory/Assets/ModelAsset.h>
#include <SGCore/Motion/MotionPlanner.h>
#include <SGCore/Memory/Assets/Materials/IMaterial.h>
#include <SGCore/Render/Mesh.h>
#include <SGCore/Coro/Task.h>
#include <SGCore/Render/PostProcess/StandardFX/SSAO.h>
#include <SGCore/Animation/AnimationsTree.h>
#include <SGCore/Animation/SkeletalAnimationNode.h>
#include <SGCore/Animation/GotoAnimationNode.h>
#include <SGCore/Animation/WhenAnimationEndAction.h>
#include <SGCore/Motion/IK/IKRootJoint.h>
#include <SGCore/Motion/IK/IKJoint.h>
#include <SGCore/Render/DebugDraw.h>
#include <SGCore/Render/RenderPipelinesManager.h>

SGCore::Coro::Task<> moveSmoothly(SGCore::ECS::entity_t entity, glm::vec3 to, float speed)
{
    using namespace std::chrono_literals;

    auto scene = SGCore::Scene::getCurrentScene();
    if(!scene) co_return;

    auto transform = scene->getECSRegistry()->get<SGCore::Transform>(entity);

    while(glm::distance(transform->m_localTransform.m_position, to) > 0.5f)
    {
        co_await 1ms;

        const auto dif = to - transform->m_localTransform.m_position;
        transform->m_localTransform.m_position += dif * speed;
    }
}

void App::onInit() noexcept
{
    const std::string demosPath = SG_STRINGIFY_MACRO(SUNGEAR_DEMOS_ROOT);

    auto scene = SGCore::Scene::getCurrentScene();
    auto mainAssetManager = SGCore::AssetManager::getInstance();

    // TEST!!!
    m_testTexture = mainAssetManager->loadAsset<SGCore::ITexture2D>("${enginePath}/Resources/textures/no_material.png");

    auto ecsRegistry = scene->getECSRegistry();

    // loading audio ================================================================================================
    m_copterSound = SGCore::AssetManager::getInstance()->loadAsset<SGCore::AudioTrackAsset>(demosPath / "Tests/ModelDraw/Resources/drone/copter.wav");
    m_copterSound->toMono();
    std::cout << m_copterSound->getSummary() << std::endl;

    // creating model ===============================================================================================

    // loading model asset
    /*auto modelAsset = SGCore::AssetManager::getInstance()->loadAsset<SGCore::ModelAsset>("${enginePath}/Tests/ModelDraw/Resources/Fast Run.fbx");
    auto modelSkeletonAsset = SGCore::AssetManager::getInstance()->loadAsset<SGCore::Skeleton>("${enginePath}/Tests/ModelDraw/Resources/Fast Run.fbx/skeletons/mixamorig:Hips");*/

    auto modelAsset = SGCore::AssetManager::getInstance()->createAndAddAsset<SGCore::ModelAsset>();

    auto& modelAssetLoadSlot = modelAsset->onLazyLoadDone += [this, demosPath, ecsRegistry](SGCore::IAsset* thisAsset) {
        auto* modelAsset = static_cast<SGCore::ModelAsset*>(thisAsset);

        // auto modelSkeletonAsset = SGCore::AssetManager::getInstance()->loadAsset<SGCore::Skeleton>("${enginePath}/Tests/ModelDraw/Resources/fsb_operator/scene.gltf/skeletons/GLTF_created_0_rootJoint");

        auto modelSkeletonAsset = SGCore::AssetManager::getInstance()->loadAsset<SGCore::Skeleton>(demosPath / "Tests/ModelDraw/Resources/drone/scene.gltf/skeletons/GLTF_created_0_rootJoint");

        // auto modelSkeletonAsset = SGCore::AssetManager::getInstance()->loadAsset<SGCore::Skeleton>("${enginePath}/Tests/ModelDraw/Resources/hu_tao_animated/scene.gltf/skeletons/_rootJoint");

        // auto modelSkeletonAsset = SGCore::AssetManager::getInstance()->loadAsset<SGCore::Skeleton>("${enginePath}/Tests/ModelDraw/Resources/Fast Run.fbx/skeletons/mixamorig:Hips");

        /*auto modelAsset = SGCore::AssetManager::getInstance()->loadAsset<SGCore::ModelAsset>("${enginePath}/Tests/ModelDraw/Resources/tec/scene.gltf");
        auto modelSkeletonAsset = SGCore::AssetManager::getInstance()->loadAsset<SGCore::Skeleton>("${enginePath}/Tests/ModelDraw/Resources/tec/scene.gltf/skeletons/GLTF_created_0_rootJoint");*/

        /*auto modelAsset = SGCore::AssetManager::getInstance()->loadAsset<SGCore::ModelAsset>("${enginePath}/Tests/ModelDraw/Resources/drone/scene.gltf");
        auto modelSkeletonAsset = SGCore::AssetManager::getInstance()->loadAsset<SGCore::Skeleton>("${enginePath}/Tests/ModelDraw/Resources/drone/scene.gltf/skeletons/GLTF_created_0_rootJoint");*/

        const auto entities = modelAsset->m_rootNode->addOnScene(SGCore::Scene::getCurrentScene());

        // adding animation
        {
            m_characterEntity = entities[0];

            auto characterEntityTransform = ecsRegistry->get<SGCore::Transform>(m_characterEntity);

            /*auto animations0 = SGCore::AssetManager::getInstance()->loadAsset<SGCore::AnimationsFile>("${enginePath}/Tests/ModelDraw/Resources/Walking.fbx");
            auto animations1 = SGCore::AssetManager::getInstance()->loadAsset<SGCore::AnimationsFile>("${enginePath}/Tests/ModelDraw/Resources/Idle.fbx");

            auto animations = SGCore::AssetManager::getInstance()->getAsset<SGCore::AnimationsFile, SGCore::AssetStorageType::BY_PATH>(
                    "${enginePath}/Tests/ModelDraw/Resources/Fast Run.fbx/animations"
            );*/

            /*auto animations0 = SGCore::AssetManager::getInstance()->loadAsset<SGCore::AnimationsFile>(
                "${enginePath}/Tests/ModelDraw/Resources/fsb_operator/scene.gltf/animations");*/

            // auto animations0 = SGCore::AssetManager::getInstance()->loadAsset<SGCore::AnimationsFile>("${enginePath}/Tests/ModelDraw/Resources/tec/scene.gltf/animations");

            auto animations0 = SGCore::AssetManager::getInstance()->loadAsset<SGCore::AnimationsFile>(demosPath / "Tests/ModelDraw/Resources/drone/scene.gltf/animations");
            // auto animations0 = SGCore::AssetManager::getInstance()->loadAsset<SGCore::AnimationsFile>("${enginePath}/Tests/ModelDraw/Resources/hu_tao_animated/scene.gltf/animations");

            auto& motionPlanner = SGCore::Scene::getCurrentScene()->getECSRegistry()->emplace<SGCore::MotionPlanner>(entities[0]);
            motionPlanner.m_skeleton = modelSkeletonAsset;

            auto& animationsTree = SGCore::Scene::getCurrentScene()->getECSRegistry()->emplace<SGCore::AnimationsTree>(entities[0]);

            auto& copterAudioSource = SGCore::Scene::getCurrentScene()->getECSRegistry()->emplace<SGCore::AudioSource>(
                entities[0]);
            copterAudioSource.create();
            copterAudioSource.attachAudioTrack(m_copterSound);
            copterAudioSource.setRolloffFactor(0.5f);
            copterAudioSource.setIsLooping(true);
            copterAudioSource.setState(SGCore::PlayableState::SG_PLAYING);
            copterAudioSource.setType(SGCore::AudioSourceType::SG_WORLD);

            // SGCore::Transform::reg_t& huTaoTransform = SGCore::Scene::getCurrentScene()->getECSRegistry()->get<SGCore::Transform>(entities[0]);

            // huTaoTransform->m_localTransform.m_scale = { 0.01, 0.01, 0.01 };

            // sample skeleton
            /*auto idleNode = SGCore::MotionPlannerNode::createNode();
            idleNode->m_isRepeated = true;
            idleNode->m_animationSpeed = 1.0f;
            idleNode->m_skeletalAnimation = animations1->m_skeletalAnimations[0];
            auto idleConnection = SGCore::MakeRef<SGCore::MotionPlannerConnection>();
            idleConnection->m_blendTime = 2.0f;
            idleConnection->m_doNotInterruptWhenInactive = true;
            idleNode->m_anyState.m_toRootConnection = idleConnection;

            auto walkNode = SGCore::MotionPlannerNode::createNode();
            walkNode->m_isRepeated = true;
            walkNode->m_animationSpeed = 1.0f;
            walkNode->m_skeletalAnimation = animations0->m_skeletalAnimations[0];

            auto runNode = SGCore::MotionPlannerNode::createNode();
            runNode->m_isRepeated = true;
            runNode->m_animationSpeed = 0.1f;
            runNode->m_skeletalAnimation = animations->m_skeletalAnimations[0];

            auto walkConnection = SGCore::MakeRef<SGCore::MotionPlannerConnection>();
            walkConnection->m_previousNode = idleNode;
            walkConnection->m_nextNode = walkNode;
            walkConnection->m_blendTime = 0.2f;
            auto walkActivationAction = SGCore::MakeRef<SGCore::KeyboardKeyDownAction>();
            walkActivationAction->m_key = SGCore::KeyboardKey::KEY_W;
            walkConnection->m_activationAction = walkActivationAction;

            auto runConnection = SGCore::MakeRef<SGCore::MotionPlannerConnection>();
            runConnection->m_previousNode = walkNode;
            runConnection->m_nextNode = runNode;
            runConnection->m_blendTime = 2.0f;
            auto runActivationAction = SGCore::MakeRef<SGCore::KeyboardKeyDownAction>();
            runActivationAction->m_key = SGCore::KeyboardKey::KEY_LEFT_SHIFT;
            runConnection->m_activationAction = runActivationAction;

            idleNode->m_connections.push_back(walkConnection);
            walkNode->m_connections.push_back(runConnection);*/

            auto idleNode = SGCore::MakeRef<SGCore::SkeletalAnimationNode>();
            idleNode->m_animationSpeed = 1.0f;
            idleNode->m_isLooping = true;
            idleNode->m_skeletalAnimation = animations0->m_skeletalAnimations[0];

            auto gotoNode0 = SGCore::MakeRef<SGCore::GotoAnimationNode>();
            gotoNode0->m_animationSpeed = 0.05f;
            gotoNode0->m_destination = glm::vec3 { 0.0f, 10.0f, 0.0f };
            gotoNode0->m_useBlend = true;
            gotoNode0->m_interpolate = true;

            auto gotoNode1 = SGCore::MakeRef<SGCore::GotoAnimationNode>();
            gotoNode1->m_animationSpeed = 0.05f;
            gotoNode1->m_destination = glm::vec3 { 0.0f, 10.0f, 10.0f };
            gotoNode1->m_useBlend = true;
            gotoNode1->m_interpolate = true;

            auto gotoConnection0 = SGCore::MakeRef<SGCore::AnimationNodeConnection>();
            gotoConnection0->m_nextNode = gotoNode1;
            gotoConnection0->m_previousNode = gotoNode0;
            /*auto gotoConnection0Activation = SGCore::MakeRef<SGCore::WhenAnimationEndAction>();
            gotoConnection0Activation->m_animationNode = gotoNode0;
            gotoConnection0Activation->m_entity = m_characterEntity;
            gotoConnection0Activation->m_inRegistry = ecsRegistry;*/
            auto gotoConnection0Activation = SGCore::MakeRef<SGCore::AlwaysTrueAction>();
            gotoConnection0->m_activationAction = gotoConnection0Activation;

            gotoNode0->m_connections.push_back(gotoConnection0);

            m_testIdleNode = idleNode;

            animationsTree.m_rootNodes.push_back(idleNode);
            animationsTree.m_rootNodes.push_back(gotoNode0);
        }
    };

    // SGCore::AssetManager::getInstance()->loadAsset<SGCore::ModelAsset>(modelAsset, SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Tests/ModelDraw/Resources/fsb_operator/scene.gltf");

    // SGCore::AssetManager::getInstance()->loadAsset<SGCore::ModelAsset>(modelAsset, SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Tests/ModelDraw/Resources/Fast Run.fbx");

    SGCore::AssetManager::getInstance()->loadAsset<SGCore::ModelAsset>(modelAsset, SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, demosPath / "Tests/ModelDraw/Resources/drone/scene.gltf");

    // SGCore::AssetManager::getInstance()->loadAsset<SGCore::ModelAsset>(modelAsset, SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Tests/ModelDraw/Resources/hu_tao_animated/scene.gltf");

    // SGCore::AssetManager::getInstance()->loadAsset<SGCore::ModelAsset>(modelAsset, SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Models/vss/scene.gltf");

    // gif animation test ======================================

    auto testGif = mainAssetManager->loadAsset<SGCore::GIF>(demosPath / "Tests/ModelDraw/Resources/test0.gif");
    std::cout << "loaded gif with " << std::to_string(testGif->m_sequence.m_frames.size()) << " frames" << std::endl;

    auto cubeTestMaterial = mainAssetManager->getOrAddAssetByAlias<SGCore::IMaterial>("cube_test_material");
    cubeTestMaterial->m_meshRenderState.m_useFacesCulling = false;

    std::vector<SGCore::ECS::entity_t> cubeEntities;
    SGCore::ECS::entity_t cubeMeshEntity;
    auto cubeModel = mainAssetManager->loadAssetWithAlias<SGCore::ModelAsset>(
        "cube_model",
        "${enginePath}/Resources/models/standard/cube.obj"
    );
    cubeModel->m_rootNode->addOnScene(scene, [&](SGCore::ECS::entity_t entity) {
        cubeEntities.push_back(entity);
        if(ecsRegistry->allOf<SGCore::Mesh>(entity))
        {
            cubeMeshEntity = entity;
        }
    });

    const auto cubeRootEntity = cubeEntities[0];

    auto cubeTransform = ecsRegistry->get<SGCore::Transform>(cubeRootEntity);
    cubeTransform->m_localTransform.m_position = { 10, 0, 0 };

    auto& cubeMesh = ecsRegistry->get<SGCore::Mesh>(cubeMeshEntity);
    cubeMesh.m_base.setMaterial(cubeTestMaterial);

    m_roboarmModel = SGCore::AssetManager::getInstance()->loadAsset<SGCore::ModelAsset>(
        demosPath / "Tests/ModelDraw/Resources/roboarm_0/scene.gltf");
    /*m_roboarmSkeleton = SGCore::AssetManager::getInstance()->loadAsset<SGCore::Skeleton>(
        demosPath / "Tests/ModelDraw/Resources/roboarm/Roboarm.gltf/skeletons/RoboArm"
    );*/
    auto roboarmAnimations = SGCore::AssetManager::getInstance()->loadAsset<SGCore::AnimationsFile>(demosPath / "Tests/ModelDraw/Resources/roboarm_0/scene.gltf/animations");

    if(!roboarmAnimations)
    {
        std::cout << "Failed to load roboarm animations" << std::endl;
    }

    m_roboarmEntity = m_roboarmModel->m_rootNode->addOnScene(scene)[0];

    auto roboarmTransform = ecsRegistry->get<SGCore::Transform>(m_roboarmEntity);
    roboarmTransform->m_localTransform.m_scale *= 12.0f;

    auto& roboarmInfo = ecsRegistry->get<SGCore::EntityBaseInfo>(m_roboarmEntity);

    auto& roboarmMotionPlanner = ecsRegistry->emplace<SGCore::MotionPlanner>(m_roboarmEntity);
    // roboarmMotionPlanner.m_skeleton = m_roboarmSkeleton;

    auto& animationsTree = SGCore::Scene::getCurrentScene()->getECSRegistry()->emplace<SGCore::AnimationsTree>(m_roboarmEntity);

    auto roboarm_rootJoint = roboarmInfo.findEntity(*ecsRegistry, "_rootJoint");
    ecsRegistry->emplace<SGCore::IKRootJoint>(roboarm_rootJoint);

    auto roboarm_joint1_00 = roboarmInfo.findEntity(*ecsRegistry, "joint1_00");
    ecsRegistry->emplace<SGCore::IKJoint>(roboarm_joint1_00);

    auto roboarm_joint2_01 = roboarmInfo.findEntity(*ecsRegistry, "joint2_01");
    ecsRegistry->emplace<SGCore::IKJoint>(roboarm_joint2_01);

    auto roboarm_joint3_02 = roboarmInfo.findEntity(*ecsRegistry, "joint3_02");
    ecsRegistry->emplace<SGCore::IKJoint>(roboarm_joint3_02);

    auto roboarm_joint4_03 = roboarmInfo.findEntity(*ecsRegistry, "joint4_03");
    ecsRegistry->emplace<SGCore::IKJoint>(roboarm_joint4_03);

    auto roboarm_joint5_04 = roboarmInfo.findEntity(*ecsRegistry, "joint5_04");
    ecsRegistry->emplace<SGCore::IKJoint>(roboarm_joint5_04);

    auto roboarm_joint6_05 = roboarmInfo.findEntity(*ecsRegistry, "joint6_05");
    auto& endJoint = ecsRegistry->emplace<SGCore::IKJoint>(roboarm_joint6_05);
    auto endJointTransform = ecsRegistry->get<SGCore::Transform>(roboarm_joint6_05);

    endJoint.m_isEndJoint = true;

    endJointTransform->m_localTransform.m_position += glm::vec3 { -100.0f, -100.0f, -100.0f };

    m_roboarmJoints.push_back(roboarm_rootJoint);
    m_roboarmJoints.push_back(roboarm_joint1_00);
    m_roboarmJoints.push_back(roboarm_joint2_01);
    m_roboarmJoints.push_back(roboarm_joint3_02);
    m_roboarmJoints.push_back(roboarm_joint4_03);
    m_roboarmJoints.push_back(roboarm_joint5_04);
    m_roboarmJoints.push_back(roboarm_joint6_05);

    /*auto takeNode = SGCore::MakeRef<SGCore::SkeletalAnimationNode>();
    takeNode->m_animationSpeed = 1.0f;
    takeNode->m_isLooping = true;
    takeNode->m_skeletalAnimation = roboarmAnimations->m_skeletalAnimations[0];

    animationsTree.m_rootNodes.push_back(takeNode);*/

    // testGif->m_sequence.calculateDelays(0.05f);

    /*auto& cubeAnimation = ecsRegistry->emplace<SGCore::FrameAnimation>(cubeMeshEntity);
    cubeAnimation.m_source = testGif;
    cubeAnimation.m_isRepeated = true;

    cubeTestMaterial->addTexture2D(SGTextureSlot::SGTT_DIFFUSE, testGif->m_sequence.m_frames[0].m_texture);*/
}

void App::onUpdate(double dt, double fixedDt)
{
    const auto currentScene = SGCore::Scene::getCurrentScene();

    if(currentScene && currentScene->getECSRegistry()->valid(m_characterEntity))
    {
        auto& characterTransform = SGCore::Scene::getCurrentScene()->getECSRegistry()->get<SGCore::Transform>(m_characterEntity);

        const float characterSpeed = 3.0f;

        if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_UP))
        {
            characterTransform->m_localTransform.m_position += characterTransform->m_worldTransform.m_up * characterSpeed * dt;
        }
        if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_DOWN))
        {
            characterTransform->m_localTransform.m_position -= characterTransform->m_worldTransform.m_up * characterSpeed * dt;
        }
        if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_LEFT))
        {
            characterTransform->m_localTransform.m_position -= characterTransform->m_worldTransform.m_right * characterSpeed * dt;
        }
        if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_RIGHT))
        {
            characterTransform->m_localTransform.m_position += characterTransform->m_worldTransform.m_right * characterSpeed * dt;
        }

        if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_M))
        {
            moveSmoothly(m_characterEntity, characterTransform->m_localTransform.m_position + characterTransform->m_localTransform.m_up * 10.0f, 0.01f);
        }
    }

    if(currentScene)
    {
        const auto ecsRegistry = currentScene->getECSRegistry();
        const auto debugDraw = SGCore::RenderPipelinesManager::instance().getCurrentRenderPipeline()->getRenderPass<SGCore::DebugDraw>();

        for(auto joint : m_roboarmJoints)
        {
            auto jointTransform = ecsRegistry->get<SGCore::Transform>(joint);

            // debugDraw->drawLine(jointTransform->m_worldTransform.m_position, jointTransform->m_worldTransform.m_position + -jointTransform->m_worldTransform.m_right * 0.2f, { 1.0f, 1.0f, 0.0f, 1.0f });
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

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_3))
    {
        if(m_testIdleNode->getState() == SGCore::PlayableState::SG_PAUSED)
        {
            m_testIdleNode->setState(SGCore::PlayableState::SG_PLAYING);
        }
        else
        {
            m_testIdleNode->setState(SGCore::PlayableState::SG_PAUSED);
        }
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_4))
    {
        auto& frameReceiver = currentScene->getECSRegistry()->get<SGCore::LayeredFrameReceiver>(getCameraEntity());
        auto ssaoEffect = frameReceiver.getDefaultLayer()->getEffect<SGCore::SSAO>();
        ssaoEffect->setEnabled(!ssaoEffect->isEnabled());
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_MINUS))
    {
        m_testIdleNode->m_animationSpeed -= 0.1f;
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_KP_ADD))
    {
        m_testIdleNode->m_animationSpeed += 0.1f;
    }

    if(SGCore::Input::PC::mouseButtonPressed(SGCore::Input::MouseButton::MOUSE_BUTTON_LEFT))
    {

    }
}

void App::onFixedUpdate(double dt, double fixedDt)
{
}
