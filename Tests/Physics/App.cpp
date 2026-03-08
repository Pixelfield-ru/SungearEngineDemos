//
// Created by stuka on 08.03.2026.
//

#include "App.h"

#include <SGCore/Input/PCInput.h>
#include <SGCore/Main/CoreMain.h>
#include <SGCore/Memory/AssetManager.h>
#include <SGCore/Memory/Assets/ModelAsset.h>
#include <SGCore/Motion/MotionPlanner.h>
#include <SGCore/Physics/Rigidbody3D.h>
#include <SGCore/Physics/PhysicsWorld3D.h>
#include <SGCore/Physics/Ragdoll3D.h>
#include <SGCore/Scene/Scene.h>

void App::createBallAndApplyImpulse(const glm::vec3& spherePos, const glm::vec3& impulse) noexcept
{
    auto scene = SGCore::Scene::getCurrentScene();
    auto ecsRegistry = scene->getECSRegistry();

    const auto sphereEntities = m_sphereModel->m_rootNode->addOnScene(scene);

    auto sphereRigidbody3D = ecsRegistry->emplace<SGCore::Rigidbody3D>(sphereEntities[2], SGCore::MakeRef<SGCore::Rigidbody3D>(scene->getSystem<SGCore::PhysicsWorld3D>()));

    SGCore::Ref<btSphereShape> sphereRigidbody3DShape = SGCore::MakeRef<btSphereShape>(1.0);
    btTransform shapeTransform;
    shapeTransform.setIdentity();
    sphereRigidbody3D->addShape(shapeTransform, sphereRigidbody3DShape);
    sphereRigidbody3D->setType(SGCore::PhysicalObjectType::OT_DYNAMIC);
    sphereRigidbody3D->m_body->setRestitution(0.9);
    sphereRigidbody3D->m_body->setFriction(1.0);
    btScalar mass = 100.0f;
    btVector3 inertia(0, 0, 0);
    sphereRigidbody3D->m_body->getCollisionShape()->calculateLocalInertia(mass, inertia);
    sphereRigidbody3D->m_body->setMassProps(mass, inertia);
    sphereRigidbody3D->reAddToWorld();

    glm::vec3 finalImpulse = impulse;
    sphereRigidbody3D->m_body->applyCentralImpulse({ finalImpulse.x, finalImpulse.y, finalImpulse.z });

    SGCore::Ref<SGCore::Transform>& sphereTransform = ecsRegistry->get<SGCore::Transform>(sphereEntities[0]);
    sphereTransform->m_ownTransform.m_position = spherePos;
}

void App::onInit() noexcept
{
    const std::filesystem::path demosPath = SG_STRINGIFY_MACRO(SUNGEAR_DEMOS_ROOT);

    auto scene = SGCore::Scene::getCurrentScene();
    auto ecsRegistry = scene->getECSRegistry();
    auto physicsWorld = scene->getSystem<SGCore::PhysicsWorld3D>();

    auto assetManager = SGCore::AssetManager::getInstance();

    m_cubeModel = assetManager->loadAssetWithAlias<SGCore::ModelAsset>(
        "cube_model",
        SGCore::CoreMain::getSungearEngineRootPath() / "Resources/models/standard/cube.obj"
    );

    m_sphereModel = assetManager->loadAssetWithAlias<SGCore::ModelAsset>(
        "sphere_model",
        "${enginePath}/Resources/models/standard/sphere.obj"
    );

    m_humanModel = assetManager->loadAsset<SGCore::ModelAsset>(
        demosPath / "Tests/ModelDraw/Resources/Idle.fbx"
    );

    m_humanSkeleton = SGCore::AssetManager::getInstance()->loadAsset<SGCore::Skeleton>(
        demosPath / "Tests/ModelDraw/Resources/Idle.fbx/skeletons/mixamorig:Hips"
    );

    // ================== creating player
    const auto cubeEntities = m_cubeModel->m_rootNode->addOnScene(scene);

    m_playerEntity = cubeEntities[0];

    auto playerTransform = ecsRegistry->get<SGCore::Transform>(m_playerEntity);

    playerTransform->m_ownTransform.m_position = { 300, 10.0f, 0.0f };
    playerTransform->m_ownTransform.m_scale = { 1.0f, 1.8f, 1.0f };

    auto playerRigidbody3D = ecsRegistry->emplace<SGCore::Rigidbody3D>(m_playerEntity, SGCore::MakeRef<SGCore::Rigidbody3D>(physicsWorld));

    // ===============================
    // ====================== creating floor

    const auto floorEntities = m_cubeModel->m_rootNode->addOnScene(scene);

    m_floorEntity = floorEntities[0];

    auto floorTransform = ecsRegistry->get<SGCore::Transform>(m_floorEntity);

    floorTransform->m_ownTransform.m_scale = { 250.0f, 1.0f, 250.0f };
    floorTransform->m_ownTransform.m_position = { 0, -50, 0 };

    auto floorRigidbody3D = ecsRegistry->emplace<SGCore::Rigidbody3D>(m_floorEntity, SGCore::MakeRef<SGCore::Rigidbody3D>(physicsWorld));

    auto floorShape = SGCore::MakeRef<btBoxShape>(btVector3(250.0f, 1.0f, 250.0f));
    btTransform shapeTransform;
    shapeTransform.setIdentity();
    floorRigidbody3D->addShape(shapeTransform, floorShape);
    floorRigidbody3D->setType(SGCore::PhysicalObjectType::OT_STATIC);
    floorRigidbody3D->m_body->setRestitution(0.2f);
    floorRigidbody3D->m_body->setFriction(1.0f);
    floorRigidbody3D->m_body->setMassProps(0.0f, { 0.0f, 0.0f, 0.0f });
    floorRigidbody3D->reAddToWorld();

    // ====================== creating human
    const auto humanEntities = m_humanModel->m_rootNode->addOnScene(scene);

    m_humanEntity = humanEntities[0];

    auto humanTransform = ecsRegistry->get<SGCore::Transform>(m_humanEntity);
    auto& humanRagdoll = ecsRegistry->emplace<SGCore::Ragdoll3D>(m_humanEntity, physicsWorld);
    auto& motionPlanner = SGCore::Scene::getCurrentScene()->getECSRegistry()->emplace<SGCore::MotionPlanner>(m_humanEntity);
    auto& humanEntityInfo = ecsRegistry->get<SGCore::EntityBaseInfo>(m_humanEntity);

    motionPlanner.m_skeleton = m_humanSkeleton;

    humanTransform->m_ownTransform.m_scale *= 0.1f;
    // humanTransform->m_ownTransform.m_position.y = -25.0f;
    humanTransform->m_ownTransform.m_position.y = 50.0f;

    const auto& humanBones = m_humanSkeleton->getAllBones();

    static auto addBoneShape = [](const SGCore::Ref<SGCore::Rigidbody3D>& boneRigidbody, float mass, float radius, SGCore::PhysicalObjectType objectType) {
        SGCore::Ref<btSphereShape> sphereRigidbody3DShape = SGCore::MakeRef<btSphereShape>(radius);
        btTransform shapeTransform;
        shapeTransform.setIdentity();
        boneRigidbody->addShape(shapeTransform, sphereRigidbody3DShape);
        boneRigidbody->setType(objectType);
        boneRigidbody->m_body->setRestitution(0.0);
        boneRigidbody->m_body->setFriction(0.8f);
        btVector3 inertia(0, 0, 0);
        boneRigidbody->m_body->getCollisionShape()->calculateLocalInertia(mass, inertia);
        boneRigidbody->m_body->setMassProps(mass, inertia);
        boneRigidbody->reAddToWorld();
    };

    const auto createBoneRigidbody = [&](SGCore::ECS::entity_t boneEntity) {
        return ecsRegistry->emplace<SGCore::Rigidbody3D>(boneEntity, SGCore::MakeRef<SGCore::Rigidbody3D>(physicsWorld));
    };

    const auto hips = humanEntityInfo.findEntity(*ecsRegistry, "mixamorig:Hips");
    auto hipsRigidbody = createBoneRigidbody(hips);
    addBoneShape(hipsRigidbody, 100.0f, 1.0f, SGCore::PhysicalObjectType::OT_DYNAMIC);

    const auto spine = humanEntityInfo.findEntity(*ecsRegistry, "mixamorig:Spine");
    auto spineRigidbody = createBoneRigidbody(spine);
    addBoneShape(spineRigidbody, 10.0f, 1.0f, SGCore::PhysicalObjectType::OT_DYNAMIC);
    // spineRigidbody->m_body->setGravity({ 0.0, 0.0, 0.0 });

    /*auto constraintInfo = humanRagdoll.addConstraint(hips, spine, *ecsRegistry);
    constraintInfo.m_constraint->setLimit(0, 100.0f);
    constraintInfo.m_constraint->setLimit(1, 100.0f);
    constraintInfo.m_constraint->setLimit(2, 100.0f);*/

    /*for(const auto& bone : humanBones)
    {
        std::cout << bone->getName() << std::endl;

        const auto boneEntity = humanEntityInfo.findEntity(*ecsRegistry, bone->getName());
        const auto boneParentEntity = bone->m_parent ? humanEntityInfo.findEntity(*ecsRegistry, bone->m_parent->getName()) : entt::null;
        if(boneEntity == entt::null) continue;

        auto boneRigidbody = ecsRegistry->emplace<SGCore::Rigidbody3D>(boneEntity, SGCore::MakeRef<SGCore::Rigidbody3D>(physicsWorld));

        addBoneShape(boneRigidbody, 10.0f, 0.1f);

        if(boneParentEntity != entt::null)
        {
            auto boneParentRigidbody = ecsRegistry->get<SGCore::Rigidbody3D>(boneParentEntity);

            auto constraintInfo = humanRagdoll.addConstraint(boneParentEntity, boneEntity, *ecsRegistry);
            // constraintInfo.m_constraint->buildJacobian();

            std::println(std::cout, "add constraint: {} + {}", std::to_underlying(boneParentEntity), std::to_underlying(boneEntity));
        }
    }*/
}

void App::onUpdate(double dt, double fixedDt) noexcept
{
    auto scene = SGCore::Scene::getCurrentScene();
    auto ecsRegistry = scene->getECSRegistry();
    auto physicsWorld = scene->getSystem<SGCore::PhysicsWorld3D>();

    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_1))
    {
        auto& cameraTransform = ecsRegistry->get<SGCore::Transform>(getCameraEntity());
        createBallAndApplyImpulse(cameraTransform->m_ownTransform.m_position, cameraTransform->m_ownTransform.m_forward * 200000.0f / 10.0f);
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_M))
    {
        auto& physicsWorld3DDebug = scene->getSystem<SGCore::PhysicsWorld3D>()->getDebugDraw();
        if (physicsWorld3DDebug->getDebugMode() == btIDebugDraw::DBG_NoDebug)
        {
            physicsWorld3DDebug->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
        }
        else
        {
            physicsWorld3DDebug->setDebugMode(btIDebugDraw::DBG_NoDebug);
        }
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_2))
    {
        physicsWorld->m_simulate = !physicsWorld->m_simulate;
    }
}

void App::onFixedUpdate(double dt, double fixedDt) noexcept
{

}