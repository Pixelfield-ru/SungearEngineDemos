//
// Created by stuka on 08.03.2026.
//

#include "App.h"

#define GLM_ENABLE_EXPERIMENTAL

#include <glm/gtx/string_cast.hpp>

#include <glm/gtx/euler_angles.hpp>
#include <SGCore/Input/PCInput.h>
#include <SGCore/Main/CoreMain.h>
#include <SGCore/Memory/AssetManager.h>
#include <SGCore/Memory/Assets/ModelAsset.h>
#include <SGCore/Motion/MotionPlanner.h>
#include <SGCore/Physics/Rigidbody3D.h>
#include <SGCore/Physics/PhysicsWorld3D.h>
#include <SGCore/Physics/Ragdoll3D.h>
#include <SGCore/Scene/Scene.h>
#include <SGCore/Transformations/TransformationsUpdater.h>
#include <SGCore/Transformations/TransformUtils.h>

#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>

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

    SGCore::TransformationsUpdater;

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
    auto& humanRagdoll = ecsRegistry->emplace<SGCore::Ragdoll3D>(m_humanEntity);
    auto& motionPlanner = SGCore::Scene::getCurrentScene()->getECSRegistry()->emplace<SGCore::MotionPlanner>(m_humanEntity);
    auto& humanEntityInfo = ecsRegistry->get<SGCore::EntityBaseInfo>(m_humanEntity);

    motionPlanner.m_skeleton = m_humanSkeleton;

    humanTransform->m_ownTransform.m_scale *= 0.1f;
    // humanTransform->m_ownTransform.m_position.y = -25.0f;
    humanTransform->m_ownTransform.m_position.y = 50.0f;

    const auto& humanBones = m_humanSkeleton->getAllBones();

    static auto addShape = [](const SGCore::Ref<SGCore::Rigidbody3D>& rigidbody, float mass, float radius, SGCore::PhysicalObjectType objectType, glm::vec3 shapeOffset = {}) {
        auto shape = SGCore::MakeRef<btSphereShape>(radius);
        btTransform shapeTransform;
        shapeTransform.setIdentity();
        shapeTransform.setOrigin({ shapeOffset.x, shapeOffset.y, shapeOffset.z });
        rigidbody->addShape(shapeTransform, shape);
        rigidbody->setType(objectType);
        rigidbody->m_body->setRestitution(0.0);
        rigidbody->m_body->setFriction(0.8f);
        btVector3 inertia(0, 0, 0);
        rigidbody->m_body->getCollisionShape()->calculateLocalInertia(mass, inertia);
        rigidbody->m_body->setMassProps(mass, inertia);
        rigidbody->reAddToWorld();
    };

    static auto addCapsuleShape = [](const SGCore::Ref<SGCore::Rigidbody3D>& rigidbody, float mass, float radius, float height, SGCore::PhysicalObjectType objectType, glm::vec3 shapeOffset = {}) {
        auto shape = SGCore::MakeRef<btCapsuleShape>(radius, height);
        btTransform shapeTransform;
        shapeTransform.setIdentity();
        shapeTransform.setOrigin({ shapeOffset.x, shapeOffset.y, shapeOffset.z });
        rigidbody->addShape(shapeTransform, shape);
        rigidbody->setType(objectType);
        rigidbody->m_body->setRestitution(0.0);
        rigidbody->m_body->setFriction(0.8f);
        btVector3 inertia(0, 0, 0);
        rigidbody->m_body->getCollisionShape()->calculateLocalInertia(mass, inertia);
        rigidbody->m_body->setMassProps(mass, inertia);
        rigidbody->reAddToWorld();
    };

    const auto createRigidbody = [&](SGCore::ECS::entity_t entity) {
        return ecsRegistry->emplace<SGCore::Rigidbody3D>(entity, SGCore::MakeRef<SGCore::Rigidbody3D>(physicsWorld));
    };

    const auto sphere0Entities = m_sphereModel->m_rootNode->addOnScene(scene);
    const auto sphere1Entities = m_sphereModel->m_rootNode->addOnScene(scene);
    const auto sphere2Entities = m_sphereModel->m_rootNode->addOnScene(scene);

    m_sphere0Entity = sphere0Entities[0];
    m_sphere1Entity = sphere1Entities[0];
    m_sphere2Entity = sphere2Entities[0];

    auto sphere0Transform =  ecsRegistry->get<SGCore::Transform>(m_sphere0Entity);
    auto sphere1Transform =  ecsRegistry->get<SGCore::Transform>(m_sphere1Entity);
    auto sphere2Transform =  ecsRegistry->get<SGCore::Transform>(m_sphere2Entity);


    sphere0Transform->m_ownTransform.m_position.y += 20.0f;

    sphere1Transform->m_ownTransform.m_scale *= 2.0f;
    sphere1Transform->m_ownTransform.m_position.x = 20.0f;

    sphere2Transform->m_ownTransform.m_scale *= 2.0f;
    sphere2Transform->m_ownTransform.m_position.x = -20.0f;
    sphere2Transform->m_ownTransform.m_position.z = -20.0f;

    const auto sphere0Rigidbody = createRigidbody(m_sphere0Entity);
    const auto sphere1Rigidbody = createRigidbody(m_sphere1Entity);
    const auto sphere2Rigidbody = createRigidbody(m_sphere2Entity);

    addShape(sphere0Rigidbody, 100.0f, 1.0f, SGCore::PhysicalObjectType::OT_DYNAMIC);
    addShape(sphere1Rigidbody, 100.0f, 4.0f, SGCore::PhysicalObjectType::OT_DYNAMIC);
    addShape(sphere2Rigidbody, 100.0f, 2.0f, SGCore::PhysicalObjectType::OT_DYNAMIC);

    auto& sphere1BaseInfo = ecsRegistry->get<SGCore::EntityBaseInfo>(m_sphere1Entity);
    auto& sphere2BaseInfo = ecsRegistry->get<SGCore::EntityBaseInfo>(m_sphere2Entity);
    // sphere1BaseInfo.setParent(m_sphere0Entity, *ecsRegistry);
    // sphere1BaseInfo.setParent(sphere0Entities[1], *ecsRegistry);
    sphere1BaseInfo.setParent(m_sphere2Entity, *ecsRegistry);
    sphere2BaseInfo.setParent(m_sphere0Entity, *ecsRegistry);

    // adding ragdoll

    const auto hips = humanEntityInfo.findEntity(*ecsRegistry, "mixamorig:Hips");
    auto hipsRigidbody = createRigidbody(hips);
    addShape(hipsRigidbody, 15.0f, 1.0f, SGCore::PhysicalObjectType::OT_DYNAMIC, { 0.0f, 0.0f, 0.0f });

    const auto spine = humanEntityInfo.findEntity(*ecsRegistry, "mixamorig:Spine");
    auto spineRigidbody = createRigidbody(spine);
    // addShape(spineRigidbody, 8.0f, 1.0f, SGCore::PhysicalObjectType::OT_DYNAMIC);
    addCapsuleShape(spineRigidbody, 8.0f, 1.0f, 2.0f, SGCore::PhysicalObjectType::OT_DYNAMIC, { 0.0f, 2.0f, 0.0f });
    // spineRigidbody->m_body->setGravity({ 0.0, 0.0, 0.0 });

    {
        auto constraintInfo = hipsRigidbody->addConeTwistConstraint(*spineRigidbody, *ecsRegistry, false);
        auto constraint = std::static_pointer_cast<btConeTwistConstraint>(constraintInfo.m_constraint);
        constraint->setLimit(glm::radians(90.0f), // swing span1 (вперед-назад)
                             glm::radians(30.0f), // swing span2 (в стороны)
                             0.8f);
    }

    {
        auto constraintInfo = hipsRigidbody->addPointToPointConstraint(*spineRigidbody, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, *ecsRegistry, false);
        auto constraint = std::static_pointer_cast<btPoint2PointConstraint>(constraintInfo.m_constraint);
        constraint->m_setting.m_tau = 0.1f;
    }

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
    auto sphere0Rigidbody = ecsRegistry->get<SGCore::Rigidbody3D>(m_sphere0Entity);
    auto sphere0Transform = ecsRegistry->get<SGCore::Transform>(m_sphere0Entity);
    auto sphere1Rigidbody = ecsRegistry->get<SGCore::Rigidbody3D>(m_sphere1Entity);
    auto sphere1Transform = ecsRegistry->get<SGCore::Transform>(m_sphere1Entity);

    // std::println(std::cout, "sphere1Transform pos: {}", glm::to_string(sphere1Transform->m_finalTransform.m_position));

    auto& cameraTransform = ecsRegistry->get<SGCore::Transform>(getCameraEntity());

    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_1))
    {
        createBallAndApplyImpulse(cameraTransform->m_finalTransform.m_position, cameraTransform->m_finalTransform.m_forward * 200000.0f / 10.0f);
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

    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_3))
    {
        /*auto& rigidbodyTransform = sphere0Rigidbody->m_body->getWorldTransform();

        const auto btRotation = rigidbodyTransform.getRotation();
        const auto glmRotation = glm::quat(btRotation.w(), btRotation.x(), btRotation.y(), btRotation.z());

        glm::vec3 eulerAngles(glm::radians(0.0f),
                          glm::radians(1.0f),
                          glm::radians(1.0f));

        const auto rotated = glm::quat(eulerAngles) * glmRotation;

        rigidbodyTransform.setRotation({ rotated.x, rotated.y, rotated.z, rotated.w });*/

        const auto impulseDir = cameraTransform->m_ownTransform.m_forward * 2000.0f / 10.0f;

        sphere0Rigidbody->m_body->applyCentralImpulse({ impulseDir.x, impulseDir.y, impulseDir.z });
    }

    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_4))
    {
        sphere0Transform->m_ownTransform.m_position += glm::vec3 { 1.0f };

        /*auto& rigidbodyTransform = sphere0Rigidbody->m_body->getWorldTransform();

        const auto btPos = rigidbodyTransform.getOrigin();
        auto glmPos = glm::vec3(btPos.x(), btPos.y(), btPos.z());

        glmPos += glm::vec3(1.0f, 1.0f, 0.0f);

        rigidbodyTransform.setOrigin({ glmPos.x, glmPos.y, glmPos.z });*/
    }

    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_5))
    {
        glm::vec3 eulerAngles(glm::radians(0.0f),
                          glm::radians(1.0f),
                          glm::radians(1.0f));

        sphere0Transform->m_ownTransform.m_rotation = glm::quat(eulerAngles) * sphere0Transform->m_ownTransform.m_rotation;


        // rigidbodyTransform.setRotation({ rotated.x, rotated.y, rotated.z, rotated.w });
    }

    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_6))
    {
        auto& rigidbodyTransform = sphere0Rigidbody->m_body->getWorldTransform();

        const auto rotated = glm::identity<glm::quat>();

        rigidbodyTransform.setRotation({ rotated.x, rotated.y, rotated.z, rotated.w });
    }

    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_7))
    {
        auto& rigidbodyTransform = sphere1Rigidbody->m_body->getWorldTransform();

        const auto btRotation = rigidbodyTransform.getRotation();
        const auto glmRotation = glm::quat(btRotation.w(), btRotation.x(), btRotation.y(), btRotation.z());

        glm::vec3 eulerAngles(glm::radians(0.0f),
                          glm::radians(1.0f),
                          glm::radians(1.0f));

        const auto rotated = glm::quat(eulerAngles) * glmRotation;

        rigidbodyTransform.setRotation({ rotated.x, rotated.y, rotated.z, rotated.w });
    }

    /*auto humanTransform = ecsRegistry->get<SGCore::Transform>(m_humanEntity);

    glm::vec3 eulerAngles(glm::radians(1.0f),
                          glm::radians(1.0f),
                          glm::radians(1.0f));

    humanTransform->m_ownTransform.m_rotation = glm::quat(eulerAngles) * humanTransform->m_ownTransform.m_rotation;*/
}

void App::onFixedUpdate(double dt, double fixedDt) noexcept
{
}