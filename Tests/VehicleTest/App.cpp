//
// Created by stuka on 25.03.2026.
//

#include "App.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/scalar_multiplication.hpp> // used for operator* for vec * double

#include <SGCore/Input/PCInput.h>
#include <SGCore/Physics/Rigidbody3D.h>
#include <SGCore/Scene/Scene.h>
#include <SGCore/Transformations/Controllable3D.h>
#include <SGCore/Physics/PhysicsWorld3D.h>

void App::onInit() noexcept
{
    const std::string demosPath = SG_STRINGIFY_MACRO(SUNGEAR_DEMOS_ROOT);

    auto scene = SGCore::Scene::getCurrentScene();
    auto ecsRegistry = scene->getECSRegistry();
    auto assetManager = SGCore::AssetManager::getInstance();

    ecsRegistry->remove<SGCore::Controllable3D>(getCameraEntity());

    m_cubeModel = assetManager->loadAssetWithAlias<SGCore::ModelAsset>(
        "cube_model",
        SGCore::CoreMain::getSungearEngineRootPath() / "Resources/models/standard/cube.obj"
    );

    const auto cubeEntity0 = m_cubeModel->m_rootNode->addOnScene(scene)[0];
    m_playerEntity = m_cubeModel->m_rootNode->addOnScene(scene)[0];

    static auto addShape = [](const SGCore::Ref<SGCore::Rigidbody3D>& rigidbody, float mass, const glm::vec3& halfExtents, SGCore::PhysicalObjectType objectType, glm::vec3 shapeOffset = {}) {
        auto shape = SGCore::MakeRef<btBoxShape>(btVector3 { halfExtents.x, halfExtents.y, halfExtents.z });
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

    auto physicsWorld = scene->getSystem<SGCore::PhysicsWorld3D>();

    auto cubeEntity0Transform = ecsRegistry->get<SGCore::Transform>(cubeEntity0);
    auto cubeEntity1Transform = ecsRegistry->get<SGCore::Transform>(m_playerEntity);

    auto cubeEntity0Rigidbody = ecsRegistry->emplace<SGCore::Rigidbody3D>(cubeEntity0, SGCore::MakeRef<SGCore::Rigidbody3D>(physicsWorld));
    auto cubeEntity1Rigidbody = ecsRegistry->emplace<SGCore::Rigidbody3D>(m_playerEntity, SGCore::MakeRef<SGCore::Rigidbody3D>(physicsWorld));

    cubeEntity1Rigidbody->m_body->setActivationState(DISABLE_DEACTIVATION);

    addShape(cubeEntity0Rigidbody, 0.0f, { 500.0f, 1.0f, 500.0f }, SGCore::PhysicalObjectType::OT_STATIC);
    addShape(cubeEntity1Rigidbody, 10.0f, { 1.0f, 1.0f, 1.0f }, SGCore::PhysicalObjectType::OT_DYNAMIC);

    cubeEntity0Transform->m_ownTransform.m_scale = { 1000.0f, 1.0f, 1000.0f};
    cubeEntity1Transform->m_ownTransform.m_scale = { 1.0f, 1.0f, 1.0f };
    cubeEntity1Transform->m_ownTransform.m_position.y += 10.0f;

    ecsRegistry->get<SGCore::EntityBaseInfo>(getCameraEntity()).setParent(m_playerEntity, *ecsRegistry);
    auto cameraTransform = ecsRegistry->get<SGCore::Transform>(getCameraEntity());
    cameraTransform->m_ownTransform.m_position = { 0.0f, 5.0f, 10.0f };
    cameraTransform->m_ownTransform.m_rotation *= glm::quat({ glm::radians(-20.0f), glm::radians(0.0f), glm::radians(0.0f) });
    // cameraTransform->m_ownTransform.m_yawPitchRoll = { -20.0f, 0.0f, 0.0f };
}

void App::onUpdate(double dt, double fixedDt)
{
    SGCore::CoreMain::getWindow().setTitle("Vehicle Test. FPS: " + std::to_string(SGCore::CoreMain::getFPS()));

    auto scene = SGCore::Scene::getCurrentScene();
    auto ecsRegistry = scene->getECSRegistry();

    auto playerTransform = ecsRegistry->get<SGCore::Transform>(m_playerEntity);
    auto playerRigidbody = ecsRegistry->get<SGCore::Rigidbody3D>(m_playerEntity);
    auto cameraTransform = ecsRegistry->get<SGCore::Transform>(getCameraEntity());

    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_W))
    {
        playerTransform->m_ownTransform.m_position += playerTransform->m_finalTransform.m_forward * m_playerSpeed * dt;
    }
    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_A))
    {
        playerTransform->m_ownTransform.m_position += playerTransform->m_finalTransform.m_right * m_playerSpeed * dt;
    }
    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_D))
    {
        playerTransform->m_ownTransform.m_position -= playerTransform->m_finalTransform.m_right * m_playerSpeed * dt;
    }
    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_S))
    {
        playerTransform->m_ownTransform.m_position -= playerTransform->m_finalTransform.m_forward * m_playerSpeed * dt;
    }

    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_R))
    {
        playerTransform->m_ownTransform.m_position = { 0.0f, 10.0f, 0.0f };
        playerTransform->m_ownTransform.m_rotation = glm::identity<glm::quat>();
    }

    if(SGCore::Input::PC::keyboardKeyPressed(SGCore::Input::KeyboardKey::KEY_SPACE))
    {
        playerRigidbody->m_body->applyCentralImpulse({ 0.0f, 1000.0f, 0.0f });
    }

    if(SGCore::Input::PC::keyboardKeyPressed(SGCore::Input::KeyboardKey::KEY_ESCAPE))
    {
        SGCore::CoreMain::getWindow().setHideAndCentralizeCursor(!SGCore::CoreMain::getWindow().isHideAndCentralizeCursor());
    }

    const glm::vec3 playerRotation = {
        0.0f,
        -glm::radians((float) SGCore::Input::PC::getCursorPositionDeltaX() * m_mouseSensitivity),
        0.0f
    };

    const glm::vec3 cameraRotation = {
        -glm::radians((float) SGCore::Input::PC::getCursorPositionDeltaY() * m_mouseSensitivity),
        0.0f,
        0.0f
    };

    playerTransform->m_ownTransform.m_rotation *= glm::quat(playerRotation);
    cameraTransform->m_ownTransform.m_rotation *= glm::quat(cameraRotation);
}

void App::onFixedUpdate(double dt, double fixedDt)
{

}