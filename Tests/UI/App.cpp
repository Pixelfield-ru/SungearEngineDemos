//
// Created by stuka on 23.03.2026.
//

#include "App.h"

#include <SGCore/Graphics/API/ITexture2D.h>
#include <SGCore/Input/PCInput.h>
#include <SGCore/Scene/Scene.h>
#include <SGCore/UI/UIComponent.h>
#include <SGCore/UI/UIDocument.h>
#include <SGCore/Utils/Macroses.h>
#include <SGCore/Render/RenderingBase.h>

std::string App::m_myText;

void App::onInit() noexcept
{
    const std::string demosPath = SG_STRINGIFY_MACRO(SUNGEAR_DEMOS_ROOT);

    m_cssFile = SGCore::AssetManager::getInstance()->loadAsset<SGCore::UI::CSSFile>(demosPath / "Tests/UI/Resources/test.css");
    m_uiDocument = SGCore::AssetManager::getInstance()->loadAsset<SGCore::UI::UIDocument>(demosPath / "Tests/UI/Resources/test.xml");
    m_testTexture = SGCore::AssetManager::getInstance()->loadAsset<SGCore::ITexture2D>("${enginePath}/Resources/textures/no_material.png");

    m_uiDocument->m_bindingsStorage.bind("myText", &m_myText);

    auto ecsRegistry = SGCore::Scene::getCurrentScene()->getECSRegistry();

    auto uiEntity = ecsRegistry->create();

    auto& uiComponent = ecsRegistry->emplace<SGCore::UI::UIComponent>(uiEntity);
    uiComponent.m_document = m_uiDocument;
    uiComponent.m_attachedToCamera = getCameraEntity();

    glfwInit();

    glfwSetCharCallback(SGCore::CoreMain::getWindow().getNativeHandle(), [](GLFWwindow* window, unsigned int c) {
        std::cout << "char: " << c << std::endl;

        m_myText += c;
    });

    const char* error { };
    glfwGetError(&error);

    if(error)
    {
        std::cout << "glfw error after glfwSetCharCallback: " << error << std::endl;
    }

    SGCore::Input::PC::onKeyboardKeyEvent() += [](SGCore::Window& inWindow, SGCore::Input::KeyboardKey key, int scancode, SGCore::Input::KeyState state, int mods) {
        if(key == SGCore::Input::KeyboardKey::KEY_BACKSPACE && (state == SGCore::Input::KeyState::KS_REPEAT || state == SGCore::Input::KeyState::KS_PRESSED) && !m_myText.empty())
        {
            m_myText.erase(m_myText.length() - 1);
        }
        else if(key == SGCore::Input::KeyboardKey::KEY_ENTER && (state == SGCore::Input::KeyState::KS_REPEAT || state == SGCore::Input::KeyState::KS_PRESSED))
        {
            m_myText += U'\n';
        }
    };
}

void App::onUpdate(double dt, double fixedDt) noexcept
{
    SGCore::CoreMain::getWindow().setTitle("UI Test. FPS: " + std::to_string(SGCore::CoreMain::getFPS()));

    auto ecsRegistry = SGCore::Scene::getCurrentScene()->getECSRegistry();

    auto& cameraRenderingBase = ecsRegistry->get<SGCore::RenderingBase>(getCameraEntity());

    int windowSizeX = 0;
    int windowSizeY = 0;

    SGCore::CoreMain::getWindow().getSize(windowSizeX, windowSizeY);

    cameraRenderingBase.m_left = -windowSizeX / 2.0f;
    cameraRenderingBase.m_right = windowSizeX / 2.0f;
    cameraRenderingBase.m_top = windowSizeY / 2.0f;
    cameraRenderingBase.m_bottom = -windowSizeY / 2.0f;
    cameraRenderingBase.m_zNear = -100;
    cameraRenderingBase.m_zFar = 100;

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_1))
    {
        m_uiDocument->reloadFromDisk();
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_2))
    {
        auto shaders = SGCore::AssetManager::getInstance()->getAssetsWithType<SGCore::IShader>();
        for(const auto& shader : shaders)
        {
            shader->reloadFromDisk();
        }
    }
}

void App::onFixedUpdate(double dt, double fixedDt) noexcept
{

}