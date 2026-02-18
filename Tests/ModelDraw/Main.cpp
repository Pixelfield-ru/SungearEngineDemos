//
// Created by stuka on 06.02.2025.
//

#include "Main.h"

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/ext.hpp>

#include <SGCore/Memory/AssetManager.h>
#include <SGCore/UI/CSS/CSSFile.h>
#include <SGCore/Main/CoreMain.h>
#include <SGCore/Scene/Scene.h>
#include <SGCore/Render/RenderPipelinesManager.h>
#include <SGCore/Render/PBRRP/PBRRenderPipeline.h>
#include <SGCore/Render/LayeredFrameReceiver.h>
#include <SGCore/Render/RenderingBase.h>

#include <SGCore/Actions/KeyboardKeyDownAction.h>
#include <SGCore/Actions/KeyboardKeyReleasedAction.h>
#include <SGCore/Motion/MotionPlanner.h>
#include <SGCore/Motion/MotionPlannerConnection.h>
#include <SGCore/Audio/AudioListener.h>
#include <SGCore/Audio/AudioSource.h>
#include <SGCore/Coro/CoroScheduler.h>
#include <SGCore/Graphics/API/IFrameBuffer.h>
#include <SGCore/Graphics/API/ITexture2D.h>
#include <SGCore/Input/PCInput.h>
#include <SGCore/Memory/Assets/ModelAsset.h>
#include <SGCore/Render/Camera3D.h>
#include <SGCore/Render/Alpha/TransparentEntityTag.h>
#include <SGCore/Render/Picking/Pickable.h>
#include <SGCore/Render/SpacePartitioning/IgnoreOctrees.h>
#include <SGCore/Serde/Components/NonSavable.h>
#include <SGCore/Transformations/Controllable3D.h>
#include <SGCore/Render/Mesh.h>
#include <SGCore/Render/Atmosphere/Atmosphere.h>
#include <SGCore/Memory/Assets/Materials/IMaterial.h>
#include <SGCore/Graphics/API/ICubemapTexture.h>
#include <SGCore/Memory/Assets/AnimationsFile.h>
#include <SGCore/Memory/Assets/AudioTrackAsset.h>
#include <SGCore/Coro/Task.h>
#include <SGCore/Memory/Assets/GIF.h>
#include <SGCore/Animation2D/FrameAnimation.h>

#include "App.h"

#if SG_PLATFORM_OS_WINDOWS
#ifdef __cplusplus
extern "C" {
#endif
#include <windows.h>
    __declspec(dllexport) DWORD NvOptimusEnablement = 1;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
#ifdef __cplusplus
}
#endif
#endif

int main()
{
    App app;
    app.start(true);

    return 0;
}
