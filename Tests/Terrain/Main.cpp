//
// Created by stuka on 19.04.2025.
//

#include "Main.h"

#include <stb_image_write.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/string_cast.hpp>
#include <algorithm>

#include "SGCore/ECS/Utils.h"
#include "SGCore/Main/CoreMain.h"
#include "SGCore/Memory/AssetManager.h"
#include "SGCore/Memory/Assets/ModelAsset.h"
#include "SGCore/Render/Camera3D.h"
#include "SGCore/Render/RenderPipelinesManager.h"
#include "SGCore/Render/PBRRP/PBRRenderPipeline.h"
#include "SGCore/Scene/Scene.h"
#include "SGCore/Serde/Components/NonSavable.h"
#include "SGCore/Transformations/Controllable3D.h"
#include "SGCore/Graphics/API/ICubemapTexture.h"
#include "SGCore/Input/PCInput.h"
#include "SGCore/Memory/Assets/Materials/IMaterial.h"
#include "SGCore/Physics/Rigidbody3D.h"
#include "SGCore/Render/Alpha/TransparentEntityTag.h"
#include "SGCore/Render/Picking/Pickable.h"
#include "SGCore/Render/SpacePartitioning/IgnoreOctrees.h"
#include "SGCore/Render/RenderingBase.h"
#include "SGCore/Render/LayeredFrameReceiver.h"
#include "SGCore/Render/Mesh.h"
#include "SGCore/Render/Terrain/Terrain.h"
#include "SGCore/Render/Atmosphere/Atmosphere.h"
#include "SGCore/Physics/PhysicsWorld3D.h"

#include "SGCore/Navigation/NavGrid3D.h"
#include "SGCore/Navigation/NavMesh/NavMesh.h"
#include "SGCore/Navigation/NavMesh/Steps/InputFilteringStep.h"
#include "SGCore/Navigation/NavMesh/Steps/VoxelizationStep.h"
#include "SGCore/Render/DebugDraw.h"
#include "SGCore/Render/Lighting/SpotLight.h"
#include "SGCore/Render/Terrain/TerrainUtils.h"

#include "SGCore/Render/Volumetric/VolumetricFog.h"

#include "SGCore/Render/RenderAbilities/EnableDecalPass.h"
#include "SGCore/Render/RenderAbilities/EnableTerrainPass.h"
#include "SGCore/Render/RenderAbilities/EnableVolumetricPass.h"

#include "SGCore/Math/Noise/FastNoiseLite.h"

#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>

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

SGCore::Ref<SGCore::Scene> scene;
SGCore::AssetRef<SGCore::IShader> screenShader;
SGCore::Ref<SGCore::IMeshData> quadMeshData;
SGCore::AssetRef<SGCore::IMeshData> terrainMeshData;
SGCore::AssetRef<SGCore::ModelAsset> testModelAsset;
SGCore::MeshRenderState quadMeshRenderState;

SGCore::Ref<SGCore::ITexture2D> attachmentToDisplay;

SGCore::ECS::entity_t mainCamera;
SGCore::ECS::entity_t lightEntity;
SGCore::ECS::entity_t terrainEntity;
SGCore::ECS::entity_t atmosphereEntity;
SGCore::ECS::entity_t terrainDecalEntity;
SGCore::ECS::entity_t cloudsEntity;

SGCore::AssetRef<SGCore::ITexture2D> terrainHeightmapTex;

SGCore::AssetRef<SGCore::ITexture2D> terrainDecalDiffuseTex0;
SGCore::AssetRef<SGCore::ITexture2D> terrainDecalDiffuseTex1;
SGCore::AssetRef<SGCore::ITexture2D> terrainDecalAOTex;
SGCore::AssetRef<SGCore::ITexture2D> terrainDecalHeightTex;
SGCore::AssetRef<SGCore::ITexture2D> terrainDecalMetallicTex;
SGCore::AssetRef<SGCore::ITexture2D> terrainDecalNormalTex;
SGCore::AssetRef<SGCore::ITexture2D> terrainDecalRoughnessTex;

SGCore::AssetRef<SGCore::IMaterial> terrainDecalMaterial;

SGCore::AssetRef<SGCore::ITexture2D> terrainDiffuseTex;
SGCore::AssetRef<SGCore::ITexture2D> terrainDisplacementTex;
SGCore::AssetRef<SGCore::ITexture2D> terrainNormalsTex;
SGCore::AssetRef<SGCore::ITexture2D> terrainAORoughnessMetalTex;
SGCore::AssetRef<SGCore::ITexture2D> terrainTilingNoiseTex;

std::vector<SGCore::Vertex> terrainDisplacedVertices;
std::vector<std::uint32_t> terrainDisplacedIndices;

std::vector<float> terrainDisplacementData;

float terrainGrowSpeed = 0.001f;
float terrainLowerSpeed = 0.001f;

enum class TerrainOp
{
    TERRAIN_GROW,
    TERRAIN_LOWER
};

TerrainOp currentTerrainOp = TerrainOp::TERRAIN_GROW;

void createBallAndApplyImpulse(const glm::vec3& spherePos,
    const glm::vec3& impulse) noexcept
{
    auto sphereModel = SGCore::AssetManager::getInstance()->loadAsset<SGCore::ModelAsset>("sphere_model");

    std::vector<SGCore::ECS::entity_t> sphereEntities;
    sphereModel->m_rootNode->addOnScene(scene, SG_LAYER_OPAQUE_NAME,
        [&sphereEntities](const SGCore::ECS::entity_t& entity)
        {
            sphereEntities.push_back(entity);
        }
    );

    auto sphereRigidbody3D = scene->getECSRegistry()->emplace<SGCore::Rigidbody3D>(sphereEntities[2],
        SGCore::MakeRef<SGCore::Rigidbody3D>(
            scene->getSystem<SGCore::PhysicsWorld3D>()));

    SGCore::Ref<btSphereShape> sphereRigidbody3DShape = SGCore::MakeRef<btSphereShape>(1.0);
    btTransform sphereShapeTransform;
    sphereShapeTransform.setIdentity();
    sphereRigidbody3D->addShape(sphereShapeTransform, sphereRigidbody3DShape);
    sphereRigidbody3D->setType(SGCore::PhysicalObjectType::OT_DYNAMIC);
    sphereRigidbody3D->m_body->setRestitution(0.9);
    btScalar mass = 100.0f;
    btVector3 inertia(0, 0, 0);
    sphereRigidbody3D->m_body->getCollisionShape()->calculateLocalInertia(mass, inertia);
    sphereRigidbody3D->m_body->setMassProps(mass, inertia);
    sphereRigidbody3D->reAddToWorld();

    glm::vec3 finalImpulse = impulse;
    sphereRigidbody3D->m_body->applyCentralImpulse({ finalImpulse.x, finalImpulse.y, finalImpulse.z });

    SGCore::Ref<SGCore::Transform>& sphereTransform = scene->getECSRegistry()->get<SGCore::Transform>(sphereEntities[0]);
    sphereTransform->m_ownTransform.m_position = spherePos;
}

void regenerateTerrainPhysicalMesh(SGCore::ECS::entity_t terrainEntity)
{
    auto& terrainComponent = scene->getECSRegistry()->get<SGCore::Terrain>(terrainEntity);
    auto& terrainMesh = scene->getECSRegistry()->get<SGCore::Mesh>(terrainEntity);
    auto& terrainRigidbody = scene->getECSRegistry()->get<SGCore::Rigidbody3D>(terrainEntity);

    terrainRigidbody->removeFromWorld();

    // generating terrain physical mesh
    terrainComponent.generatePhysicalMesh(terrainMesh, 5);

    SGCore::Ref<btBvhTriangleMeshShape> terrainRigidbodyShape = SGCore::MakeRef<btBvhTriangleMeshShape>(terrainMeshData->m_physicalMesh.get(), true);
    btTransform terrainShapeTransform;
    terrainShapeTransform.setIdentity();
    terrainRigidbody->removeAllShapes();
    terrainRigidbody->addShape(terrainShapeTransform, terrainRigidbodyShape);

    terrainRigidbody->reAddToWorld();
}

void regenerateTerrainNavGrid(SGCore::ECS::entity_t terrainEntity)
{
    // remove to generate
    return;

    terrainDisplacedVertices.clear();
    terrainDisplacedIndices.clear();

    const auto ecsRegistry = scene->getECSRegistry();

    auto& terrainComponent = ecsRegistry->get<SGCore::Terrain>(terrainEntity);
    // auto& terrainNavGrid = ecsRegistry->get<SGCore::Navigation::NavGrid3D>(terrainEntity);
    auto& terrainNavMesh = ecsRegistry->get<SGCore::Navigation::NavMesh>(terrainEntity);

    SGCore::TerrainUtils::calculateVerticesUsingDisplacementMap(terrainComponent, terrainDisplacementTex.get(), 5, terrainDisplacedVertices, terrainDisplacedIndices);

    std::vector<SGCore::MathPrimitivesUtils::Triangle<>> inputTriangles;

    for(size_t i = 0; i < terrainDisplacedVertices.size(); i += 4)
    {
        const auto& v0 = terrainDisplacedVertices[terrainDisplacedIndices[i] + 0];
        const auto& v1 = terrainDisplacedVertices[terrainDisplacedIndices[i] + 1];
        const auto& v2 = terrainDisplacedVertices[terrainDisplacedIndices[i] + 2];
        const auto& v3 = terrainDisplacedVertices[terrainDisplacedIndices[i] + 3];

        SGCore::MathPrimitivesUtils::Triangle<> tri0 {
            .m_vertices = { v0.m_position, v1.m_position, v2.m_position }
        };

        SGCore::MathPrimitivesUtils::Triangle<> tri1 {
            .m_vertices = { v1.m_position, v2.m_position, v3.m_position }
        };

        tri0.calculateNormal();
        tri1.calculateNormal();

        inputTriangles.push_back(tri0);
        inputTriangles.push_back(tri1);
    }

    terrainNavMesh.useStandardSteps();
    terrainNavMesh.build(inputTriangles);


    // terrainNavGrid.build(terrainDisplacementTex.get(), terrainComponent.m_heightScale, terrainMeshData->m_aabb, *ecsRegistry);
    // terrainNavGrid.build(terrainDisplacedVertices, terrainDisplacedIndices, 4, terrainMeshData->m_aabb, *ecsRegistry);

    // std::cout << "terrain nav grid nodes count: " << terrainNavGrid.m_nodes.size() << std::endl;
}

std::pair<SGCore::AssetRef<SGCore::ITexture2D>, SGCore::AssetRef<SGCore::ITexture2D>> generateMultiOctaveCloudTexture3D(
    int lowFreqSize, int highFreqSize, int seed)
{
    FastNoiseLite noises[3];

    // низкочастотный шум перлина
    noises[0].SetNoiseType(FastNoiseLite::NoiseType_Perlin);
    noises[0].SetSeed(seed + 300);
    noises[0].SetFrequency(6.0f);
    noises[0].SetFractalType(FastNoiseLite::FractalType_FBm);
    noises[0].SetFractalOctaves(4);
    noises[0].SetFractalLacunarity(2.8f);
    noises[0].SetFractalGain(0.5f);

    // высокочастотный шум вороного
    noises[1].SetNoiseType(FastNoiseLite::NoiseType_Cellular);
    noises[1].SetSeed(seed + 1000);
    noises[1].SetFrequency(3.0f);
    noises[1].SetFractalType(FastNoiseLite::FractalType_FBm);
    noises[1].SetFractalOctaves(10);

    // низкочастотный шум вороного
    noises[2].SetNoiseType(FastNoiseLite::NoiseType_Cellular);
    noises[2].SetSeed(seed + 1500);
    noises[2].SetFrequency(1.0f);
    noises[2].SetFractalType(FastNoiseLite::FractalType_FBm);
    noises[2].SetFractalOctaves(10);

    static const auto saturate = [](auto value) {
        return std::clamp(value, 0.0f, 1.0f);
    };

    static const auto getVorleyNoise = [](float value) {
        return saturate((1.0f - value) * 0.5f - 0.25f);
        // return saturate(1.0f - value);
    };

    static const auto remap = [](float value, float minValue, float maxValue, float newMinValue, float newMaxValue) {
        return newMinValue + (value - minValue) / (maxValue - minValue) * (newMaxValue - newMinValue);
    };

    auto lowFreqNoiseData = std::vector<std::uint8_t>(lowFreqSize * lowFreqSize * lowFreqSize);
    auto highFreqNoiseData = std::vector<std::uint8_t>(highFreqSize * highFreqSize * highFreqSize);

    for(size_t z = 0; z < lowFreqSize; ++z)
    {
        for(size_t y = 0; y < lowFreqSize; ++y)
        {
            for(size_t x = 0; x < lowFreqSize; ++x)
            {
                size_t index = x + (y * lowFreqSize) + (z * lowFreqSize * lowFreqSize);

                const float invWidth = 1.0f / static_cast<float>(std::max(1, lowFreqSize - 1));
                const float invHeight = 1.0f / static_cast<float>(std::max(1, lowFreqSize - 1));
                const float invDepth = 1.0f / static_cast<float>(std::max(1, lowFreqSize - 1));

                const auto fx = static_cast<float>(x) * invWidth;
                const auto fy = static_cast<float>(y) * invHeight;
                const auto fz = static_cast<float>(z) * invDepth;

                float perlin = noises[0].GetNoise(fx, fy, fz);
                
                float voronoi0 = noises[2].GetNoise(fx * 2.0f, fy * 2.0f , fz * 2.0f);
                float worley0 = getVorleyNoise(voronoi0);
                // ворлея с меньшим масштабом
                float worley1 = getVorleyNoise(noises[2].GetNoise(fx * 4.0f, fy * 4.0f, fz * 4.0f));
                // ворлея с ещё меньшим масшатбом
                float worley2 = getVorleyNoise(noises[2].GetNoise(fx * 6.0f, fy * 6.0f, fz * 6.0f));

                // std::cout << "vorley: " << worley0 << ", voronoi: " << voronoi0 << std::endl;

                // объединяем 4 шума в одно значение
                const auto finalValue = remap(perlin, (worley0 * 0.625f + worley1 * 0.25f + worley2 * 0.125f) - 1.0f, 1.0f, 0.0f, 1.0f);

                // lowFreqNoiseData[index] = static_cast<std::uint8_t>(remap(worley0, -1.0f, 1.0f, 0.0f, 1.0f) * 255.0f);
                // lowFreqNoiseData[index] = static_cast<std::uint8_t>(worley0 * 255.0f);
                // lowFreqNoiseData[index] = static_cast<std::uint8_t>(voronoi0 * 255.0f);
                lowFreqNoiseData[index] = std::clamp<std::uint8_t>(static_cast<std::uint8_t>(saturate(finalValue) * 255.0f * 1.5f), 0, 255);
            }
        }
    }

    /*FastNoiseLite noises[4];

    noises[0].SetNoiseType(FastNoiseLite::NoiseType_OpenSimplex2S);
    noises[0].SetSeed(seed);
    noises[0].SetFrequency(0.6f);
    noises[0].SetFractalType(FastNoiseLite::FractalType_FBm);
    noises[0].SetFractalOctaves(4);
    noises[0].SetFractalLacunarity(2.0f);
    noises[0].SetFractalGain(0.5f);
    noises[0].SetFractalWeightedStrength(0.0f);
    noises[0].SetRotationType3D(FastNoiseLite::RotationType3D_ImproveXYPlanes);

    noises[1].SetNoiseType(FastNoiseLite::NoiseType_Cellular);
    noises[1].SetSeed(seed + 1000);
    noises[1].SetFrequency(1.0f);
    // noises[1].SetFractalType(FastNoiseLite::FractalType_None);
    noises[1].SetCellularDistanceFunction(FastNoiseLite::CellularDistanceFunction_Euclidean);
    noises[1].SetCellularReturnType(FastNoiseLite::CellularReturnType_Distance2Div);
    noises[1].SetCellularJitter(0.85f);

    noises[2].SetNoiseType(FastNoiseLite::NoiseType_OpenSimplex2);
    noises[2].SetSeed(seed + 2000);
    noises[2].SetFrequency(8.0f);
    noises[2].SetFractalType(FastNoiseLite::FractalType_FBm);
    noises[2].SetFractalOctaves(2);
    noises[2].SetFractalLacunarity(2.2f);
    noises[2].SetFractalGain(0.4f);

    noises[3].SetNoiseType(FastNoiseLite::NoiseType_Perlin);
    noises[3].SetSeed(seed + 300);
    noises[3].SetFrequency(6.0f);
    noises[3].SetFractalType(FastNoiseLite::FractalType_FBm);
    noises[3].SetFractalOctaves(4);
    noises[3].SetFractalLacunarity(2.8f);
    noises[3].SetFractalGain(0.5f);

    float* noiseData = new float[width * height * depth * 4];

    static const auto saturate = [](float f) {
        return std::clamp(f, 0.0f, 1.0f);
    };

    static const auto increaseContrast = [](float value, float contrast) {
        float midpoint = 0.5;
        return (value - midpoint) * contrast + midpoint;
    };

    const float invWidth = 1.0f / static_cast<float>(std::max(1, width - 1));
    const float invHeight = 1.0f / static_cast<float>(std::max(1, height - 1));
    const float invDepth = 1.0f / static_cast<float>(std::max(1, depth - 1));
    const float warpAmp = 0.15f;

#pragma omp parallel for collapse(3)
    for(size_t z = 0; z < depth; ++z)
    {
        for(size_t y = 0; y < height; ++y)
        {
            for(size_t x = 0; x < width; ++x)
            {
                size_t index = (x + (y * width) + (z * width * height)) * 4;

                const float fx = static_cast<float>(x) * invWidth;
                const float fy = static_cast<float>(y) * invHeight;
                const float fz = static_cast<float>(z) * invDepth;

                const float warpX = noises[0].GetNoise(fx, fy, fz);
                const float warpY = noises[0].GetNoise(fx + 31.4f, fy + 17.1f, fz + 5.3f);
                const float warpZ = noises[0].GetNoise(fx + 11.7f, fy + 73.2f, fz + 19.9f);

                const float wx = fx + warpX * warpAmp;
                const float wy = fy + warpY * warpAmp;
                const float wz = fz + warpZ * warpAmp;

                const float perlinValue = noises[3].GetNoise(wx, wy, wz) * 0.5f + 0.5f;
                const float worleyBase = noises[1].GetNoise(wx, wy, wz) * 0.5f + 0.5f;
                const float worleyDetail = noises[2].GetNoise(wx, wy, wz) * 0.5f + 0.5f;
                const float coverage = noises[0].GetNoise(fx, fy, fz) * 0.5f + 0.5f;

                const float baseClouds = saturate(increaseContrast(1.0f - worleyBase, 1.6f));
                const float detailClouds = saturate(increaseContrast(1.0f - worleyDetail, 1.3f));
                const float coverageClouds = saturate(increaseContrast(coverage, 1.2f));

                noiseData[index + 0] = baseClouds;
                noiseData[index + 1] = detailClouds;
                noiseData[index + 2] = perlinValue;
                noiseData[index + 3] = coverageClouds;
            }
        }
    }*/

    auto lowFreqTexture = SGCore::AssetManager::getInstance()->getOrAddAssetByAlias<SGCore::ITexture2D>("cloud_texture_3d_low_freq");
    lowFreqTexture->m_layersCount = lowFreqSize;
    lowFreqTexture->m_type = SGTextureType::SG_TEXTURE3D;
    lowFreqTexture->m_dataType = SGGDataType::SGG_UNSIGNED_BYTE;
    lowFreqTexture->create(lowFreqNoiseData.data(), lowFreqSize, lowFreqSize, 1,
                        SGGColorInternalFormat::SGG_R8,
                        SGGColorFormat::SGG_R);

    auto highFreqTexture = SGCore::AssetManager::getInstance()->getOrAddAssetByAlias<SGCore::ITexture2D>("cloud_texture_3d_high_freq");
    highFreqTexture->m_layersCount = highFreqSize;
    highFreqTexture->m_type = SGTextureType::SG_TEXTURE3D;
    highFreqTexture->m_dataType = SGGDataType::SGG_UNSIGNED_BYTE;
    highFreqTexture->create(highFreqNoiseData.data(), highFreqSize, highFreqSize, 1,
                        SGGColorInternalFormat::SGG_R8,
                        SGGColorFormat::SGG_R);

    return { lowFreqTexture, highFreqTexture };
}

void coreInit()
{
    auto mainAssetManager = SGCore::AssetManager::getInstance();

    auto& pipelinesManager = SGCore::RenderPipelinesManager::instance();

    // setting pipeline that will render our scene
    auto pbrrpPipeline = pipelinesManager.createRenderPipeline<SGCore::PBRRenderPipeline>();
    pipelinesManager.registerRenderPipeline(pbrrpPipeline);
    pipelinesManager.setCurrentRenderPipeline<SGCore::PBRRenderPipeline>();

    // creating scene
    scene = SGCore::MakeRef<SGCore::Scene>();
    scene->createDefaultSystems();

    // setting this scene as current
    SGCore::Scene::setCurrentScene(scene);

    scene->getSystem<SGCore::PhysicsWorld3D>()->getDebugDraw()->setDebugMode(btIDebugDraw::DBG_NoDebug);

    auto ecsRegistry = scene->getECSRegistry();

    mainAssetManager->loadAssetWithAlias<SGCore::ModelAsset>(
        "cube_model",
        "${enginePath}/Resources/models/standard/cube.obj"
    );

    mainAssetManager->loadAssetWithAlias<SGCore::ModelAsset>(
        "sphere_model",
        "${enginePath}/Resources/models/standard/sphere.obj"
    );

    screenShader = mainAssetManager->loadAsset<SGCore::IShader>("${enginePath}/Resources/sg_shaders/features/screen.sgshader");

    // terrainDecalDiffuseTex = mainAssetManager->loadAsset<SGCore::ITexture2D>(SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Resources/textures/spotted_rust/spotted-rust_albedo.png");
    terrainDecalDiffuseTex0 = mainAssetManager->loadAsset<SGCore::ITexture2D>(SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Resources/textures/terrain_editor/grow.png");
    terrainDecalDiffuseTex1 = mainAssetManager->loadAsset<SGCore::ITexture2D>(SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Resources/textures/terrain_editor/lower.png");
    terrainDecalAOTex = mainAssetManager->loadAsset<SGCore::ITexture2D>(SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Resources/textures/spotted_rust/spotted-rust_ao.png");
    terrainDecalHeightTex = mainAssetManager->loadAsset<SGCore::ITexture2D>(SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Resources/textures/spotted_rust/spotted-rust_height.png");
    terrainDecalMetallicTex = mainAssetManager->loadAsset<SGCore::ITexture2D>(SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Resources/textures/spotted_rust/spotted-rust_metallic.png");
    terrainDecalNormalTex = mainAssetManager->loadAsset<SGCore::ITexture2D>(SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Resources/textures/spotted_rust/spotted-rust_normal-ogl.png");
    terrainDecalRoughnessTex = mainAssetManager->loadAsset<SGCore::ITexture2D>(SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Resources/textures/spotted_rust/spotted-rust_roughness.png");

    terrainDiffuseTex = mainAssetManager->loadAsset<SGCore::ITexture2D>(SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Resources/textures/test_terrain/diffuse.png");
    terrainHeightmapTex = mainAssetManager->loadAsset<SGCore::ITexture2D>(SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Resources/textures/test_terrain/displacement.png");
    terrainNormalsTex = mainAssetManager->loadAsset<SGCore::ITexture2D>(SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Resources/textures/test_terrain/normals.png");
    terrainAORoughnessMetalTex = mainAssetManager->loadAsset<SGCore::ITexture2D>(SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Resources/textures/test_terrain/ao_roughness_metal.png");
    terrainTilingNoiseTex = mainAssetManager->loadAsset<SGCore::ITexture2D>(SGCore::AssetsLoadPolicy::PARALLEL_THEN_LAZYLOAD, "${enginePath}/Resources/textures/test_terrain/tiling_noise.png");
    // terrainDisplacementTex = mainAssetManager->loadAsset<SGCore::ITexture2D>("${enginePath}/Resources/textures/test_heightmap0.png");

    terrainDisplacementTex = mainAssetManager->getOrAddAssetByAlias<SGCore::ITexture2D>("test_heightmap");

    if(std::filesystem::exists("terrain_displacement.bin"))
    {
        size_t texByteSize = 0;
        const auto* texBinContent = SGCore::FileUtils::readBytes("terrain_displacement.bin", texByteSize);

        terrainDisplacementData.resize(1000 * 1000);

        memcpy(terrainDisplacementData.data(), texBinContent, texByteSize);

        delete texBinContent;

        /*terrainDisplacementTex = mainAssetManager->loadAsset<SGCore::ITexture2D>("terrain_displacement.hdr");

        terrainDisplacementData.resize(1000 * 1000);

        memcpy(terrainDisplacementData.data(), terrainDisplacementTex->getData(), terrainDisplacementTex->getWidth() * terrainDisplacementTex->getHeight() * sizeof(float));*/
        // terrainDisplacementTex->getData()
    }
    else
    {
        terrainDisplacementData.resize(1000 * 1000);
    }

    terrainDisplacementTex->m_dataType = SGGDataType::SGG_FLOAT;
    terrainDisplacementTex->create(terrainDisplacementData.data(), 1000, 1000, 1, SGGColorInternalFormat::SGG_R32_FLOAT, SGGColorFormat::SGG_R);

    // creating camera entity
    mainCamera = ecsRegistry->create();

    // creating components for entity
    auto cameraTransform = ecsRegistry->emplace<SGCore::Transform>(mainCamera, SGCore::MakeRef<SGCore::Transform>());
    ecsRegistry->emplace<SGCore::NonSavable>(mainCamera);
    ecsRegistry->emplace<SGCore::Camera3D>(mainCamera, SGCore::MakeRef<SGCore::Camera3D>());
    ecsRegistry->emplace<SGCore::RenderingBase>(mainCamera, SGCore::MakeRef<SGCore::RenderingBase>())->m_zFar = 70000.0;
    ecsRegistry->emplace<SGCore::Controllable3D>(mainCamera)/*.m_movementSpeed = 100.0f*/;
    auto& cameraReceiver = ecsRegistry->emplace<SGCore::LayeredFrameReceiver>(mainCamera);

    attachmentToDisplay = cameraReceiver.m_layersFXFrameBuffer->getAttachment(SGFrameBufferAttachmentType::SGG_COLOR_ATTACHMENT7);

    lightEntity = ecsRegistry->create();

    auto& lightTransform = ecsRegistry->emplace<SGCore::Transform>(lightEntity, SGCore::MakeRef<SGCore::Transform>());
    auto& spotLight = ecsRegistry->emplace<SGCore::SpotLight>(lightEntity);
    auto& lightRenderingBase = ecsRegistry->emplace<SGCore::RenderingBase>(lightEntity, SGCore::MakeRef<SGCore::RenderingBase>());

    spotLight.m_base.m_intensity = 10000.0f;

    // lightTransform->m_ownTransform.m_position = { 0.0, 3.0, 0.0 };

    ecsRegistry->get<SGCore::EntityBaseInfo>(mainCamera).addChild(lightEntity, *ecsRegistry);

    // creating skybox ==============================================================================================

    // creating cubemap and skybox material =====

    auto standardCubemap = mainAssetManager->getOrAddAssetByAlias<SGCore::ICubemapTexture>("standard_skybox0");

    standardCubemap->m_parts.push_back(mainAssetManager->loadAsset<SGCore::ITexture2D>(
            "${enginePath}/Resources/textures/skyboxes/skybox0/standard_skybox0_xleft.png"
    ));
    standardCubemap->m_parts.push_back(mainAssetManager->loadAsset<SGCore::ITexture2D>(
            "${enginePath}/Resources/textures/skyboxes/skybox0/standard_skybox0_xright.png"
    ));

    standardCubemap->m_parts.push_back(mainAssetManager->loadAsset<SGCore::ITexture2D>(
            "${enginePath}/Resources/textures/skyboxes/skybox0/standard_skybox0_ytop.png"
    ));
    standardCubemap->m_parts.push_back(mainAssetManager->loadAsset<SGCore::ITexture2D>(
            "${enginePath}/Resources/textures/skyboxes/skybox0/standard_skybox0_ybottom.png"
    ));

    standardCubemap->m_parts.push_back(mainAssetManager->loadAsset<SGCore::ITexture2D>(
            "${enginePath}/Resources/textures/skyboxes/skybox0/standard_skybox0_zfront.png"
    ));
    standardCubemap->m_parts.push_back(mainAssetManager->loadAsset<SGCore::ITexture2D>(
            "${enginePath}/Resources/textures/skyboxes/skybox0/standard_skybox0_zback.png"
    ));

    standardCubemap->create();

    auto standardCubemapMaterial = mainAssetManager->getOrAddAssetByAlias<SGCore::IMaterial>("standard_skybox_material0");
    standardCubemapMaterial->m_shaders["GeometryPass"] =
            mainAssetManager->loadAsset<SGCore::IShader>(
                    *pipelinesManager.getCurrentRenderPipeline()->m_shadersPaths["SkyboxShader"]);
    standardCubemapMaterial->m_meshRenderState.m_useFacesCulling = false;
    standardCubemapMaterial->addTexture2D(SGTextureSlot::SGTT_SKYBOX, standardCubemap);

    // ======

    std::vector<SGCore::ECS::entity_t> skyboxEntities;
    auto cubeModel =  SGCore::AssetManager::getInstance()->loadAsset<SGCore::ModelAsset>("sphere_model");
    cubeModel->m_rootNode->addOnScene(scene, SG_LAYER_OPAQUE_NAME, [&skyboxEntities](const auto& entity) {
        skyboxEntities.push_back(entity);
        scene->getECSRegistry()->emplace<SGCore::IgnoreOctrees>(entity);
        scene->getECSRegistry()->remove<SGCore::Pickable>(entity);
        scene->getECSRegistry()->remove<SGCore::TransparentEntityTag>(entity);
    });

    atmosphereEntity = skyboxEntities[2];

    auto& skyboxMesh = scene->getECSRegistry()->get<SGCore::Mesh>(atmosphereEntity);
    auto& atmosphereScattering = scene->getECSRegistry()->emplace<SGCore::Atmosphere>(atmosphereEntity);
    atmosphereScattering.m_sunRotation.x = 0.0;
    skyboxMesh.m_base.setMaterial(standardCubemapMaterial);

    auto& skyboxTransform = scene->getECSRegistry()->get<SGCore::Transform>(atmosphereEntity);

    skyboxTransform->m_ownTransform.m_scale = { 65000, 65000, 65000 };

    // =================================================================

    // creating terrain material ==============================

    auto standardTerrainMaterial = mainAssetManager->getOrAddAssetByAlias<SGCore::IMaterial>("standard_terrain_material");
    standardTerrainMaterial->m_shaders["GeometryPass"] =
            mainAssetManager->loadAsset<SGCore::IShader>(
                    *pipelinesManager.getCurrentRenderPipeline()->m_shadersPaths["StandardTerrainShader"]);
    standardTerrainMaterial->m_meshRenderState.m_useFacesCulling = false;
    standardTerrainMaterial->m_meshRenderState.m_facesCullingPolygonsOrder = SGPolygonsOrder::SGG_CCW;
    //standardTerrainMaterial->m_meshRenderState.m_useIndices = false;
    standardTerrainMaterial->m_meshRenderState.m_drawMode = SGDrawMode::SGG_PATCHES;
    standardTerrainMaterial->addTexture2D(SGTextureSlot::SGTT_DIFFUSE, terrainDiffuseTex);
    standardTerrainMaterial->addTexture2D(SGTextureSlot::SGTT_DISPLACEMENT, terrainDisplacementTex);
    standardTerrainMaterial->addTexture2D(SGTextureSlot::SGTT_NORMALS, terrainNormalsTex);
    standardTerrainMaterial->addTexture2D(SGTextureSlot::SGTT_LIGHTMAP, terrainAORoughnessMetalTex);
    standardTerrainMaterial->addTexture2D(SGTextureSlot::SGTT_DIFFUSE_ROUGHNESS, terrainAORoughnessMetalTex);
    standardTerrainMaterial->addTexture2D(SGTextureSlot::SGTT_METALNESS, terrainAORoughnessMetalTex);
    standardTerrainMaterial->addTexture2D(SGTextureSlot::SGTT_HEIGHT, terrainHeightmapTex);
    standardTerrainMaterial->addTexture2D(SGTextureSlot::SGTT_NOISE, terrainTilingNoiseTex);

    // creating terrain material ==============================

    auto testMaterial = mainAssetManager->getOrAddAssetByAlias<SGCore::IMaterial>("test_material_0");
    testMaterial->m_shaders["GeometryPass"] =
            mainAssetManager->loadAsset<SGCore::IShader>(
                    *pipelinesManager.getCurrentRenderPipeline()->m_shadersPaths["StandardMeshShader"]);
    testMaterial->m_meshRenderState.m_useFacesCulling = false;
    testMaterial->m_meshRenderState.m_facesCullingPolygonsOrder = SGPolygonsOrder::SGG_CCW;
    //standardTerrainMaterial->m_meshRenderState.m_useIndices = false;
    testMaterial->m_meshRenderState.m_drawMode = SGDrawMode::SGG_TRIANGLES;
    /*testMaterial->addTexture2D(SGTextureSlot::SGTT_DIFFUSE, terrainDiffuseTex);
    testMaterial->addTexture2D(SGTextureSlot::SGTT_DISPLACEMENT, terrainDisplacementTex);
    testMaterial->addTexture2D(SGTextureSlot::SGTT_NORMALS, terrainNormalsTex);
    testMaterial->addTexture2D(SGTextureSlot::SGTT_LIGHTMAP, terrainAORoughnessMetalTex);
    testMaterial->addTexture2D(SGTextureSlot::SGTT_DIFFUSE_ROUGHNESS, terrainAORoughnessMetalTex);
    testMaterial->addTexture2D(SGTextureSlot::SGTT_METALNESS, terrainAORoughnessMetalTex);
    testMaterial->addTexture2D(SGTextureSlot::SGTT_HEIGHT, terrainHeightmapTex);*/

    // creating terrain entity ================================

    terrainEntity = ecsRegistry->create();

    auto& terrainTransform = ecsRegistry->emplace<SGCore::Transform>(terrainEntity, SGCore::MakeRef<SGCore::Transform>());
    ecsRegistry->emplace<SGCore::EnableTerrainPass>(terrainEntity);
    // terrainTransform->m_ownTransform.m_scale = { 0.1, 0.1, 0.1 };
    ecsRegistry->emplace<SGCore::NonSavable>(terrainEntity);
    auto& terrainMesh = ecsRegistry->emplace<SGCore::Mesh>(terrainEntity);
    auto& terrainComponent = ecsRegistry->emplace<SGCore::Terrain>(terrainEntity);
    // auto& terrainNavGrid = ecsRegistry->emplace<SGCore::Navigation::NavGrid3D>(terrainEntity);
    auto& terrainNavMesh = ecsRegistry->emplace<SGCore::Navigation::NavMesh>(terrainEntity);
    terrainNavMesh.m_config.m_cellHeight = 5.0f;
    terrainNavMesh.m_config.m_cellSize = 5.0f;

    // creating terrain mesh ====
    terrainMeshData = mainAssetManager->createAndAddAsset<SGCore::IMeshData>();

    terrainComponent.generate(terrainMeshData, 40, 40, 100);

    terrainMesh.m_base.setMeshData(terrainMeshData);
    terrainMesh.m_base.setMaterial(standardTerrainMaterial);

    std::cout << "terrain aabb min: " << glm::to_string(terrainMeshData->m_aabb.m_min) << ", terrain aabb max: " << glm::to_string(terrainMeshData->m_aabb.m_max) << std::endl;

    // terrainNavGrid.m_cellSize = 10.0f;

    regenerateTerrainNavGrid(terrainEntity);

    // creating rigidbody for terrain
    auto terrainRigidbody = scene->getECSRegistry()->emplace<SGCore::Rigidbody3D>(terrainEntity,
        SGCore::MakeRef<SGCore::Rigidbody3D>(scene->getSystem<SGCore::PhysicsWorld3D>()));

    // generating terrain physical mesh
    terrainComponent.generatePhysicalMesh(terrainMesh, 10);

    SGCore::Ref<btBvhTriangleMeshShape> terrainRigidbodyShape = SGCore::MakeRef<btBvhTriangleMeshShape>(terrainMeshData->m_physicalMesh.get(), true);
    btTransform terrainShapeTransform;
    terrainShapeTransform.setIdentity();
    terrainRigidbody->addShape(terrainShapeTransform, terrainRigidbodyShape);
    btScalar mass = 0.0f;
    btVector3 inertia(0, 0, 0);
    terrainRigidbody->m_body->setMassProps(mass, inertia);
    terrainRigidbody->reAddToWorld();

    // ==========================

    // terrainTransform->m_ownTransform.m_scale = { 1.0f, 1.0f, 1.0f };

    // =================================================================
    // ================================================================= test
    // =================================================================

    testModelAsset = mainAssetManager->loadAssetWithAlias<SGCore::ModelAsset>(
        "quad_test",
        "${enginePath}/Resources/models/standard/cube.obj"
    );

    std::vector<SGCore::ECS::entity_t> testQuadEntities;
    testModelAsset->m_rootNode->addOnScene(scene, SG_LAYER_OPAQUE_NAME, [&testQuadEntities, &testMaterial](const auto& entity) {
        testQuadEntities.push_back(entity);
        scene->getECSRegistry()->emplace<SGCore::IgnoreOctrees>(entity);
        scene->getECSRegistry()->remove<SGCore::Pickable>(entity);
        scene->getECSRegistry()->remove<SGCore::TransparentEntityTag>(entity);

        auto* meshComponent = scene->getECSRegistry()->tryGet<SGCore::Mesh>(entity);
        if(meshComponent)
        {
            meshComponent->m_base.setMaterial(testMaterial);
        }
    });

    scene->getECSRegistry()->get<SGCore::Transform>(testQuadEntities[0])->m_ownTransform.m_position.y += 10.0f;

    // creating decal !!! ==============================================

    terrainDecalEntity = SGCore::ECS::Utils::createDecal(*ecsRegistry.get());

    terrainDecalMaterial = mainAssetManager->getOrAddAssetByAlias<SGCore::IMaterial>("terrain_decal_material_0");
    terrainDecalMaterial->addTexture2D(SGTextureSlot::SGTT_DIFFUSE, terrainDecalDiffuseTex0);
    /*terrainDecalMaterial->addTexture2D(SGTextureSlot::SGTT_LIGHTMAP, terrainDecalAOTex);
    terrainDecalMaterial->addTexture2D(SGTextureSlot::SGTT_HEIGHT, terrainDecalHeightTex);
    terrainDecalMaterial->addTexture2D(SGTextureSlot::SGTT_METALNESS, terrainDecalMetallicTex);
    terrainDecalMaterial->addTexture2D(SGTextureSlot::SGTT_NORMALS, terrainDecalNormalTex);
    terrainDecalMaterial->addTexture2D(SGTextureSlot::SGTT_DIFFUSE_ROUGHNESS, terrainDecalRoughnessTex);*/
    terrainDecalMaterial->setMetallicFactor(0.0f);
    terrainDecalMaterial->setShininess(0.0f);
    terrainDecalMaterial->setRoughnessFactor(1.0f);

    auto& decalMesh = ecsRegistry->get<SGCore::Mesh>(terrainDecalEntity);
    decalMesh.m_base.setMaterial(terrainDecalMaterial);

    auto& decalTransform = ecsRegistry->get<SGCore::Transform>(terrainDecalEntity);
    ecsRegistry->emplace<SGCore::EnableDecalPass>(terrainDecalEntity);
    // decalTransform->m_ownTransform.m_scale.x *= 10.0;
    // decalTransform->m_ownTransform.m_scale.z *= 10.0;
    // decalTransform->m_ownTransform.m_scale.y = 5.0;

    // =================================================================
    // =================================================================
    // =================================================================

    // creating volumetric clouds

    cloudsEntity = ecsRegistry->create();

    auto& volumetricFog = ecsRegistry->emplace<SGCore::VolumetricFog>(cloudsEntity);
    auto& volumetricMesh = ecsRegistry->emplace<SGCore::Mesh>(cloudsEntity);
    auto& cloudsTransform = ecsRegistry->emplace<SGCore::Transform>(cloudsEntity, SGCore::MakeRef<SGCore::Transform>());
    ecsRegistry->emplace<SGCore::EnableVolumetricPass>(cloudsEntity);
    cloudsTransform->m_ownTransform.m_position.y += 15000.0f;
    // cloudsTransform->m_ownTransform.m_position.x += 1000.0f;
    cloudsTransform->m_ownTransform.m_scale.x = 60000.0f;
    cloudsTransform->m_ownTransform.m_scale.z = 60000.0f;
    cloudsTransform->m_ownTransform.m_scale.y = 2000.0f;

    const auto standardCloudsMaterial = mainAssetManager->getOrAddAssetByAlias<SGCore::IMaterial>("standard_volumetric_clouds_material");
    const auto standardCubeModel = mainAssetManager->loadAsset<SGCore::ModelAsset>("${enginePath}/Resources/models/standard/cube.obj");

    standardCloudsMaterial->m_meshRenderState.m_useFacesCulling = false;
    standardCloudsMaterial->addTexture2D(SGTextureSlot::SGTT_NOISE, generateMultiOctaveCloudTexture3D(128, 32, 42).first);

    volumetricMesh.m_base.setMeshData(standardCubeModel->m_rootNode->findMesh("cube"));
    volumetricMesh.m_base.setMaterial(standardCloudsMaterial);

    // =================================================================
    // =================================================================
    // =================================================================

    quadMeshData = SGCore::Ref<SGCore::IMeshData>(SGCore::CoreMain::getRenderer()->createMeshData());

    quadMeshData->m_vertices.resize(4);

    quadMeshData->m_vertices[0] = {
        .m_position = glm::vec3 { -1, -1, 0.0f }
    };

    quadMeshData->m_vertices[1] = {
        .m_position = glm::vec3 { -1, 1, 0.0f }
    };

    quadMeshData->m_vertices[2] = {
        .m_position = glm::vec3 { 1, 1, 0.0f }
    };

    quadMeshData->m_vertices[3] = {
        .m_position = glm::vec3 { 1, -1, 0.0f }
    };

    quadMeshData->m_indices.resize(6);

    quadMeshData->m_indices[0] = 0;
    quadMeshData->m_indices[1] = 2;
    quadMeshData->m_indices[2] = 1;
    quadMeshData->m_indices[3] = 0;
    quadMeshData->m_indices[4] = 3;
    quadMeshData->m_indices[5] = 2;

    quadMeshData->prepare();
}

void saveTerrainDisplacementMap() noexcept
{
    const int displacementTexWidth = terrainDisplacementTex->getWidth();
    const int displacementTexHeight = terrainDisplacementTex->getHeight();

    SGCore::FileUtils::writeBytes("terrain_displacement.bin", 0, reinterpret_cast<char*>(terrainDisplacementTex->getData()), displacementTexWidth * displacementTexHeight * sizeof(float) ,false);

    /*std::vector<float> rgbData;
    rgbData.resize(displacementTexWidth * displacementTexHeight * 1);

    for(int i = 0; i < displacementTexWidth * displacementTexHeight; ++i)
    {
        rgbData[i] = terrainDisplacementData[i];
    }

    int result = stbi_write_hdr("terrain_displacement.hdr", displacementTexWidth, displacementTexHeight, 1, rgbData.data());
    std::cout << "result of saving displacement: " << result << std::endl;*/
}

void onUpdate(const double& dt, const double& fixedDt)
{
    SGCore::CoreMain::getWindow().setTitle("Tesselation Test. FPS: " + std::to_string(SGCore::CoreMain::getFPS()));

    if(SGCore::Scene::getCurrentScene())
    {
        SGCore::Scene::getCurrentScene()->update(dt, fixedDt);

        SGCore::CoreMain::getRenderer()->renderTextureOnScreen(attachmentToDisplay.get());
    }

    /*if(mainInputListener->keyboardKeyDown(SGCore::KeyboardKey::KEY_LEFT))
    {
        scene->getECSRegistry()->get<SGCore::Transform>(mainCamera)->m_ownTransform.m_yawPitchRoll.y -= 0.5f;
    }

    if(mainInputListener->keyboardKeyDown(SGCore::KeyboardKey::KEY_RIGHT))
    {
        scene->getECSRegistry()->get<SGCore::Transform>(mainCamera)->m_ownTransform.m_yawPitchRoll.y += 0.5f;
    }*/

    auto& decalTransform = scene->getECSRegistry()->get<SGCore::Transform>(terrainDecalEntity);
    auto& terrainTransform = scene->getECSRegistry()->get<SGCore::Transform>(terrainEntity);
    auto& terrain = scene->getECSRegistry()->get<SGCore::Terrain>(terrainEntity);
    // auto& terrainNavGrid = scene->getECSRegistry()->get<SGCore::Navigation::NavGrid3D>(terrainEntity);
    auto& terrainNavMesh = scene->getECSRegistry()->get<SGCore::Navigation::NavMesh>(terrainEntity);

    const auto debugDraw = SGCore::RenderPipelinesManager::instance().getCurrentRenderPipeline()->getRenderPass<SGCore::DebugDraw>();
    const auto& physicsDebugDraw = scene->getSystem<SGCore::PhysicsWorld3D>()->getDebugDraw();

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_3))
    {
        // terrainTransform->m_ownTransform.m_rotation = glm::identity<glm::quat>();
        terrainTransform->m_ownTransform.m_rotation = glm::angleAxis(glm::radians(90.0f), glm::vec3(0, 0, 1)) * terrainTransform->m_ownTransform.m_rotation;

    }

    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_9))
    {
        auto& atmosphereScattering = scene->getECSRegistry()->get<SGCore::Atmosphere>(atmosphereEntity);
        atmosphereScattering.m_sunRotation.x += 10.0f * dt;
    }

    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_0))
    {
        auto& atmosphereScattering = scene->getECSRegistry()->get<SGCore::Atmosphere>(atmosphereEntity);
        atmosphereScattering.m_sunRotation.x -= 10.0f * dt;
    }

    {
        auto& layeredFrameReceiver = scene->getECSRegistry()->get<SGCore::LayeredFrameReceiver>(mainCamera);

        int windowSizeX;
        int windowSizeY;
        SGCore::CoreMain::getWindow().getSize(windowSizeX, windowSizeY);

        const glm::vec2 cursorPos {
            SGCore::Input::PC::getCursorPositionX(),
            windowSizeY - SGCore::Input::PC::getCursorPositionY()
        };

        const auto& attachment4 = layeredFrameReceiver.m_layersFrameBuffer->getAttachment(SGFrameBufferAttachmentType::SGG_COLOR_ATTACHMENT4);

        const glm::vec2 cursorRelativePos = {
            cursorPos.x / windowSizeX * (attachment4->getWidth()),
            cursorPos.y / windowSizeY * (attachment4->getHeight())
        };

        const auto worldPos = layeredFrameReceiver.m_layersFrameBuffer->readPixelsFromAttachment(cursorRelativePos, SGFrameBufferAttachmentType::SGG_COLOR_ATTACHMENT4);
        const auto surfaceNormal = layeredFrameReceiver.m_layersFrameBuffer->readPixelsFromAttachment(cursorRelativePos, SGFrameBufferAttachmentType::SGG_COLOR_ATTACHMENT6);

        /*std::cout << "pos: " << worldPos << ", cursor pos: " << cursorRelativePos <<
                ", attachment4->getWidth(): " << attachment4->getWidth() <<
                ", attachment4->getHeight(): " << attachment4->getHeight() <<
                ", surface normal: " << surfaceNormal <<
                std::endl;*/

        glm::vec3 zDir = glm::normalize(surfaceNormal);
        glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);

        // fixing normal
        if (glm::abs(glm::dot(zDir, up)) > 0.999f)
            up = glm::vec3(1.0f, 0.0f, 0.0f);

        glm::vec3 xDir = glm::normalize(glm::cross(up, zDir));
        glm::vec3 yDir = glm::cross(zDir, xDir);

        glm::mat3 rotationMatrix(xDir, yDir, zDir);
        glm::quat rotation = glm::quat_cast(rotationMatrix);

        decalTransform->m_ownTransform.m_position = worldPos;
        decalTransform->m_ownTransform.m_rotation = rotation * glm::angleAxis(glm::radians(-90.0f), glm::vec3 { 1.0f, 0.0f, 0.0f });

        if(SGCore::Input::PC::mouseButtonDown(SGCore::Input::MouseButton::MOUSE_BUTTON_LEFT))
        {
            const glm::vec3 decalRelativePos = worldPos - terrainTransform->m_finalTransform.m_position + glm::vec3 { terrain.getPatchSize() / 2.0f, 0.0f, terrain.getPatchSize() / 2.0f };

            const glm::vec2 pixelSize {
                float(terrain.getPatchSize() * terrain.getSize().x) / (float) terrainDisplacementTex->getWidth(),
                float(terrain.getPatchSize() * terrain.getSize().y) / (float) terrainDisplacementTex->getHeight()
            };

            const glm::ivec2 indices { std::floor(decalRelativePos.x / pixelSize.x), std::floor(decalRelativePos.z / pixelSize.y) };

            std::cout << "indices: " << indices.x << ", " << indices.y << std::endl;

            const float paintRadius = (decalTransform->m_ownTransform.m_scale.x / pixelSize.x);
            const int growRegionHalfSizeX = (int) paintRadius;
            const int growRegionHalfSizeY = (int) paintRadius;

            switch(currentTerrainOp)
            {
                case TerrainOp::TERRAIN_GROW:
                {
                    for(int y = -growRegionHalfSizeY; y < growRegionHalfSizeY + 1; ++y)
                    {
                        for(int x = -growRegionHalfSizeX; x < growRegionHalfSizeX + 1; ++x)
                        {
                            if(glm::length(glm::vec2 { x, y }) < paintRadius)
                            {
                                const size_t finalIndex = indices.x + x + (indices.y + y) * terrainDisplacementTex->getWidth();
                                // const size_t finalIndex = indices.x + (indices.y) * terrainDisplacementTex->getWidth();
                                if(finalIndex > 0 && finalIndex < terrainDisplacementData.size())
                                {
                                    terrainDisplacementData[finalIndex] += terrainGrowSpeed;
                                }
                            }
                        }
                    }

                    break;
                }
                case TerrainOp::TERRAIN_LOWER:
                {
                    for(int y = -growRegionHalfSizeY; y < growRegionHalfSizeY + 1; ++y)
                    {
                        for(int x = -growRegionHalfSizeX; x < growRegionHalfSizeX + 1; ++x)
                        {
                            if(glm::length(glm::vec2 { x, y }) < paintRadius)
                            {
                                const size_t finalIndex = indices.x + x + (indices.y + y) * terrainDisplacementTex->getWidth();
                                if(finalIndex > 0 && finalIndex < terrainDisplacementData.size())
                                {
                                    terrainDisplacementData[finalIndex] -= terrainLowerSpeed;
                                }
                            }
                        }
                    }

                    break;
                }
            }

            terrainDisplacementTex->bind(0);
            terrainDisplacementTex->subTextureData(reinterpret_cast<const std::uint8_t*>(terrainDisplacementData.data()), terrainDisplacementTex->getWidth(), terrainDisplacementTex->getHeight(), 0, 0);
        }

        if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_F1))
        {
            currentTerrainOp = TerrainOp::TERRAIN_GROW;
            terrainDecalMaterial->replaceTexture(SGTextureSlot::SGTT_DIFFUSE, 0, terrainDecalDiffuseTex0);
        }
        else if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_F2))
        {
            currentTerrainOp = TerrainOp::TERRAIN_LOWER;
            terrainDecalMaterial->replaceTexture(SGTextureSlot::SGTT_DIFFUSE, 0, terrainDecalDiffuseTex1);
        }

        if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_PAGE_UP))
        {
            switch(currentTerrainOp)
            {
                case TerrainOp::TERRAIN_GROW:
                {
                    terrainGrowSpeed += 0.001f;
                    std::cout << "current terrain grow speed: " << terrainGrowSpeed << std::endl;
                    break;
                }
                case TerrainOp::TERRAIN_LOWER:
                {
                    terrainLowerSpeed += 0.001f;
                    std::cout << "current terrain lower speed: " << terrainLowerSpeed << std::endl;
                    break;
                }
            }
        }

        if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_PAGE_DOWN))
        {
            switch(currentTerrainOp)
            {
                case TerrainOp::TERRAIN_GROW:
                {
                    terrainGrowSpeed -= 0.001f;
                    std::cout << "current terrain grow speed: " << terrainGrowSpeed << std::endl;
                    break;
                }
                case TerrainOp::TERRAIN_LOWER:
                {
                    terrainLowerSpeed -= 0.001f;
                    std::cout << "current terrain lower speed: " << terrainLowerSpeed << std::endl;
                    break;
                }
            }
        }

        if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_EQUAL))
        {
            decalTransform->m_ownTransform.m_scale += 0.1f;
        }
        else if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_MINUS))
        {
            decalTransform->m_ownTransform.m_scale -= 0.1f;
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

    if(SGCore::Input::PC::keyboardKeyDown(SGCore::Input::KeyboardKey::KEY_4))
    {
        auto& cameraTransform = scene->getECSRegistry()->get<SGCore::Transform>(mainCamera);
        createBallAndApplyImpulse(cameraTransform->m_ownTransform.m_position, cameraTransform->m_ownTransform.m_forward * 200000.0f / 10.0f);
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_5))
    {
        regenerateTerrainPhysicalMesh(terrainEntity);
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_6))
    {
        if(physicsDebugDraw->getDebugMode() != btIDebugDraw::DBG_NoDebug)
        {
            physicsDebugDraw->setDebugMode(btIDebugDraw::DBG_NoDebug);
        }
        else
        {
            physicsDebugDraw->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
        }
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_7))
    {
        saveTerrainDisplacementMap();
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_F3))
    {
        // terrainNavGrid.applyModelMatrix(terrainTransform->m_finalTransform.m_animatedModelMatrix);
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_F4))
    {
        if(debugDraw->m_mode != SGCore::DebugDrawMode::NO_DEBUG)
        {
            debugDraw->m_mode = SGCore::DebugDrawMode::NO_DEBUG;
        }
        else
        {
            debugDraw->m_mode = SGCore::DebugDrawMode::WIREFRAME;
        }
    }

    if(SGCore::Input::PC::keyboardKeyReleased(SGCore::Input::KeyboardKey::KEY_F5))
    {
        regenerateTerrainNavGrid(terrainEntity);
    }

    /*if(debugDraw->m_mode != SGCore::DebugDrawMode::NO_DEBUG)
    {
        const auto inputFilteringStep = terrainNavMesh.getStep<SGCore::Navigation::InputFilteringStep>();
        const auto voxelizationStep = terrainNavMesh.getStep<SGCore::Navigation::VoxelizationStep>();

        const auto& navMeshConfig = terrainNavMesh.m_config;

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


            // debugDraw->drawAABB(min, max, { 0.47, 0.87, 0.78, 1.0 });
            if(voxel.m_isWalkable)
            {
                debugDraw->drawAABB(min, max, { 0.47, 0.87, 0.78, 1.0 });
            }
            else
            {
                debugDraw->drawAABB(min, max, { 1.0, 0.0, 0.0, 1.0 });
            }
        }
    }*/

    // debugDraw->drawLine({ 0, 0, 0 }, { 0, 10, 0 },  { 0.91, 0.40, 0.42, 1.0 });
}

void onFixedUpdate(const double& dt, const double& fixedDt)
{
    if(SGCore::Scene::getCurrentScene())
    {
        SGCore::Scene::getCurrentScene()->fixedUpdate(dt, fixedDt);
    }
}

int main()
{
    SGCore::CoreMain::onInit.connect<&coreInit>();
    SGCore::CoreMain::getRenderTimer().onUpdate.connect<&onUpdate>();
    SGCore::CoreMain::getFixedTimer().onUpdate.connect<&onFixedUpdate>();

    SGCore::CoreMain::init();
    SGCore::CoreMain::startCycle();

    return 0;
}
