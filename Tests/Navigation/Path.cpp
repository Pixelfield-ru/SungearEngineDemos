//
// Created by stuka on 06.07.2026.
//

#include "Path.h"

#include <SGCore/ECS/Registry.h>
#include <SGCore/Transformations/Transform.h>

SGCore::Coro::Task<bool> Path::npcGoto(SGCore::Ref<SGCore::ECS::registry_t> registry, SGCore::ECS::entity_t npcEntity,
                                       glm::vec3 position, float speed)
{
    auto& npcTransform = registry->get<SGCore::Transform>(npcEntity);
    auto currentThread = SGCore::Threading::ThreadsManager::currentThread();

    while(glm::distance(npcTransform.m_worldTransform.m_position, position) > 0.5f)
    {
        co_await SGCore::Coro::returnToCaller();

        const auto dif = glm::normalize(position - npcTransform.m_worldTransform.m_position);
        npcTransform.m_localTransform.m_position += dif * speed * float(currentThread->getDeltaTime());
        // npcTransform.m_localTransform.m_position += ;

        if(m_stop) break;
    }

    co_return true;
}

SGCore::Coro::Task<> Path::npcMoveByPath(SGCore::Ref<SGCore::ECS::registry_t> registry, SGCore::ECS::entity_t npcEntity, float speed)
{
    for(const auto& pos : m_points)
    {
        co_await npcGoto(registry, npcEntity, pos, speed);

        if(m_stop) break;
    }
}
