//
// Created by stuka on 06.07.2026.
//

#pragma once

#include <vector>
#include <glm/vec3.hpp>
#include <SGCore/Coro/Task.h>

struct Path
{
    std::vector<glm::vec3> m_points;
    bool m_stop {};

    SGCore::Coro::Task<bool> npcGoto(SGCore::Ref<SGCore::ECS::registry_t> registry, SGCore::ECS::entity_t npcEntity, glm::vec3 position, float speed);
    SGCore::Coro::Task<> npcMoveByPath(SGCore::Ref<SGCore::ECS::registry_t> registry, SGCore::ECS::entity_t npcEntity, float speed);
};
