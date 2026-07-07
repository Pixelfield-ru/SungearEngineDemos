//
// Created by stuka on 08.07.2026.
//

#pragma once

#include <string>

#include <glm/vec3.hpp>
#include <glm/gtx/quaternion.hpp>

struct TransformPacket
{
    static inline std::string type_name = "TransformPacket";
    glm::vec3 m_position { };
    glm::quat m_rotation { };
};