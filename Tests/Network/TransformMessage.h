//
// Created by stuka on 08.07.2026.
//

#pragma once

#include <string>

#include <glm/vec3.hpp>
#include <glm/gtx/quaternion.hpp>
#include <SGCore/Utils/StaticTypeID.h>

#pragma push(pack, 1)
struct TransformMessage
{
    sg_implement_nonvirtual_type_id(TransformMessage);

    glm::vec3 m_position { };
    glm::quat m_rotation { };
};
#pragma pop(pack)