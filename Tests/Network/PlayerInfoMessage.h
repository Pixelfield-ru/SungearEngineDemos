//
// Created by stuka on 13.07.2026.
//

#pragma once

#include <cstdint>

#include <SGCore/Utils/StaticTypeID.h>

#pragma push(pack, 1)
struct PlayerInfoMessage
{
    sg_implement_nonvirtual_type_id(PlayerInfoMessage);

    std::int32_t m_playerID {};
};
#pragma pop(pack)