//
// Created by stuka on 11.07.2026.
//

#pragma once

#include <array>

#include <SGCore/Utils/StaticTypeID.h>

#pragma push(pack, 1)
struct AnyMessage
{
    sg_implement_nonvirtual_type_id(AnyMessage);

    std::array<std::uint8_t, 512> m_message {};
};
#pragma pop(pack)
