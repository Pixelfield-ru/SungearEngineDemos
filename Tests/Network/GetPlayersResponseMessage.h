//
// Created by stuka on 16.07.2026.
//

#pragma once

#include <array>
#include <SGCore/Utils/StaticTypeID.h>

#pragma push(pack, 1)
struct GetPlayersResponseMessage
{
    sg_implement_nonvirtual_type_id(GetPlayersResponseMessage);

    static constexpr bool use_rudp = true;

    std::array<std::int64_t, 40> m_players;
    std::int32_t m_playersCount {};
};
#pragma pop(pack)