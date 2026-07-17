//
// Created by stuka on 16.07.2026.
//

#pragma once

#include <SGCore/Utils/StaticTypeID.h>

#pragma push(pack, 1)
struct GetPlayersMessage
{
    sg_implement_nonvirtual_type_id(GetPlayersMessage);

    static constexpr bool use_rudp = true;
};
#pragma pop(pack)