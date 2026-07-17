//
// Created by stuka on 15.07.2026.
//

#pragma once

#include <SGCore/Utils/StaticTypeID.h>

#pragma push(pack, 1)
struct AuthResponseMessage
{
    sg_implement_nonvirtual_type_id(AuthResponseMessage);

    static constexpr bool use_rudp = true;

    std::int64_t m_sessionID = -1;
};
#pragma pop(pack)