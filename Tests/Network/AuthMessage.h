//
// Created by stuka on 15.07.2026.
//

#pragma once

#include <SGCore/Utils/StaticTypeID.h>

#pragma push(pack, 1)
struct AuthMessage
{
    sg_implement_nonvirtual_type_id(AuthMessage);

    static constexpr bool use_rudp = true;
    static constexpr bool use_for_auth = true;
};
#pragma pop(pack)