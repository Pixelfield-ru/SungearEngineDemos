//
// Created by stuka on 06.02.2025.
//

#include "Main.h"
#include "App.h"

#if SG_PLATFORM_OS_WINDOWS
#ifdef __cplusplus
extern "C" {
#endif
#include <windows.h>
    __declspec(dllexport) DWORD NvOptimusEnablement = 1;
    __declspec(dllexport) int AmdPowerXpressRequestHighPerformance = 1;
#ifdef __cplusplus
}
#endif
#endif

int main()
{
    App app;
    app.start(true);

    return 0;
}
