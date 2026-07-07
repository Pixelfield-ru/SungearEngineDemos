//
// Created by stuka on 08.07.2026.
//

#include "App.h"

int main(int argc, char* argv[])
{
    StartupType startupType = StartupType::CLIENT;
    if(argc == 1)
    {
        if(!std::strcmp(argv[0], "--server"))
        {
            startupType = StartupType::SERVER;
        }
    }

    App app;
    app.start(true);

    return 0;
}