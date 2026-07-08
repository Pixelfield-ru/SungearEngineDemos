//
// Created by stuka on 08.07.2026.
//

#include "App.h"

int main(int argc, char* argv[])
{
    StartupType startupType = StartupType::CLIENT;
    if(argc == 2)
    {
        if(!std::strcmp(argv[1], "--server"))
        {
            startupType = StartupType::SERVER;
            std::cout << "got --server arg" << std::endl;
        }
    }

    /*for(int i = 0; i < argc; i++)
    {
        std::cout << "arg: " << argv[i] << std::endl;
    }*/

    App app;
    app.m_startupType = startupType;
    // app.start(startupType == StartupType::CLIENT);
    app.start(false);

    return 0;
}