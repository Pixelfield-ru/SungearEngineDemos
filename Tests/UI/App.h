//
// Created by stuka on 23.03.2026.
//

#pragma once

#include <SGCore/Main/BasicApp.h>
#include <SGCore/Memory/AssetRef.h>
#include <SGCore/UI/UIDocument.h>
#include <SGCore/UI/CSS/CSSFile.h>

struct App final : SGCore::BasicApp
{
    void onInit() noexcept override;
    void onUpdate(double dt, double fixedDt) noexcept override;
    void onFixedUpdate(double dt, double fixedDt) noexcept override;

private:
    SGCore::AssetRef<SGCore::UI::CSSFile> m_cssFile;
    SGCore::AssetRef<SGCore::UI::UIDocument> m_uiDocument;
    SGCore::AssetRef<SGCore::ITexture2D> m_testTexture;
    static std::string m_myText;
};
