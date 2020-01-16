// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// The MIT License (MIT)
//
// Copyright (c) 2019 www.open3d.org
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.
// ----------------------------------------------------------------------------

#pragma once

#include <filament/utils/Entity.h>

#include <functional>
#include <memory>
#include <unordered_map>

#include "Open3D/Geometry/Image.h"
#include "Open3D/Visualization/Rendering/Renderer.h"

namespace filament {
namespace backend {
enum class PixelDataFormat : uint8_t;
enum class PixelDataType : uint8_t;
}

class Engine;
class Renderer;
class Scene;
class SwapChain;
class VertexBuffer;
}  // namespace filament

namespace open3d {
namespace visualization {

class FilamentMaterialModifier;
class FilamentResourceManager;
class FilamentScene;
class FilamentView;

class FilamentRenderer : public Renderer {
public:
    struct HeadlessModeSettings {
        size_t width;
        size_t height;

        void* buffer;
        size_t bufferSize;
        // Defaults to RGB
        filament::backend::PixelDataFormat pixelFormat;
        // This is per channel, defaults to UBYTE
        filament::backend::PixelDataType pixelType;

        std::function<void(const HeadlessModeSettings&)> onReady;

        HeadlessModeSettings();
    };

    FilamentRenderer(filament::Engine& engine,
                     void* nativeDrawable,
                     FilamentResourceManager& resourceManager);

    FilamentRenderer(filament::Engine& engine,
                     FilamentResourceManager& resourceManager,
                     HeadlessModeSettings&& headlessSettings);
    ~FilamentRenderer() override;

    SceneHandle CreateScene() override;
    Scene* GetScene(const SceneHandle& id) const override;
    void DestroyScene(const SceneHandle& id) override;

    void BeginFrame() override;
    void Draw() override;
    void EndFrame() override;

    MaterialHandle AddMaterial(const void* materialData,
                               size_t dataSize) override;
    MaterialHandle AddMaterial(const MaterialLoadRequest& request) override;
    MaterialModifier& ModifyMaterial(const MaterialHandle& id) override;
    MaterialModifier& ModifyMaterial(const MaterialInstanceHandle& id) override;

    // Removes scene from scenes list and draws it last
    // WARNING: will destroy previous gui scene if there was any
    void ConvertToGuiScene(const SceneHandle& id);
    FilamentScene* GetGuiScene() const { return guiScene_.get(); }

private:
    filament::Engine& engine_;
    filament::Renderer* renderer_ = nullptr;
    filament::SwapChain* swapChain_ = nullptr;

    std::unordered_map<REHandle_abstract, std::unique_ptr<FilamentScene>>
            scenes_;
    std::unique_ptr<FilamentScene> guiScene_;

    std::unique_ptr<FilamentMaterialModifier> materialsModifier_;
    FilamentResourceManager& resourceManager_;

    bool frameStarted_ = false;

    bool headlessMode_ = false;
    struct HeadlessMode {
        HeadlessModeSettings settings;

        static void FilamentReadyCb(void* buffer, size_t size, void* user);
    } headless_;
};

// Simple helper class to save each rendered frame to file.
// Helper's lifespan should be longer than Renderer's.
// Path is empty by default
// Filename is "frame"
// So, by default frame would be saved to "frame<number>.png"
class HeadlessRenderHelper {
public:
    using Settings = FilamentRenderer::HeadlessModeSettings;

    HeadlessRenderHelper(size_t width, size_t height);
// TBD:
//    HeadlessRenderHelper(size_t width,
//                         size_t height,
//                         filament::backend::PixelDataFormat pixelFormat,
//                         filament::backend::PixelDataType pixelType);

    void SetPath(std::string& path);
    void SetFilename(std::string& name);
    void SetQuality(int quality);

    Settings MakeSettings() const;

private:
    geometry::Image img_;
    Settings settings_;
    std::uint32_t framesCounter_ = 1;

    int quality_ = 100;
    std::string path_;
    std::string filename_ = "frame";

    void OnReadyCallback(const Settings&);
};


}  // namespace visualization
}  // namespace open3d