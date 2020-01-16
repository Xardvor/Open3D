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

#include "FilamentRenderer.h"

#include <filament/Engine.h>
#include <filament/LightManager.h>
#include <filament/RenderableManager.h>
#include <filament/Renderer.h>
#include <filament/Scene.h>

#include "FilamentCamera.h"
#include "FilamentEntitiesMods.h"
#include "FilamentResourceManager.h"
#include "FilamentScene.h"
#include "FilamentView.h"

#include "Open3D/IO/ClassIO/ImageIO.h"

namespace open3d {
namespace visualization {

FilamentRenderer::HeadlessModeSettings::HeadlessModeSettings()
    : width(0u),
      height(0u),
      buffer(nullptr),
      bufferSize(0u),
      pixelFormat(filament::backend::PixelDataFormat::RGB),
      pixelType(filament::backend::PixelDataType::UBYTE) {}

FilamentRenderer::FilamentRenderer(filament::Engine& aEngine,
                                   void* nativeDrawable,
                                   FilamentResourceManager& aResourceManager)
    : engine_(aEngine), resourceManager_(aResourceManager) {
    swapChain_ = engine_.createSwapChain(nativeDrawable);
    renderer_ = engine_.createRenderer();

    materialsModifier_ = std::make_unique<FilamentMaterialModifier>();
}

FilamentRenderer::FilamentRenderer(filament::Engine& aEngine,
                                   FilamentResourceManager& aResourceManager,
                                   HeadlessModeSettings&& headlessSettings)
    : engine_(aEngine), resourceManager_(aResourceManager) {
    headlessMode_ = true;
    headless_.settings = std::move(headlessSettings);

    swapChain_ = engine_.createSwapChain(headless_.settings.width,
                                         headless_.settings.height);
    renderer_ = engine_.createRenderer();

    materialsModifier_ = std::make_unique<FilamentMaterialModifier>();
}

FilamentRenderer::~FilamentRenderer() {
    scenes_.clear();

    engine_.destroy(renderer_);
    engine_.destroy(swapChain_);
}

SceneHandle FilamentRenderer::CreateScene() {
    auto handle = SceneHandle::Next();
    scenes_[handle] =
            std::make_unique<FilamentScene>(engine_, resourceManager_);

    return handle;
}

Scene* FilamentRenderer::GetScene(const SceneHandle& id) const {
    auto found = scenes_.find(id);
    if (found != scenes_.end()) {
        return found->second.get();
    }

    return nullptr;
}

void FilamentRenderer::DestroyScene(const SceneHandle& id) {
    scenes_.erase(id);
}

void FilamentRenderer::BeginFrame() {
    frameStarted_ = renderer_->beginFrame(swapChain_);
}

void FilamentRenderer::Draw() {
    if (frameStarted_) {
        for (const auto& pair : scenes_) {
            pair.second->Draw(*renderer_);
        }

        if (guiScene_) {
            guiScene_->Draw(*renderer_);
        }

        if (headlessMode_) {
            using namespace filament;
            using namespace backend;

            // memset(headless_.settings.buffer, 0, headless_.settings.bufferSize);

            PixelBufferDescriptor pd(headless_.settings.buffer,
                                     headless_.settings.bufferSize,
                                     PixelDataFormat::RGB, PixelDataType::UBYTE,
                                     HeadlessMode::FilamentReadyCb, &headless_);

            renderer_->readPixels(0, 0, headless_.settings.width,
                                  headless_.settings.height, std::move(pd));
        }
    }
}

void FilamentRenderer::EndFrame() {
    if (frameStarted_) {
        renderer_->endFrame();

        if (headlessMode_) {
            engine_.flushAndWait();
        }
    }
}

MaterialHandle FilamentRenderer::AddMaterial(const void* materialData,
                                             const size_t dataSize) {
    return resourceManager_.CreateMaterial(materialData, dataSize);
}

MaterialHandle FilamentRenderer::AddMaterial(
        const MaterialLoadRequest& request) {
    return resourceManager_.CreateMaterial(request);
}

MaterialModifier& FilamentRenderer::ModifyMaterial(const MaterialHandle& id) {
    materialsModifier_->Reset();

    auto instanceId = resourceManager_.CreateMaterialInstance(id);

    if (instanceId) {
        auto wMaterialInstance =
                resourceManager_.GetMaterialInstance(instanceId);
        materialsModifier_->InitWithMaterialInstance(wMaterialInstance.lock(),
                                                     instanceId);
    }

    return *materialsModifier_;
}

MaterialModifier& FilamentRenderer::ModifyMaterial(
        const MaterialInstanceHandle& id) {
    materialsModifier_->Reset();

    auto wMaterialInstance = resourceManager_.GetMaterialInstance(id);
    if (!wMaterialInstance.expired()) {
        materialsModifier_->InitWithMaterialInstance(wMaterialInstance.lock(),
                                                     id);
    }

    return *materialsModifier_;
}

void FilamentRenderer::ConvertToGuiScene(const SceneHandle& id) {
    auto found = scenes_.find(id);
    if (found != scenes_.end()) {
        // TODO: Warning on guiScene != nullptr

        guiScene_ = std::move(found->second);
        scenes_.erase(found);
    }

    // TODO: assert
}

void FilamentRenderer::HeadlessMode::FilamentReadyCb(void* buffer,
                                                     size_t size,
                                                     void* user) {
    auto self = static_cast<HeadlessMode*>(user);
    self->settings.onReady(self->settings);

    // free(buffer);
}

HeadlessRenderHelper::HeadlessRenderHelper(size_t w, size_t h) {
    settings_.width = w;
    settings_.height = h;

    img_.Prepare(w, h, 3, sizeof(std::uint8_t));

    settings_.bufferSize = w * h * 3 * sizeof(std::uint8_t);
    settings_.buffer = img_.data_.data();

    settings_.onReady = std::bind(&HeadlessRenderHelper::OnReadyCallback, this,
                                  std::placeholders::_1);
}

//HeadlessRenderHelper::HeadlessRenderHelper(
//        size_t width,
//        size_t height,
//        filament::backend::PixelDataFormat pixelFormat,
//        filament::backend::PixelDataType pixelType) {}

void HeadlessRenderHelper::SetPath(std::string& path) {
    path_ = path;
}

void HeadlessRenderHelper::SetFilename(std::string& name) {
    filename_ = name;
}

void HeadlessRenderHelper::SetQuality(int quality) {
    quality_ = quality;
}

HeadlessRenderHelper::Settings HeadlessRenderHelper::MakeSettings() const {
    return settings_;
}

void HeadlessRenderHelper::OnReadyCallback(const Settings&) {
    io::WriteImage(path_ + filename_ + std::to_string(framesCounter_) + ".png", img_, quality_);
    ++framesCounter_;
}

}  // namespace visualization
}  // namespace open3d
