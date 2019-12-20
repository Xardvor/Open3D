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

#include "Open3D/Open3D.h"

#include "Open3D/Visualization/Rendering/AbstractRenderInterface.h"
#include "Open3D/Visualization/Rendering/Camera.h"
#include "Open3D/Visualization/Rendering/CameraManipulator.h"
#include "Open3D/Visualization/Rendering/RendererStructs.h"
#include "Open3D/Visualization/Rendering/Scene.h"

#if !defined(WIN32)
#    include <unistd.h>
#else
#    include <io.h>
#endif
#include <fcntl.h>

using namespace open3d;

namespace {
std::string getIOErrorString(int errnoVal) {
    switch (errnoVal) {
        case EPERM:
            return "Operation not permitted";
        case EACCES:
            return "Access denied";
        case EAGAIN:
            return "EAGAIN";
#if !defined(WIN32)
        case EDQUOT:
            return "Over quota";
#endif
        case EEXIST:
            return "File already exists";
        case EFAULT:
            return "Bad filename pointer";
        case EINTR:
            return "open() interrupted by a signal";
        case EIO:
            return "I/O error";
        case ELOOP:
            return "Too many symlinks, could be a loop";
        case EMFILE:
            return "Process is out of file descriptors";
        case ENAMETOOLONG:
            return "Filename is too long";
        case ENFILE:
            return "File system table is full";
        case ENOENT:
            return "No such file or directory";
        case ENOSPC:
            return "No space available to create file";
        case ENOTDIR:
            return "Bad path";
        case EOVERFLOW:
            return "File is too big";
        case EROFS:
            return "Can't modify file on read-only filesystem";
        default: {
            std::stringstream s;
            s << "IO error " << errnoVal << " (see sys/errno.h)";
            return s.str();
        }
    }
}

bool readBinaryFile(const std::string& path,
                    std::vector<char>* bytes,
                    std::string* errorStr) {
    bytes->clear();
    if (errorStr) {
        *errorStr = "";
    }

    // Open file
    int fd = open(path.c_str(), O_RDONLY);
    if (fd == -1) {
        if (errorStr) {
            *errorStr = getIOErrorString(errno);
        }
        return false;
    }

    // Get file size
    auto filesize = (size_t)lseek(fd, 0, SEEK_END);
    lseek(fd, 0, SEEK_SET);  // reset file pointer back to beginning

    // Read data
    bytes->resize(filesize);
    read(fd, bytes->data(), filesize);

    // We're done, close and return
    close(fd);
    return true;
}

class DemoWindow : public gui::Window {
    using Super = Window;

    enum MenuIds { FILE_OPEN, FILE_CLOSE, VIEW_WIREFRAME, VIEW_MESH };

public:
    DemoWindow() : gui::Window("Filament Demo", 1280, 720) {
        auto& theme = GetTheme();
        gui::Margins noMargins(0);
        gui::Margins margins(theme.defaultMargin);

        // Menu
        menubar_ = std::make_shared<gui::Menu>();
        auto fileMenu = std::make_shared<gui::Menu>();
        fileMenu->AddItem("Open", "Ctrl-O", FILE_OPEN);
        fileMenu->AddSeparator();
        fileMenu->AddItem("Close", "Ctrl-W", FILE_CLOSE);
        menubar_->AddMenu("File", fileMenu);
        auto viewMenu = std::make_shared<gui::Menu>();
        viewMenu->AddItem("Wireframe", "", VIEW_WIREFRAME);
        viewMenu->AddItem("Mesh", "", VIEW_MESH);
        menubar_->AddMenu("View", viewMenu);
        SetMenubar(menubar_);
        this->OnMenuItemSelected = [this](gui::Menu::ItemId id) {
            this->OnMenuItem(id);
        };

        // Right panel
        verticesLabel_ = std::make_shared<gui::Label>("No vertices");
        trianglesLabel_ = std::make_shared<gui::Label>("No triangles");

        auto redLabel = std::make_shared<gui::Label>("Red");
        redSlider_ = std::make_shared<gui::Slider>(gui::Slider::DOUBLE);
        redSlider_->SetLimits(0.f, 1.f);
        redSlider_->SetValue(1.f);
        auto blueLabel = std::make_shared<gui::Label>("Blue");
        blueSlider_ = std::make_shared<gui::Slider>(gui::Slider::DOUBLE);
        blueSlider_->SetLimits(0.f, 1.f);
        blueSlider_->SetValue(1.f);
        auto greenLabel = std::make_shared<gui::Label>("Green");
        greenSlider_ = std::make_shared<gui::Slider>(gui::Slider::DOUBLE);
        greenSlider_->SetLimits(0.f, 1.f);
        greenSlider_->SetValue(1.f);

        auto roughnessLabel = std::make_shared<gui::Label>("Roughness");
        roughnessSlider_ = std::make_shared<gui::Slider>(gui::Slider::DOUBLE);
        roughnessSlider_->SetLimits(0.f, 1.f);
        roughnessSlider_->SetValue(0.5f);

        auto clearCoatLabel = std::make_shared<gui::Label>("Clear coat");
        clearCoatSlider_ = std::make_shared<gui::Slider>(gui::Slider::DOUBLE);
        clearCoatSlider_->SetLimits(0.f, 1.f);
        clearCoatSlider_->SetValue(1.f);

        auto clearCoatRoughnessLabel =
                std::make_shared<gui::Label>("Clear coat roughness");
        clearCoatRoughnessSlider_ =
                std::make_shared<gui::Slider>(gui::Slider::DOUBLE);
        clearCoatRoughnessSlider_->SetLimits(0.f, 1.f);
        clearCoatRoughnessSlider_->SetValue(0.01f);

        auto pointSizeLabel = std::make_shared<gui::Label>("Point size");
        pointSizeSlider_ = std::make_shared<gui::Slider>(gui::Slider::DOUBLE);
        pointSizeSlider_->SetLimits(1.f, 44.f);

        int spacing = theme.defaultLayoutSpacing;

        auto showMesh = std::make_shared<gui::Button>("Mesh");
        showMesh->OnClicked = std::bind(&DemoWindow::ShowTriangleMesh, this);
        auto showPointcloud = std::make_shared<gui::Button>("Pointcloud");
        showPointcloud->OnClicked =
                std::bind(&DemoWindow::ShowPointCloud, this);

        auto applyMaterial = std::make_shared<gui::Button>("Apply");
        applyMaterial->OnClicked = std::bind(&DemoWindow::ChangeMaterial, this);

        rightPanel_ = std::make_shared<gui::Vert>(
                0, noMargins,
                std::vector<std::shared_ptr<gui::Widget>>(
                        {std::make_shared<gui::Vert>(
                                 0, margins,
                                 std::vector<std::shared_ptr<gui::Widget>>(
                                         {verticesLabel_, trianglesLabel_,
                                          redSlider_, redLabel, blueSlider_,
                                          blueLabel, greenSlider_, greenLabel,
                                          roughnessSlider_, roughnessLabel,
                                          clearCoatSlider_, clearCoatLabel,
                                          clearCoatRoughnessSlider_,
                                          clearCoatRoughnessLabel,
                                          pointSizeSlider_, pointSizeLabel,
                                          gui::Vert::MakeFixed(spacing),
                                          applyMaterial})),
                         gui::Vert::MakeStretch(),
                         std::make_shared<gui::Vert>(
                                 spacing, margins,
                                 std::vector<std::shared_ptr<gui::Widget>>(
                                         {showMesh, showPointcloud}))}));
        AddChild(rightPanel_);

        LoadDefaultMaterials();
        LoadDefaultMeshes();

        // Create scene
        const auto sceneId = GetRenderer().CreateScene();
        auto scene = GetRenderer().GetScene(sceneId);
        sceneWidget_ = std::make_shared<gui::SceneWidget>(*scene);
        sceneWidget_->SetBackgroundColor(gui::Color(0.5, 0.5, 1.0));

        sceneWidget_->GetCameraManipulator()->SetFov(100.0f);
        sceneWidget_->GetCameraManipulator()->SetNearPlane(0.1f);
        sceneWidget_->GetCameraManipulator()->SetFarPlane(1000.0f);
        sceneWidget_->GetCameraManipulator()->LookAt({0, 0, 0}, {80, 0, 80});

        // Create light
        visualization::LightDescription lightDescription;
        lightDescription.intensity = 100000;
        lightDescription.direction = {-0.707, -.707, 0.0};
        lightDescription.customAttributes["custom_type"] = "SUN";

        scene->AddLight(lightDescription);

        AddChild(sceneWidget_);

        ShowTriangleMesh();
    }

    void OnMenuItem(gui::Menu::ItemId id) {
        switch (id) {
            case FILE_CLOSE:
                this->Close();
                break;
            default:
                break;
        }
    }

protected:
    void Layout(const gui::Theme& theme) override {
        auto contentRect = GetContentRect();

        auto rightSize = rightPanel_->CalcPreferredSize(theme);
        gui::Rect rightRect(contentRect.width - rightSize.width, contentRect.y,
                            rightSize.width, contentRect.height);
        rightPanel_->SetFrame(rightRect);

        sceneWidget_->SetFrame(gui::Rect(contentRect.x, contentRect.y,
                                         rightRect.x, contentRect.height));

        Super::Layout(theme);
    }

    void OnMouseMove(const gui::MouseMoveEvent& e) override {
        Super::OnMouseMove(e);

        if (controlsState_.mode == ControlsState::Mode::None) {
            return;
        }

        controlsState_.lastMouseX = controlsState_.mouseX;
        controlsState_.lastMouseY = controlsState_.mouseY;

        controlsState_.mouseX = e.x;
        controlsState_.mouseY = e.y;

        controlsState_.mouseMovePending = true;
    }

    void OnMouseButton(const gui::MouseButtonEvent& e) override {
        Super::OnMouseButton(e);

        if (e.button == gui::MouseButton::RIGHT) {
            const bool isDown = (e.type == gui::MouseButtonEvent::DOWN);

            if (!controlsState_.rmbDown && isDown) {
                controlsState_.lastMouseX = e.x;
                controlsState_.lastMouseY = e.y;

                controlsState_.mouseX = e.x;
                controlsState_.mouseY = e.y;
            }

            controlsState_.rmbDown = isDown;
        } else if (e.button == gui::MouseButton::LEFT) {
            const bool isDown = (e.type == gui::MouseButtonEvent::DOWN);

            if (!controlsState_.lmbDown && isDown && !controlsState_.rmbDown) {
                controlsState_.lastMouseX = e.x;
                controlsState_.lastMouseY = e.y;

                controlsState_.mouseX = e.x;
                controlsState_.mouseY = e.y;
            }

            controlsState_.lmbDown = isDown;
        }

        // Prefer flying over interacting
        if (controlsState_.rmbDown) {
            controlsState_.mode = ControlsState::Mode::Fly;
        } else if (controlsState_.lmbDown) {
            controlsState_.mode = ControlsState::Mode::Interact;
        } else {
            controlsState_.mode = ControlsState::Mode::None;
        }
    }

    void OnMouseWheel(const gui::MouseWheelEvent& e) override {
        Super::OnMouseWheel(e);
    }

    void OnKey(const gui::KeyEvent& e) override {
        Super::OnKey(e);

        if (e.type == gui::KeyEvent::DOWN) {
            controlsState_.pressedKeys.insert(e.key);
            if (e.key == 'r') {
                controlsState_.mode = ControlsState::Mode::None;

                sceneWidget_->GetScene()->SetEntityTransform(
                        geometryHandle,
                        visualization::Scene::Transform::Identity());

                FitGeometryToView();
            }
        } else {
            controlsState_.pressedKeys.erase(e.key);
        }
    }

    DrawResult OnDraw(float dtSec) override {
        auto scene = sceneWidget_->GetScene();

        const float mouseDx = controlsState_.mouseX - controlsState_.lastMouseX;
        const float mouseDy = controlsState_.mouseY - controlsState_.lastMouseY;

        if (controlsState_.mouseMovePending &&
            controlsState_.mode == ControlsState::Mode::Interact) {
            auto eTransform = scene->GetEntityTransform(geometryHandle);
            //auto pos = eTransform.translation();

            const float rsx = M_PI * 25.f;
            const float rsy = M_PI * 25.f;

            const float dYaw = mouseDx * dtSec * M_PI / 180.f * rsx;
            const float dPitch = mouseDy * dtSec * M_PI / 180.f * rsy;

            auto cameraman = sceneWidget_->GetCameraManipulator();

            Eigen::AngleAxisf yawAngle(dYaw, cameraman->GetUpVector());
            Eigen::AngleAxisf pitchAngle(dPitch, cameraman->GetForwardVector());

            eTransform = eTransform.rotate(yawAngle * pitchAngle);
            scene->SetEntityTransform(geometryHandle, eTransform);

        } else if (controlsState_.mode == ControlsState::Mode::Fly) {
            auto cameraman = sceneWidget_->GetCameraManipulator();

            float dx = 0.f;
            float dz = 0.f;
#define IS_KEY_PRESSED(Key) (controlsState_.pressedKeys.count((Key)) > 0)
            if (IS_KEY_PRESSED('w')) {
                dz += 1.f;
            }
            if (IS_KEY_PRESSED('s')) {
                dz -= 1.f;
            }

            if (IS_KEY_PRESSED('a')) {
                dx -= 1.f;
            }
            if (IS_KEY_PRESSED('d')) {
                dx += 1.f;
            }
#undef IS_KEY_PRESSED

            auto f = cameraman->GetForwardVector();

            if (controlsState_.mouseMovePending) {
                const float rsx = M_PI * 10.f;
                const float rsy = M_PI * 10.f;

                const float dYaw = -mouseDx * dtSec * M_PI / 180.f * rsx;
                const float dPitch = -mouseDy * dtSec * M_PI / 180.f * rsy;

                Eigen::AngleAxisf yawAngle(dYaw, cameraman->GetUpVector());
                Eigen::AngleAxisf pitchAngle(dPitch,
                                             cameraman->GetLeftVector());
                f = pitchAngle * yawAngle * f;

                cameraman->SetForwardVector(f);
            }

            const float speed = 100.f;
            auto l = cameraman->GetLeftVector();
            Eigen::Vector3f offset = (l * dx + f * dz) * speed * dtSec;
            auto p = cameraman->GetPosition();
            cameraman->SetPosition(p + offset);
        }

        controlsState_.mouseMovePending = false;

        return Super::OnDraw(dtSec);
    }

private:
    struct ControlsState {
        enum class Mode { None, Fly, Interact };

        std::unordered_set<std::uint32_t> pressedKeys;
        bool lmbDown = false;
        bool rmbDown = false;

        float mouseWheel = 0.f;
        int mouseX = 0;
        int mouseY = 0;
        bool mouseMovePending = false;

        int lastMouseX = 0;
        int lastMouseY = 0;

        Mode mode = Mode::None;
    };

    std::shared_ptr<gui::Menu> menubar_;
    std::shared_ptr<gui::SceneWidget> sceneWidget_;
    std::shared_ptr<gui::Vert> rightPanel_;
    std::shared_ptr<gui::Label> verticesLabel_;
    std::shared_ptr<gui::Label> trianglesLabel_;
    std::shared_ptr<gui::Slider> redSlider_;
    std::shared_ptr<gui::Slider> blueSlider_;
    std::shared_ptr<gui::Slider> greenSlider_;
    std::shared_ptr<gui::Slider> roughnessSlider_;
    std::shared_ptr<gui::Slider> clearCoatSlider_;
    std::shared_ptr<gui::Slider> clearCoatRoughnessSlider_;
    std::shared_ptr<gui::Slider> pointSizeSlider_;

    visualization::GeometryHandle geometryHandle;
    std::unique_ptr<geometry::Geometry3D> customGeometry;
    std::unique_ptr<geometry::TriangleMesh> defaultMesh;
    std::unique_ptr<geometry::PointCloud> defaultPointCloud;

    visualization::MaterialHandle customMaterial;
    visualization::MaterialHandle defaultMeshMaterial;
    visualization::MaterialInstanceHandle iDefaultMeshMaterial;
    visualization::MaterialHandle defaultPointCloudMaterial;
    visualization::MaterialInstanceHandle iDefaultPointCloudMaterial;

    ControlsState controlsState_;

    void LoadDefaultMaterials() {
        std::string errorStr;
        std::vector<char> materialData;
        const std::string resourcePath =
                gui::Application::GetInstance().GetResourcePath();

        if (!readBinaryFile(resourcePath + "/nonmetal.filamat", &materialData,
                            &errorStr)) {
            utility::LogWarning("Failed to read default mesh material ({})",
                                errorStr.data());
        } else {
            defaultMeshMaterial = GetRenderer().AddMaterial(
                    materialData.data(), materialData.size());

            iDefaultMeshMaterial =
                    GetRenderer()
                            .ModifyMaterial(defaultMeshMaterial)
                            .SetColor("baseColor", {1.f, 1.f, 1.f})
                            .SetParameter("roughness", 0.5f)
                            .SetParameter("clearCoat", 1.f)
                            .SetParameter("clearCoatRoughness", 0.01f)
                            .Finish();
        }

        materialData.clear();
        if (!readBinaryFile(resourcePath + "/pointcloud.filamat", &materialData,
                            &errorStr)) {
            utility::LogWarning(
                    "Failed to read default pointcloud material ({})",
                    errorStr.data());
        } else {
            defaultPointCloudMaterial = GetRenderer().AddMaterial(
                    materialData.data(), materialData.size());

            iDefaultPointCloudMaterial =
                    GetRenderer()
                            .ModifyMaterial(defaultPointCloudMaterial)
                            .SetParameter("pointSize", 1.f)
                            .Finish();
        }
    }

    void LoadDefaultMeshes() {
        const std::string resourcePath =
                gui::Application::GetInstance().GetResourcePath();

        defaultMesh.reset();
        auto mesh = std::make_unique<geometry::TriangleMesh>();
        const auto meshPath = resourcePath + "/knot.ply";
        if (!io::ReadTriangleMesh(meshPath, *mesh)) {
            utility::LogWarning("Failed to load mesh from {}", meshPath.data());
        } else {
            mesh->ComputeVertexNormals();
            mesh->NormalizeNormals();
            defaultMesh = std::move(mesh);
        }

        defaultPointCloud.reset();
        auto pointcloud = std::make_unique<geometry::PointCloud>();
        const auto pointcloudPath = resourcePath + "/fragment.ply";
        if (!io::ReadPointCloud(pointcloudPath, *pointcloud)) {
            utility::LogWarning("Failed to load pointcloud from {}",
                                pointcloudPath.data());
        } else {
            pointcloud->NormalizeNormals();
            defaultPointCloud = std::move(pointcloud);
        }
    }

    void ShowTriangleMesh() {
        auto scene = sceneWidget_->GetScene();

        if (defaultMesh) {
            scene->RemoveGeometry(geometryHandle);
            geometryHandle =
                    scene->AddGeometry(*defaultMesh, iDefaultMeshMaterial);

            auto s = fmt::format("Vertices: {}", defaultMesh->vertices_.size());
            verticesLabel_->SetText(s.data());

            s = fmt::format("Tris: {}", defaultMesh->triangles_.size());
            trianglesLabel_->SetText(s.data());
        }

        FitGeometryToView();
    }

    void ShowPointCloud() {
        auto scene = sceneWidget_->GetScene();

        if (defaultPointCloud) {
            scene->RemoveGeometry(geometryHandle);
            geometryHandle = scene->AddGeometry(*defaultPointCloud,
                                                iDefaultPointCloudMaterial);

            auto s = fmt::format("Vertices: {}",
                                 defaultPointCloud->points_.size());
            verticesLabel_->SetText(s.data());

            trianglesLabel_->SetText("Tris: none");
        }

        FitGeometryToView();
    }

    void FitGeometryToView() {
        auto cameraman = sceneWidget_->GetCameraManipulator();

        auto transform =
                sceneWidget_->GetScene()->GetEntityTransform(geometryHandle);
        auto geometryPos = transform.translation();
        Eigen::Vector3f forward = geometryPos - cameraman->GetPosition();
        forward.normalize();

        auto bs = sceneWidget_->GetScene()->GetEntityBoundingSphere(
                geometryHandle);
        const float zoom = bs.second / tanf(cameraman->GetFov() / 2.f);
        Eigen::Vector3f eye = geometryPos + zoom * forward;

        cameraman->LookAt(bs.first, eye);
    }

    void ChangeMaterial() {
        GetRenderer()
                .ModifyMaterial(iDefaultMeshMaterial)
                .SetColor("baseColor", {(float)redSlider_->GetDoubleValue(),
                                        (float)blueSlider_->GetDoubleValue(),
                                        (float)greenSlider_->GetDoubleValue()})
                .SetParameter("roughness", (float)roughnessSlider_->GetDoubleValue())
                .SetParameter("clearCoat", (float)clearCoatSlider_->GetDoubleValue())
                .SetParameter("clearCoatRoughness", (float)clearCoatRoughnessSlider_->GetDoubleValue())
                .Finish();

        GetRenderer()
                .ModifyMaterial(iDefaultPointCloudMaterial)
                .SetParameter("pointSize", (float)pointSizeSlider_->GetDoubleValue())
                .Finish();
    }
};
}

int main(int argc, const char *argv[]) {
    auto &app = gui::Application::GetInstance();
    app.Initialize(argc, argv);

    auto w = std::make_shared<DemoWindow>();
    app.AddWindow(w);
    w->Show();

    app.Run();
    return 0;
}
