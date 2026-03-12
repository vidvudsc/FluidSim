#include "imgui_panel.h"

#include "imgui.h"
#include "rlImGui.h"

static void ApplyGasSimTheme(void)
{
    ImGuiStyle &style = ImGui::GetStyle();
    style.WindowRounding = 0.0f;
    style.ChildRounding = 0.0f;
    style.FrameRounding = 0.0f;
    style.PopupRounding = 0.0f;
    style.GrabRounding = 0.0f;
    style.ScrollbarRounding = 0.0f;
    style.TabRounding = 0.0f;
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 0.0f;
    style.PopupBorderSize = 1.0f;
    style.WindowPadding = ImVec2(14.0f, 14.0f);
    style.FramePadding = ImVec2(12.0f, 8.0f);
    style.ItemSpacing = ImVec2(10.0f, 10.0f);
    style.ItemInnerSpacing = ImVec2(8.0f, 6.0f);

    ImVec4 *colors = style.Colors;
    colors[ImGuiCol_Text] = ImVec4(0.93f, 0.96f, 1.00f, 1.00f);
    colors[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.58f, 0.69f, 1.00f);
    colors[ImGuiCol_WindowBg] = ImVec4(0.04f, 0.05f, 0.08f, 0.95f);
    colors[ImGuiCol_ChildBg] = ImVec4(0.06f, 0.08f, 0.12f, 0.90f);
    colors[ImGuiCol_PopupBg] = ImVec4(0.05f, 0.07f, 0.11f, 0.98f);
    colors[ImGuiCol_Border] = ImVec4(0.16f, 0.27f, 0.44f, 0.80f);
    colors[ImGuiCol_FrameBg] = ImVec4(0.08f, 0.11f, 0.18f, 1.00f);
    colors[ImGuiCol_FrameBgHovered] = ImVec4(0.12f, 0.20f, 0.34f, 1.00f);
    colors[ImGuiCol_FrameBgActive] = ImVec4(0.15f, 0.28f, 0.49f, 1.00f);
    colors[ImGuiCol_TitleBg] = ImVec4(0.07f, 0.12f, 0.22f, 1.00f);
    colors[ImGuiCol_TitleBgActive] = ImVec4(0.11f, 0.22f, 0.42f, 1.00f);
    colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.06f, 0.10f, 0.18f, 0.95f);
    colors[ImGuiCol_CheckMark] = ImVec4(0.55f, 0.78f, 1.00f, 1.00f);
    colors[ImGuiCol_SliderGrab] = ImVec4(0.35f, 0.62f, 0.98f, 1.00f);
    colors[ImGuiCol_SliderGrabActive] = ImVec4(0.58f, 0.82f, 1.00f, 1.00f);
    colors[ImGuiCol_Button] = ImVec4(0.12f, 0.22f, 0.38f, 1.00f);
    colors[ImGuiCol_ButtonHovered] = ImVec4(0.18f, 0.32f, 0.54f, 1.00f);
    colors[ImGuiCol_ButtonActive] = ImVec4(0.20f, 0.40f, 0.70f, 1.00f);
    colors[ImGuiCol_Header] = ImVec4(0.11f, 0.21f, 0.36f, 1.00f);
    colors[ImGuiCol_HeaderHovered] = ImVec4(0.16f, 0.30f, 0.51f, 1.00f);
    colors[ImGuiCol_HeaderActive] = ImVec4(0.20f, 0.38f, 0.66f, 1.00f);
    colors[ImGuiCol_Separator] = ImVec4(0.17f, 0.29f, 0.47f, 0.85f);
    colors[ImGuiCol_ResizeGrip] = ImVec4(0.31f, 0.56f, 0.90f, 0.25f);
    colors[ImGuiCol_ResizeGripHovered] = ImVec4(0.42f, 0.69f, 0.98f, 0.78f);
    colors[ImGuiCol_ResizeGripActive] = ImVec4(0.50f, 0.76f, 1.00f, 0.95f);
    colors[ImGuiCol_Tab] = ImVec4(0.08f, 0.13f, 0.22f, 1.00f);
    colors[ImGuiCol_TabHovered] = ImVec4(0.18f, 0.31f, 0.52f, 1.00f);
    colors[ImGuiCol_TabActive] = ImVec4(0.14f, 0.24f, 0.41f, 1.00f);
}

static void DrawCombo(const char *label, const char *preview, const char *const *items, int itemCount,
    int currentItem, int *selectedItem, bool *changed, bool disabled)
{
    if (disabled) {
        ImGui::BeginDisabled(true);
    }

    if (ImGui::BeginCombo(label, preview)) {
        for (int i = 0; i < itemCount; ++i) {
            bool isSelected = (i == currentItem);
            if (ImGui::Selectable(items[i], isSelected)) {
                *selectedItem = i;
                *changed = true;
            }
            if (isSelected) {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndCombo();
    }

    if (disabled) {
        ImGui::EndDisabled();
    }
}

void UiPanelSetup(void)
{
    rlImGuiSetup(true);

    ImGuiIO &io = ImGui::GetIO();
    io.ConfigWindowsMoveFromTitleBarOnly = true;

    Vector2 dpiScale = GetWindowScaleDPI();
    const float scale = (dpiScale.x > 0.0f) ? dpiScale.x : 1.0f;
    if (scale > 1.01f) {
        ImGui::GetStyle().ScaleAllSizes(scale);
        io.FontGlobalScale = 1.0f / scale;
    }

    ApplyGasSimTheme();
}

void UiPanelShutdown(void)
{
    rlImGuiShutdown();
}

void UiPanelBegin(float deltaTime)
{
    rlImGuiBeginDelta(deltaTime);
}

void UiPanelEnd(void)
{
    rlImGuiEnd();
}

UiCaptureState UiPanelGetCaptureState(void)
{
    ImGuiIO &io = ImGui::GetIO();
    UiCaptureState capture = {
        io.WantCaptureMouse ||
            ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow) ||
            ImGui::IsAnyItemHovered() ||
            ImGui::IsAnyItemActive(),
        io.WantCaptureKeyboard ||
            ImGui::IsAnyItemActive() ||
            ImGui::IsWindowFocused(ImGuiFocusedFlags_AnyWindow),
    };
    return capture;
}

void UiPanelDraw(const UiPanelState *state, UiPanelActions *actions)
{
    static const char *const backendLabels[] = {"CPU", "GPU"};
    static const char *const cpuCountLabels[] = {"10k", "20k", "30k", "40k", "50k"};
    static const int cpuCounts[] = {10000, 20000, 30000, 40000, 50000};
    static const char *const gpuCountLabels[] = {"30k", "60k", "120k", "180k", "250k"};
    static const int gpuCounts[] = {30000, 60000, 120000, 180000, 250000};
    static const char *const modeLabels[] = {"Water tank", "Gas tank", "Wind tunnel"};
    static const char *const obstacleLabels[] = {"Circle", "Airfoil", "Car", "Rectangle"};
    static const char *const viewLabels[] = {"Particles", "Smoothing"};
    static const char *const colorLabels[] = {"Material", "Temperature", "Pressure", "Speed", "Density"};

    *actions = {};

    int backend = state->backend;
    int mode = state->mode;
    int obstacleModel = state->obstacleModel;
    float obstacleAngleDegrees = state->obstacleAngleDegrees;
    float obstacleRectWidth = state->obstacleRectWidth;
    float obstacleRectHeight = state->obstacleRectHeight;
    int viewMode = state->viewMode;
    int colorMode = state->colorMode;
    float windSpeedScale = state->windSpeedScale;
    bool acousticsEnabled = state->acousticsEnabled;
    float speakerFrequency = state->speakerFrequency;
    float speakerAmplitude = state->speakerAmplitude;
    float acousticSoundSpeed = state->acousticSoundSpeed;
    float acousticMachLimit = state->acousticMachLimit;
    float acousticViscosityScale = state->acousticViscosityScale;
    float acousticDragScale = state->acousticDragScale;
    float audioMonitorPitchHz = state->audioMonitorPitchHz;
    float speakerWidth = state->speakerWidth;
    float speakerHeight = state->speakerHeight;
    float micRadius = state->micRadius;
    bool audioOutputEnabled = state->audioOutputEnabled;

    const bool gpuAvailable = state->gpuBackendAvailable;
    const int *countValues = (backend == 1) ? gpuCounts : cpuCounts;
    const char *const *countLabels = (backend == 1) ? gpuCountLabels : cpuCountLabels;
    const int countLabelCount = 5;
    int countIndex = 0;
    for (int i = 0; i < countLabelCount; ++i) {
        if (countValues[i] == state->targetParticleCount) {
            countIndex = i;
            break;
        }
    }

    ImGui::SetNextWindowPos(ImVec2(18.0f, 18.0f), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(404.0f, 548.0f), ImGuiCond_FirstUseEver);

    if (ImGui::Begin("Controls")) {
        ImGui::SetWindowFontScale(1.38f);
        ImGui::Text("Particles %d / %d", state->actualParticleCount, state->targetParticleCount);
        ImGui::Text("FPS %.1f", state->fps);
        ImGui::Text("Step %.2f ms", (float)state->lastSimStepMs);
        if (state->flowMach > 0.0f) {
            ImGui::Text("Mach %.2f", state->flowMach);
        }
        ImGui::SetWindowFontScale(1.18f);

        if (ImGui::CollapsingHeader("Simulation", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool backendChanged = false;
            DrawCombo("Backend", backendLabels[backend], backendLabels, 2, backend, &backend, &backendChanged, false);
            if (!gpuAvailable) {
                ImGui::TextDisabled("GPU backend unavailable in this build.");
            }
            if (backendChanged) {
                if (backend == 0 || gpuAvailable) {
                    actions->setBackend = true;
                    actions->backend = backend;
                } else {
                    backend = state->backend;
                }
            }

            countValues = (backend == 1) ? gpuCounts : cpuCounts;
            countLabels = (backend == 1) ? gpuCountLabels : cpuCountLabels;
            countIndex = 0;
            for (int i = 0; i < countLabelCount; ++i) {
                if (countValues[i] == state->targetParticleCount) {
                    countIndex = i;
                    break;
                }
            }

            bool countChanged = false;
            DrawCombo("Particle count", countLabels[countIndex], countLabels, countLabelCount, countIndex, &countIndex,
                &countChanged, false);
            if (countChanged) {
                actions->setTargetParticleCount = true;
                actions->targetParticleCount = countValues[countIndex];
            }

            bool paused = state->paused;
            if (ImGui::Checkbox("Paused", &paused)) {
                actions->setPaused = true;
                actions->paused = paused;
            }
            ImGui::SameLine();
            if (ImGui::Button("Respawn")) {
                actions->requestReset = true;
            }
        }

        if (ImGui::CollapsingHeader("Mode", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool modeChanged = false;
            DrawCombo("Scene", modeLabels[mode], modeLabels, UI_MODE_COUNT, mode, &mode, &modeChanged, false);
            if (modeChanged) {
                actions->setMode = true;
                actions->mode = mode;
            }

            bool obstacleChanged = false;
            DrawCombo("Wind shape", obstacleLabels[obstacleModel], obstacleLabels, 4, obstacleModel, &obstacleModel,
                &obstacleChanged, mode != UI_MODE_WIND_TUNNEL);
            if (obstacleChanged) {
                actions->setObstacleModel = true;
                actions->obstacleModel = obstacleModel;
            }

            if (mode != UI_MODE_WIND_TUNNEL) {
                ImGui::TextDisabled("Switch scene to Wind tunnel to use shape and speed.");
            }

            if (mode == UI_MODE_WIND_TUNNEL) {
                if (ImGui::SliderFloat("Tunnel speed", &windSpeedScale, 0.35f, 2.50f, "%.2fx")) {
                    actions->setWindSpeedScale = true;
                    actions->windSpeedScale = windSpeedScale;
                }
                ImGui::TextDisabled("Current Mach %.2f", state->flowMach);

                if (ImGui::SliderFloat("Obstacle angle", &obstacleAngleDegrees, -180.0f, 180.0f, "%.0f deg")) {
                    actions->setObstacleAngleDegrees = true;
                    actions->obstacleAngleDegrees = obstacleAngleDegrees;
                }

                ImGui::BeginDisabled(obstacleModel != 3);
                if (ImGui::SliderFloat("Rect width", &obstacleRectWidth, 40.0f, 360.0f, "%.0f px")) {
                    actions->setObstacleRectWidth = true;
                    actions->obstacleRectWidth = obstacleRectWidth;
                }
                if (ImGui::SliderFloat("Rect height", &obstacleRectHeight, 24.0f, 220.0f, "%.0f px")) {
                    actions->setObstacleRectHeight = true;
                    actions->obstacleRectHeight = obstacleRectHeight;
                }
                ImGui::EndDisabled();
            }
        }

        if (ImGui::CollapsingHeader("Visuals", ImGuiTreeNodeFlags_DefaultOpen)) {
            bool viewChanged = false;
            DrawCombo("View", viewLabels[viewMode], viewLabels, 2, viewMode, &viewMode, &viewChanged, false);
            if (viewChanged) {
                actions->setViewMode = true;
                actions->viewMode = viewMode;
            }

            bool colorChanged = false;
            DrawCombo("Color", colorLabels[colorMode], colorLabels, 5, colorMode, &colorMode, &colorChanged, false);
            if (colorChanged) {
                actions->setColorMode = true;
                actions->colorMode = colorMode;
            }
        }

        if (ImGui::CollapsingHeader("Acoustics", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (!state->acousticsAvailable) {
                ImGui::TextDisabled("Acoustics are available only in Gas tank mode.");
            }

            ImGui::BeginDisabled(!state->acousticsAvailable);

            if (ImGui::Checkbox("Enable speaker + mic", &acousticsEnabled)) {
                actions->setAcousticsEnabled = true;
                actions->acousticsEnabled = acousticsEnabled;
            }

            if (ImGui::SliderFloat("Frequency", &speakerFrequency, 0.2f, 40.0f, "%.2f Hz")) {
                actions->setSpeakerFrequency = true;
                actions->speakerFrequency = speakerFrequency;
            }

            if (ImGui::SliderFloat("Amplitude", &speakerAmplitude, 1.0f, 42.0f, "%.1f px")) {
                actions->setSpeakerAmplitude = true;
                actions->speakerAmplitude = speakerAmplitude;
            }
            ImGui::Text("Effective amp %.2f px", state->effectiveSpeakerAmplitude);

            if (ImGui::SliderFloat("Acoustic c", &acousticSoundSpeed, 52.0f, 420.0f, "%.0f")) {
                actions->setAcousticSoundSpeed = true;
                actions->acousticSoundSpeed = acousticSoundSpeed;
            }

            if (ImGui::SliderFloat("Mach cap", &acousticMachLimit, 0.10f, 0.60f, "%.2f c")) {
                actions->setAcousticMachLimit = true;
                actions->acousticMachLimit = acousticMachLimit;
            }

            if (ImGui::SliderFloat("Viscosity x", &acousticViscosityScale, 0.05f, 1.00f, "%.2f")) {
                actions->setAcousticViscosityScale = true;
                actions->acousticViscosityScale = acousticViscosityScale;
            }

            if (ImGui::SliderFloat("Drag x", &acousticDragScale, 0.00f, 1.00f, "%.2f")) {
                actions->setAcousticDragScale = true;
                actions->acousticDragScale = acousticDragScale;
            }

            if (ImGui::SliderFloat("Speaker width", &speakerWidth, 8.0f, 80.0f, "%.0f px")) {
                actions->setSpeakerWidth = true;
                actions->speakerWidth = speakerWidth;
            }

            if (ImGui::SliderFloat("Speaker height", &speakerHeight, 24.0f, 220.0f, "%.0f px")) {
                actions->setSpeakerHeight = true;
                actions->speakerHeight = speakerHeight;
            }

            if (ImGui::SliderFloat("Mic radius", &micRadius, 8.0f, 80.0f, "%.0f px")) {
                actions->setMicRadius = true;
                actions->micRadius = micRadius;
            }

            if (ImGui::Checkbox("Output mic to speakers", &audioOutputEnabled)) {
                actions->setAudioOutputEnabled = true;
                actions->audioOutputEnabled = audioOutputEnabled;
            }
            if (!state->audioOutputAvailable) {
                ImGui::TextDisabled("Speaker output unavailable.");
            }
            if (ImGui::SliderFloat("Monitor pitch", &audioMonitorPitchHz, 0.0f, 1000.0f, "%.0f Hz")) {
                actions->setAudioMonitorPitchHz = true;
                actions->audioMonitorPitchHz = audioMonitorPitchHz;
            }
            ImGui::TextDisabled("0 Hz plays the raw low-frequency mic signal.");

            ImGui::Text("Mic signal %.3f", state->micSignal);
            if (state->micWaveform != nullptr && state->micWaveformCount > 1) {
                ImGui::PlotLines("Waveform", state->micWaveform, state->micWaveformCount, 0, NULL, -1.0f, 1.0f,
                    ImVec2(0.0f, 96.0f));
            }
            ImGui::TextDisabled("Drag the speaker rectangle and mic circle in the tank.");

            ImGui::EndDisabled();
        }
    }
    ImGui::End();
}
