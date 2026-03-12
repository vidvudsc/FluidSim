#ifndef IMGUI_PANEL_H
#define IMGUI_PANEL_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum UiSimMode {
    UI_MODE_WATER_TANK = 0,
    UI_MODE_GAS_TANK,
    UI_MODE_WIND_TUNNEL,
    UI_MODE_COUNT
} UiSimMode;

typedef struct UiCaptureState {
    bool wantsMouse;
    bool wantsKeyboard;
} UiCaptureState;

typedef struct UiPanelState {
    int backend;
    int targetParticleCount;
    int actualParticleCount;
    int mode;
    int obstacleModel;
    int viewMode;
    int colorMode;
    bool paused;
    bool gpuBackendAvailable;
    float windSpeedScale;
    bool acousticsAvailable;
    bool acousticsEnabled;
    float speakerFrequency;
    float speakerAmplitude;
    float effectiveSpeakerAmplitude;
    float speakerWidth;
    float speakerHeight;
    float micRadius;
    float micSignal;
    float acousticSoundSpeed;
    float acousticMachLimit;
    float acousticViscosityScale;
    float acousticDragScale;
    float audioMonitorPitchHz;
    bool audioOutputAvailable;
    bool audioOutputEnabled;
    const float *micWaveform;
    int micWaveformCount;
    float fps;
    double lastSimStepMs;
} UiPanelState;

typedef struct UiPanelActions {
    bool setBackend;
    int backend;
    bool setTargetParticleCount;
    int targetParticleCount;
    bool setMode;
    int mode;
    bool setObstacleModel;
    int obstacleModel;
    bool setViewMode;
    int viewMode;
    bool setColorMode;
    int colorMode;
    bool setPaused;
    bool paused;
    bool setWindSpeedScale;
    float windSpeedScale;
    bool setAcousticsEnabled;
    bool acousticsEnabled;
    bool setSpeakerFrequency;
    float speakerFrequency;
    bool setSpeakerAmplitude;
    float speakerAmplitude;
    bool setAcousticSoundSpeed;
    float acousticSoundSpeed;
    bool setAcousticMachLimit;
    float acousticMachLimit;
    bool setAcousticViscosityScale;
    float acousticViscosityScale;
    bool setAcousticDragScale;
    float acousticDragScale;
    bool setAudioMonitorPitchHz;
    float audioMonitorPitchHz;
    bool setSpeakerWidth;
    float speakerWidth;
    bool setSpeakerHeight;
    float speakerHeight;
    bool setMicRadius;
    float micRadius;
    bool setAudioOutputEnabled;
    bool audioOutputEnabled;
    bool requestReset;
} UiPanelActions;

void UiPanelSetup(void);
void UiPanelShutdown(void);
void UiPanelBegin(float deltaTime);
void UiPanelEnd(void);
UiCaptureState UiPanelGetCaptureState(void);
void UiPanelDraw(const UiPanelState *state, UiPanelActions *actions);

#ifdef __cplusplus
}
#endif

#endif
