#include "raylib.h"
#include "imgui_panel.h"
#include "rlgl.h"

#include <dispatch/dispatch.h>
#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(__APPLE__)
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>
#endif

#define WINDOW_WIDTH 1400
#define WINDOW_HEIGHT 820
#define MAX_PARTICLES 250000
#define DEFAULT_TARGET_PARTICLES 30000
#define CPU_COUNT_PRESET_1 10000
#define CPU_COUNT_PRESET_2 20000
#define CPU_COUNT_PRESET_3 30000
#define CPU_COUNT_PRESET_4 40000
#define CPU_COUNT_PRESET_5 50000
#define GPU_COUNT_PRESET_1 30000
#define GPU_COUNT_PRESET_2 60000
#define GPU_COUNT_PRESET_3 120000
#define GPU_COUNT_PRESET_4 180000
#define GPU_COUNT_PRESET_5 250000
#define MAX_SIM_STEPS_PER_FRAME 8
#define PARALLEL_THRESHOLD 4096
#define DIAGNOSTIC_REFRESH_INTERVAL 4
#define SMOOTH_VIEW_FULL_DRAW_LIMIT 50000
#define GPU_SMOOTH_VIEW_FULL_DRAW_LIMIT MAX_PARTICLES
#define PI_F 3.14159265358979323846f
#define TEMP_MIN 0.35f
#define TEMP_MAX 2.50f
#define MIC_WAVEFORM_SAMPLES 192
#define AUDIO_OUTPUT_SAMPLE_RATE 48000
#define AUDIO_OUTPUT_BUFFER_FRAMES 4096
#define AUDIO_OUTPUT_GAIN 0.55f

typedef enum MaterialPreset {
    MATERIAL_WATER = 0,
    MATERIAL_GAS,
    MATERIAL_COUNT
} MaterialPreset;

typedef enum ViewMode {
    VIEW_PARTICLES = 0,
    VIEW_SMOOTHING_RADIUS,
    VIEW_MODE_COUNT
} ViewMode;

typedef enum ColorMode {
    COLOR_MATERIAL = 0,
    COLOR_TEMPERATURE,
    COLOR_PRESSURE,
    COLOR_SPEED,
    COLOR_DENSITY,
    COLOR_MODE_COUNT
} ColorMode;

typedef enum SimulationBackend {
    SIM_BACKEND_CPU = 0,
    SIM_BACKEND_GPU,
    SIM_BACKEND_COUNT
} SimulationBackend;

typedef enum SimulationScene {
    SCENE_TANK = 0,
    SCENE_WIND_TUNNEL,
    SCENE_COUNT
} SimulationScene;

typedef enum ObstacleModel {
    OBSTACLE_CIRCLE = 0,
    OBSTACLE_AIRFOIL,
    OBSTACLE_CAR,
    OBSTACLE_RECTANGLE,
    OBSTACLE_MODEL_COUNT
} ObstacleModel;

typedef enum DragTarget {
    DRAG_TARGET_NONE = 0,
    DRAG_TARGET_SPEAKER,
    DRAG_TARGET_MIC,
} DragTarget;

typedef struct MaterialParams {
    const char *name;
    float spacing;
    float supportRadius;
    float particleRadius;
    float restDensity;
    float soundSpeed;
    float kinematicViscosity;
    float gravity;
    float buoyancy;
    float temperatureDiffusion;
    float thermalExpansion;
    float ambientTemperature;
    float initialTemperatureGradient;
    float globalDrag;
    float wallBounce;
    float wallFriction;
    float timeStep;
} MaterialParams;

typedef struct Diagnostics {
    float minDensity;
    float maxDensity;
    float minPressure;
    float maxPressure;
    float minSpeed;
    float maxSpeed;
    float minTemperature;
    float maxTemperature;
    float avgDensity;
    float avgPressure;
    float avgSpeed;
    float avgTemperature;
} Diagnostics;

typedef struct ParticleSystem {
    int maxParticles;
    int particleCount;
    int targetParticleCount;
    float worldWidth;
    float worldHeight;
    Rectangle bounds;
    MaterialPreset preset;
    MaterialPreset tankPreset;
    MaterialParams params;
    ViewMode viewMode;
    ColorMode colorMode;
    SimulationBackend activeBackend;
    SimulationScene scene;
    ObstacleModel obstacleModel;
    bool paused;
    bool gpuBackendAvailable;
    float timeScale;
    float accumulator;
    float mass;
    float supportRadiusSquared;
    float pressureStiffness;
    float densityKernel;
    float pressureKernelGrad;
    float viscosityKernelLap;
    float mouseRadius;
    float mouseRadiusSquared;
    float mouseStrength;
    Vector2 mousePosition;
    bool mouseActive;
    Vector2 obstacleCenter;
    float obstacleRadius;
    float obstacleAngleDegrees;
    float obstacleRectWidth;
    float obstacleRectHeight;
    float obstacleShell;
    float obstacleStrength;
    float obstacleDamping;
    float flowTargetSpeed;
    float flowDrive;
    float flowSpeedScale;
    bool acousticsEnabled;
    bool acousticHandlesInitialized;
    float simulationTime;
    Vector2 speakerBaseCenter;
    float speakerWidth;
    float speakerHeight;
    float speakerFrequency;
    float speakerAmplitude;
    float speakerShell;
    float speakerStrength;
    float speakerDamping;
    Vector2 micPosition;
    float micRadius;
    float micSignal;
    float micBaseline;
    float micWaveform[MIC_WAVEFORM_SAMPLES];
    float micWaveformDisplay[MIC_WAVEFORM_SAMPLES];
    int micWaveformHead;
    int micWaveformCount;
    float baseSoundSpeed;
    float baseKinematicViscosity;
    float baseGlobalDrag;
    float acousticSoundSpeed;
    float acousticMachLimit;
    float acousticViscosityScale;
    float acousticDragScale;
    bool audioOutputEnabled;
    bool audioOutputReady;
    AudioStream micAudioStream;
    volatile float audioOutputSignal;
    volatile float audioOutputState;
    volatile float audioMonitorPitchHz;
    volatile float audioMonitorPhase;
    DragTarget dragTarget;
    Vector2 dragOffset;

    int gridWidth;
    int gridHeight;
    int cellCount;
    float cellSize;
    float invCellSize;

    float *x;
    float *y;
    float *vx;
    float *vy;
    float *ax;
    float *ay;
    float *density;
    float *pressure;
    float *temperature;
    float *temperatureRate;
    float *xsphVX;
    float *xsphVY;
    float *sortedX;
    float *sortedY;
    float *sortedVX;
    float *sortedVY;
    float *sortedTemperature;
    float *sortedDensity;
    float *sortedPressure;
    int *particleCells;
    int *sortedCellIndices;
    int *sortedIndices;
    int *cellCounts;
    int *cellStarts;
    int *cellOffsets;
    int *cellNeighborCounts;
    int *cellNeighbors;

    Texture2D particleTexture;
    Texture2D supportTexture;
    Diagnostics stats;
    double lastSimStepMs;
    float lastStepDt;
    int framesUntilDiagnostics;
    bool scalarFieldsDirty;
    const char *backendNotice;
    double backendNoticeUntil;
    void *gpuBackend;
} ParticleSystem;

typedef struct GpuSimParams {
    uint32_t particleCount;
    uint32_t gridWidth;
    uint32_t gridHeight;
    uint32_t cellCount;
    uint32_t preset;
    uint32_t scene;
    uint32_t obstacleModel;
    uint32_t mouseActive;
    float boundsX;
    float boundsY;
    float boundsWidth;
    float boundsHeight;
    float supportRadius;
    float supportRadiusSquared;
    float particleRadius;
    float restDensity;
    float soundSpeed;
    float kinematicViscosity;
    float gravity;
    float buoyancy;
    float temperatureDiffusion;
    float thermalExpansion;
    float ambientTemperature;
    float globalDrag;
    float wallBounce;
    float wallFriction;
    float mass;
    float pressureStiffness;
    float densityKernel;
    float pressureKernelGrad;
    float viscosityKernelLap;
    float mouseRadius;
    float mouseRadiusSquared;
    float mouseStrength;
    float mouseX;
    float mouseY;
    float obstacleCenterX;
    float obstacleCenterY;
    float obstacleRadius;
    float obstacleAngleCos;
    float obstacleAngleSin;
    float obstacleRectHalfWidth;
    float obstacleRectHalfHeight;
    float obstacleShell;
    float obstacleStrength;
    float obstacleDamping;
    float flowTargetSpeed;
    float flowDrive;
    uint32_t acousticsEnabled;
    float speakerCenterX;
    float speakerCenterY;
    float speakerVelocityX;
    float speakerVelocityY;
    float speakerHalfWidth;
    float speakerHalfHeight;
    float speakerShell;
    float speakerStrength;
    float speakerDamping;
    float simulationTime;
    float substepDt;
} GpuSimParams;

#if defined(__APPLE__)
@interface MetalGpuBackend : NSObject

- (instancetype)initWithSystem:(ParticleSystem *)system error:(NSString * _Nullable * _Nullable)error;
- (void)invalidateGridBindings;
- (BOOL)prepareForSystem:(ParticleSystem *)system error:(NSString * _Nullable * _Nullable)error;
- (BOOL)runBuildDensityForceForSystem:(ParticleSystem *)system includeForce:(BOOL)includeForce error:(NSString * _Nullable * _Nullable)error;
- (BOOL)runStepForSystem:(ParticleSystem *)system dt:(float)dt error:(NSString * _Nullable * _Nullable)error;

@end
#endif

static int ClampCellCoord(int value, int maxValue);
static void BuildGrid(ParticleSystem *system);
static float SampleDensityAtSortedSlot(const ParticleSystem *system, int slot, float sampleMass);
static float PressureFromState(const ParticleSystem *system, float density, float temperature);
static void ResetSimulation(ParticleSystem *system, MaterialPreset preset);
static bool InitializeGpuBackend(ParticleSystem *system);
static void ShutdownGpuBackend(ParticleSystem *system);
static void InvalidateGpuBackendState(ParticleSystem *system);
static bool StepSimulationGPU(ParticleSystem *system, float frameDelta);
static bool InitializeAudioOutput(ParticleSystem *system);
static void ShutdownAudioOutput(ParticleSystem *system);

static const MaterialParams MATERIAL_PRESETS[MATERIAL_COUNT] = {
    [MATERIAL_WATER] = {
        .name = "Water-like",
        .spacing = 6.1f,
        .supportRadius = 13.0f,
        .particleRadius = 1.9f,
        .restDensity = 1.0f,
        .soundSpeed = 92.0f,
        .kinematicViscosity = 18.0f,
        .gravity = 950.0f,
        .buoyancy = 70.0f,
        .temperatureDiffusion = 0.18f,
        .thermalExpansion = 0.010f,
        .ambientTemperature = 1.0f,
        .initialTemperatureGradient = 0.02f,
        .globalDrag = 0.16f,
        .wallBounce = 0.03f,
        .wallFriction = 0.82f,
        .timeStep = 1.0f / 240.0f,
    },
    [MATERIAL_GAS] = {
        .name = "Gas-like",
        .spacing = 8.0f,
        .supportRadius = 16.0f,
        .particleRadius = 1.5f,
        .restDensity = 1.0f,
        .soundSpeed = 52.0f,
        .kinematicViscosity = 0.22f,
        .gravity = 0.0f,
        .buoyancy = 0.0f,
        .temperatureDiffusion = 0.65f,
        .thermalExpansion = 0.020f,
        .ambientTemperature = 1.0f,
        .initialTemperatureGradient = 0.0f,
        .globalDrag = 0.14f,
        .wallBounce = 0.35f,
        .wallFriction = 0.995f,
        .timeStep = 1.0f / 240.0f,
    },
};

static ParticleSystem *gAudioOutputSystem = NULL;

static float ClampFloat(float value, float minValue, float maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static int ClampInt(int value, int minValue, int maxValue)
{
    if (value < minValue) {
        return minValue;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static float Saturate(float value)
{
    return ClampFloat(value, 0.0f, 1.0f);
}

static float Pow7(float x)
{
    const float x2 = x * x;
    const float x4 = x2 * x2;
    return x4 * x2 * x;
}

static float HashNoise(int index)
{
    uint32_t x = (uint32_t)index * 747796405u + 2891336453u;
    x = ((x >> ((x >> 28u) + 4u)) ^ x) * 277803737u;
    x = (x >> 22u) ^ x;
    return (float)(x & 0x00FFFFFFu) / (float)0x00FFFFFFu;
}

static uint32_t HashUint32(uint32_t x)
{
    x = x * 747796405u + 2891336453u;
    x = ((x >> ((x >> 28u) + 4u)) ^ x) * 277803737u;
    return (x >> 22u) ^ x;
}

static Color ColorRamp(float value)
{
    const float hue = 220.0f - 220.0f * Saturate(value);
    return ColorFromHSV(hue, 0.85f, 1.0f);
}

static float RangeLerp(float minValue, float maxValue, float value)
{
    const float denom = maxValue - minValue;
    if (fabsf(denom) < 1e-6f) {
        return 0.0f;
    }
    return (value - minValue) / denom;
}

static float MinFloat3(float a, float b, float c)
{
    return fminf(a, fminf(b, c));
}

static bool ShouldParallelize(int count)
{
    return count >= PARALLEL_THRESHOLD;
}

static bool ColorModeNeedsFreshDensity(ColorMode colorMode)
{
    return colorMode == COLOR_PRESSURE || colorMode == COLOR_DENSITY;
}

static UiSimMode CurrentUiMode(const ParticleSystem *system)
{
    if (system->scene == SCENE_WIND_TUNNEL) {
        return UI_MODE_WIND_TUNNEL;
    }
    return (system->preset == MATERIAL_GAS) ? UI_MODE_GAS_TANK : UI_MODE_WATER_TANK;
}

static bool SceneIsWindTunnel(const ParticleSystem *system)
{
    return system->scene == SCENE_WIND_TUNNEL;
}

static bool AcousticsAvailable(const ParticleSystem *system)
{
    return system->scene == SCENE_TANK && system->preset == MATERIAL_GAS;
}

static bool AcousticsActive(const ParticleSystem *system)
{
    return AcousticsAvailable(system) && system->acousticsEnabled;
}

static float EffectiveAcousticSoundSpeed(const ParticleSystem *system)
{
    const float baseSoundSpeed = (system->baseSoundSpeed > 0.0f) ? system->baseSoundSpeed : system->params.soundSpeed;
    if (!AcousticsActive(system)) {
        return baseSoundSpeed;
    }
    return fmaxf(baseSoundSpeed, system->acousticSoundSpeed);
}

static float EffectiveSpeakerAmplitude(const ParticleSystem *system)
{
    if (!AcousticsActive(system)) {
        return system->speakerAmplitude;
    }

    const float frequency = fmaxf(system->speakerFrequency, 1e-4f);
    const float maxSurfaceSpeed = ClampFloat(system->acousticMachLimit, 0.05f, 1.25f) *
        fmaxf(EffectiveAcousticSoundSpeed(system), 1e-4f);
    const float cappedAmplitude = maxSurfaceSpeed / (2.0f * PI_F * frequency);
    return fminf(system->speakerAmplitude, cappedAmplitude);
}

static float RectangleSignedDistance(Vector2 p, Vector2 halfSize)
{
    const Vector2 d = {fabsf(p.x) - halfSize.x, fabsf(p.y) - halfSize.y};
    const float ox = fmaxf(d.x, 0.0f);
    const float oy = fmaxf(d.y, 0.0f);
    return sqrtf(ox * ox + oy * oy) + fminf(fmaxf(d.x, d.y), 0.0f);
}

static Rectangle SpeakerBaseRect(const ParticleSystem *system)
{
    const float halfWidth = system->speakerWidth * 0.5f;
    const float halfHeight = system->speakerHeight * 0.5f;
    return (Rectangle){
        system->speakerBaseCenter.x - halfWidth,
        system->speakerBaseCenter.y - halfHeight,
        system->speakerWidth,
        system->speakerHeight,
    };
}

static void ClampAcousticAnchors(ParticleSystem *system)
{
    const float padding = system->params.particleRadius * 2.0f;
    const float minX = system->bounds.x + padding;
    const float maxX = system->bounds.x + system->bounds.width - padding;
    const float minY = system->bounds.y + padding;
    const float maxY = system->bounds.y + system->bounds.height - padding;
    const float halfWidth = system->speakerWidth * 0.5f;
    const float halfHeight = system->speakerHeight * 0.5f;
    const float amplitude = EffectiveSpeakerAmplitude(system);
    system->speakerBaseCenter.x = ClampFloat(system->speakerBaseCenter.x,
        minX + halfWidth + amplitude,
        maxX - halfWidth - amplitude);
    system->speakerBaseCenter.y = ClampFloat(system->speakerBaseCenter.y,
        minY + halfHeight,
        maxY - halfHeight);
    system->micPosition.x = ClampFloat(system->micPosition.x, minX, maxX);
    system->micPosition.y = ClampFloat(system->micPosition.y, minY, maxY);
}

static void GetSpeakerState(const ParticleSystem *system, Vector2 *center, Vector2 *velocity, Vector2 *halfSize)
{
    const float phase = system->simulationTime * system->speakerFrequency * 2.0f * PI_F;
    const float amplitude = EffectiveSpeakerAmplitude(system);
    const float displacement = amplitude * sinf(phase);
    const float angularFrequency = system->speakerFrequency * 2.0f * PI_F;
    const float speed = amplitude * angularFrequency * cosf(phase);
    if (center != NULL) {
        *center = (Vector2){system->speakerBaseCenter.x + displacement, system->speakerBaseCenter.y};
    }
    if (velocity != NULL) {
        *velocity = (Vector2){speed, 0.0f};
    }
    if (halfSize != NULL) {
        *halfSize = (Vector2){system->speakerWidth * 0.5f, system->speakerHeight * 0.5f};
    }
}

static float SpeakerPeakSurfaceSpeed(const ParticleSystem *system)
{
    const float angularFrequency = system->speakerFrequency * 2.0f * PI_F;
    return fabsf(EffectiveSpeakerAmplitude(system) * angularFrequency);
}

static float SpeakerSignedDistance(const ParticleSystem *system, float x, float y)
{
    Vector2 center;
    Vector2 halfSize;
    GetSpeakerState(system, &center, NULL, &halfSize);
    const Vector2 local = {x - center.x, y - center.y};
    return RectangleSignedDistance(local, halfSize);
}

static Vector2 SpeakerNormal(const ParticleSystem *system, float x, float y)
{
    const float epsilon = fmaxf(system->params.particleRadius * 0.65f, 0.75f);
    const float dx = SpeakerSignedDistance(system, x + epsilon, y) - SpeakerSignedDistance(system, x - epsilon, y);
    const float dy = SpeakerSignedDistance(system, x, y + epsilon) - SpeakerSignedDistance(system, x, y - epsilon);
    const float length = sqrtf(dx * dx + dy * dy);
    if (length > 1e-6f) {
        return (Vector2){dx / length, dy / length};
    }
    return (Vector2){1.0f, 0.0f};
}

static void ResetMicrophoneHistory(ParticleSystem *system)
{
    memset(system->micWaveform, 0, sizeof(system->micWaveform));
    memset(system->micWaveformDisplay, 0, sizeof(system->micWaveformDisplay));
    system->micWaveformHead = 0;
    system->micWaveformCount = 0;
    system->micSignal = 0.0f;
    system->micBaseline = 0.0f;
    system->audioOutputSignal = 0.0f;
    system->audioOutputState = 0.0f;
    system->audioMonitorPhase = 0.0f;
}

static void PushMicrophoneSample(ParticleSystem *system, float signal)
{
    system->micSignal = signal;
    system->audioOutputSignal = signal;
    system->micWaveform[system->micWaveformHead] = signal;
    system->micWaveformHead = (system->micWaveformHead + 1) % MIC_WAVEFORM_SAMPLES;
    if (system->micWaveformCount < MIC_WAVEFORM_SAMPLES) {
        system->micWaveformCount += 1;
    }

    const int start = (system->micWaveformHead - system->micWaveformCount + MIC_WAVEFORM_SAMPLES) % MIC_WAVEFORM_SAMPLES;
    for (int i = 0; i < system->micWaveformCount; ++i) {
        const int index = (start + i) % MIC_WAVEFORM_SAMPLES;
        system->micWaveformDisplay[i] = system->micWaveform[index];
    }
}

static void SampleMicrophone(ParticleSystem *system)
{
    if (!AcousticsActive(system)) {
        return;
    }

    float weightedPressure = 0.0f;
    float totalWeight = 0.0f;
    const float radius2 = system->micRadius * system->micRadius;
    for (int i = 0; i < system->particleCount; ++i) {
        const float dx = system->x[i] - system->micPosition.x;
        const float dy = system->y[i] - system->micPosition.y;
        const float r2 = dx * dx + dy * dy;
        if (r2 < radius2) {
            const float weight = 1.0f - sqrtf(r2 / fmaxf(radius2, 1e-6f));
            weightedPressure += system->pressure[i] * weight;
            totalWeight += weight;
        }
    }

    const float sample = (totalWeight > 1e-5f) ? (weightedPressure / totalWeight) : 0.0f;
    if (system->micWaveformCount == 0) {
        system->micBaseline = sample;
    } else {
        system->micBaseline = 0.985f * system->micBaseline + 0.015f * sample;
    }

    const float scale = fmaxf(system->params.soundSpeed * system->params.soundSpeed * system->params.restDensity * 0.65f, 1e-4f);
    PushMicrophoneSample(system, ClampFloat((sample - system->micBaseline) / scale, -1.0f, 1.0f));
}

static void MicAudioStreamCallback(void *bufferData, unsigned int frames)
{
    float *samples = (float *)bufferData;
    if (samples == NULL) {
        return;
    }

    ParticleSystem *system = gAudioOutputSystem;
    if (system == NULL || !system->audioOutputReady || !system->audioOutputEnabled || !AcousticsActive(system)) {
        memset(samples, 0, (size_t)frames * sizeof(float));
        return;
    }

    float state = system->audioOutputState;
    float phase = system->audioMonitorPhase;
    const float carrierHz = system->audioMonitorPitchHz;
    for (unsigned int i = 0; i < frames; ++i) {
        const float target = system->audioOutputSignal;
        state += (target - state) * 0.035f;
        float output = state;
        if (carrierHz > 1.0f) {
            phase += (2.0f * PI_F * carrierHz) / (float)AUDIO_OUTPUT_SAMPLE_RATE;
            if (phase > 2.0f * PI_F) {
                phase = fmodf(phase, 2.0f * PI_F);
            }
            output = state * sinf(phase);
        }
        samples[i] = ClampFloat(output * AUDIO_OUTPUT_GAIN, -1.0f, 1.0f);
    }
    system->audioOutputState = state;
    system->audioMonitorPhase = phase;
}

static bool InitializeAudioOutput(ParticleSystem *system)
{
    if (system->audioOutputReady) {
        if (!IsAudioStreamPlaying(system->micAudioStream)) {
            PlayAudioStream(system->micAudioStream);
        }
        gAudioOutputSystem = system;
        return true;
    }

    if (!IsAudioDeviceReady()) {
        SetAudioStreamBufferSizeDefault(AUDIO_OUTPUT_BUFFER_FRAMES);
        InitAudioDevice();
    }
    if (!IsAudioDeviceReady()) {
        return false;
    }

    system->micAudioStream = LoadAudioStream(AUDIO_OUTPUT_SAMPLE_RATE, 32, 1);
    if (!IsAudioStreamValid(system->micAudioStream)) {
        memset(&system->micAudioStream, 0, sizeof(system->micAudioStream));
        return false;
    }

    SetAudioStreamVolume(system->micAudioStream, 1.0f);
    SetAudioStreamCallback(system->micAudioStream, MicAudioStreamCallback);
    PlayAudioStream(system->micAudioStream);
    system->audioOutputReady = true;
    gAudioOutputSystem = system;
    return true;
}

static void ShutdownAudioOutput(ParticleSystem *system)
{
    if (gAudioOutputSystem == system) {
        gAudioOutputSystem = NULL;
    }

    if (system->audioOutputReady) {
        StopAudioStream(system->micAudioStream);
        UnloadAudioStream(system->micAudioStream);
        memset(&system->micAudioStream, 0, sizeof(system->micAudioStream));
        system->audioOutputReady = false;
    }

    if (IsAudioDeviceReady()) {
        CloseAudioDevice();
    }
}

static int BackendMinTargetParticleCount(SimulationBackend backend)
{
    return (backend == SIM_BACKEND_GPU) ? GPU_COUNT_PRESET_1 : CPU_COUNT_PRESET_1;
}

static int BackendMaxTargetParticleCount(const ParticleSystem *system, SimulationBackend backend)
{
    const int backendLimit = (backend == SIM_BACKEND_GPU) ? GPU_COUNT_PRESET_5 : CPU_COUNT_PRESET_5;
    return ClampInt(backendLimit, 1, system->maxParticles);
}

static int CountPresetForBackend(SimulationBackend backend, int presetIndex)
{
    switch (presetIndex) {
        case 0:
            return (backend == SIM_BACKEND_GPU) ? GPU_COUNT_PRESET_1 : CPU_COUNT_PRESET_1;
        case 1:
            return (backend == SIM_BACKEND_GPU) ? GPU_COUNT_PRESET_2 : CPU_COUNT_PRESET_2;
        case 2:
            return (backend == SIM_BACKEND_GPU) ? GPU_COUNT_PRESET_3 : CPU_COUNT_PRESET_3;
        case 3:
            return (backend == SIM_BACKEND_GPU) ? GPU_COUNT_PRESET_4 : CPU_COUNT_PRESET_4;
        case 4:
        default:
            return (backend == SIM_BACKEND_GPU) ? GPU_COUNT_PRESET_5 : CPU_COUNT_PRESET_5;
    }
}

static int EffectiveTargetParticleCount(const ParticleSystem *system)
{
    return ClampInt(system->targetParticleCount,
        BackendMinTargetParticleCount(system->activeBackend),
        BackendMaxTargetParticleCount(system, system->activeBackend));
}

static float MouseInteractionScale(const ParticleSystem *system)
{
    const float countRatio = (float)EffectiveTargetParticleCount(system) / (float)DEFAULT_TARGET_PARTICLES;
    return ClampFloat(sqrtf(fmaxf(countRatio, 1.0f)), 1.0f, 3.2f);
}

static int SmoothViewDrawLimit(const ParticleSystem *system)
{
    return (system->activeBackend == SIM_BACKEND_GPU)
        ? GPU_SMOOTH_VIEW_FULL_DRAW_LIMIT
        : SMOOTH_VIEW_FULL_DRAW_LIMIT;
}

static float SmoothViewCoverageScale(const ParticleSystem *system)
{
    if (system->viewMode != VIEW_SMOOTHING_RADIUS || system->particleCount <= SMOOTH_VIEW_FULL_DRAW_LIMIT) {
        return 1.0f;
    }

    const float ratio = (float)system->particleCount / (float)SMOOTH_VIEW_FULL_DRAW_LIMIT;
    return ClampFloat(1.0f + 0.045f * log2f(fmaxf(ratio, 1.0f)), 1.0f, 1.18f);
}

static unsigned char SmoothViewAlpha(const ParticleSystem *system, int drawStride)
{
    const float countScale = sqrtf(fmaxf((float)system->particleCount / (float)SMOOTH_VIEW_FULL_DRAW_LIMIT, 1.0f));
    const float alpha = (30.0f / countScale) * sqrtf((float)drawStride);
    return (unsigned char)ClampInt((int)lroundf(alpha), 10, 128);
}

static bool ShouldDrawSmoothingSample(int index, int drawStride)
{
    if (drawStride <= 1) {
        return true;
    }

    return (HashUint32((uint32_t)index + 0x9E3779B9u) % (uint32_t)drawStride) == 0u;
}

static const Vector2 AIRFOIL_OUTLINE_POINTS[] = {
    {1.9919f, 0.0067f},
    {1.9415f, 0.0171f},
    {1.8914f, 0.0272f},
    {1.7439f, 0.0560f},
    {1.6478f, 0.0738f},
    {1.4138f, 0.1149f},
    {1.2779f, 0.1372f},
    {0.9748f, 0.1828f},
    {0.8103f, 0.2051f},
    {0.4608f, 0.2467f},
    {0.2788f, 0.2653f},
    {-0.0905f, 0.2950f},
    {-0.2748f, 0.3054f},
    {-0.6368f, 0.3150f},
    {-0.8115f, 0.3130f},
    {-1.1373f, 0.2944f},
    {-1.2856f, 0.2785f},
    {-1.5474f, 0.2348f},
    {-1.6584f, 0.2079f},
    {-1.8354f, 0.1478f},
    {-1.9000f, 0.1157f},
    {-1.9791f, 0.0496f},
    {-1.9899f, -0.0149f},
    {-1.9699f, -0.0445f},
    {-1.8805f, -0.0946f},
    {-1.8122f, -0.1148f},
    {-1.6312f, -0.1454f},
    {-1.5198f, -0.1557f},
    {-1.2613f, -0.1671f},
    {-1.1163f, -0.1686f},
    {-0.7994f, -0.1646f},
    {-0.6299f, -0.1598f},
    {-0.2757f, -0.1467f},
    {-0.0941f, -0.1383f},
    {0.2717f, -0.1185f},
    {0.4525f, -0.1078f},
    {0.8006f, -0.0861f},
    {0.9650f, -0.0755f},
    {1.2690f, -0.0554f},
    {1.4058f, -0.0463f},
    {1.6419f, -0.0302f},
    {1.7392f, -0.0235f},
    {1.8889f, -0.0130f},
    {1.9399f, -0.0093f},
    {1.9916f, 0.0019f},
};

static const int AIRFOIL_OUTLINE_POINT_COUNT =
    (int)(sizeof(AIRFOIL_OUTLINE_POINTS) / sizeof(AIRFOIL_OUTLINE_POINTS[0]));

static const Vector2 CAR_OUTLINE_POINTS[] = {
    {-2.1600f, 0.1150f},
    {-2.0600f, -0.0550f},
    {-1.9800f, -0.1250f},
    {-1.8750f, -0.1750f},
    {-1.5950f, -0.2300f},
    {-1.2500f, -0.2850f},
    {-1.0700f, -0.3350f},
    {-0.8850f, -0.3950f},
    {-0.4900f, -0.5150f},
    {-0.0500f, -0.5600f},
    {0.1700f, -0.5600f},
    {0.3850f, -0.5450f},
    {0.7900f, -0.4800f},
    {1.1300f, -0.4000f},
    {1.2700f, -0.3600f},
    {1.4000f, -0.3250f},
    {1.6400f, -0.2850f},
    {1.8800f, -0.3250f},
    {2.0000f, -0.3750f},
    {2.1050f, -0.2550f},
    {2.1600f, -0.1100f},
    {2.1150f, 0.1200f},
    {2.0250f, 0.1600f},
    {1.8800f, 0.1850f},
    {1.4550f, 0.2000f},
    {0.9500f, 0.2000f},
    {0.6900f, 0.2000f},
    {0.4300f, 0.2000f},
    {-0.0900f, 0.2000f},
    {-0.6150f, 0.2000f},
    {-0.8850f, 0.2000f},
    {-1.1400f, 0.2000f},
    {-1.6000f, 0.1950f},
    {-1.9700f, 0.1750f},
    {-2.1100f, 0.1650f},
};

static const int CAR_OUTLINE_POINT_COUNT =
    (int)(sizeof(CAR_OUTLINE_POINTS) / sizeof(CAR_OUTLINE_POINTS[0]));

static float DistanceSquaredToSegment(Vector2 p, Vector2 a, Vector2 b)
{
    const Vector2 ab = {b.x - a.x, b.y - a.y};
    const Vector2 ap = {p.x - a.x, p.y - a.y};
    const float abLengthSquared = ab.x * ab.x + ab.y * ab.y;
    const float t = (abLengthSquared > 1e-8f)
        ? ClampFloat((ap.x * ab.x + ap.y * ab.y) / abLengthSquared, 0.0f, 1.0f)
        : 0.0f;
    const Vector2 closest = {a.x + ab.x * t, a.y + ab.y * t};
    const float dx = p.x - closest.x;
    const float dy = p.y - closest.y;
    return dx * dx + dy * dy;
}

static float PolygonSignedDistanceScaled(const Vector2 *points, int pointCount, Vector2 p, float scale)
{
    float minDistanceSquared = FLT_MAX;
    bool inside = false;

    for (int i = 0, j = pointCount - 1; i < pointCount; j = i++) {
        const Vector2 a = {points[i].x * scale, points[i].y * scale};
        const Vector2 b = {points[j].x * scale, points[j].y * scale};
        minDistanceSquared = fminf(minDistanceSquared, DistanceSquaredToSegment(p, a, b));

        const float yDelta = b.y - a.y;
        const bool intersects = ((a.y > p.y) != (b.y > p.y)) &&
            (p.x < (b.x - a.x) * (p.y - a.y) / (fabsf(yDelta) > 1e-6f ? yDelta : 1e-6f) + a.x);
        if (intersects) {
            inside = !inside;
        }
    }

    const float distance = sqrtf(fmaxf(minDistanceSquared, 0.0f));
    return inside ? -distance : distance;
}

static Vector2 RotateVector(Vector2 p, float radians)
{
    const float c = cosf(radians);
    const float s = sinf(radians);
    return (Vector2){
        c * p.x - s * p.y,
        s * p.x + c * p.y,
    };
}

static float ObstacleAngleRadians(const ParticleSystem *system)
{
    return system->obstacleAngleDegrees * (PI_F / 180.0f);
}

static Vector2 ObstacleWorldToLocal(const ParticleSystem *system, float x, float y)
{
    const Vector2 p = {
        x - system->obstacleCenter.x,
        y - system->obstacleCenter.y,
    };
    return RotateVector(p, -ObstacleAngleRadians(system));
}

static Vector2 ObstacleLocalToWorld(const ParticleSystem *system, Vector2 local)
{
    const Vector2 rotated = RotateVector(local, ObstacleAngleRadians(system));
    return (Vector2){
        system->obstacleCenter.x + rotated.x,
        system->obstacleCenter.y + rotated.y,
    };
}

static float ObstacleSignedDistanceLocal(const ParticleSystem *system, Vector2 p)
{
    switch (system->obstacleModel) {
        case OBSTACLE_AIRFOIL:
            return PolygonSignedDistanceScaled(AIRFOIL_OUTLINE_POINTS, AIRFOIL_OUTLINE_POINT_COUNT, (Vector2){p.x, -p.y},
                system->obstacleRadius);
        case OBSTACLE_CAR: {
            const float r = system->obstacleRadius;
            return PolygonSignedDistanceScaled(CAR_OUTLINE_POINTS, CAR_OUTLINE_POINT_COUNT, p, r);
        }
        case OBSTACLE_RECTANGLE:
            return RectangleSignedDistance(p, (Vector2){
                system->obstacleRectWidth * 0.5f,
                system->obstacleRectHeight * 0.5f,
            });
        case OBSTACLE_CIRCLE:
        default:
            return sqrtf(p.x * p.x + p.y * p.y) - system->obstacleRadius;
    }
}

static float ObstacleSignedDistance(const ParticleSystem *system, float x, float y)
{
    return ObstacleSignedDistanceLocal(system, ObstacleWorldToLocal(system, x, y));
}

static Vector2 ObstacleNormal(const ParticleSystem *system, float x, float y)
{
    const float epsilon = fmaxf(system->params.particleRadius * 0.65f, 0.75f);
    const float dx = ObstacleSignedDistance(system, x + epsilon, y) - ObstacleSignedDistance(system, x - epsilon, y);
    const float dy = ObstacleSignedDistance(system, x, y + epsilon) - ObstacleSignedDistance(system, x, y - epsilon);
    const float length = sqrtf(dx * dx + dy * dy);
    if (length > 1e-6f) {
        return (Vector2){dx / length, dy / length};
    }

    const Vector2 fallback = {x - system->obstacleCenter.x, y - system->obstacleCenter.y};
    const float fallbackLength = sqrtf(fallback.x * fallback.x + fallback.y * fallback.y);
    if (fallbackLength > 1e-6f) {
        return (Vector2){fallback.x / fallbackLength, fallback.y / fallbackLength};
    }
    return (Vector2){1.0f, 0.0f};
}

static void ConfigureSceneParameters(ParticleSystem *system)
{
    system->obstacleCenter = (Vector2){0.0f, 0.0f};
    system->obstacleRadius = 0.0f;
    system->obstacleShell = 0.0f;
    system->obstacleStrength = 0.0f;
    system->obstacleDamping = 0.0f;
    system->flowTargetSpeed = 0.0f;
    system->flowDrive = 0.0f;

    if (!SceneIsWindTunnel(system)) {
        return;
    }

    system->obstacleCenter = (Vector2){
        system->bounds.x + system->bounds.width * 0.43f,
        system->bounds.y + system->bounds.height * 0.50f,
    };
    switch (system->obstacleModel) {
        case OBSTACLE_AIRFOIL:
            system->obstacleRadius = fminf(system->bounds.width, system->bounds.height) * 0.060f;
            system->obstacleShell = system->params.supportRadius * 1.08f;
            system->obstacleStrength = system->params.soundSpeed * system->params.soundSpeed * 0.42f;
            system->obstacleDamping = system->params.soundSpeed * 0.55f / fmaxf(system->params.supportRadius, 1.0f);
            break;
        case OBSTACLE_CAR:
            system->obstacleRadius = fminf(system->bounds.width, system->bounds.height) * 0.062f;
            system->obstacleShell = system->params.supportRadius * 1.42f;
            system->obstacleStrength = system->params.soundSpeed * system->params.soundSpeed * 0.66f;
            system->obstacleDamping = system->params.soundSpeed * 0.78f / fmaxf(system->params.supportRadius, 1.0f);
            break;
        case OBSTACLE_RECTANGLE:
            system->obstacleRadius = 0.5f * fmaxf(system->obstacleRectWidth, system->obstacleRectHeight);
            system->obstacleShell = system->params.supportRadius * 1.12f;
            system->obstacleStrength = system->params.soundSpeed * system->params.soundSpeed * 0.44f;
            system->obstacleDamping = system->params.soundSpeed * 0.58f / fmaxf(system->params.supportRadius, 1.0f);
            break;
        case OBSTACLE_CIRCLE:
        default:
            system->obstacleRadius = fminf(system->bounds.width, system->bounds.height) * 0.085f;
            system->obstacleShell = system->params.supportRadius * 1.55f;
            system->obstacleStrength = system->params.soundSpeed * system->params.soundSpeed * 0.55f;
            system->obstacleDamping = system->params.soundSpeed * 0.70f / fmaxf(system->params.supportRadius, 1.0f);
            break;
    }
    const float flowScale = fmaxf(system->flowSpeedScale, 0.35f);
    system->flowTargetSpeed = system->params.soundSpeed * 1.10f * flowScale;
    system->flowDrive = system->params.soundSpeed * system->params.soundSpeed * 0.18f * flowScale /
        fmaxf(system->params.supportRadius, 1.0f);
}

static float WindTunnelProfile(const ParticleSystem *system, float y)
{
    const float top = system->bounds.y + system->params.particleRadius;
    const float bottom = system->bounds.y + system->bounds.height - system->params.particleRadius;
    const float t = ClampFloat(RangeLerp(top, bottom, y), 0.0f, 1.0f);
    const float centered = 2.0f * t - 1.0f;
    return fmaxf(0.12f, 1.0f - centered * centered);
}

static void RespawnWindTunnelParticle(ParticleSystem *system, int index)
{
    const float radius = system->params.particleRadius;
    const float left = system->bounds.x + radius;
    const float top = system->bounds.y + radius;
    const float bottom = system->bounds.y + system->bounds.height - radius;
    const float inletDepth = fmaxf(system->params.supportRadius * 1.6f, radius * 4.0f);
    const float tunnelHeight = fmaxf(bottom - top, radius * 2.0f);
    const int seedBase = (int)floorf(system->simulationTime * 1536.0f) + index * 97;
    const float baseY = top + HashNoise(seedBase + 17) * tunnelHeight;
    const float yJitter = (HashNoise(seedBase + 41) - 0.5f) * system->params.spacing * 0.22f;
    const float particleY = ClampFloat(baseY + yJitter, top, bottom);
    const float profile = WindTunnelProfile(system, particleY);

    system->x[index] = left + HashNoise(seedBase + 73) * inletDepth;
    system->y[index] = particleY;
    system->vx[index] = system->flowTargetSpeed * profile +
        (HashNoise(seedBase + 109) - 0.5f) * system->params.soundSpeed * 0.020f;
    system->vy[index] = (HashNoise(seedBase + 149) - 0.5f) * system->params.soundSpeed * 0.014f;
    system->ax[index] = system->flowDrive * profile;
    system->ay[index] = 0.0f;
    system->temperature[index] = ClampFloat(
        system->params.ambientTemperature + (HashNoise(seedBase + 181) - 0.5f) * 0.02f,
        TEMP_MIN,
        TEMP_MAX);
    system->temperatureRate[index] = 0.0f;
    system->xsphVX[index] = 0.0f;
    system->xsphVY[index] = 0.0f;
    system->density[index] = system->params.restDensity;
    system->pressure[index] = PressureFromState(system, system->density[index], system->temperature[index]);
}

static void AddSceneForceContribution(const ParticleSystem *system, float x, float y, float vx, float vy,
    float *ax, float *ay)
{
    if (!SceneIsWindTunnel(system)) {
        return;
    }

    const float profile = WindTunnelProfile(system, y);
    const float xNorm = ClampFloat(RangeLerp(system->bounds.x, system->bounds.x + system->bounds.width, x), 0.0f, 1.0f);
    const float driveBias = 1.18f - 0.22f * xNorm;
    *ax += system->flowDrive * profile * driveBias *
        (1.0f - vx / fmaxf(system->flowTargetSpeed, 1e-4f));
    *ay -= system->flowDrive * 0.030f * vy;

    const float signedDistance = ObstacleSignedDistance(system, x, y);
    if (signedDistance < system->obstacleShell) {
        const Vector2 normal = ObstacleNormal(system, x, y);
        const float falloff = ClampFloat(1.0f - signedDistance / system->obstacleShell, 0.0f, 1.8f);
        const float repulse = system->obstacleStrength * falloff * falloff;
        const float vn = vx * normal.x + vy * normal.y;
        const float damp = -system->obstacleDamping * fminf(vn, 0.0f);
        *ax += normal.x * (repulse + damp);
        *ay += normal.y * (repulse + damp);
    }
}

static void AddSpeakerForceContribution(const ParticleSystem *system, float x, float y, float vx, float vy,
    float *ax, float *ay)
{
    if (!AcousticsActive(system)) {
        return;
    }

    const float signedDistance = SpeakerSignedDistance(system, x, y);
    if (signedDistance >= system->speakerShell) {
        return;
    }

    Vector2 speakerVelocity;
    GetSpeakerState(system, NULL, &speakerVelocity, NULL);
    const Vector2 normal = SpeakerNormal(system, x, y);
    const float falloff = ClampFloat(1.0f - signedDistance / system->speakerShell, 0.0f, 1.8f);
    const float repulse = system->speakerStrength * falloff * falloff;
    const float relativeVn = (vx - speakerVelocity.x) * normal.x + (vy - speakerVelocity.y) * normal.y;
    const float damp = -system->speakerDamping * fminf(relativeVn, 0.0f);
    *ax += normal.x * (repulse + damp);
    *ay += normal.y * (repulse + damp);
}

static void ResolveObstacle(ParticleSystem *system, int index)
{
    if (!SceneIsWindTunnel(system)) {
        return;
    }

    const float baseSurfaceOffset = system->params.particleRadius *
        ((system->obstacleModel == OBSTACLE_CAR) ? 1.20f : 0.70f);
    Vector2 normal = {1.0f, 0.0f};
    bool resolved = false;

    for (int iteration = 0; iteration < ((system->obstacleModel == OBSTACLE_CAR) ? 4 : 2); ++iteration) {
        const float signedDistance = ObstacleSignedDistance(system, system->x[index], system->y[index]);
        if (signedDistance >= baseSurfaceOffset) {
            break;
        }

        normal = ObstacleNormal(system, system->x[index], system->y[index]);
        const float pushOut = (baseSurfaceOffset - signedDistance) +
            system->params.particleRadius * ((system->obstacleModel == OBSTACLE_CAR) ? 0.18f : 0.08f);
        system->x[index] += normal.x * pushOut;
        system->y[index] += normal.y * pushOut;
        resolved = true;
    }

    if (!resolved) {
        return;
    }

    const float vn = system->vx[index] * normal.x + system->vy[index] * normal.y;
    if (vn < 0.0f) {
        system->vx[index] -= vn * normal.x;
        system->vy[index] -= vn * normal.y;
    }
    system->vx[index] *= 0.985f;
    system->vy[index] *= 0.985f;
}

static void ResolveSpeaker(ParticleSystem *system, int index)
{
    if (!AcousticsActive(system)) {
        return;
    }

    const float surfaceOffset = system->params.particleRadius * 0.70f;
    const float signedDistance = SpeakerSignedDistance(system, system->x[index], system->y[index]);
    if (signedDistance >= surfaceOffset) {
        return;
    }

    Vector2 speakerVelocity;
    GetSpeakerState(system, NULL, &speakerVelocity, NULL);
    const Vector2 normal = SpeakerNormal(system, system->x[index], system->y[index]);
    const float pushOut = surfaceOffset - signedDistance;
    system->x[index] += normal.x * pushOut;
    system->y[index] += normal.y * pushOut;

    const float relativeVn =
        (system->vx[index] - speakerVelocity.x) * normal.x +
        (system->vy[index] - speakerVelocity.y) * normal.y;
    if (relativeVn < 0.0f) {
        system->vx[index] -= relativeVn * normal.x;
        system->vy[index] -= relativeVn * normal.y;
    }
    system->vx[index] = 0.992f * system->vx[index] + 0.008f * speakerVelocity.x;
    system->vy[index] = 0.992f * system->vy[index] + 0.008f * speakerVelocity.y;
}

static void SetBackendNotice(ParticleSystem *system, const char *message)
{
    system->backendNotice = message;
    system->backendNoticeUntil = GetTime() + 2.0;
}

static void SetSimulationScene(ParticleSystem *system, SimulationScene scene)
{
    if (system->scene == scene) {
        return;
    }

    system->scene = scene;
    if (scene == SCENE_WIND_TUNNEL) {
        SetBackendNotice(system, "Wind tunnel scene enabled.");
        ResetSimulation(system, MATERIAL_GAS);
    } else {
        SetBackendNotice(system, "Tank scene enabled.");
        ResetSimulation(system, system->tankPreset);
    }
}

static void ToggleSimulationScene(ParticleSystem *system)
{
    const SimulationScene nextScene = (system->scene == SCENE_TANK)
        ? SCENE_WIND_TUNNEL
        : SCENE_TANK;
    SetSimulationScene(system, nextScene);
}

static void SetObstacleModel(ParticleSystem *system, ObstacleModel obstacleModel)
{
    if (system->obstacleModel == obstacleModel) {
        return;
    }

    system->obstacleModel = obstacleModel;
    switch (system->obstacleModel) {
        case OBSTACLE_AIRFOIL:
            SetBackendNotice(system, "Obstacle: Airfoil");
            break;
        case OBSTACLE_CAR:
            SetBackendNotice(system, "Obstacle: Car");
            break;
        case OBSTACLE_RECTANGLE:
            SetBackendNotice(system, "Obstacle: Rectangle");
            break;
        case OBSTACLE_CIRCLE:
        default:
            SetBackendNotice(system, "Obstacle: Circle");
            break;
    }
    if (SceneIsWindTunnel(system)) {
        ResetSimulation(system, MATERIAL_GAS);
    }
}

static void CycleObstacleModel(ParticleSystem *system)
{
    SetObstacleModel(system, (ObstacleModel)((system->obstacleModel + 1) % OBSTACLE_MODEL_COUNT));
}

static void SetSimulationBackend(ParticleSystem *system, SimulationBackend backend)
{
    if (backend == SIM_BACKEND_GPU && !system->gpuBackendAvailable) {
        SetBackendNotice(system, "GPU solver is not available in this build. Still using CPU.");
        system->activeBackend = SIM_BACKEND_CPU;
        return;
    }

    if (system->activeBackend != backend) {
        system->activeBackend = backend;
        const int clampedTarget = ClampInt(system->targetParticleCount,
            BackendMinTargetParticleCount(backend),
            BackendMaxTargetParticleCount(system, backend));
        const bool targetChanged = (clampedTarget != system->targetParticleCount);
        system->targetParticleCount = clampedTarget;
        SetBackendNotice(system, (backend == SIM_BACKEND_GPU)
            ? (targetChanged
                ? "Switched to GPU solver. Count range is now 30k to 250k."
                : "Switched to GPU solver.")
            : (targetChanged
                ? "Switched to CPU solver. Count range is now 10k to 50k."
                : "Switched to CPU solver."));
        if (targetChanged) {
            ResetSimulation(system, system->preset);
        }
    }
}

static void ToggleSimulationBackend(ParticleSystem *system)
{
    const SimulationBackend nextBackend = (system->activeBackend == SIM_BACKEND_CPU)
        ? SIM_BACKEND_GPU
        : SIM_BACKEND_CPU;
    if (nextBackend == SIM_BACKEND_GPU) {
        (void)InitializeGpuBackend(system);
    }
    SetSimulationBackend(system, nextBackend);
}

static void SetTargetParticleCount(ParticleSystem *system, int targetParticleCount)
{
    const int clampedTarget = ClampInt(targetParticleCount,
        BackendMinTargetParticleCount(system->activeBackend),
        BackendMaxTargetParticleCount(system, system->activeBackend));
    if (system->targetParticleCount == clampedTarget) {
        return;
    }

    system->targetParticleCount = clampedTarget;
    ResetSimulation(system, system->preset);
}

static void SetWindSpeedScale(ParticleSystem *system, float flowSpeedScale)
{
    const float clamped = ClampFloat(flowSpeedScale, 0.35f, 2.50f);
    if (fabsf(system->flowSpeedScale - clamped) < 1e-4f) {
        return;
    }

    system->flowSpeedScale = clamped;
    ConfigureSceneParameters(system);
    InvalidateGpuBackendState(system);
    SetBackendNotice(system, "Wind tunnel speed updated.");
}

static void SetObstacleAngleDegrees(ParticleSystem *system, float obstacleAngleDegrees)
{
    const float clamped = ClampFloat(obstacleAngleDegrees, -180.0f, 180.0f);
    if (fabsf(system->obstacleAngleDegrees - clamped) < 1e-4f) {
        return;
    }

    system->obstacleAngleDegrees = clamped;
    ConfigureSceneParameters(system);
    InvalidateGpuBackendState(system);
}

static void SetObstacleRectangleWidth(ParticleSystem *system, float obstacleRectWidth)
{
    const float clamped = ClampFloat(obstacleRectWidth, 40.0f, 360.0f);
    if (fabsf(system->obstacleRectWidth - clamped) < 1e-4f) {
        return;
    }

    system->obstacleRectWidth = clamped;
    ConfigureSceneParameters(system);
    InvalidateGpuBackendState(system);
}

static void SetObstacleRectangleHeight(ParticleSystem *system, float obstacleRectHeight)
{
    const float clamped = ClampFloat(obstacleRectHeight, 24.0f, 220.0f);
    if (fabsf(system->obstacleRectHeight - clamped) < 1e-4f) {
        return;
    }

    system->obstacleRectHeight = clamped;
    ConfigureSceneParameters(system);
    InvalidateGpuBackendState(system);
}

#if defined(__APPLE__)
static NSString *LoadMetalSource(void)
{
    NSString *workingPath = @"src/gpu_backend.metal";
    NSError *error = nil;
    NSString *source = [NSString stringWithContentsOfFile:workingPath
                                                 encoding:NSUTF8StringEncoding
                                                    error:&error];
    if (source != nil) {
        return source;
    }

    NSString *executablePath = [[NSBundle mainBundle] executablePath];
    if (executablePath != nil) {
        NSString *bundleRelativePath = [[[[executablePath stringByDeletingLastPathComponent]
            stringByAppendingPathComponent:@"../src"] stringByStandardizingPath]
            stringByAppendingPathComponent:@"gpu_backend.metal"];
        source = [NSString stringWithContentsOfFile:bundleRelativePath
                                           encoding:NSUTF8StringEncoding
                                              error:&error];
        if (source != nil) {
            return source;
        }
    }

    return nil;
}

static GpuSimParams MakeGpuSimParams(const ParticleSystem *system, float dt)
{
    Vector2 speakerCenter = {0.0f, 0.0f};
    Vector2 speakerVelocity = {0.0f, 0.0f};
    Vector2 speakerHalfSize = {0.0f, 0.0f};
    const float obstacleAngle = ObstacleAngleRadians(system);
    const float obstacleCos = cosf(obstacleAngle);
    const float obstacleSin = sinf(obstacleAngle);
    if (AcousticsActive(system)) {
        GetSpeakerState(system, &speakerCenter, &speakerVelocity, &speakerHalfSize);
    }

    return (GpuSimParams){
        .particleCount = (uint32_t)system->particleCount,
        .gridWidth = (uint32_t)system->gridWidth,
        .gridHeight = (uint32_t)system->gridHeight,
        .cellCount = (uint32_t)system->cellCount,
        .preset = (uint32_t)system->preset,
        .scene = (uint32_t)system->scene,
        .obstacleModel = (uint32_t)system->obstacleModel,
        .mouseActive = system->mouseActive ? 1u : 0u,
        .boundsX = system->bounds.x,
        .boundsY = system->bounds.y,
        .boundsWidth = system->bounds.width,
        .boundsHeight = system->bounds.height,
        .supportRadius = system->params.supportRadius,
        .supportRadiusSquared = system->supportRadiusSquared,
        .particleRadius = system->params.particleRadius,
        .restDensity = system->params.restDensity,
        .soundSpeed = system->params.soundSpeed,
        .kinematicViscosity = system->params.kinematicViscosity,
        .gravity = system->params.gravity,
        .buoyancy = system->params.buoyancy,
        .temperatureDiffusion = system->params.temperatureDiffusion,
        .thermalExpansion = system->params.thermalExpansion,
        .ambientTemperature = system->params.ambientTemperature,
        .globalDrag = system->params.globalDrag,
        .wallBounce = system->params.wallBounce,
        .wallFriction = system->params.wallFriction,
        .mass = system->mass,
        .pressureStiffness = system->pressureStiffness,
        .densityKernel = system->densityKernel,
        .pressureKernelGrad = system->pressureKernelGrad,
        .viscosityKernelLap = system->viscosityKernelLap,
        .mouseRadius = system->mouseRadius,
        .mouseRadiusSquared = system->mouseRadiusSquared,
        .mouseStrength = system->mouseStrength,
        .mouseX = system->mousePosition.x,
        .mouseY = system->mousePosition.y,
        .obstacleCenterX = system->obstacleCenter.x,
        .obstacleCenterY = system->obstacleCenter.y,
        .obstacleRadius = system->obstacleRadius,
        .obstacleAngleCos = obstacleCos,
        .obstacleAngleSin = obstacleSin,
        .obstacleRectHalfWidth = system->obstacleRectWidth * 0.5f,
        .obstacleRectHalfHeight = system->obstacleRectHeight * 0.5f,
        .obstacleShell = system->obstacleShell,
        .obstacleStrength = system->obstacleStrength,
        .obstacleDamping = system->obstacleDamping,
        .flowTargetSpeed = system->flowTargetSpeed,
        .flowDrive = system->flowDrive,
        .acousticsEnabled = AcousticsActive(system) ? 1u : 0u,
        .speakerCenterX = speakerCenter.x,
        .speakerCenterY = speakerCenter.y,
        .speakerVelocityX = speakerVelocity.x,
        .speakerVelocityY = speakerVelocity.y,
        .speakerHalfWidth = speakerHalfSize.x,
        .speakerHalfHeight = speakerHalfSize.y,
        .speakerShell = system->speakerShell,
        .speakerStrength = system->speakerStrength,
        .speakerDamping = system->speakerDamping,
        .simulationTime = system->simulationTime,
        .substepDt = dt,
    };
}

@implementation MetalGpuBackend {
    id<MTLDevice> _device;
    id<MTLCommandQueue> _commandQueue;
    id<MTLLibrary> _library;
    id<MTLComputePipelineState> _clearCountsPSO;
    id<MTLComputePipelineState> _computeCellsPSO;
    id<MTLComputePipelineState> _scatterIndicesPSO;
    id<MTLComputePipelineState> _gatherSortedPSO;
    id<MTLComputePipelineState> _densityPressurePSO;
    id<MTLComputePipelineState> _forcePSO;
    id<MTLComputePipelineState> _integratePSO;
    id<MTLBuffer> _paramsBuffer;

    id<MTLBuffer> _xBuffer;
    id<MTLBuffer> _yBuffer;
    id<MTLBuffer> _vxBuffer;
    id<MTLBuffer> _vyBuffer;
    id<MTLBuffer> _axBuffer;
    id<MTLBuffer> _ayBuffer;
    id<MTLBuffer> _densityBuffer;
    id<MTLBuffer> _pressureBuffer;
    id<MTLBuffer> _temperatureBuffer;
    id<MTLBuffer> _temperatureRateBuffer;
    id<MTLBuffer> _xsphVXBuffer;
    id<MTLBuffer> _xsphVYBuffer;
    id<MTLBuffer> _sortedXBuffer;
    id<MTLBuffer> _sortedYBuffer;
    id<MTLBuffer> _sortedVXBuffer;
    id<MTLBuffer> _sortedVYBuffer;
    id<MTLBuffer> _sortedTemperatureBuffer;
    id<MTLBuffer> _sortedDensityBuffer;
    id<MTLBuffer> _sortedPressureBuffer;
    id<MTLBuffer> _particleCellsBuffer;
    id<MTLBuffer> _sortedCellIndicesBuffer;
    id<MTLBuffer> _sortedIndicesBuffer;
    id<MTLBuffer> _cellCountsBuffer;
    id<MTLBuffer> _cellStartsBuffer;
    id<MTLBuffer> _cellOffsetsBuffer;
    id<MTLBuffer> _cellNeighborCountsBuffer;
    id<MTLBuffer> _cellNeighborsBuffer;

    const void *_cellCountsPtr;
    const void *_cellStartsPtr;
    const void *_cellOffsetsPtr;
    const void *_cellNeighborCountsPtr;
    const void *_cellNeighborsPtr;
    NSUInteger _cellCountsLength;
    NSUInteger _cellStartsLength;
    NSUInteger _cellOffsetsLength;
    NSUInteger _cellNeighborCountsLength;
    NSUInteger _cellNeighborsLength;
}

- (id<MTLBuffer>)wrapBufferForPointer:(void *)pointer
                               length:(NSUInteger)length
                                label:(NSString *)label
                                error:(NSString * _Nullable * _Nullable)error
{
    if (pointer == NULL || length == 0) {
        if (error != NULL) {
            *error = [NSString stringWithFormat:@"Missing storage for %@", label];
        }
        return nil;
    }

    id<MTLBuffer> buffer = [_device newBufferWithBytesNoCopy:pointer
                                                      length:length
                                                     options:MTLResourceStorageModeShared
                                                 deallocator:nil];
    if (buffer == nil) {
        if (error != NULL) {
            *error = [NSString stringWithFormat:@"Failed to wrap %@", label];
        }
        return nil;
    }

    buffer.label = label;
    return buffer;
}

- (BOOL)createFixedBuffersForSystem:(ParticleSystem *)system error:(NSString * _Nullable * _Nullable)error
{
    const NSUInteger particleBytes = (NSUInteger)system->maxParticles * sizeof(float);
    const NSUInteger particleIntBytes = (NSUInteger)system->maxParticles * sizeof(int);

    _xBuffer = [self wrapBufferForPointer:system->x length:particleBytes label:@"x" error:error];
    _yBuffer = [self wrapBufferForPointer:system->y length:particleBytes label:@"y" error:error];
    _vxBuffer = [self wrapBufferForPointer:system->vx length:particleBytes label:@"vx" error:error];
    _vyBuffer = [self wrapBufferForPointer:system->vy length:particleBytes label:@"vy" error:error];
    _axBuffer = [self wrapBufferForPointer:system->ax length:particleBytes label:@"ax" error:error];
    _ayBuffer = [self wrapBufferForPointer:system->ay length:particleBytes label:@"ay" error:error];
    _densityBuffer = [self wrapBufferForPointer:system->density length:particleBytes label:@"density" error:error];
    _pressureBuffer = [self wrapBufferForPointer:system->pressure length:particleBytes label:@"pressure" error:error];
    _temperatureBuffer = [self wrapBufferForPointer:system->temperature length:particleBytes label:@"temperature" error:error];
    _temperatureRateBuffer = [self wrapBufferForPointer:system->temperatureRate length:particleBytes label:@"temperatureRate" error:error];
    _xsphVXBuffer = [self wrapBufferForPointer:system->xsphVX length:particleBytes label:@"xsphVX" error:error];
    _xsphVYBuffer = [self wrapBufferForPointer:system->xsphVY length:particleBytes label:@"xsphVY" error:error];
    _sortedXBuffer = [self wrapBufferForPointer:system->sortedX length:particleBytes label:@"sortedX" error:error];
    _sortedYBuffer = [self wrapBufferForPointer:system->sortedY length:particleBytes label:@"sortedY" error:error];
    _sortedVXBuffer = [self wrapBufferForPointer:system->sortedVX length:particleBytes label:@"sortedVX" error:error];
    _sortedVYBuffer = [self wrapBufferForPointer:system->sortedVY length:particleBytes label:@"sortedVY" error:error];
    _sortedTemperatureBuffer = [self wrapBufferForPointer:system->sortedTemperature length:particleBytes label:@"sortedTemperature" error:error];
    _sortedDensityBuffer = [self wrapBufferForPointer:system->sortedDensity length:particleBytes label:@"sortedDensity" error:error];
    _sortedPressureBuffer = [self wrapBufferForPointer:system->sortedPressure length:particleBytes label:@"sortedPressure" error:error];
    _particleCellsBuffer = [self wrapBufferForPointer:system->particleCells length:particleIntBytes label:@"particleCells" error:error];
    _sortedCellIndicesBuffer = [self wrapBufferForPointer:system->sortedCellIndices length:particleIntBytes label:@"sortedCellIndices" error:error];
    _sortedIndicesBuffer = [self wrapBufferForPointer:system->sortedIndices length:particleIntBytes label:@"sortedIndices" error:error];

    return _xBuffer != nil && _yBuffer != nil && _vxBuffer != nil && _vyBuffer != nil &&
        _axBuffer != nil && _ayBuffer != nil && _densityBuffer != nil && _pressureBuffer != nil &&
        _temperatureBuffer != nil && _temperatureRateBuffer != nil &&
        _xsphVXBuffer != nil && _xsphVYBuffer != nil &&
        _sortedXBuffer != nil && _sortedYBuffer != nil &&
        _sortedVXBuffer != nil && _sortedVYBuffer != nil &&
        _sortedTemperatureBuffer != nil && _sortedDensityBuffer != nil && _sortedPressureBuffer != nil &&
        _particleCellsBuffer != nil && _sortedCellIndicesBuffer != nil && _sortedIndicesBuffer != nil;
}

- (BOOL)rebindGridBuffer:(id<MTLBuffer> __strong *)buffer
                   track:(const void **)trackedPointer
                   bytes:(NSUInteger *)trackedLength
                 pointer:(void *)pointer
                  length:(NSUInteger)length
                   label:(NSString *)label
                   error:(NSString * _Nullable * _Nullable)error
{
    if (*buffer != nil && *trackedPointer == pointer && *trackedLength == length) {
        return YES;
    }

    *buffer = [self wrapBufferForPointer:pointer length:length label:label error:error];
    if (*buffer == nil) {
        return NO;
    }

    *trackedPointer = pointer;
    *trackedLength = length;
    return YES;
}

- (id<MTLComputePipelineState>)pipelineNamed:(NSString *)name error:(NSString * _Nullable * _Nullable)error
{
    id<MTLFunction> function = [_library newFunctionWithName:name];
    if (function == nil) {
        if (error != NULL) {
            *error = [NSString stringWithFormat:@"Missing Metal kernel %@", name];
        }
        return nil;
    }

    NSError *pipelineError = nil;
    id<MTLComputePipelineState> pipeline = [_device newComputePipelineStateWithFunction:function error:&pipelineError];
    if (pipeline == nil && error != NULL) {
        *error = [NSString stringWithFormat:@"Failed to create pipeline %@: %@", name, pipelineError.localizedDescription];
    }
    return pipeline;
}

- (instancetype)initWithSystem:(ParticleSystem *)system error:(NSString * _Nullable * _Nullable)error
{
    self = [super init];
    if (self == nil) {
        if (error != NULL) {
            *error = @"Failed to initialize Metal backend object.";
        }
        return nil;
    }

    _device = MTLCreateSystemDefaultDevice();
    if (_device == nil) {
        if (error != NULL) {
            *error = @"No Metal device is available.";
        }
        return nil;
    }

    _commandQueue = [_device newCommandQueue];
    if (_commandQueue == nil) {
        if (error != NULL) {
            *error = @"Failed to create a Metal command queue.";
        }
        return nil;
    }

    NSString *source = LoadMetalSource();
    if (source == nil) {
        if (error != NULL) {
            *error = @"Could not load src/gpu_backend.metal.";
        }
        return nil;
    }

    MTLCompileOptions *options = [[MTLCompileOptions alloc] init];
    options.mathMode = MTLMathModeFast;

    NSError *libraryError = nil;
    _library = [_device newLibraryWithSource:source options:options error:&libraryError];
    if (_library == nil) {
        if (error != NULL) {
            *error = [NSString stringWithFormat:@"Failed to compile Metal shaders: %@", libraryError.localizedDescription];
        }
        return nil;
    }

    _clearCountsPSO = [self pipelineNamed:@"clearCounts" error:error];
    _computeCellsPSO = [self pipelineNamed:@"computeParticleCells" error:error];
    _scatterIndicesPSO = [self pipelineNamed:@"scatterSortedIndices" error:error];
    _gatherSortedPSO = [self pipelineNamed:@"gatherSortedData" error:error];
    _densityPressurePSO = [self pipelineNamed:@"computeDensityPressure" error:error];
    _forcePSO = [self pipelineNamed:@"computeForces" error:error];
    _integratePSO = [self pipelineNamed:@"integrateParticles" error:error];
    if (_clearCountsPSO == nil || _computeCellsPSO == nil || _scatterIndicesPSO == nil ||
        _gatherSortedPSO == nil || _densityPressurePSO == nil || _forcePSO == nil || _integratePSO == nil) {
        return nil;
    }

    _paramsBuffer = [_device newBufferWithLength:sizeof(GpuSimParams) options:MTLResourceStorageModeShared];
    if (_paramsBuffer == nil) {
        if (error != NULL) {
            *error = @"Failed to allocate GPU parameter buffer.";
        }
        return nil;
    }

    if (![self createFixedBuffersForSystem:system error:error]) {
        return nil;
    }

    if (![self prepareForSystem:system error:error]) {
        return nil;
    }

    return self;
}

- (void)invalidateGridBindings
{
    _cellCountsBuffer = nil;
    _cellStartsBuffer = nil;
    _cellOffsetsBuffer = nil;
    _cellNeighborCountsBuffer = nil;
    _cellNeighborsBuffer = nil;
    _cellCountsPtr = NULL;
    _cellStartsPtr = NULL;
    _cellOffsetsPtr = NULL;
    _cellNeighborCountsPtr = NULL;
    _cellNeighborsPtr = NULL;
    _cellCountsLength = 0;
    _cellStartsLength = 0;
    _cellOffsetsLength = 0;
    _cellNeighborCountsLength = 0;
    _cellNeighborsLength = 0;
}

- (BOOL)prepareForSystem:(ParticleSystem *)system error:(NSString * _Nullable * _Nullable)error
{
    if (![self rebindGridBuffer:&_cellCountsBuffer
                          track:&_cellCountsPtr
                          bytes:&_cellCountsLength
                        pointer:system->cellCounts
                         length:(NSUInteger)system->cellCount * sizeof(int)
                          label:@"cellCounts"
                          error:error]) {
        return NO;
    }
    if (![self rebindGridBuffer:&_cellStartsBuffer
                          track:&_cellStartsPtr
                          bytes:&_cellStartsLength
                        pointer:system->cellStarts
                         length:(NSUInteger)(system->cellCount + 1) * sizeof(int)
                          label:@"cellStarts"
                          error:error]) {
        return NO;
    }
    if (![self rebindGridBuffer:&_cellOffsetsBuffer
                          track:&_cellOffsetsPtr
                          bytes:&_cellOffsetsLength
                        pointer:system->cellOffsets
                         length:(NSUInteger)system->cellCount * sizeof(int)
                          label:@"cellOffsets"
                          error:error]) {
        return NO;
    }
    if (![self rebindGridBuffer:&_cellNeighborCountsBuffer
                          track:&_cellNeighborCountsPtr
                          bytes:&_cellNeighborCountsLength
                        pointer:system->cellNeighborCounts
                         length:(NSUInteger)system->cellCount * sizeof(int)
                          label:@"cellNeighborCounts"
                          error:error]) {
        return NO;
    }
    if (![self rebindGridBuffer:&_cellNeighborsBuffer
                          track:&_cellNeighborsPtr
                          bytes:&_cellNeighborsLength
                        pointer:system->cellNeighbors
                         length:(NSUInteger)system->cellCount * 9u * sizeof(int)
                          label:@"cellNeighbors"
                          error:error]) {
        return NO;
    }

    return YES;
}

- (void)writeParams:(const GpuSimParams *)params
{
    memcpy(_paramsBuffer.contents, params, sizeof(*params));
}

- (void)dispatchCount:(NSUInteger)count
             pipeline:(id<MTLComputePipelineState>)pipeline
              encoder:(id<MTLComputeCommandEncoder>)encoder
{
    if (count == 0) {
        return;
    }

    const NSUInteger threadCount = MIN((NSUInteger)256, pipeline.maxTotalThreadsPerThreadgroup);
    [encoder setComputePipelineState:pipeline];
    [encoder dispatchThreads:MTLSizeMake(count, 1, 1)
      threadsPerThreadgroup:MTLSizeMake(threadCount, 1, 1)];
}

- (BOOL)finishCommandBuffer:(id<MTLCommandBuffer>)commandBuffer error:(NSString * _Nullable * _Nullable)error
{
    [commandBuffer commit];
    [commandBuffer waitUntilCompleted];
    if (commandBuffer.status != MTLCommandBufferStatusCompleted) {
        if (error != NULL) {
            NSString *errorText = (commandBuffer.error.localizedDescription != nil)
                ? commandBuffer.error.localizedDescription
                : @"Unknown Metal command failure.";
            *error = [NSString stringWithFormat:@"Metal command buffer failed: %@", errorText];
        }
        return NO;
    }
    return YES;
}

- (BOOL)runBuildDensityForceForSystem:(ParticleSystem *)system includeForce:(BOOL)includeForce error:(NSString * _Nullable * _Nullable)error
{
    if (![self prepareForSystem:system error:error]) {
        return NO;
    }

    GpuSimParams params = MakeGpuSimParams(system, 0.0f);
    [self writeParams:&params];

    id<MTLCommandBuffer> countBuffer = [_commandQueue commandBuffer];
    id<MTLComputeCommandEncoder> countEncoder = [countBuffer computeCommandEncoder];
    [countEncoder setBuffer:_cellCountsBuffer offset:0 atIndex:0];
    [countEncoder setBuffer:_paramsBuffer offset:0 atIndex:1];
    [self dispatchCount:(NSUInteger)system->cellCount pipeline:_clearCountsPSO encoder:countEncoder];

    [countEncoder setBuffer:_xBuffer offset:0 atIndex:0];
    [countEncoder setBuffer:_yBuffer offset:0 atIndex:1];
    [countEncoder setBuffer:_particleCellsBuffer offset:0 atIndex:2];
    [countEncoder setBuffer:_cellCountsBuffer offset:0 atIndex:3];
    [countEncoder setBuffer:_paramsBuffer offset:0 atIndex:4];
    [self dispatchCount:(NSUInteger)system->particleCount pipeline:_computeCellsPSO encoder:countEncoder];
    [countEncoder endEncoding];

    if (![self finishCommandBuffer:countBuffer error:error]) {
        return NO;
    }

    system->cellStarts[0] = 0;
    for (int cell = 0; cell < system->cellCount; ++cell) {
        system->cellStarts[cell + 1] = system->cellStarts[cell] + system->cellCounts[cell];
        system->cellOffsets[cell] = system->cellStarts[cell];
    }

    id<MTLCommandBuffer> simBuffer = [_commandQueue commandBuffer];
    id<MTLComputeCommandEncoder> encoder = [simBuffer computeCommandEncoder];

    [encoder setBuffer:_particleCellsBuffer offset:0 atIndex:0];
    [encoder setBuffer:_cellOffsetsBuffer offset:0 atIndex:1];
    [encoder setBuffer:_sortedIndicesBuffer offset:0 atIndex:2];
    [encoder setBuffer:_paramsBuffer offset:0 atIndex:3];
    [self dispatchCount:(NSUInteger)system->particleCount pipeline:_scatterIndicesPSO encoder:encoder];

    [encoder setBuffer:_sortedIndicesBuffer offset:0 atIndex:0];
    [encoder setBuffer:_particleCellsBuffer offset:0 atIndex:1];
    [encoder setBuffer:_xBuffer offset:0 atIndex:2];
    [encoder setBuffer:_yBuffer offset:0 atIndex:3];
    [encoder setBuffer:_vxBuffer offset:0 atIndex:4];
    [encoder setBuffer:_vyBuffer offset:0 atIndex:5];
    [encoder setBuffer:_temperatureBuffer offset:0 atIndex:6];
    [encoder setBuffer:_sortedXBuffer offset:0 atIndex:7];
    [encoder setBuffer:_sortedYBuffer offset:0 atIndex:8];
    [encoder setBuffer:_sortedVXBuffer offset:0 atIndex:9];
    [encoder setBuffer:_sortedVYBuffer offset:0 atIndex:10];
    [encoder setBuffer:_sortedTemperatureBuffer offset:0 atIndex:11];
    [encoder setBuffer:_sortedCellIndicesBuffer offset:0 atIndex:12];
    [encoder setBuffer:_paramsBuffer offset:0 atIndex:13];
    [self dispatchCount:(NSUInteger)system->particleCount pipeline:_gatherSortedPSO encoder:encoder];

    [encoder setBuffer:_sortedXBuffer offset:0 atIndex:0];
    [encoder setBuffer:_sortedYBuffer offset:0 atIndex:1];
    [encoder setBuffer:_sortedTemperatureBuffer offset:0 atIndex:2];
    [encoder setBuffer:_sortedCellIndicesBuffer offset:0 atIndex:3];
    [encoder setBuffer:_cellStartsBuffer offset:0 atIndex:4];
    [encoder setBuffer:_cellNeighborCountsBuffer offset:0 atIndex:5];
    [encoder setBuffer:_cellNeighborsBuffer offset:0 atIndex:6];
    [encoder setBuffer:_sortedIndicesBuffer offset:0 atIndex:7];
    [encoder setBuffer:_densityBuffer offset:0 atIndex:8];
    [encoder setBuffer:_pressureBuffer offset:0 atIndex:9];
    [encoder setBuffer:_sortedDensityBuffer offset:0 atIndex:10];
    [encoder setBuffer:_sortedPressureBuffer offset:0 atIndex:11];
    [encoder setBuffer:_paramsBuffer offset:0 atIndex:12];
    [self dispatchCount:(NSUInteger)system->particleCount pipeline:_densityPressurePSO encoder:encoder];

    if (includeForce) {
        [encoder setBuffer:_sortedIndicesBuffer offset:0 atIndex:0];
        [encoder setBuffer:_sortedXBuffer offset:0 atIndex:1];
        [encoder setBuffer:_sortedYBuffer offset:0 atIndex:2];
        [encoder setBuffer:_sortedVXBuffer offset:0 atIndex:3];
        [encoder setBuffer:_sortedVYBuffer offset:0 atIndex:4];
        [encoder setBuffer:_sortedTemperatureBuffer offset:0 atIndex:5];
        [encoder setBuffer:_sortedDensityBuffer offset:0 atIndex:6];
        [encoder setBuffer:_sortedPressureBuffer offset:0 atIndex:7];
        [encoder setBuffer:_sortedCellIndicesBuffer offset:0 atIndex:8];
        [encoder setBuffer:_cellStartsBuffer offset:0 atIndex:9];
        [encoder setBuffer:_cellNeighborCountsBuffer offset:0 atIndex:10];
        [encoder setBuffer:_cellNeighborsBuffer offset:0 atIndex:11];
        [encoder setBuffer:_axBuffer offset:0 atIndex:12];
        [encoder setBuffer:_ayBuffer offset:0 atIndex:13];
        [encoder setBuffer:_temperatureRateBuffer offset:0 atIndex:14];
        [encoder setBuffer:_xsphVXBuffer offset:0 atIndex:15];
        [encoder setBuffer:_xsphVYBuffer offset:0 atIndex:16];
        [encoder setBuffer:_paramsBuffer offset:0 atIndex:17];
        [self dispatchCount:(NSUInteger)system->particleCount pipeline:_forcePSO encoder:encoder];
    }

    [encoder endEncoding];
    return [self finishCommandBuffer:simBuffer error:error];
}

- (BOOL)runStepForSystem:(ParticleSystem *)system dt:(float)dt error:(NSString * _Nullable * _Nullable)error
{
    if (![self prepareForSystem:system error:error]) {
        return NO;
    }

    GpuSimParams params = MakeGpuSimParams(system, dt);
    [self writeParams:&params];

    id<MTLCommandBuffer> countBuffer = [_commandQueue commandBuffer];
    id<MTLComputeCommandEncoder> countEncoder = [countBuffer computeCommandEncoder];
    [countEncoder setBuffer:_cellCountsBuffer offset:0 atIndex:0];
    [countEncoder setBuffer:_paramsBuffer offset:0 atIndex:1];
    [self dispatchCount:(NSUInteger)system->cellCount pipeline:_clearCountsPSO encoder:countEncoder];

    [countEncoder setBuffer:_xBuffer offset:0 atIndex:0];
    [countEncoder setBuffer:_yBuffer offset:0 atIndex:1];
    [countEncoder setBuffer:_particleCellsBuffer offset:0 atIndex:2];
    [countEncoder setBuffer:_cellCountsBuffer offset:0 atIndex:3];
    [countEncoder setBuffer:_paramsBuffer offset:0 atIndex:4];
    [self dispatchCount:(NSUInteger)system->particleCount pipeline:_computeCellsPSO encoder:countEncoder];
    [countEncoder endEncoding];

    if (![self finishCommandBuffer:countBuffer error:error]) {
        return NO;
    }

    system->cellStarts[0] = 0;
    for (int cell = 0; cell < system->cellCount; ++cell) {
        system->cellStarts[cell + 1] = system->cellStarts[cell] + system->cellCounts[cell];
        system->cellOffsets[cell] = system->cellStarts[cell];
    }

    id<MTLCommandBuffer> simBuffer = [_commandQueue commandBuffer];
    id<MTLComputeCommandEncoder> encoder = [simBuffer computeCommandEncoder];

    [encoder setBuffer:_particleCellsBuffer offset:0 atIndex:0];
    [encoder setBuffer:_cellOffsetsBuffer offset:0 atIndex:1];
    [encoder setBuffer:_sortedIndicesBuffer offset:0 atIndex:2];
    [encoder setBuffer:_paramsBuffer offset:0 atIndex:3];
    [self dispatchCount:(NSUInteger)system->particleCount pipeline:_scatterIndicesPSO encoder:encoder];

    [encoder setBuffer:_sortedIndicesBuffer offset:0 atIndex:0];
    [encoder setBuffer:_particleCellsBuffer offset:0 atIndex:1];
    [encoder setBuffer:_xBuffer offset:0 atIndex:2];
    [encoder setBuffer:_yBuffer offset:0 atIndex:3];
    [encoder setBuffer:_vxBuffer offset:0 atIndex:4];
    [encoder setBuffer:_vyBuffer offset:0 atIndex:5];
    [encoder setBuffer:_temperatureBuffer offset:0 atIndex:6];
    [encoder setBuffer:_sortedXBuffer offset:0 atIndex:7];
    [encoder setBuffer:_sortedYBuffer offset:0 atIndex:8];
    [encoder setBuffer:_sortedVXBuffer offset:0 atIndex:9];
    [encoder setBuffer:_sortedVYBuffer offset:0 atIndex:10];
    [encoder setBuffer:_sortedTemperatureBuffer offset:0 atIndex:11];
    [encoder setBuffer:_sortedCellIndicesBuffer offset:0 atIndex:12];
    [encoder setBuffer:_paramsBuffer offset:0 atIndex:13];
    [self dispatchCount:(NSUInteger)system->particleCount pipeline:_gatherSortedPSO encoder:encoder];

    [encoder setBuffer:_sortedXBuffer offset:0 atIndex:0];
    [encoder setBuffer:_sortedYBuffer offset:0 atIndex:1];
    [encoder setBuffer:_sortedTemperatureBuffer offset:0 atIndex:2];
    [encoder setBuffer:_sortedCellIndicesBuffer offset:0 atIndex:3];
    [encoder setBuffer:_cellStartsBuffer offset:0 atIndex:4];
    [encoder setBuffer:_cellNeighborCountsBuffer offset:0 atIndex:5];
    [encoder setBuffer:_cellNeighborsBuffer offset:0 atIndex:6];
    [encoder setBuffer:_sortedIndicesBuffer offset:0 atIndex:7];
    [encoder setBuffer:_densityBuffer offset:0 atIndex:8];
    [encoder setBuffer:_pressureBuffer offset:0 atIndex:9];
    [encoder setBuffer:_sortedDensityBuffer offset:0 atIndex:10];
    [encoder setBuffer:_sortedPressureBuffer offset:0 atIndex:11];
    [encoder setBuffer:_paramsBuffer offset:0 atIndex:12];
    [self dispatchCount:(NSUInteger)system->particleCount pipeline:_densityPressurePSO encoder:encoder];

    [encoder setBuffer:_sortedIndicesBuffer offset:0 atIndex:0];
    [encoder setBuffer:_sortedXBuffer offset:0 atIndex:1];
    [encoder setBuffer:_sortedYBuffer offset:0 atIndex:2];
    [encoder setBuffer:_sortedVXBuffer offset:0 atIndex:3];
    [encoder setBuffer:_sortedVYBuffer offset:0 atIndex:4];
    [encoder setBuffer:_sortedTemperatureBuffer offset:0 atIndex:5];
    [encoder setBuffer:_sortedDensityBuffer offset:0 atIndex:6];
    [encoder setBuffer:_sortedPressureBuffer offset:0 atIndex:7];
    [encoder setBuffer:_sortedCellIndicesBuffer offset:0 atIndex:8];
    [encoder setBuffer:_cellStartsBuffer offset:0 atIndex:9];
    [encoder setBuffer:_cellNeighborCountsBuffer offset:0 atIndex:10];
    [encoder setBuffer:_cellNeighborsBuffer offset:0 atIndex:11];
    [encoder setBuffer:_axBuffer offset:0 atIndex:12];
    [encoder setBuffer:_ayBuffer offset:0 atIndex:13];
    [encoder setBuffer:_temperatureRateBuffer offset:0 atIndex:14];
    [encoder setBuffer:_xsphVXBuffer offset:0 atIndex:15];
    [encoder setBuffer:_xsphVYBuffer offset:0 atIndex:16];
    [encoder setBuffer:_paramsBuffer offset:0 atIndex:17];
    [self dispatchCount:(NSUInteger)system->particleCount pipeline:_forcePSO encoder:encoder];

    [encoder setBuffer:_xBuffer offset:0 atIndex:0];
    [encoder setBuffer:_yBuffer offset:0 atIndex:1];
    [encoder setBuffer:_vxBuffer offset:0 atIndex:2];
    [encoder setBuffer:_vyBuffer offset:0 atIndex:3];
    [encoder setBuffer:_axBuffer offset:0 atIndex:4];
    [encoder setBuffer:_ayBuffer offset:0 atIndex:5];
    [encoder setBuffer:_temperatureBuffer offset:0 atIndex:6];
    [encoder setBuffer:_temperatureRateBuffer offset:0 atIndex:7];
    [encoder setBuffer:_xsphVXBuffer offset:0 atIndex:8];
    [encoder setBuffer:_xsphVYBuffer offset:0 atIndex:9];
    [encoder setBuffer:_paramsBuffer offset:0 atIndex:10];
    [self dispatchCount:(NSUInteger)system->particleCount pipeline:_integratePSO encoder:encoder];

    [encoder endEncoding];
    return [self finishCommandBuffer:simBuffer error:error];
}

@end

static bool InitializeGpuBackend(ParticleSystem *system)
{
    if (system->gpuBackend != NULL) {
        return true;
    }

    NSString *error = nil;
    MetalGpuBackend *backend = [[MetalGpuBackend alloc] initWithSystem:system error:&error];
    if (backend == nil) {
        if (error != nil) {
            fprintf(stderr, "Metal backend init failed: %s\n", error.UTF8String);
        }
        system->gpuBackendAvailable = false;
        return false;
    }

    system->gpuBackend = (__bridge_retained void *)backend;
    system->gpuBackendAvailable = true;
    return true;
}

static void ShutdownGpuBackend(ParticleSystem *system)
{
    if (system->gpuBackend != NULL) {
        CFBridgingRelease(system->gpuBackend);
        system->gpuBackend = NULL;
    }
    system->gpuBackendAvailable = false;
}

static void InvalidateGpuBackendState(ParticleSystem *system)
{
    if (system->gpuBackend != NULL) {
        MetalGpuBackend *backend = (__bridge MetalGpuBackend *)system->gpuBackend;
        [backend invalidateGridBindings];
    }
}

static bool RunGpuDensityForce(ParticleSystem *system, bool includeForce)
{
    if (!InitializeGpuBackend(system)) {
        return false;
    }

    MetalGpuBackend *backend = (__bridge MetalGpuBackend *)system->gpuBackend;
    NSString *error = nil;
    if (![backend runBuildDensityForceForSystem:system includeForce:includeForce error:&error]) {
        if (error != nil) {
            fprintf(stderr, "Metal density/force step failed: %s\n", error.UTF8String);
        }
        return false;
    }
    return true;
}

static bool RunGpuStep(ParticleSystem *system, float dt)
{
    if (!InitializeGpuBackend(system)) {
        return false;
    }

    MetalGpuBackend *backend = (__bridge MetalGpuBackend *)system->gpuBackend;
    NSString *error = nil;
    if (![backend runStepForSystem:system dt:dt error:&error]) {
        if (error != nil) {
            fprintf(stderr, "Metal step failed: %s\n", error.UTF8String);
        }
        return false;
    }
    return true;
}
#else
static bool InitializeGpuBackend(ParticleSystem *system)
{
    (void)system;
    return false;
}

static void ShutdownGpuBackend(ParticleSystem *system)
{
    (void)system;
}

static void InvalidateGpuBackendState(ParticleSystem *system)
{
    (void)system;
}

static bool RunGpuDensityForce(ParticleSystem *system, bool includeForce)
{
    (void)system;
    (void)includeForce;
    return false;
}

static bool RunGpuStep(ParticleSystem *system, float dt)
{
    (void)system;
    (void)dt;
    return false;
}
#endif

static Texture2D CreateSoftCircleTexture(int size, float exponent)
{
    Color *pixels = (Color *)MemAlloc((size_t)size * (size_t)size * sizeof(Color));
    if (pixels == NULL) {
        fprintf(stderr, "Failed to allocate texture pixels\n");
        exit(EXIT_FAILURE);
    }

    const float center = ((float)size - 1.0f) * 0.5f;
    const float invRadius = 1.0f / center;

    for (int y = 0; y < size; ++y) {
        for (int x = 0; x < size; ++x) {
            const float dx = ((float)x - center) * invRadius;
            const float dy = ((float)y - center) * invRadius;
            const float distance = sqrtf(dx * dx + dy * dy);
            float alpha = 0.0f;

            if (distance <= 1.0f) {
                alpha = powf(1.0f - distance, exponent);
            }

            pixels[y * size + x] = (Color){255, 255, 255, (unsigned char)(255.0f * alpha)};
        }
    }

    Image image = {
        .data = pixels,
        .width = size,
        .height = size,
        .mipmaps = 1,
        .format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8,
    };

    Texture2D texture = LoadTextureFromImage(image);
    UnloadImage(image);
    return texture;
}

static void AllocateSystem(ParticleSystem *system, int maxParticles)
{
    memset(system, 0, sizeof(*system));
    system->maxParticles = maxParticles;
    system->viewMode = VIEW_PARTICLES;
    system->colorMode = COLOR_MATERIAL;
    system->activeBackend = SIM_BACKEND_CPU;
    system->scene = SCENE_TANK;
    system->obstacleModel = OBSTACLE_CIRCLE;
    system->obstacleAngleDegrees = 0.0f;
    system->obstacleRectWidth = 158.0f;
    system->obstacleRectHeight = 62.0f;
    system->tankPreset = MATERIAL_WATER;
    system->gpuBackendAvailable = false;
    system->timeScale = 1.0f;
    system->flowSpeedScale = 1.0f;
    system->acousticsEnabled = false;
    system->acousticHandlesInitialized = false;
    system->simulationTime = 0.0f;
    system->speakerWidth = 18.0f;
    system->speakerHeight = 120.0f;
    system->speakerFrequency = 2.4f;
    system->speakerAmplitude = 10.0f;
    system->speakerShell = 0.0f;
    system->speakerStrength = 0.0f;
    system->speakerDamping = 0.0f;
    system->micRadius = 22.0f;
    system->micSignal = 0.0f;
    system->micBaseline = 0.0f;
    system->micWaveformHead = 0;
    system->micWaveformCount = 0;
    system->baseSoundSpeed = 0.0f;
    system->baseKinematicViscosity = 0.0f;
    system->baseGlobalDrag = 0.0f;
    system->acousticSoundSpeed = 180.0f;
    system->acousticMachLimit = 0.30f;
    system->acousticViscosityScale = 0.40f;
    system->acousticDragScale = 0.35f;
    system->audioOutputEnabled = true;
    system->audioOutputReady = false;
    memset(&system->micAudioStream, 0, sizeof(system->micAudioStream));
    system->audioOutputSignal = 0.0f;
    system->audioOutputState = 0.0f;
    system->audioMonitorPitchHz = 180.0f;
    system->audioMonitorPhase = 0.0f;
    system->dragTarget = DRAG_TARGET_NONE;

    system->x = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->y = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->vx = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->vy = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->ax = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->ay = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->density = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->pressure = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->temperature = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->temperatureRate = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->xsphVX = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->xsphVY = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->sortedX = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->sortedY = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->sortedVX = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->sortedVY = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->sortedTemperature = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->sortedDensity = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->sortedPressure = (float *)MemAlloc((size_t)maxParticles * sizeof(float));
    system->particleCells = (int *)MemAlloc((size_t)maxParticles * sizeof(int));
    system->sortedCellIndices = (int *)MemAlloc((size_t)maxParticles * sizeof(int));
    system->sortedIndices = (int *)MemAlloc((size_t)maxParticles * sizeof(int));

    if (system->x == NULL || system->y == NULL || system->vx == NULL || system->vy == NULL ||
        system->ax == NULL || system->ay == NULL || system->density == NULL || system->pressure == NULL ||
        system->temperature == NULL || system->temperatureRate == NULL ||
        system->xsphVX == NULL || system->xsphVY == NULL ||
        system->sortedX == NULL || system->sortedY == NULL || system->sortedVX == NULL ||
        system->sortedVY == NULL || system->sortedTemperature == NULL ||
        system->sortedDensity == NULL || system->sortedPressure == NULL ||
        system->particleCells == NULL || system->sortedCellIndices == NULL ||
        system->sortedIndices == NULL) {
        fprintf(stderr, "Failed to allocate particle buffers\n");
        exit(EXIT_FAILURE);
    }
}

static void RebuildGrid(ParticleSystem *system)
{
    const int newGridWidth = (int)ceilf(system->worldWidth / system->params.supportRadius) + 2;
    const int newGridHeight = (int)ceilf(system->worldHeight / system->params.supportRadius) + 2;
    const int newCellCount = newGridWidth * newGridHeight;
    const bool cellCountChanged = (newCellCount != system->cellCount);
    const bool gridShapeChanged = (newGridWidth != system->gridWidth) || (newGridHeight != system->gridHeight);

    if (cellCountChanged) {
        if (system->cellCounts != NULL) {
            MemFree(system->cellCounts);
        }
        if (system->cellStarts != NULL) {
            MemFree(system->cellStarts);
        }
        if (system->cellOffsets != NULL) {
            MemFree(system->cellOffsets);
        }
        if (system->cellNeighborCounts != NULL) {
            MemFree(system->cellNeighborCounts);
        }
        if (system->cellNeighbors != NULL) {
            MemFree(system->cellNeighbors);
        }

        system->cellCounts = (int *)MemAlloc((size_t)newCellCount * sizeof(int));
        system->cellStarts = (int *)MemAlloc((size_t)(newCellCount + 1) * sizeof(int));
        system->cellOffsets = (int *)MemAlloc((size_t)newCellCount * sizeof(int));
        system->cellNeighborCounts = (int *)MemAlloc((size_t)newCellCount * sizeof(int));
        system->cellNeighbors = (int *)MemAlloc((size_t)newCellCount * 9u * sizeof(int));

        if (system->cellCounts == NULL || system->cellStarts == NULL || system->cellOffsets == NULL ||
            system->cellNeighborCounts == NULL || system->cellNeighbors == NULL) {
            fprintf(stderr, "Failed to allocate grid cells\n");
            exit(EXIT_FAILURE);
        }
        system->cellCount = newCellCount;
    }

    system->gridWidth = newGridWidth;
    system->gridHeight = newGridHeight;
    system->cellSize = system->params.supportRadius;
    system->invCellSize = 1.0f / system->cellSize;

    if (cellCountChanged || gridShapeChanged) {
        for (int cellY = 0; cellY < system->gridHeight; ++cellY) {
            for (int cellX = 0; cellX < system->gridWidth; ++cellX) {
                const int cellIndex = cellY * system->gridWidth + cellX;
                const int neighborBase = cellIndex * 9;
                int neighborCount = 0;

                for (int offsetY = -1; offsetY <= 1; ++offsetY) {
                    const int neighborY = cellY + offsetY;
                    if (neighborY < 0 || neighborY >= system->gridHeight) {
                        continue;
                    }

                    const int rowBase = neighborY * system->gridWidth;
                    for (int offsetX = -1; offsetX <= 1; ++offsetX) {
                        const int neighborX = cellX + offsetX;
                        if (neighborX < 0 || neighborX >= system->gridWidth) {
                            continue;
                        }

                        system->cellNeighbors[neighborBase + neighborCount] = rowBase + neighborX;
                        ++neighborCount;
                    }
                }

                system->cellNeighborCounts[cellIndex] = neighborCount;
            }
        }
    }
}

static float LocalRestDensity(const ParticleSystem *system, float temperature)
{
    const float delta = temperature - system->params.ambientTemperature;
    const float adjusted = system->params.restDensity * (1.0f - system->params.thermalExpansion * delta);
    return fmaxf(0.35f * system->params.restDensity, adjusted);
}

static float TargetAverageDensity(const ParticleSystem *system)
{
    if (system->preset == MATERIAL_GAS) {
        return 0.55f * system->params.restDensity;
    }

    return system->params.restDensity;
}

static float DensityFloor(const ParticleSystem *system)
{
    return (system->preset == MATERIAL_GAS)
        ? 0.10f * system->params.restDensity
        : 0.50f * system->params.restDensity;
}

static float BoundaryMassScale(const ParticleSystem *system)
{
    (void)system;
    return 1.15f;
}

static float XsphBlend(const ParticleSystem *system)
{
    switch (system->preset) {
        case MATERIAL_GAS:
            return 0.0f;
        case MATERIAL_WATER:
        default:
            return 0.055f;
    }
}

static void UpdateMassAndEquationOfState(ParticleSystem *system)
{
    if (AcousticsActive(system)) {
        system->params.soundSpeed = EffectiveAcousticSoundSpeed(system);
        system->params.kinematicViscosity = fmaxf(0.01f,
            system->baseKinematicViscosity * ClampFloat(system->acousticViscosityScale, 0.01f, 4.0f));
        system->params.globalDrag = fmaxf(0.0f,
            system->baseGlobalDrag * ClampFloat(system->acousticDragScale, 0.0f, 4.0f));
    } else {
        if (system->baseSoundSpeed > 0.0f) {
            system->params.soundSpeed = system->baseSoundSpeed;
        }
        if (system->baseKinematicViscosity > 0.0f) {
            system->params.kinematicViscosity = system->baseKinematicViscosity;
        }
        if (system->baseGlobalDrag >= 0.0f) {
            system->params.globalDrag = system->baseGlobalDrag;
        }
    }

    const float h = system->params.supportRadius;
    const float interactionScale = MouseInteractionScale(system);
    system->supportRadiusSquared = system->params.supportRadius * system->params.supportRadius;
    system->mass = system->params.restDensity * system->params.spacing * system->params.spacing;
    system->pressureStiffness = system->params.restDensity *
        system->params.soundSpeed * system->params.soundSpeed / 7.0f;
    system->densityKernel = 4.0f / (PI_F * powf(h, 8.0f));
    system->pressureKernelGrad = -10.0f / (PI_F * powf(h, 5.0f));
    system->viscosityKernelLap = 40.0f / (PI_F * powf(h, 5.0f));
    system->mouseRadius = h * 6.0f * interactionScale;
    system->mouseRadiusSquared = system->mouseRadius * system->mouseRadius;
    system->mouseStrength = system->params.soundSpeed * system->params.soundSpeed * 2.6f * interactionScale;
    system->speakerShell = h * 1.25f;
    system->speakerStrength = system->params.soundSpeed * system->params.soundSpeed * 1.25f;
    system->speakerDamping = system->params.soundSpeed * 1.45f / fmaxf(h, 1.0f);
}

static void RefreshDerivedSimulationParameters(ParticleSystem *system)
{
    UpdateMassAndEquationOfState(system);
    ConfigureSceneParameters(system);
    ClampAcousticAnchors(system);
    system->scalarFieldsDirty = true;
}

static void AddBoundaryDensityContribution(const ParticleSystem *system, float x, float y, float *density)
{
    if (system->preset == MATERIAL_GAS) {
        return;
    }

    const float radius = system->params.particleRadius;
    const float left = system->bounds.x + radius;
    const float right = system->bounds.x + system->bounds.width - radius;
    const float top = system->bounds.y + radius;
    const float bottom = system->bounds.y + system->bounds.height - radius;
    const float boundaryMass = system->mass * BoundaryMassScale(system);
    const float h2 = system->supportRadiusSquared;

    const float distances[4] = {
        x - left,
        right - x,
        y - top,
        bottom - y,
    };

    for (int wall = 0; wall < 4; ++wall) {
        if (SceneIsWindTunnel(system) && wall < 2) {
            continue;
        }
        const float mirroredDistance = 2.0f * distances[wall];
        const float r2 = mirroredDistance * mirroredDistance;
        if (r2 < h2) {
            const float diff = h2 - r2;
            *density += boundaryMass * system->densityKernel * diff * diff * diff;
        }
    }
}

static void AddBoundaryForceContribution(const ParticleSystem *system, float x, float y, float vx, float vy,
    float rhoi, float pressureI, float temperature, float *ax, float *ay)
{
    const float radius = system->params.particleRadius;
    const float left = system->bounds.x + radius;
    const float right = system->bounds.x + system->bounds.width - radius;
    const float top = system->bounds.y + radius;
    const float bottom = system->bounds.y + system->bounds.height - radius;
    const float h = system->params.supportRadius;
    const float h2 = system->supportRadiusSquared;

    const float distances[4] = {
        x - left,
        right - x,
        y - top,
        bottom - y,
    };
    const Vector2 normals[4] = {
        {1.0f, 0.0f},
        {-1.0f, 0.0f},
        {0.0f, 1.0f},
        {0.0f, -1.0f},
    };

    if (system->preset == MATERIAL_GAS) {
        const float wallK = 2.2f * system->params.soundSpeed * system->params.soundSpeed / h;
        const float wallDamp = 1.2f * system->params.soundSpeed / h;

        for (int wall = 0; wall < 4; ++wall) {
            if (SceneIsWindTunnel(system) && wall < 2) {
                continue;
            }
            const float d = distances[wall];
            if (d < h) {
                const float falloff = 1.0f - d / h;
                const float vn = vx * normals[wall].x + vy * normals[wall].y;
                const float repulse = wallK * falloff * falloff;
                const float damp = -wallDamp * fminf(vn, 0.0f);
                *ax += normals[wall].x * (repulse + damp);
                *ay += normals[wall].y * (repulse + damp);
            }
        }
        return;
    }

    const float boundaryMass = system->mass * BoundaryMassScale(system);
    const float boundaryDensity = LocalRestDensity(system, temperature);
    const float boundaryPressure = fmaxf(pressureI, 0.10f * system->pressureStiffness);

    for (int wall = 0; wall < 4; ++wall) {
        if (SceneIsWindTunnel(system) && wall < 2) {
            continue;
        }
        const float mirroredDistance = 2.0f * distances[wall];
        const float r2 = mirroredDistance * mirroredDistance;
        if (r2 > 1e-8f && r2 < h2) {
            const float r = sqrtf(r2);
            const float influence = h - r;
            const float pressureTerm = -boundaryMass *
                ((pressureI / (rhoi * rhoi)) + (boundaryPressure / (boundaryDensity * boundaryDensity))) *
                system->pressureKernelGrad * influence * influence;
            const float viscosityTerm = system->params.kinematicViscosity * boundaryMass *
                system->viscosityKernelLap * influence / boundaryDensity;
            *ax += pressureTerm * normals[wall].x;
            *ay += pressureTerm * normals[wall].y;
            *ax -= viscosityTerm * vx * fabsf(normals[wall].x);
            *ay -= viscosityTerm * vy * fabsf(normals[wall].y);
        }
    }
}

static float PressureFromState(const ParticleSystem *system, float density, float temperature)
{
    if (system->preset == MATERIAL_GAS) {
        const float normalizedTemperature = fmaxf(0.30f, temperature);
        const float gasStiffness = 0.65f * system->params.soundSpeed * system->params.soundSpeed;
        return gasStiffness * density * normalizedTemperature;
    }

    const float ratio = density / LocalRestDensity(system, temperature);
    const float pressure = system->pressureStiffness * (Pow7(ratio) - 1.0f);
    return fmaxf(0.0f, pressure);
}

static void CalibrateParticleMass(ParticleSystem *system)
{
    if (system->preset == MATERIAL_GAS) {
        return;
    }

    BuildGrid(system);

    double densitySum = 0.0;
    for (int slot = 0; slot < system->particleCount; ++slot) {
        densitySum += (double)SampleDensityAtSortedSlot(system, slot, 1.0f);
    }

    const float averageDensity = (float)(densitySum / (double)system->particleCount);
    if (averageDensity > 1e-6f) {
        system->mass = TargetAverageDensity(system) / averageDensity;
    }
}

static void ResetSimulation(ParticleSystem *system, MaterialPreset preset)
{
    InvalidateGpuBackendState(system);
    if (preset != MATERIAL_GAS || !SceneIsWindTunnel(system)) {
        system->tankPreset = preset;
    }
    if (SceneIsWindTunnel(system)) {
        preset = MATERIAL_GAS;
    }
    system->preset = preset;
    system->params = MATERIAL_PRESETS[preset];
    system->worldWidth = (float)WINDOW_WIDTH;
    system->worldHeight = (float)WINDOW_HEIGHT;
    system->bounds = (Rectangle){
        .x = 50.0f,
        .y = 50.0f,
        .width = system->worldWidth - 100.0f,
        .height = system->worldHeight - 100.0f,
    };
    system->paused = false;
    system->accumulator = 0.0f;
    system->mouseActive = false;
    system->dragTarget = DRAG_TARGET_NONE;
    system->simulationTime = 0.0f;
    system->lastStepDt = system->params.timeStep;
    system->framesUntilDiagnostics = 0;
    system->scalarFieldsDirty = true;

    const float spacing = system->params.spacing;
    float fillWidthRatio = 0.48f;
    float fillHeightRatio = 0.68f;
    float originX = system->bounds.x + system->bounds.width * 0.07f;
    float originY = system->bounds.y + system->bounds.height * 0.06f;

    if (SceneIsWindTunnel(system)) {
        fillWidthRatio = 0.995f;
        fillHeightRatio = 0.82f;
        originX = system->bounds.x + system->bounds.width * 0.002f;
        originY = system->bounds.y + system->bounds.height * 0.09f;
    } else if (preset == MATERIAL_GAS) {
        fillWidthRatio = 0.98f;
        fillHeightRatio = 0.96f;
        originX = system->bounds.x + system->bounds.width * 0.01f;
        originY = system->bounds.y + system->bounds.height * 0.02f;
    }

    const float fillWidth = system->bounds.width * fillWidthRatio;
    const float fillHeight = system->bounds.height * fillHeightRatio;
    const int targetParticles = EffectiveTargetParticleCount(system);
    const float targetSpacing = sqrtf((fillWidth * fillHeight) / (float)targetParticles);
    const float resolutionScale = ClampFloat(targetSpacing / spacing, 0.18f, 1.75f);

    system->params.spacing *= resolutionScale;
    system->params.supportRadius *= resolutionScale;
    system->params.particleRadius = fmaxf(0.55f, system->params.particleRadius * resolutionScale);
    system->params.timeStep = fmaxf(1.0f / 720.0f, system->params.timeStep * fmaxf(resolutionScale, 0.45f));
    system->baseSoundSpeed = system->params.soundSpeed;
    system->baseKinematicViscosity = system->params.kinematicViscosity;
    system->baseGlobalDrag = system->params.globalDrag;

    RefreshDerivedSimulationParameters(system);
    if (!system->acousticHandlesInitialized) {
        system->speakerBaseCenter = (Vector2){
            system->bounds.x + system->bounds.width * 0.18f,
            system->bounds.y + system->bounds.height * 0.50f,
        };
        system->micPosition = (Vector2){
            system->bounds.x + system->bounds.width * 0.78f,
            system->bounds.y + system->bounds.height * 0.50f,
        };
        system->acousticHandlesInitialized = true;
    }
    ClampAcousticAnchors(system);
    ResetMicrophoneHistory(system);
    RebuildGrid(system);

    const float scaledSpacing = system->params.spacing;
    const int countX = (int)floorf(fillWidth / scaledSpacing);
    const int countY = (int)floorf(fillHeight / scaledSpacing);
    const float blockHeight = fmaxf(1.0f, (float)(countY - 1) * scaledSpacing);

    system->particleCount = 0;
    for (int y = 0; y < countY && system->particleCount < targetParticles; ++y) {
        for (int x = 0; x < countX && system->particleCount < targetParticles; ++x) {
            const int seedIndex = y * countX + x;
            const float jitterScale = (preset == MATERIAL_GAS) ? 0.16f : 0.08f;
            const float jitterX = (HashNoise(seedIndex * 2 + 11) - 0.5f) * scaledSpacing * jitterScale;
            const float jitterY = (HashNoise(seedIndex * 2 + 29) - 0.5f) * scaledSpacing * jitterScale;
            const float particleX = originX + (float)x * scaledSpacing + jitterX;
            const float particleY = originY + (float)y * scaledSpacing + jitterY;

            if (SceneIsWindTunnel(system)) {
                if (ObstacleSignedDistance(system, particleX, particleY) < system->params.particleRadius * 1.2f) {
                    continue;
                }
            } else if (AcousticsActive(system)) {
                if (SpeakerSignedDistance(system, particleX, particleY) < system->params.particleRadius * 1.2f) {
                    continue;
                }
            }

            const int index = system->particleCount++;
            const float verticalMix = RangeLerp(originY, originY + blockHeight, particleY);
            const float randomVelX = (HashNoise(index * 5 + 101) - 0.5f);
            const float randomVelY = (HashNoise(index * 5 + 211) - 0.5f);

            system->x[index] = particleX;
            system->y[index] = particleY;
            if (SceneIsWindTunnel(system)) {
                const float baseFlow = system->flowTargetSpeed * WindTunnelProfile(system, particleY);
                system->vx[index] = baseFlow + randomVelX * system->params.soundSpeed * 0.035f;
                system->vy[index] = randomVelY * system->params.soundSpeed * 0.025f;
            } else {
                system->vx[index] = (preset == MATERIAL_GAS) ? randomVelX * system->params.soundSpeed * 0.10f : 0.0f;
                system->vy[index] = (preset == MATERIAL_GAS)
                    ? randomVelY * system->params.soundSpeed * 0.10f
                    : 0.0f;
            }
            system->ax[index] = 0.0f;
            system->ay[index] = 0.0f;
            system->density[index] = system->params.restDensity;
            system->pressure[index] = 0.0f;
            system->temperature[index] = ClampFloat(
                system->params.ambientTemperature +
                    system->params.initialTemperatureGradient * (1.0f - verticalMix) +
                    (HashNoise(index * 3 + 7) - 0.5f) * 0.03f,
                TEMP_MIN,
                TEMP_MAX);
            system->temperatureRate[index] = 0.0f;
            system->particleCells[index] = 0;
        }
    }

    CalibrateParticleMass(system);
    BuildGrid(system);
}

static int ClampCellCoord(int value, int maxValue)
{
    if (value < 0) {
        return 0;
    }
    if (value > maxValue) {
        return maxValue;
    }
    return value;
}

static void BuildGrid(ParticleSystem *system)
{
    float *restrict sortedX = system->sortedX;
    float *restrict sortedY = system->sortedY;
    float *restrict sortedVX = system->sortedVX;
    float *restrict sortedVY = system->sortedVY;
    float *restrict sortedTemperature = system->sortedTemperature;
    int *restrict sortedCellIndices = system->sortedCellIndices;
    const float *restrict x = system->x;
    const float *restrict y = system->y;
    const float *restrict vx = system->vx;
    const float *restrict vy = system->vy;
    const float *restrict temperature = system->temperature;
    const int *restrict sortedIndices = system->sortedIndices;

    memset(system->cellCounts, 0, (size_t)system->cellCount * sizeof(int));

    for (int i = 0; i < system->particleCount; ++i) {
        const int cellX = ClampCellCoord((int)(system->x[i] * system->invCellSize), system->gridWidth - 1);
        const int cellY = ClampCellCoord((int)(system->y[i] * system->invCellSize), system->gridHeight - 1);
        const int cellIndex = cellY * system->gridWidth + cellX;
        system->particleCells[i] = cellIndex;
        system->cellCounts[cellIndex] += 1;
    }

    system->cellStarts[0] = 0;
    for (int cell = 0; cell < system->cellCount; ++cell) {
        system->cellStarts[cell + 1] = system->cellStarts[cell] + system->cellCounts[cell];
        system->cellOffsets[cell] = system->cellStarts[cell];
    }

    for (int i = 0; i < system->particleCount; ++i) {
        const int cellIndex = system->particleCells[i];
        system->sortedIndices[system->cellOffsets[cellIndex]++] = i;
    }

    for (int slot = 0; slot < system->particleCount; ++slot) {
        const int particleIndex = sortedIndices[slot];
        sortedX[slot] = x[particleIndex];
        sortedY[slot] = y[particleIndex];
        sortedVX[slot] = vx[particleIndex];
        sortedVY[slot] = vy[particleIndex];
        sortedTemperature[slot] = temperature[particleIndex];
        sortedCellIndices[slot] = system->particleCells[particleIndex];
    }
}

static float SampleDensityAtSortedSlot(const ParticleSystem *system, int slot, float sampleMass)
{
    const float *restrict sortedX = system->sortedX;
    const float *restrict sortedY = system->sortedY;
    const int *restrict cellStarts = system->cellStarts;
    const int *restrict cellNeighborCounts = system->cellNeighborCounts;
    const int *restrict cellNeighbors = system->cellNeighbors;
    const float xi = sortedX[slot];
    const float yi = sortedY[slot];
    const int baseCell = system->sortedCellIndices[slot];
    const int neighborBase = baseCell * 9;
    const int neighborCount = cellNeighborCounts[baseCell];
    float density = 0.0f;

    for (int neighbor = 0; neighbor < neighborCount; ++neighbor) {
        const int cellIndex = cellNeighbors[neighborBase + neighbor];
        const int start = cellStarts[cellIndex];
        const int end = cellStarts[cellIndex + 1];

        for (int cursor = start; cursor < end; ++cursor) {
            const float dx = xi - sortedX[cursor];
            const float dy = yi - sortedY[cursor];
            const float r2 = dx * dx + dy * dy;

            if (r2 < system->supportRadiusSquared) {
                const float diff = system->supportRadiusSquared - r2;
                density += sampleMass * system->densityKernel * diff * diff * diff;
            }
        }
    }

    AddBoundaryDensityContribution(system, xi, yi, &density);
    return density;
}

static void ComputeDensityAndPressureParticle(ParticleSystem *system, int slot)
{
    const int particleIndex = system->sortedIndices[slot];
    const float density = SampleDensityAtSortedSlot(system, slot, system->mass);
    const float clampedDensity = fmaxf(DensityFloor(system), density);

    system->density[particleIndex] = clampedDensity;
    system->pressure[particleIndex] = PressureFromState(system, clampedDensity, system->sortedTemperature[slot]);
    system->sortedDensity[slot] = clampedDensity;
    system->sortedPressure[slot] = system->pressure[particleIndex];
}

static void ComputeDensityAndPressure(ParticleSystem *system)
{
    if (ShouldParallelize(system->particleCount)) {
        dispatch_apply((size_t)system->particleCount,
            dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0),
            ^(size_t index) {
                ComputeDensityAndPressureParticle(system, (int)index);
            });
    } else {
        for (int i = 0; i < system->particleCount; ++i) {
            ComputeDensityAndPressureParticle(system, i);
        }
    }
}

static void ComputeForceParticle(ParticleSystem *system, int slot)
{
    const float h = system->params.supportRadius;
    const float h2 = system->supportRadiusSquared;
    const float spikyGrad = system->pressureKernelGrad;
    const float viscLap = system->viscosityKernelLap;
    const float *restrict sortedX = system->sortedX;
    const float *restrict sortedY = system->sortedY;
    const float *restrict sortedVX = system->sortedVX;
    const float *restrict sortedVY = system->sortedVY;
    const float *restrict sortedTemperature = system->sortedTemperature;
    const float *restrict sortedDensity = system->sortedDensity;
    const float *restrict sortedPressure = system->sortedPressure;
    const int *restrict cellStarts = system->cellStarts;
    const int *restrict cellNeighborCounts = system->cellNeighborCounts;
    const int *restrict cellNeighbors = system->cellNeighbors;
    const bool mouseActive = system->mouseActive;
    const float mouseX = system->mousePosition.x;
    const float mouseY = system->mousePosition.y;
    const float mouseRadius = system->mouseRadius;
    const float mouseRadiusSquared = system->mouseRadiusSquared;
    const float mouseStrength = system->mouseStrength;
    const int particleIndex = system->sortedIndices[slot];

    const float xi = sortedX[slot];
    const float yi = sortedY[slot];
    const float vxi = sortedVX[slot];
    const float vyi = sortedVY[slot];
    const float rhoi = sortedDensity[slot];
    const float pressureI = sortedPressure[slot];
    const float temperatureI = sortedTemperature[slot];
    const int baseCell = system->sortedCellIndices[slot];
    const int neighborBase = baseCell * 9;
    const int neighborCount = cellNeighborCounts[baseCell];

    float ax = -system->params.globalDrag * vxi;
    float ay = system->params.gravity -
        system->params.buoyancy * (temperatureI - system->params.ambientTemperature) -
        system->params.globalDrag * vyi;
    float temperatureRate = 0.0f;
    float xsphVX = 0.0f;
    float xsphVY = 0.0f;

    for (int neighbor = 0; neighbor < neighborCount; ++neighbor) {
        const int cellIndex = cellNeighbors[neighborBase + neighbor];
        const int start = cellStarts[cellIndex];
        const int end = cellStarts[cellIndex + 1];

        for (int cursor = start; cursor < end; ++cursor) {
            if (cursor != slot) {
                const float dx = xi - sortedX[cursor];
                const float dy = yi - sortedY[cursor];
                const float r2 = dx * dx + dy * dy;

                if (r2 > 1e-8f && r2 < h2) {
                    const float r = sqrtf(r2);
                    const float influence = h - r;
                    const float invR = 1.0f / r;
                    const float neighborDensity = sortedDensity[cursor];
                    const float pressureTerm = -system->mass *
                        ((pressureI / (rhoi * rhoi)) +
                         (sortedPressure[cursor] / (neighborDensity * neighborDensity))) *
                        spikyGrad * influence * influence;
                    const float viscosityTerm = system->params.kinematicViscosity * system->mass *
                        viscLap * influence / neighborDensity;
                    const float densityKernelValue = system->densityKernel * (h2 - r2) * (h2 - r2) * (h2 - r2);

                    ax += pressureTerm * dx * invR;
                    ay += pressureTerm * dy * invR;

                    ax += viscosityTerm * (sortedVX[cursor] - vxi);
                    ay += viscosityTerm * (sortedVY[cursor] - vyi);

                    temperatureRate += system->params.temperatureDiffusion * system->mass *
                        (sortedTemperature[cursor] - temperatureI) *
                        viscLap * influence / (rhoi * neighborDensity);

                    xsphVX += system->mass * (sortedVX[cursor] - vxi) *
                        densityKernelValue / neighborDensity;
                    xsphVY += system->mass * (sortedVY[cursor] - vyi) *
                        densityKernelValue / neighborDensity;
                }
            }
        }
    }

    AddBoundaryForceContribution(system, xi, yi, vxi, vyi, rhoi, pressureI, temperatureI, &ax, &ay);
    AddSceneForceContribution(system, xi, yi, vxi, vyi, &ax, &ay);
    AddSpeakerForceContribution(system, xi, yi, vxi, vyi, &ax, &ay);

    if (mouseActive) {
        const float dx = xi - mouseX;
        const float dy = yi - mouseY;
        const float r2 = dx * dx + dy * dy;

        if (r2 > 1e-8f && r2 < mouseRadiusSquared) {
            const float r = sqrtf(r2);
            const float falloff = 1.0f - (r / mouseRadius);
            const float impulse = mouseStrength * falloff * falloff;
            const float invR = 1.0f / r;
            ax += dx * invR * impulse;
            ay += dy * invR * impulse;
        }
    }

    system->ax[particleIndex] = ax;
    system->ay[particleIndex] = ay;
    system->temperatureRate[particleIndex] = temperatureRate;
    system->xsphVX[particleIndex] = xsphVX;
    system->xsphVY[particleIndex] = xsphVY;
}

static void ComputeForces(ParticleSystem *system)
{
    if (ShouldParallelize(system->particleCount)) {
        dispatch_apply((size_t)system->particleCount,
            dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0),
            ^(size_t index) {
                ComputeForceParticle(system, (int)index);
            });
    } else {
        for (int i = 0; i < system->particleCount; ++i) {
            ComputeForceParticle(system, i);
        }
    }
}

static void ResolveBounds(ParticleSystem *system, int index)
{
    const float radius = system->params.particleRadius;
    const float left = system->bounds.x + radius;
    const float right = system->bounds.x + system->bounds.width - radius;
    const float top = system->bounds.y + radius;
    const float bottom = system->bounds.y + system->bounds.height - radius;

    if (SceneIsWindTunnel(system)) {
        if (system->x[index] < left || system->x[index] > right) {
            RespawnWindTunnelParticle(system, index);
            return;
        }
    } else if (system->x[index] < left) {
        system->x[index] = left;
        system->vx[index] = fabsf(system->vx[index]) * system->params.wallBounce;
        system->vy[index] *= system->params.wallFriction;
    } else if (system->x[index] > right) {
        system->x[index] = right;
        system->vx[index] = -fabsf(system->vx[index]) * system->params.wallBounce;
        system->vy[index] *= system->params.wallFriction;
    }

    if (system->y[index] < top) {
        system->y[index] = top;
        system->vy[index] = fabsf(system->vy[index]) * system->params.wallBounce;
        system->vx[index] *= system->params.wallFriction;
    } else if (system->y[index] > bottom) {
        system->y[index] = bottom;
        system->vy[index] = -fabsf(system->vy[index]) * system->params.wallBounce;
        system->vx[index] *= system->params.wallFriction;
    }
}

static void IntegrateParticle(ParticleSystem *system, int i, float dt)
{
    system->vx[i] += system->ax[i] * dt;
    system->vy[i] += system->ay[i] * dt;

    const float maxSpeed = (system->preset == MATERIAL_GAS)
        ? system->params.soundSpeed * 5.0f
        : system->params.soundSpeed * 2.5f;
    const float speed2 = system->vx[i] * system->vx[i] + system->vy[i] * system->vy[i];
    if (speed2 > maxSpeed * maxSpeed) {
        const float scale = maxSpeed / sqrtf(speed2);
        system->vx[i] *= scale;
        system->vy[i] *= scale;
    }

    const float advVx = system->vx[i] + XsphBlend(system) * system->xsphVX[i];
    const float advVy = system->vy[i] + XsphBlend(system) * system->xsphVY[i];
    system->x[i] += advVx * dt;
    system->y[i] += advVy * dt;
    system->temperature[i] = ClampFloat(
        system->temperature[i] + system->temperatureRate[i] * dt,
        TEMP_MIN,
        TEMP_MAX);

    if (system->preset != MATERIAL_GAS) {
        system->vx[i] *= 0.9992f;
        system->vy[i] *= 0.9992f;
    }

    ResolveBounds(system, i);
    ResolveObstacle(system, i);
    ResolveSpeaker(system, i);
}

static float ComputeAdaptiveTimeStep(const ParticleSystem *system, float maxStep)
{
    float maxSpeed = 0.0f;
    float maxAcceleration = 0.0f;

    for (int i = 0; i < system->particleCount; ++i) {
        const float speed = sqrtf(system->vx[i] * system->vx[i] + system->vy[i] * system->vy[i]);
        const float acceleration = sqrtf(system->ax[i] * system->ax[i] + system->ay[i] * system->ay[i]);
        maxSpeed = fmaxf(maxSpeed, speed);
        maxAcceleration = fmaxf(maxAcceleration, acceleration);
    }

    const float h = system->params.supportRadius;
    const float cflDt = 0.30f * h / fmaxf(system->params.soundSpeed + maxSpeed, 1e-4f);
    const float forceDt = 0.25f * sqrtf(h / fmaxf(maxAcceleration, 1e-4f));
    const float viscDt = 0.125f * h * h / fmaxf(system->params.kinematicViscosity, 1e-4f);
    float stepLimit = MinFloat3(maxStep, cflDt, fminf(forceDt, viscDt));
    float minDt = system->params.timeStep * 0.20f;

    if (AcousticsActive(system)) {
        const float speakerSpeed = SpeakerPeakSurfaceSpeed(system);
        const float speakerWaveDt = 0.20f * h / fmaxf(system->params.soundSpeed + speakerSpeed, 1e-4f);
        const float speakerPhaseDt = 1.0f / fmaxf(system->speakerFrequency * 24.0f, 1e-4f);
        stepLimit = fminf(stepLimit, fminf(speakerWaveDt, speakerPhaseDt));
        minDt = system->params.timeStep * 0.05f;
    }

    return ClampFloat(stepLimit, minDt, maxStep);
}

static void Integrate(ParticleSystem *system, float dt)
{
    if (ShouldParallelize(system->particleCount)) {
        dispatch_apply((size_t)system->particleCount,
            dispatch_get_global_queue(QOS_CLASS_USER_INITIATED, 0),
            ^(size_t index) {
                IntegrateParticle(system, (int)index, dt);
            });
    } else {
        for (int i = 0; i < system->particleCount; ++i) {
            IntegrateParticle(system, i, dt);
        }
    }
}

static void UpdateDiagnostics(ParticleSystem *system)
{
    Diagnostics diagnostics = {
        .minDensity = FLT_MAX,
        .maxDensity = -FLT_MAX,
        .minPressure = FLT_MAX,
        .maxPressure = -FLT_MAX,
        .minSpeed = FLT_MAX,
        .maxSpeed = -FLT_MAX,
        .minTemperature = FLT_MAX,
        .maxTemperature = -FLT_MAX,
        .avgDensity = 0.0f,
        .avgPressure = 0.0f,
        .avgSpeed = 0.0f,
        .avgTemperature = 0.0f,
    };

    for (int i = 0; i < system->particleCount; ++i) {
        const float speed = sqrtf(system->vx[i] * system->vx[i] + system->vy[i] * system->vy[i]);
        diagnostics.minDensity = fminf(diagnostics.minDensity, system->density[i]);
        diagnostics.maxDensity = fmaxf(diagnostics.maxDensity, system->density[i]);
        diagnostics.minPressure = fminf(diagnostics.minPressure, system->pressure[i]);
        diagnostics.maxPressure = fmaxf(diagnostics.maxPressure, system->pressure[i]);
        diagnostics.minSpeed = fminf(diagnostics.minSpeed, speed);
        diagnostics.maxSpeed = fmaxf(diagnostics.maxSpeed, speed);
        diagnostics.minTemperature = fminf(diagnostics.minTemperature, system->temperature[i]);
        diagnostics.maxTemperature = fmaxf(diagnostics.maxTemperature, system->temperature[i]);
        diagnostics.avgDensity += system->density[i];
        diagnostics.avgPressure += system->pressure[i];
        diagnostics.avgSpeed += speed;
        diagnostics.avgTemperature += system->temperature[i];
    }

    const float invCount = (system->particleCount > 0) ? 1.0f / (float)system->particleCount : 0.0f;
    diagnostics.avgDensity *= invCount;
    diagnostics.avgPressure *= invCount;
    diagnostics.avgSpeed *= invCount;
    diagnostics.avgTemperature *= invCount;

    system->stats = diagnostics;
}

static void StepSimulationCPU(ParticleSystem *system, float frameDelta)
{
    system->accumulator = fminf(system->accumulator + frameDelta * system->timeScale,
        system->params.timeStep * (float)MAX_SIM_STEPS_PER_FRAME);
    const float maxStep = system->params.timeStep;
    const float minStep = maxStep * 0.20f;
    int steps = 0;

    const double startTime = GetTime();
    while (system->accumulator >= minStep && steps < MAX_SIM_STEPS_PER_FRAME) {
        BuildGrid(system);
        ComputeDensityAndPressure(system);
        ComputeForces(system);
        SampleMicrophone(system);
        const float dt = ComputeAdaptiveTimeStep(system, fminf(maxStep, system->accumulator));
        system->lastStepDt = dt;
        Integrate(system, dt);
        system->simulationTime += dt;
        system->accumulator -= dt;
        ++steps;
        system->scalarFieldsDirty = true;
    }

    const bool needsFreshDensity = ColorModeNeedsFreshDensity(system->colorMode);
    const bool refreshDiagnostics = needsFreshDensity || system->framesUntilDiagnostics <= 0;

    if (system->scalarFieldsDirty && (needsFreshDensity || refreshDiagnostics)) {
        BuildGrid(system);
        ComputeDensityAndPressure(system);
        system->scalarFieldsDirty = false;
    }

    if (refreshDiagnostics) {
        UpdateDiagnostics(system);
        system->framesUntilDiagnostics = DIAGNOSTIC_REFRESH_INTERVAL;
    } else {
        system->framesUntilDiagnostics -= 1;
    }

    system->lastSimStepMs = (GetTime() - startTime) * 1000.0;
}

static bool StepSimulationGPU(ParticleSystem *system, float frameDelta)
{
    system->accumulator = fminf(system->accumulator + frameDelta * system->timeScale,
        system->params.timeStep * (float)MAX_SIM_STEPS_PER_FRAME);
    const float maxStep = system->params.timeStep;
    const float minStep = maxStep * 0.20f;
    int steps = 0;

    const double startTime = GetTime();
    while (system->accumulator >= minStep && steps < MAX_SIM_STEPS_PER_FRAME) {
        const float dt = ComputeAdaptiveTimeStep(system, fminf(maxStep, system->accumulator));
        system->lastStepDt = dt;

        if (!RunGpuStep(system, dt)) {
            SetBackendNotice(system, "GPU step failed. Falling back to CPU.");
            system->activeBackend = SIM_BACKEND_CPU;
            StepSimulationCPU(system, frameDelta);
            return false;
        }

        SampleMicrophone(system);
        system->simulationTime += dt;
        system->accumulator -= dt;
        ++steps;
        system->scalarFieldsDirty = true;
    }

    const bool needsFreshDensity = ColorModeNeedsFreshDensity(system->colorMode);
    const bool refreshDiagnostics = needsFreshDensity || system->framesUntilDiagnostics <= 0;

    if (system->scalarFieldsDirty && (needsFreshDensity || refreshDiagnostics)) {
        if (!RunGpuDensityForce(system, false)) {
            SetBackendNotice(system, "GPU refresh failed. Falling back to CPU.");
            system->activeBackend = SIM_BACKEND_CPU;
            StepSimulationCPU(system, 0.0f);
            return false;
        }
        system->scalarFieldsDirty = false;
    }

    if (refreshDiagnostics) {
        UpdateDiagnostics(system);
        system->framesUntilDiagnostics = DIAGNOSTIC_REFRESH_INTERVAL;
    } else {
        system->framesUntilDiagnostics -= 1;
    }

    system->lastSimStepMs = (GetTime() - startTime) * 1000.0;
    return true;
}

static void StepSimulation(ParticleSystem *system, float frameDelta)
{
    switch (system->activeBackend) {
        case SIM_BACKEND_GPU:
            (void)StepSimulationGPU(system, frameDelta);
            break;
        case SIM_BACKEND_CPU:
        default:
            StepSimulationCPU(system, frameDelta);
            break;
    }
}

static Color ParticleColor(const ParticleSystem *system, int index)
{
    switch (system->colorMode) {
        case COLOR_TEMPERATURE: {
            const float t = RangeLerp(system->stats.minTemperature, system->stats.maxTemperature, system->temperature[index]);
            return ColorRamp(t);
        }
        case COLOR_PRESSURE: {
            const float t = RangeLerp(system->stats.minPressure, system->stats.maxPressure, system->pressure[index]);
            return ColorRamp(t);
        }
        case COLOR_SPEED: {
            const float speed = sqrtf(system->vx[index] * system->vx[index] + system->vy[index] * system->vy[index]);
            const float t = RangeLerp(system->stats.minSpeed, system->stats.maxSpeed, speed);
            return ColorRamp(t);
        }
        case COLOR_DENSITY: {
            const float t = RangeLerp(system->stats.minDensity, system->stats.maxDensity, system->density[index]);
            return ColorRamp(t);
        }
        case COLOR_MATERIAL:
        default:
            switch (system->preset) {
                case MATERIAL_WATER:
                    return (Color){64, 168, 255, 230};
                case MATERIAL_GAS:
                default:
                    return (Color){210, 235, 255, 185};
            }
    }
}

static void DrawParticles(const ParticleSystem *system)
{
    const Texture2D texture = (system->viewMode == VIEW_PARTICLES) ? system->particleTexture : system->supportTexture;
    const int smoothDrawLimit = SmoothViewDrawLimit(system);
    const int drawStride = (system->viewMode == VIEW_SMOOTHING_RADIUS &&
            system->particleCount > smoothDrawLimit)
        ? (system->particleCount / smoothDrawLimit) + 1
        : 1;
    const float smoothCoverageScale = SmoothViewCoverageScale(system);
    const float smoothDiameterScale = (system->viewMode == VIEW_SMOOTHING_RADIUS)
        ? smoothCoverageScale * ClampFloat(1.0f + 0.18f * (float)(drawStride - 1), 1.0f, 2.0f)
        : 1.0f;
    const float diameter = (system->viewMode == VIEW_PARTICLES)
        ? system->params.particleRadius * 2.0f
        : system->params.supportRadius * 2.0f * smoothDiameterScale;

    if (system->viewMode == VIEW_SMOOTHING_RADIUS) {
        BeginBlendMode(BLEND_ADDITIVE);
    }

    rlSetTexture(texture.id);
    rlBegin(RL_QUADS);
    for (int i = 0; i < system->particleCount; ++i) {
        if (system->viewMode == VIEW_SMOOTHING_RADIUS && !ShouldDrawSmoothingSample(i, drawStride)) {
            continue;
        }

        Color color = ParticleColor(system, i);
        if (system->viewMode == VIEW_SMOOTHING_RADIUS) {
            color.a = SmoothViewAlpha(system, drawStride);
        }

        const Rectangle dest = {
            system->x[i] - diameter * 0.5f,
            system->y[i] - diameter * 0.5f,
            diameter,
            diameter,
        };

        const float left = dest.x;
        const float right = dest.x + dest.width;
        const float top = dest.y;
        const float bottom = dest.y + dest.height;

        rlColor4ub(color.r, color.g, color.b, color.a);
        rlTexCoord2f(0.0f, 0.0f);
        rlVertex2f(left, top);
        rlTexCoord2f(0.0f, 1.0f);
        rlVertex2f(left, bottom);
        rlTexCoord2f(1.0f, 1.0f);
        rlVertex2f(right, bottom);
        rlTexCoord2f(1.0f, 0.0f);
        rlVertex2f(right, top);
    }
    rlEnd();
    rlSetTexture(0);

    if (system->viewMode == VIEW_SMOOTHING_RADIUS) {
        EndBlendMode();
    }

    if (system->mouseActive) {
        DrawCircleLines((int)system->mousePosition.x, (int)system->mousePosition.y, system->mouseRadius, (Color){255, 244, 176, 210});
    }
}

static float PolygonSignedArea(const Vector2 *points, int pointCount)
{
    float area = 0.0f;
    for (int i = 0; i < pointCount; ++i) {
        const Vector2 a = points[i];
        const Vector2 b = points[(i + 1) % pointCount];
        area += a.x * b.y - b.x * a.y;
    }
    return 0.5f * area;
}

static bool PointInTriangle(Vector2 p, Vector2 a, Vector2 b, Vector2 c)
{
    const float ab = (b.x - a.x) * (p.y - a.y) - (b.y - a.y) * (p.x - a.x);
    const float bc = (c.x - b.x) * (p.y - b.y) - (c.y - b.y) * (p.x - b.x);
    const float ca = (a.x - c.x) * (p.y - c.y) - (a.y - c.y) * (p.x - c.x);
    const bool hasNeg = (ab < -1e-5f) || (bc < -1e-5f) || (ca < -1e-5f);
    const bool hasPos = (ab > 1e-5f) || (bc > 1e-5f) || (ca > 1e-5f);
    return !(hasNeg && hasPos);
}

static bool PolygonVertexIsEar(const Vector2 *points, const int *indices, int remainingCount, int vertex,
    float orientationSign)
{
    const int prevIndex = indices[(vertex + remainingCount - 1) % remainingCount];
    const int currIndex = indices[vertex];
    const int nextIndex = indices[(vertex + 1) % remainingCount];
    const Vector2 a = points[prevIndex];
    const Vector2 b = points[currIndex];
    const Vector2 c = points[nextIndex];
    const float cross = (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);

    if (cross * orientationSign <= 1e-5f) {
        return false;
    }

    for (int i = 0; i < remainingCount; ++i) {
        const int testIndex = indices[i];
        if (testIndex == prevIndex || testIndex == currIndex || testIndex == nextIndex) {
            continue;
        }
        if (PointInTriangle(points[testIndex], a, b, c)) {
            return false;
        }
    }
    return true;
}

static void DrawObstaclePolygon(const ParticleSystem *system, const Vector2 *localPoints, int pointCount,
    Color fill, Color outline)
{
    Vector2 points[128];
    int indices[128];
    if (pointCount > (int)(sizeof(points) / sizeof(points[0]))) {
        pointCount = (int)(sizeof(points) / sizeof(points[0]));
    }

    for (int i = 0; i < pointCount; ++i) {
        points[i] = ObstacleLocalToWorld(system, localPoints[i]);
        indices[i] = i;
    }

    int remainingCount = pointCount;
    const float orientationSign = (PolygonSignedArea(points, pointCount) >= 0.0f) ? 1.0f : -1.0f;
    int guard = 0;
    while (remainingCount > 2 && guard < 512) {
        bool clippedEar = false;
        for (int i = 0; i < remainingCount; ++i) {
            if (!PolygonVertexIsEar(points, indices, remainingCount, i, orientationSign)) {
                continue;
            }

            const int prevIndex = indices[(i + remainingCount - 1) % remainingCount];
            const int currIndex = indices[i];
            const int nextIndex = indices[(i + 1) % remainingCount];
            DrawTriangle(points[prevIndex], points[currIndex], points[nextIndex], fill);

            for (int shift = i; shift < remainingCount - 1; ++shift) {
                indices[shift] = indices[shift + 1];
            }
            remainingCount -= 1;
            clippedEar = true;
            break;
        }

        if (!clippedEar) {
            break;
        }
        ++guard;
    }

    if (remainingCount > 2) {
        Vector2 fillCenter = {0.0f, 0.0f};
        for (int i = 0; i < pointCount; ++i) {
            fillCenter.x += points[i].x;
            fillCenter.y += points[i].y;
        }
        fillCenter.x /= (float)pointCount;
        fillCenter.y /= (float)pointCount;
        for (int i = 0; i < pointCount; ++i) {
            const int next = (i + 1) % pointCount;
            DrawTriangle(fillCenter, points[i], points[next], fill);
        }
    }

    for (int i = 0; i < pointCount; ++i) {
        const int next = (i + 1) % pointCount;
        DrawLineEx(points[i], points[next], 2.0f, outline);
    }
}

static void DrawObstacle(const ParticleSystem *system)
{
    if (!SceneIsWindTunnel(system)) {
        return;
    }

    const Color fill = (Color){28, 34, 44, 255};
    const Color outline = (Color){190, 214, 235, 230};

    switch (system->obstacleModel) {
        case OBSTACLE_AIRFOIL: {
            const int pointCount = AIRFOIL_OUTLINE_POINT_COUNT;
            const float r = system->obstacleRadius;
            Vector2 localPoints[128];
            for (int i = 0; i < pointCount; ++i) {
                localPoints[i] = (Vector2){
                    AIRFOIL_OUTLINE_POINTS[i].x * r,
                    -AIRFOIL_OUTLINE_POINTS[i].y * r,
                };
            }
            DrawObstaclePolygon(system, localPoints, pointCount, fill, outline);
        } break;
        case OBSTACLE_CAR: {
            const float r = system->obstacleRadius;
            const int pointCount = CAR_OUTLINE_POINT_COUNT;
            Vector2 localPoints[128];
            for (int i = 0; i < pointCount; ++i) {
                localPoints[i] = (Vector2){
                    CAR_OUTLINE_POINTS[i].x * r,
                    CAR_OUTLINE_POINTS[i].y * r,
                };
            }
            DrawObstaclePolygon(system, localPoints, pointCount, fill, outline);
        } break;
        case OBSTACLE_RECTANGLE: {
            const float halfWidth = system->obstacleRectWidth * 0.5f;
            const float halfHeight = system->obstacleRectHeight * 0.5f;
            const Vector2 localPoints[4] = {
                {-halfWidth, -halfHeight},
                {halfWidth, -halfHeight},
                {halfWidth, halfHeight},
                {-halfWidth, halfHeight},
            };
            DrawObstaclePolygon(system, localPoints, 4, fill, outline);
        } break;
        case OBSTACLE_CIRCLE:
        default:
            DrawCircleV(system->obstacleCenter, system->obstacleRadius, fill);
            DrawCircleLines((int)lroundf(system->obstacleCenter.x), (int)lroundf(system->obstacleCenter.y),
                system->obstacleRadius, outline);
            break;
    }
}

static void DrawAcousticRig(const ParticleSystem *system)
{
    if (!AcousticsActive(system)) {
        return;
    }

    Vector2 speakerCenter;
    Vector2 speakerVelocity;
    Vector2 speakerHalfSize;
    GetSpeakerState(system, &speakerCenter, &speakerVelocity, &speakerHalfSize);

    const Rectangle baseRect = SpeakerBaseRect(system);
    const Rectangle speakerRect = {
        speakerCenter.x - speakerHalfSize.x,
        speakerCenter.y - speakerHalfSize.y,
        speakerHalfSize.x * 2.0f,
        speakerHalfSize.y * 2.0f,
    };

    DrawRectangleLinesEx(baseRect, 1.0f, (Color){74, 102, 136, 160});
    DrawRectangleRec(speakerRect, (Color){26, 34, 44, 240});
    DrawRectangleLinesEx(speakerRect, 2.0f, (Color){122, 212, 255, 255});
    const float velocityIndicator = ClampFloat(speakerVelocity.x * 0.015f, -28.0f, 28.0f);
    DrawLineV(speakerCenter, (Vector2){speakerCenter.x + velocityIndicator, speakerCenter.y},
        (Color){160, 225, 255, 210});

    DrawCircleV(system->micPosition, system->micRadius, (Color){18, 30, 40, 180});
    DrawCircleLines((int)lroundf(system->micPosition.x), (int)lroundf(system->micPosition.y),
        system->micRadius, (Color){255, 208, 112, 255});
    DrawCircleLines((int)lroundf(system->micPosition.x), (int)lroundf(system->micPosition.y), 7.0f,
        (Color){255, 208, 112, 255});
    DrawLineEx((Vector2){system->micPosition.x - 10.0f, system->micPosition.y},
        (Vector2){system->micPosition.x + 10.0f, system->micPosition.y}, 2.0f, (Color){255, 208, 112, 220});
    DrawLineEx((Vector2){system->micPosition.x, system->micPosition.y - 10.0f},
        (Vector2){system->micPosition.x, system->micPosition.y + 10.0f}, 2.0f, (Color){255, 208, 112, 220});
}

static void DrawHud(const ParticleSystem *system)
{
    static const char *const colorModeLabels[] = {
        "Material",
        "Temperature",
        "Pressure",
        "Speed",
        "Density",
    };

    const char *label = colorModeLabels[ClampInt((int)system->colorMode, 0, COLOR_MODE_COUNT - 1)];
    const int fontSize = 22;
    const int paddingX = 14;
    const int paddingY = 9;
    const int textWidth = MeasureText(label, fontSize);
    const int boxWidth = textWidth + paddingX * 2;
    const int boxHeight = fontSize + paddingY * 2;
    const int boxX = WINDOW_WIDTH - boxWidth - 18;
    const int boxY = 18;
    const Rectangle box = {(float)boxX, (float)boxY, (float)boxWidth, (float)boxHeight};
    DrawRectangleRec(box, (Color){9, 18, 34, 208});
    DrawRectangleLinesEx(box, 1.5f, (Color){66, 124, 184, 235});
    DrawText(label, boxX + paddingX, boxY + paddingY, fontSize, (Color){232, 242, 255, 255});
}

static void DrawBackendNotice(const ParticleSystem *system)
{
    (void)system;
}

static void ApplyTargetCountPreset(ParticleSystem *system, int targetParticleCount)
{
    SetTargetParticleCount(system, targetParticleCount);
}

static void HandlePressedKey(ParticleSystem *system, int key)
{
    switch (key) {
        case KEY_V:
            system->viewMode = (ViewMode)((system->viewMode + 1) % VIEW_MODE_COUNT);
            break;
        case KEY_C:
            system->colorMode = (ColorMode)((system->colorMode + 1) % COLOR_MODE_COUNT);
            break;
        case KEY_O:
            CycleObstacleModel(system);
            break;
        case KEY_F:
            ToggleSimulationScene(system);
            break;
        case KEY_G:
            ToggleSimulationBackend(system);
            break;
        case KEY_SPACE:
            system->paused = !system->paused;
            break;
        case KEY_R:
            ResetSimulation(system, system->preset);
            break;
        case KEY_M: {
            const MaterialPreset nextPreset = (system->preset == MATERIAL_GAS) ? MATERIAL_WATER : MATERIAL_GAS;
            ResetSimulation(system, nextPreset);
        } break;
        case KEY_ONE:
            ResetSimulation(system, MATERIAL_WATER);
            break;
        case KEY_TWO:
            ResetSimulation(system, MATERIAL_GAS);
            break;
        case KEY_FOUR:
            ApplyTargetCountPreset(system, CountPresetForBackend(system->activeBackend, 0));
            break;
        case KEY_FIVE:
            ApplyTargetCountPreset(system, CountPresetForBackend(system->activeBackend, 1));
            break;
        case KEY_SIX:
            ApplyTargetCountPreset(system, CountPresetForBackend(system->activeBackend, 2));
            break;
        case KEY_SEVEN:
            ApplyTargetCountPreset(system, CountPresetForBackend(system->activeBackend, 3));
            break;
        case KEY_EIGHT:
            ApplyTargetCountPreset(system, CountPresetForBackend(system->activeBackend, 4));
            break;
        case KEY_LEFT_BRACKET:
            system->params.kinematicViscosity = fmaxf(0.05f, system->params.kinematicViscosity * 0.85f);
            break;
        case KEY_RIGHT_BRACKET:
            system->params.kinematicViscosity = fminf(80.0f, system->params.kinematicViscosity * 1.15f);
            break;
        case KEY_MINUS:
            system->params.gravity = fmaxf(0.0f, system->params.gravity - 60.0f);
            break;
        case KEY_EQUAL:
            system->params.gravity = fminf(1400.0f, system->params.gravity + 60.0f);
            break;
        default:
            break;
    }
}

static bool ShortcutAllowedWhileUiFocused(int key)
{
    return key == KEY_SPACE || key == KEY_C || key == KEY_V;
}

static bool UpdateAcousticDragging(ParticleSystem *system, bool allowMouseInput)
{
    if (!AcousticsActive(system)) {
        system->dragTarget = DRAG_TARGET_NONE;
        return false;
    }

    if (!allowMouseInput) {
        if (!IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            system->dragTarget = DRAG_TARGET_NONE;
        }
        return (system->dragTarget != DRAG_TARGET_NONE);
    }

    const bool mouseDown = IsMouseButtonDown(MOUSE_BUTTON_LEFT);
    const bool mousePressed = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);

    if (!mouseDown) {
        system->dragTarget = DRAG_TARGET_NONE;
        return false;
    }

    Vector2 speakerCenter;
    Vector2 speakerHalfSize;
    GetSpeakerState(system, &speakerCenter, NULL, &speakerHalfSize);
    const Rectangle speakerRect = {
        speakerCenter.x - speakerHalfSize.x,
        speakerCenter.y - speakerHalfSize.y,
        speakerHalfSize.x * 2.0f,
        speakerHalfSize.y * 2.0f,
    };
    const float micHitRadius = fmaxf(system->micRadius * 0.80f, 12.0f);

    if (mousePressed && CheckCollisionPointRec(system->mousePosition, system->bounds)) {
        if (CheckCollisionPointRec(system->mousePosition, speakerRect)) {
            system->dragTarget = DRAG_TARGET_SPEAKER;
            system->dragOffset = (Vector2){
                system->mousePosition.x - speakerCenter.x,
                system->mousePosition.y - speakerCenter.y,
            };
            return true;
        }

        if (CheckCollisionPointCircle(system->mousePosition, system->micPosition, micHitRadius)) {
            system->dragTarget = DRAG_TARGET_MIC;
            system->dragOffset = (Vector2){
                system->mousePosition.x - system->micPosition.x,
                system->mousePosition.y - system->micPosition.y,
            };
            return true;
        }
    }

    if (system->dragTarget == DRAG_TARGET_SPEAKER) {
        system->speakerBaseCenter.x = system->mousePosition.x - system->dragOffset.x - (speakerCenter.x - system->speakerBaseCenter.x);
        system->speakerBaseCenter.y = system->mousePosition.y - system->dragOffset.y;
        ClampAcousticAnchors(system);
        return true;
    }

    if (system->dragTarget == DRAG_TARGET_MIC) {
        system->micPosition.x = system->mousePosition.x - system->dragOffset.x;
        system->micPosition.y = system->mousePosition.y - system->dragOffset.y;
        ClampAcousticAnchors(system);
        return true;
    }

    return false;
}

static void ProcessInput(ParticleSystem *system, bool allowMouseInput, bool allowKeyboardInput)
{
    system->mousePosition = GetMousePosition();
    const bool draggingAcousticHandle = UpdateAcousticDragging(system, allowMouseInput);
    system->mouseActive = allowMouseInput &&
        !draggingAcousticHandle &&
        IsMouseButtonDown(MOUSE_BUTTON_LEFT) &&
        CheckCollisionPointRec(system->mousePosition, system->bounds);

    for (int key = GetKeyPressed(); key != 0; key = GetKeyPressed()) {
        if (allowKeyboardInput || ShortcutAllowedWhileUiFocused(key)) {
            HandlePressedKey(system, key);
        }
    }
}

static UiPanelState BuildUiPanelState(const ParticleSystem *system)
{
    UiPanelState state = {
        .backend = (int)system->activeBackend,
        .targetParticleCount = EffectiveTargetParticleCount(system),
        .actualParticleCount = system->particleCount,
        .mode = (int)CurrentUiMode(system),
        .obstacleModel = (int)system->obstacleModel,
        .obstacleAngleDegrees = system->obstacleAngleDegrees,
        .obstacleRectWidth = system->obstacleRectWidth,
        .obstacleRectHeight = system->obstacleRectHeight,
        .viewMode = (int)system->viewMode,
        .colorMode = (int)system->colorMode,
        .paused = system->paused,
        .gpuBackendAvailable = system->gpuBackendAvailable,
        .windSpeedScale = system->flowSpeedScale,
        .flowMach = SceneIsWindTunnel(system)
            ? (system->flowTargetSpeed / fmaxf(system->params.soundSpeed, 1e-4f))
            : 0.0f,
        .acousticsAvailable = AcousticsAvailable(system),
        .acousticsEnabled = system->acousticsEnabled,
        .speakerFrequency = system->speakerFrequency,
        .speakerAmplitude = system->speakerAmplitude,
        .effectiveSpeakerAmplitude = EffectiveSpeakerAmplitude(system),
        .acousticSoundSpeed = system->acousticSoundSpeed,
        .acousticMachLimit = system->acousticMachLimit,
        .acousticViscosityScale = system->acousticViscosityScale,
        .acousticDragScale = system->acousticDragScale,
        .audioMonitorPitchHz = system->audioMonitorPitchHz,
        .speakerWidth = system->speakerWidth,
        .speakerHeight = system->speakerHeight,
        .micRadius = system->micRadius,
        .micSignal = system->micSignal,
        .audioOutputAvailable = system->audioOutputReady,
        .audioOutputEnabled = system->audioOutputEnabled && system->audioOutputReady,
        .micWaveform = system->micWaveformDisplay,
        .micWaveformCount = system->micWaveformCount,
        .fps = (float)GetFPS(),
        .lastSimStepMs = system->lastSimStepMs,
    };
    return state;
}

static void ApplyUiMode(ParticleSystem *system, UiSimMode mode)
{
    switch (mode) {
        case UI_MODE_GAS_TANK:
            system->tankPreset = MATERIAL_GAS;
            if (system->scene != SCENE_TANK) {
                system->scene = SCENE_TANK;
                SetBackendNotice(system, "Gas tank enabled.");
                ResetSimulation(system, MATERIAL_GAS);
            } else if (system->preset != MATERIAL_GAS) {
                SetBackendNotice(system, "Gas tank enabled.");
                ResetSimulation(system, MATERIAL_GAS);
            }
            break;
        case UI_MODE_WIND_TUNNEL:
            SetSimulationScene(system, SCENE_WIND_TUNNEL);
            break;
        case UI_MODE_WATER_TANK:
        default:
            system->tankPreset = MATERIAL_WATER;
            if (system->scene != SCENE_TANK) {
                system->scene = SCENE_TANK;
                SetBackendNotice(system, "Water tank enabled.");
                ResetSimulation(system, MATERIAL_WATER);
            } else if (system->preset != MATERIAL_WATER) {
                SetBackendNotice(system, "Water tank enabled.");
                ResetSimulation(system, MATERIAL_WATER);
            }
            break;
    }
}

static void ApplyUiPanelActions(ParticleSystem *system, const UiPanelActions *actions)
{
    bool refreshAcousticTuning = false;

    if (actions->setBackend) {
        if (actions->backend == SIM_BACKEND_GPU) {
            (void)InitializeGpuBackend(system);
        }
        SetSimulationBackend(system, (SimulationBackend)actions->backend);
    }

    if (actions->setTargetParticleCount) {
        SetTargetParticleCount(system, actions->targetParticleCount);
    }

    if (actions->setMode) {
        ApplyUiMode(system, (UiSimMode)actions->mode);
    }

    if (actions->setObstacleModel) {
        SetObstacleModel(system, (ObstacleModel)actions->obstacleModel);
    }

    if (actions->setObstacleAngleDegrees) {
        SetObstacleAngleDegrees(system, actions->obstacleAngleDegrees);
    }

    if (actions->setObstacleRectWidth) {
        SetObstacleRectangleWidth(system, actions->obstacleRectWidth);
    }

    if (actions->setObstacleRectHeight) {
        SetObstacleRectangleHeight(system, actions->obstacleRectHeight);
    }

    if (actions->setViewMode) {
        system->viewMode = (ViewMode)actions->viewMode;
    }

    if (actions->setColorMode) {
        system->colorMode = (ColorMode)actions->colorMode;
    }

    if (actions->setPaused) {
        system->paused = actions->paused;
    }

    if (actions->setWindSpeedScale) {
        SetWindSpeedScale(system, actions->windSpeedScale);
    }

    if (actions->setAcousticsEnabled) {
        system->acousticsEnabled = actions->acousticsEnabled;
        ResetMicrophoneHistory(system);
        if (AcousticsAvailable(system)) {
            ResetSimulation(system, system->preset);
        }
    }

    if (actions->setSpeakerFrequency) {
        system->speakerFrequency = ClampFloat(actions->speakerFrequency, 0.1f, 40.0f);
        ClampAcousticAnchors(system);
    }

    if (actions->setSpeakerAmplitude) {
        system->speakerAmplitude = ClampFloat(actions->speakerAmplitude, 1.0f, 42.0f);
        ClampAcousticAnchors(system);
    }

    if (actions->setAcousticSoundSpeed) {
        system->acousticSoundSpeed = ClampFloat(actions->acousticSoundSpeed, 52.0f, 420.0f);
        refreshAcousticTuning = true;
    }

    if (actions->setAcousticMachLimit) {
        system->acousticMachLimit = ClampFloat(actions->acousticMachLimit, 0.10f, 0.60f);
        ClampAcousticAnchors(system);
    }

    if (actions->setAcousticViscosityScale) {
        system->acousticViscosityScale = ClampFloat(actions->acousticViscosityScale, 0.05f, 1.00f);
        refreshAcousticTuning = true;
    }

    if (actions->setAcousticDragScale) {
        system->acousticDragScale = ClampFloat(actions->acousticDragScale, 0.00f, 1.00f);
        refreshAcousticTuning = true;
    }

    if (actions->setSpeakerWidth) {
        system->speakerWidth = ClampFloat(actions->speakerWidth, 8.0f, 80.0f);
        ClampAcousticAnchors(system);
        if (AcousticsActive(system)) {
            ResetSimulation(system, system->preset);
        }
    }

    if (actions->setSpeakerHeight) {
        system->speakerHeight = ClampFloat(actions->speakerHeight, 24.0f, 220.0f);
        ClampAcousticAnchors(system);
        if (AcousticsActive(system)) {
            ResetSimulation(system, system->preset);
        }
    }

    if (actions->setMicRadius) {
        system->micRadius = ClampFloat(actions->micRadius, 8.0f, 80.0f);
    }

    if (actions->setAudioOutputEnabled) {
        if (actions->audioOutputEnabled && !system->audioOutputReady) {
            (void)InitializeAudioOutput(system);
        }
        system->audioOutputEnabled = actions->audioOutputEnabled && system->audioOutputReady;
    }

    if (actions->setAudioMonitorPitchHz) {
        system->audioMonitorPitchHz = ClampFloat(actions->audioMonitorPitchHz, 0.0f, 1000.0f);
    }

    if (refreshAcousticTuning) {
        RefreshDerivedSimulationParameters(system);
    }

    if (actions->requestReset) {
        ResetSimulation(system, system->preset);
    }
}

static void Cleanup(ParticleSystem *system)
{
    ShutdownAudioOutput(system);
    ShutdownGpuBackend(system);
    if (system->particleTexture.id != 0) {
        UnloadTexture(system->particleTexture);
    }
    if (system->supportTexture.id != 0) {
        UnloadTexture(system->supportTexture);
    }
    if (system->cellCounts != NULL) {
        MemFree(system->cellCounts);
    }
    if (system->cellStarts != NULL) {
        MemFree(system->cellStarts);
    }
    if (system->cellOffsets != NULL) {
        MemFree(system->cellOffsets);
    }
    if (system->cellNeighborCounts != NULL) {
        MemFree(system->cellNeighborCounts);
    }
    if (system->cellNeighbors != NULL) {
        MemFree(system->cellNeighbors);
    }
    if (system->x != NULL) {
        MemFree(system->x);
    }
    if (system->y != NULL) {
        MemFree(system->y);
    }
    if (system->vx != NULL) {
        MemFree(system->vx);
    }
    if (system->vy != NULL) {
        MemFree(system->vy);
    }
    if (system->ax != NULL) {
        MemFree(system->ax);
    }
    if (system->ay != NULL) {
        MemFree(system->ay);
    }
    if (system->density != NULL) {
        MemFree(system->density);
    }
    if (system->pressure != NULL) {
        MemFree(system->pressure);
    }
    if (system->temperature != NULL) {
        MemFree(system->temperature);
    }
    if (system->temperatureRate != NULL) {
        MemFree(system->temperatureRate);
    }
    if (system->xsphVX != NULL) {
        MemFree(system->xsphVX);
    }
    if (system->xsphVY != NULL) {
        MemFree(system->xsphVY);
    }
    if (system->sortedX != NULL) {
        MemFree(system->sortedX);
    }
    if (system->sortedY != NULL) {
        MemFree(system->sortedY);
    }
    if (system->sortedVX != NULL) {
        MemFree(system->sortedVX);
    }
    if (system->sortedVY != NULL) {
        MemFree(system->sortedVY);
    }
    if (system->sortedTemperature != NULL) {
        MemFree(system->sortedTemperature);
    }
    if (system->sortedDensity != NULL) {
        MemFree(system->sortedDensity);
    }
    if (system->sortedPressure != NULL) {
        MemFree(system->sortedPressure);
    }
    if (system->particleCells != NULL) {
        MemFree(system->particleCells);
    }
    if (system->sortedCellIndices != NULL) {
        MemFree(system->sortedCellIndices);
    }
    if (system->sortedIndices != NULL) {
        MemFree(system->sortedIndices);
    }
}

int main(void)
{
    SetTraceLogLevel(LOG_WARNING);
    SetConfigFlags(FLAG_VSYNC_HINT);
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "GasSim - 2D SPH");
    SetTargetFPS(144);

    ParticleSystem system;
    AllocateSystem(&system, MAX_PARTICLES);
    system.targetParticleCount = DEFAULT_TARGET_PARTICLES;
    system.particleTexture = CreateSoftCircleTexture(24, 1.6f);
    system.supportTexture = CreateSoftCircleTexture(48, 1.8f);
    if (!InitializeAudioOutput(&system)) {
        system.audioOutputEnabled = false;
    }
    ResetSimulation(&system, MATERIAL_WATER);
    if (InitializeGpuBackend(&system)) {
        SetSimulationBackend(&system, SIM_BACKEND_GPU);
    }
    UiPanelSetup();
    BuildGrid(&system);
    ComputeDensityAndPressure(&system);
    UpdateDiagnostics(&system);
    system.scalarFieldsDirty = false;
    system.framesUntilDiagnostics = DIAGNOSTIC_REFRESH_INTERVAL;

    while (!WindowShouldClose()) {
        const float frameDelta = ClampFloat(GetFrameTime(), 0.0f, 1.0f / 20.0f);
        BeginDrawing();
        ClearBackground((Color){5, 10, 18, 255});

        UiPanelBegin(frameDelta);

        const UiPanelState panelState = BuildUiPanelState(&system);
        UiPanelActions panelActions;
        UiPanelDraw(&panelState, &panelActions);
        const UiCaptureState captureState = UiPanelGetCaptureState();

        ApplyUiPanelActions(&system, &panelActions);
        ProcessInput(&system, !captureState.wantsMouse, !captureState.wantsKeyboard);

        if (!system.paused) {
            StepSimulation(&system, frameDelta);
        } else {
            const bool needsFreshDensity = ColorModeNeedsFreshDensity(system.colorMode);
            const bool refreshDiagnostics = needsFreshDensity || system.framesUntilDiagnostics <= 0;

            if (system.scalarFieldsDirty && (needsFreshDensity || refreshDiagnostics)) {
                BuildGrid(&system);
                ComputeDensityAndPressure(&system);
                system.scalarFieldsDirty = false;
            }

            if (refreshDiagnostics) {
                UpdateDiagnostics(&system);
                system.framesUntilDiagnostics = DIAGNOSTIC_REFRESH_INTERVAL;
            } else {
                system.framesUntilDiagnostics -= 1;
            }

            system.lastSimStepMs = 0.0;
            system.lastStepDt = 0.0f;
        }

        DrawRectangleRounded(system.bounds, 0.02f, 8, (Color){12, 20, 32, 255});
        DrawRectangleRoundedLinesEx(system.bounds, 0.02f, 8, 2.0f, (Color){85, 128, 170, 255});
        DrawObstacle(&system);
        DrawParticles(&system);
        DrawAcousticRig(&system);
        DrawHud(&system);
        DrawBackendNotice(&system);

        if (system.paused) {
            DrawText("PAUSED", WINDOW_WIDTH / 2 - 54, 24, 28, (Color){255, 215, 90, 255});
        }

        UiPanelEnd();
        EndDrawing();
    }

    UiPanelShutdown();
    Cleanup(&system);
    CloseWindow();
    return 0;
}
