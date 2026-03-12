#include <metal_stdlib>

using namespace metal;

struct GpuSimParams {
    uint particleCount;
    uint gridWidth;
    uint gridHeight;
    uint cellCount;
    uint preset;
    uint scene;
    uint obstacleModel;
    uint mouseActive;
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
    uint acousticsEnabled;
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
};

constant uint MATERIAL_WATER = 0u;
constant uint MATERIAL_GAS = 1u;
constant uint SCENE_TANK = 0u;
constant uint SCENE_WIND_TUNNEL = 1u;
constant uint OBSTACLE_CIRCLE = 0u;
constant uint OBSTACLE_AIRFOIL = 1u;
constant uint OBSTACLE_CAR = 2u;
constant uint OBSTACLE_RECTANGLE = 3u;
constant float TEMP_MIN = 0.35f;
constant float TEMP_MAX = 2.50f;
constant float PI_F = 3.14159265358979323846f;
constant uint AIRFOIL_OUTLINE_POINT_COUNT = 45u;
constant float2 AIRFOIL_OUTLINE_POINTS[AIRFOIL_OUTLINE_POINT_COUNT] = {
    float2(1.9919f, 0.0067f),
    float2(1.9415f, 0.0171f),
    float2(1.8914f, 0.0272f),
    float2(1.7439f, 0.0560f),
    float2(1.6478f, 0.0738f),
    float2(1.4138f, 0.1149f),
    float2(1.2779f, 0.1372f),
    float2(0.9748f, 0.1828f),
    float2(0.8103f, 0.2051f),
    float2(0.4608f, 0.2467f),
    float2(0.2788f, 0.2653f),
    float2(-0.0905f, 0.2950f),
    float2(-0.2748f, 0.3054f),
    float2(-0.6368f, 0.3150f),
    float2(-0.8115f, 0.3130f),
    float2(-1.1373f, 0.2944f),
    float2(-1.2856f, 0.2785f),
    float2(-1.5474f, 0.2348f),
    float2(-1.6584f, 0.2079f),
    float2(-1.8354f, 0.1478f),
    float2(-1.9000f, 0.1157f),
    float2(-1.9791f, 0.0496f),
    float2(-1.9899f, -0.0149f),
    float2(-1.9699f, -0.0445f),
    float2(-1.8805f, -0.0946f),
    float2(-1.8122f, -0.1148f),
    float2(-1.6312f, -0.1454f),
    float2(-1.5198f, -0.1557f),
    float2(-1.2613f, -0.1671f),
    float2(-1.1163f, -0.1686f),
    float2(-0.7994f, -0.1646f),
    float2(-0.6299f, -0.1598f),
    float2(-0.2757f, -0.1467f),
    float2(-0.0941f, -0.1383f),
    float2(0.2717f, -0.1185f),
    float2(0.4525f, -0.1078f),
    float2(0.8006f, -0.0861f),
    float2(0.9650f, -0.0755f),
    float2(1.2690f, -0.0554f),
    float2(1.4058f, -0.0463f),
    float2(1.6419f, -0.0302f),
    float2(1.7392f, -0.0235f),
    float2(1.8889f, -0.0130f),
    float2(1.9399f, -0.0093f),
    float2(1.9916f, 0.0019f),
};
constant uint CAR_OUTLINE_POINT_COUNT = 35u;
constant float2 CAR_OUTLINE_POINTS[CAR_OUTLINE_POINT_COUNT] = {
    float2(-2.1600f, 0.1150f),
    float2(-2.0600f, -0.0550f),
    float2(-1.9800f, -0.1250f),
    float2(-1.8750f, -0.1750f),
    float2(-1.5950f, -0.2300f),
    float2(-1.2500f, -0.2850f),
    float2(-1.0700f, -0.3350f),
    float2(-0.8850f, -0.3950f),
    float2(-0.4900f, -0.5150f),
    float2(-0.0500f, -0.5600f),
    float2(0.1700f, -0.5600f),
    float2(0.3850f, -0.5450f),
    float2(0.7900f, -0.4800f),
    float2(1.1300f, -0.4000f),
    float2(1.2700f, -0.3600f),
    float2(1.4000f, -0.3250f),
    float2(1.6400f, -0.2850f),
    float2(1.8800f, -0.3250f),
    float2(2.0000f, -0.3750f),
    float2(2.1050f, -0.2550f),
    float2(2.1600f, -0.1100f),
    float2(2.1150f, 0.1200f),
    float2(2.0250f, 0.1600f),
    float2(1.8800f, 0.1850f),
    float2(1.4550f, 0.2000f),
    float2(0.9500f, 0.2000f),
    float2(0.6900f, 0.2000f),
    float2(0.4300f, 0.2000f),
    float2(-0.0900f, 0.2000f),
    float2(-0.6150f, 0.2000f),
    float2(-0.8850f, 0.2000f),
    float2(-1.1400f, 0.2000f),
    float2(-1.6000f, 0.1950f),
    float2(-1.9700f, 0.1750f),
    float2(-2.1100f, 0.1650f),
};

inline float clampf(float value, float minValue, float maxValue)
{
    return min(max(value, minValue), maxValue);
}

inline int clampCellCoord(int value, uint maxValue)
{
    return clamp(value, 0, (int)maxValue);
}

inline float pow7(float x)
{
    float x2 = x * x;
    float x4 = x2 * x2;
    return x4 * x2 * x;
}

inline uint hashUint32(uint x)
{
    x = x * 747796405u + 2891336453u;
    x = ((x >> ((x >> 28u) + 4u)) ^ x) * 277803737u;
    return (x >> 22u) ^ x;
}

inline float hashNoise(uint x)
{
    return float(hashUint32(x) & 0x00FFFFFFu) / float(0x00FFFFFFu);
}

inline float sdRoundedBox(float2 p, float2 halfExtents, float radius)
{
    float2 q = abs(p) - halfExtents + radius;
    float2 outside = max(q, float2(0.0f));
    return length(outside) + min(max(q.x, q.y), 0.0f) - radius;
}

inline float distanceSquaredToSegment(float2 p, float2 a, float2 b)
{
    float2 ab = b - a;
    float2 ap = p - a;
    float abLengthSquared = dot(ab, ab);
    float t = (abLengthSquared > 1e-8f)
        ? clamp(dot(ap, ab) / abLengthSquared, 0.0f, 1.0f)
        : 0.0f;
    float2 closest = a + ab * t;
    float2 delta = p - closest;
    return dot(delta, delta);
}

inline float polygonSignedDistanceScaled(constant float2 *points, uint pointCount, float2 p, float scale)
{
    float minDistanceSquared = INFINITY;
    bool inside = false;

    for (uint i = 0u, j = pointCount - 1u; i < pointCount; j = i++) {
        float2 a = points[i] * scale;
        float2 b = points[j] * scale;
        minDistanceSquared = min(minDistanceSquared, distanceSquaredToSegment(p, a, b));

        float yDelta = b.y - a.y;
        bool intersects = ((a.y > p.y) != (b.y > p.y)) &&
            (p.x < (b.x - a.x) * (p.y - a.y) / (fabs(yDelta) > 1e-6f ? yDelta : 1e-6f) + a.x);
        if (intersects) {
            inside = !inside;
        }
    }

    float distance = sqrt(max(minDistanceSquared, 0.0f));
    return inside ? -distance : distance;
}

inline float localRestDensity(constant GpuSimParams &params, float temperature)
{
    float delta = temperature - params.ambientTemperature;
    float adjusted = params.restDensity * (1.0f - params.thermalExpansion * delta);
    return max(0.35f * params.restDensity, adjusted);
}

inline float densityFloor(constant GpuSimParams &params)
{
    return (params.preset == MATERIAL_GAS)
        ? 0.10f * params.restDensity
        : 0.50f * params.restDensity;
}

inline float xsphBlend(constant GpuSimParams &params)
{
    switch (params.preset) {
        case MATERIAL_GAS:
            return 0.0f;
        case MATERIAL_WATER:
        default:
            return 0.055f;
    }
}

inline bool sceneIsWindTunnel(constant GpuSimParams &params)
{
    return params.scene == SCENE_WIND_TUNNEL;
}

inline float windTunnelProfile(constant GpuSimParams &params, float y)
{
    float top = params.boundsY + params.particleRadius;
    float bottom = params.boundsY + params.boundsHeight - params.particleRadius;
    float t = clampf((y - top) / max(bottom - top, 1e-6f), 0.0f, 1.0f);
    float centered = 2.0f * t - 1.0f;
    return max(0.12f, 1.0f - centered * centered);
}

inline void respawnWindTunnelParticle(constant GpuSimParams &params,
    uint particleIndex,
    thread float &x,
    thread float &y,
    thread float &vx,
    thread float &vy,
    thread float &temperature)
{
    float radius = params.particleRadius;
    float left = params.boundsX + radius;
    float top = params.boundsY + radius;
    float bottom = params.boundsY + params.boundsHeight - radius;
    float inletDepth = max(params.supportRadius * 1.6f, radius * 4.0f);
    float tunnelHeight = max(bottom - top, radius * 2.0f);
    uint timeKey = (uint)floor(params.simulationTime * 1536.0f);
    uint seedBase = timeKey + particleIndex * 97u;
    float baseY = top + hashNoise(seedBase + 17u) * tunnelHeight;
    float yJitter = (hashNoise(seedBase + 41u) - 0.5f) * params.supportRadius * 0.11f;
    float particleY = clampf(baseY + yJitter, top, bottom);
    float profile = windTunnelProfile(params, particleY);

    x = left + hashNoise(seedBase + 73u) * inletDepth;
    y = particleY;
    vx = params.flowTargetSpeed * profile +
        (hashNoise(seedBase + 109u) - 0.5f) * params.soundSpeed * 0.020f;
    vy = (hashNoise(seedBase + 149u) - 0.5f) * params.soundSpeed * 0.014f;
    temperature = clampf(
        params.ambientTemperature + (hashNoise(seedBase + 181u) - 0.5f) * 0.02f,
        TEMP_MIN,
        TEMP_MAX);
}

inline float2 rotateByAngle(float2 p, float cosAngle, float sinAngle)
{
    return float2(
        cosAngle * p.x - sinAngle * p.y,
        sinAngle * p.x + cosAngle * p.y
    );
}

inline float2 obstacleWorldToLocal(constant GpuSimParams &params, float x, float y)
{
    float2 p = float2(x - params.obstacleCenterX, y - params.obstacleCenterY);
    return float2(
        params.obstacleAngleCos * p.x + params.obstacleAngleSin * p.y,
        -params.obstacleAngleSin * p.x + params.obstacleAngleCos * p.y
    );
}

inline float obstacleSignedDistanceLocal(constant GpuSimParams &params, float2 p)
{
    switch (params.obstacleModel) {
        case OBSTACLE_AIRFOIL:
            return polygonSignedDistanceScaled(AIRFOIL_OUTLINE_POINTS, AIRFOIL_OUTLINE_POINT_COUNT, float2(p.x, -p.y),
                params.obstacleRadius);
        case OBSTACLE_CAR: {
            float r = params.obstacleRadius;
            return polygonSignedDistanceScaled(CAR_OUTLINE_POINTS, CAR_OUTLINE_POINT_COUNT, p, r);
        }
        case OBSTACLE_RECTANGLE:
            return sdRoundedBox(p, float2(params.obstacleRectHalfWidth, params.obstacleRectHalfHeight), 0.0f);
        case OBSTACLE_CIRCLE:
        default:
            return length(p) - params.obstacleRadius;
    }
}

inline float obstacleSignedDistance(constant GpuSimParams &params, float x, float y)
{
    return obstacleSignedDistanceLocal(params, obstacleWorldToLocal(params, x, y));
}

inline float speakerSignedDistance(constant GpuSimParams &params, float x, float y)
{
    float2 p = float2(x - params.speakerCenterX, y - params.speakerCenterY);
    float2 halfSize = float2(params.speakerHalfWidth, params.speakerHalfHeight);
    float2 q = abs(p) - halfSize;
    float2 outside = max(q, float2(0.0f));
    return length(outside) + min(max(q.x, q.y), 0.0f);
}

inline float2 speakerNormal(constant GpuSimParams &params, float x, float y)
{
    float epsilon = max(params.particleRadius * 0.65f, 0.75f);
    float dx = speakerSignedDistance(params, x + epsilon, y) - speakerSignedDistance(params, x - epsilon, y);
    float dy = speakerSignedDistance(params, x, y + epsilon) - speakerSignedDistance(params, x, y - epsilon);
    float2 normal = float2(dx, dy);
    float len = length(normal);
    if (len > 1e-6f) {
        return normal / len;
    }
    return float2(1.0f, 0.0f);
}

inline float2 obstacleNormal(constant GpuSimParams &params, float x, float y)
{
    float epsilon = max(params.particleRadius * 0.65f, 0.75f);
    float dx = obstacleSignedDistance(params, x + epsilon, y) - obstacleSignedDistance(params, x - epsilon, y);
    float dy = obstacleSignedDistance(params, x, y + epsilon) - obstacleSignedDistance(params, x, y - epsilon);
    float2 normal = float2(dx, dy);
    float len = length(normal);
    if (len > 1e-6f) {
        return normal / len;
    }

    float2 fallback = float2(x - params.obstacleCenterX, y - params.obstacleCenterY);
    float fallbackLen = length(fallback);
    if (fallbackLen > 1e-6f) {
        return fallback / fallbackLen;
    }
    return float2(1.0f, 0.0f);
}

inline void addBoundaryDensityContribution(constant GpuSimParams &params, float x, float y, thread float &density)
{
    if (params.preset == MATERIAL_GAS) {
        return;
    }

    float radius = params.particleRadius;
    float left = params.boundsX + radius;
    float right = params.boundsX + params.boundsWidth - radius;
    float top = params.boundsY + radius;
    float bottom = params.boundsY + params.boundsHeight - radius;
    float boundaryMass = params.mass * 1.15f;
    float h2 = params.supportRadiusSquared;

    float distances[4] = {
        x - left,
        right - x,
        y - top,
        bottom - y,
    };

    for (uint wall = 0; wall < 4; ++wall) {
        if (sceneIsWindTunnel(params) && wall < 2u) {
            continue;
        }
        float mirroredDistance = 2.0f * distances[wall];
        float r2 = mirroredDistance * mirroredDistance;
        if (r2 < h2) {
            float diff = h2 - r2;
            density += boundaryMass * params.densityKernel * diff * diff * diff;
        }
    }
}

inline float pressureFromState(constant GpuSimParams &params, float density, float temperature)
{
    if (params.preset == MATERIAL_GAS) {
        float normalizedTemperature = max(0.30f, temperature);
        float gasStiffness = 0.65f * params.soundSpeed * params.soundSpeed;
        return gasStiffness * density * normalizedTemperature;
    }

    float ratio = density / localRestDensity(params, temperature);
    float pressure = params.pressureStiffness * (pow7(ratio) - 1.0f);
    return max(0.0f, pressure);
}

inline void addBoundaryForceContribution(constant GpuSimParams &params,
    float x, float y, float vx, float vy,
    float rhoi, float pressureI, float temperature,
    thread float &ax, thread float &ay)
{
    float radius = params.particleRadius;
    float left = params.boundsX + radius;
    float right = params.boundsX + params.boundsWidth - radius;
    float top = params.boundsY + radius;
    float bottom = params.boundsY + params.boundsHeight - radius;
    float h = params.supportRadius;
    float h2 = params.supportRadiusSquared;

    float distances[4] = {
        x - left,
        right - x,
        y - top,
        bottom - y,
    };
    float2 normals[4] = {
        float2(1.0f, 0.0f),
        float2(-1.0f, 0.0f),
        float2(0.0f, 1.0f),
        float2(0.0f, -1.0f),
    };

    if (params.preset == MATERIAL_GAS) {
        float wallK = 2.2f * params.soundSpeed * params.soundSpeed / h;
        float wallDamp = 1.2f * params.soundSpeed / h;

        for (uint wall = 0; wall < 4; ++wall) {
            if (sceneIsWindTunnel(params) && wall < 2u) {
                continue;
            }
            float d = distances[wall];
            if (d < h) {
                float falloff = 1.0f - d / h;
                float vn = vx * normals[wall].x + vy * normals[wall].y;
                float repulse = wallK * falloff * falloff;
                float damp = -wallDamp * min(vn, 0.0f);
                ax += normals[wall].x * (repulse + damp);
                ay += normals[wall].y * (repulse + damp);
            }
        }
        return;
    }

    float boundaryMass = params.mass * 1.15f;
    float boundaryDensity = localRestDensity(params, temperature);
    float boundaryPressure = max(pressureI, 0.10f * params.pressureStiffness);

    for (uint wall = 0; wall < 4; ++wall) {
        if (sceneIsWindTunnel(params) && wall < 2u) {
            continue;
        }
        float mirroredDistance = 2.0f * distances[wall];
        float r2 = mirroredDistance * mirroredDistance;
        if (r2 > 1e-8f && r2 < h2) {
            float r = sqrt(r2);
            float influence = h - r;
            float pressureTerm = -boundaryMass *
                ((pressureI / (rhoi * rhoi)) + (boundaryPressure / (boundaryDensity * boundaryDensity))) *
                params.pressureKernelGrad * influence * influence;
            float viscosityTerm = params.kinematicViscosity * boundaryMass *
                params.viscosityKernelLap * influence / boundaryDensity;
            ax += pressureTerm * normals[wall].x;
            ay += pressureTerm * normals[wall].y;
            ax -= viscosityTerm * vx * fabs(normals[wall].x);
            ay -= viscosityTerm * vy * fabs(normals[wall].y);
        }
    }
}

inline void addSceneForceContribution(constant GpuSimParams &params, float x, float y, float vx, float vy,
    thread float &ax, thread float &ay)
{
    if (!sceneIsWindTunnel(params)) {
        return;
    }

    float profile = windTunnelProfile(params, y);
    float xNorm = clampf((x - params.boundsX) / max(params.boundsWidth, 1e-6f), 0.0f, 1.0f);
    float driveBias = 1.18f - 0.22f * xNorm;
    ax += params.flowDrive * profile * driveBias *
        (1.0f - vx / max(params.flowTargetSpeed, 1e-4f));
    ay -= params.flowDrive * 0.030f * vy;

    float signedDistance = obstacleSignedDistance(params, x, y);
    if (signedDistance < params.obstacleShell) {
        float2 normal = obstacleNormal(params, x, y);
        float falloff = clampf(1.0f - signedDistance / params.obstacleShell, 0.0f, 1.8f);
        float repulse = params.obstacleStrength * falloff * falloff;
        float vn = vx * normal.x + vy * normal.y;
        float damp = -params.obstacleDamping * min(vn, 0.0f);
        ax += normal.x * (repulse + damp);
        ay += normal.y * (repulse + damp);
    }
}

inline void addSpeakerForceContribution(constant GpuSimParams &params, float x, float y, float vx, float vy,
    thread float &ax, thread float &ay)
{
    if (params.acousticsEnabled == 0u) {
        return;
    }

    float signedDistance = speakerSignedDistance(params, x, y);
    if (signedDistance >= params.speakerShell) {
        return;
    }

    float2 normal = speakerNormal(params, x, y);
    float falloff = clampf(1.0f - signedDistance / params.speakerShell, 0.0f, 1.8f);
    float repulse = params.speakerStrength * falloff * falloff;
    float relativeVn = (vx - params.speakerVelocityX) * normal.x + (vy - params.speakerVelocityY) * normal.y;
    float damp = -params.speakerDamping * min(relativeVn, 0.0f);
    ax += normal.x * (repulse + damp);
    ay += normal.y * (repulse + damp);
}

inline void resolveObstacle(constant GpuSimParams &params, thread float &x, thread float &y, thread float &vx, thread float &vy)
{
    if (!sceneIsWindTunnel(params)) {
        return;
    }

    float surfaceOffset = params.particleRadius *
        ((params.obstacleModel == OBSTACLE_CAR) ? 1.20f : 0.70f);
    float2 normal = float2(1.0f, 0.0f);
    bool resolved = false;

    uint maxIterations = (params.obstacleModel == OBSTACLE_CAR) ? 4u : 2u;
    for (uint iteration = 0u; iteration < maxIterations; ++iteration) {
        float signedDistance = obstacleSignedDistance(params, x, y);
        if (signedDistance >= surfaceOffset) {
            break;
        }

        normal = obstacleNormal(params, x, y);
        float pushOut = (surfaceOffset - signedDistance) +
            params.particleRadius * ((params.obstacleModel == OBSTACLE_CAR) ? 0.18f : 0.08f);
        x += normal.x * pushOut;
        y += normal.y * pushOut;
        resolved = true;
    }

    if (!resolved) {
        return;
    }

    float vn = vx * normal.x + vy * normal.y;
    if (vn < 0.0f) {
        vx -= vn * normal.x;
        vy -= vn * normal.y;
    }
    vx *= 0.985f;
    vy *= 0.985f;
}

inline void resolveSpeaker(constant GpuSimParams &params, thread float &x, thread float &y, thread float &vx, thread float &vy)
{
    if (params.acousticsEnabled == 0u) {
        return;
    }

    float surfaceOffset = params.particleRadius * 0.70f;
    float signedDistance = speakerSignedDistance(params, x, y);
    if (signedDistance >= surfaceOffset) {
        return;
    }

    float2 normal = speakerNormal(params, x, y);
    float pushOut = surfaceOffset - signedDistance;
    x += normal.x * pushOut;
    y += normal.y * pushOut;

    float relativeVn = (vx - params.speakerVelocityX) * normal.x + (vy - params.speakerVelocityY) * normal.y;
    if (relativeVn < 0.0f) {
        vx -= relativeVn * normal.x;
        vy -= relativeVn * normal.y;
    }

    vx = 0.992f * vx + 0.008f * params.speakerVelocityX;
    vy = 0.992f * vy + 0.008f * params.speakerVelocityY;
}

inline void resolveBounds(constant GpuSimParams &params,
    uint particleIndex,
    thread float &x,
    thread float &y,
    thread float &vx,
    thread float &vy,
    thread float &temperature)
{
    float radius = params.particleRadius;
    float left = params.boundsX + radius;
    float right = params.boundsX + params.boundsWidth - radius;
    float top = params.boundsY + radius;
    float bottom = params.boundsY + params.boundsHeight - radius;

    if (sceneIsWindTunnel(params)) {
        if (x < left || x > right) {
            respawnWindTunnelParticle(params, particleIndex, x, y, vx, vy, temperature);
            return;
        }
    } else if (x < left) {
        x = left;
        vx = fabs(vx) * params.wallBounce;
        vy *= params.wallFriction;
    } else if (x > right) {
        x = right;
        vx = -fabs(vx) * params.wallBounce;
        vy *= params.wallFriction;
    }

    if (y < top) {
        y = top;
        vy = fabs(vy) * params.wallBounce;
        vx *= params.wallFriction;
    } else if (y > bottom) {
        y = bottom;
        vy = -fabs(vy) * params.wallBounce;
        vx *= params.wallFriction;
    }

    resolveObstacle(params, x, y, vx, vy);
}

kernel void clearCounts(device atomic_uint *cellCounts [[buffer(0)]],
    constant GpuSimParams &params [[buffer(1)]],
    uint id [[thread_position_in_grid]])
{
    if (id >= params.cellCount) {
        return;
    }

    atomic_store_explicit(&cellCounts[id], 0u, memory_order_relaxed);
}

kernel void computeParticleCells(const device float *x [[buffer(0)]],
    const device float *y [[buffer(1)]],
    device int *particleCells [[buffer(2)]],
    device atomic_uint *cellCounts [[buffer(3)]],
    constant GpuSimParams &params [[buffer(4)]],
    uint id [[thread_position_in_grid]])
{
    if (id >= params.particleCount) {
        return;
    }

    float invCellSize = 1.0f / params.supportRadius;
    int cellX = clampCellCoord((int)(x[id] * invCellSize), params.gridWidth - 1);
    int cellY = clampCellCoord((int)(y[id] * invCellSize), params.gridHeight - 1);
    int cellIndex = cellY * (int)params.gridWidth + cellX;
    particleCells[id] = cellIndex;
    atomic_fetch_add_explicit(&cellCounts[cellIndex], 1u, memory_order_relaxed);
}

kernel void scatterSortedIndices(const device int *particleCells [[buffer(0)]],
    device atomic_uint *cellOffsets [[buffer(1)]],
    device int *sortedIndices [[buffer(2)]],
    constant GpuSimParams &params [[buffer(3)]],
    uint id [[thread_position_in_grid]])
{
    if (id >= params.particleCount) {
        return;
    }

    uint slot = atomic_fetch_add_explicit(&cellOffsets[particleCells[id]], 1u, memory_order_relaxed);
    sortedIndices[slot] = (int)id;
}

kernel void gatherSortedData(const device int *sortedIndices [[buffer(0)]],
    const device int *particleCells [[buffer(1)]],
    const device float *x [[buffer(2)]],
    const device float *y [[buffer(3)]],
    const device float *vx [[buffer(4)]],
    const device float *vy [[buffer(5)]],
    const device float *temperature [[buffer(6)]],
    device float *sortedX [[buffer(7)]],
    device float *sortedY [[buffer(8)]],
    device float *sortedVX [[buffer(9)]],
    device float *sortedVY [[buffer(10)]],
    device float *sortedTemperature [[buffer(11)]],
    device int *sortedCellIndices [[buffer(12)]],
    constant GpuSimParams &params [[buffer(13)]],
    uint slot [[thread_position_in_grid]])
{
    if (slot >= params.particleCount) {
        return;
    }

    int particleIndex = sortedIndices[slot];
    sortedX[slot] = x[particleIndex];
    sortedY[slot] = y[particleIndex];
    sortedVX[slot] = vx[particleIndex];
    sortedVY[slot] = vy[particleIndex];
    sortedTemperature[slot] = temperature[particleIndex];
    sortedCellIndices[slot] = particleCells[particleIndex];
}

kernel void computeDensityPressure(const device float *sortedX [[buffer(0)]],
    const device float *sortedY [[buffer(1)]],
    const device float *sortedTemperature [[buffer(2)]],
    const device int *sortedCellIndices [[buffer(3)]],
    const device int *cellStarts [[buffer(4)]],
    const device int *cellNeighborCounts [[buffer(5)]],
    const device int *cellNeighbors [[buffer(6)]],
    const device int *sortedIndices [[buffer(7)]],
    device float *density [[buffer(8)]],
    device float *pressure [[buffer(9)]],
    device float *sortedDensity [[buffer(10)]],
    device float *sortedPressure [[buffer(11)]],
    constant GpuSimParams &params [[buffer(12)]],
    uint slot [[thread_position_in_grid]])
{
    if (slot >= params.particleCount) {
        return;
    }

    float xi = sortedX[slot];
    float yi = sortedY[slot];
    int baseCell = sortedCellIndices[slot];
    int neighborBase = baseCell * 9;
    int neighborCount = cellNeighborCounts[baseCell];
    float rho = 0.0f;

    for (int neighbor = 0; neighbor < neighborCount; ++neighbor) {
        int cellIndex = cellNeighbors[neighborBase + neighbor];
        int start = cellStarts[cellIndex];
        int end = cellStarts[cellIndex + 1];

        for (int cursor = start; cursor < end; ++cursor) {
            float dx = xi - sortedX[cursor];
            float dy = yi - sortedY[cursor];
            float r2 = dx * dx + dy * dy;
            if (r2 < params.supportRadiusSquared) {
                float diff = params.supportRadiusSquared - r2;
                rho += params.mass * params.densityKernel * diff * diff * diff;
            }
        }
    }

    addBoundaryDensityContribution(params, xi, yi, rho);

    float clampedDensity = max(densityFloor(params), rho);
    int particleIndex = sortedIndices[slot];
    float particlePressure = pressureFromState(params, clampedDensity, sortedTemperature[slot]);
    density[particleIndex] = clampedDensity;
    pressure[particleIndex] = particlePressure;
    sortedDensity[slot] = clampedDensity;
    sortedPressure[slot] = particlePressure;
}

kernel void computeForces(const device int *sortedIndices [[buffer(0)]],
    const device float *sortedX [[buffer(1)]],
    const device float *sortedY [[buffer(2)]],
    const device float *sortedVX [[buffer(3)]],
    const device float *sortedVY [[buffer(4)]],
    const device float *sortedTemperature [[buffer(5)]],
    const device float *sortedDensity [[buffer(6)]],
    const device float *sortedPressure [[buffer(7)]],
    const device int *sortedCellIndices [[buffer(8)]],
    const device int *cellStarts [[buffer(9)]],
    const device int *cellNeighborCounts [[buffer(10)]],
    const device int *cellNeighbors [[buffer(11)]],
    device float *axBuffer [[buffer(12)]],
    device float *ayBuffer [[buffer(13)]],
    device float *temperatureRateBuffer [[buffer(14)]],
    device float *xsphVXBuffer [[buffer(15)]],
    device float *xsphVYBuffer [[buffer(16)]],
    constant GpuSimParams &params [[buffer(17)]],
    uint slot [[thread_position_in_grid]])
{
    if (slot >= params.particleCount) {
        return;
    }

    float h = params.supportRadius;
    float h2 = params.supportRadiusSquared;
    float xi = sortedX[slot];
    float yi = sortedY[slot];
    float vxi = sortedVX[slot];
    float vyi = sortedVY[slot];
    float rhoi = sortedDensity[slot];
    float pressureI = sortedPressure[slot];
    float temperatureI = sortedTemperature[slot];
    int baseCell = sortedCellIndices[slot];
    int neighborBase = baseCell * 9;
    int neighborCount = cellNeighborCounts[baseCell];

    float ax = -params.globalDrag * vxi;
    float ay = params.gravity - params.buoyancy * (temperatureI - params.ambientTemperature) - params.globalDrag * vyi;
    float temperatureRate = 0.0f;
    float xsphVX = 0.0f;
    float xsphVY = 0.0f;

    for (int neighbor = 0; neighbor < neighborCount; ++neighbor) {
        int cellIndex = cellNeighbors[neighborBase + neighbor];
        int start = cellStarts[cellIndex];
        int end = cellStarts[cellIndex + 1];

        for (int cursor = start; cursor < end; ++cursor) {
            if ((int)slot == cursor) {
                continue;
            }

            float dx = xi - sortedX[cursor];
            float dy = yi - sortedY[cursor];
            float r2 = dx * dx + dy * dy;

            if (r2 > 1e-8f && r2 < h2) {
                float r = sqrt(r2);
                float influence = h - r;
                float invR = 1.0f / r;
                float neighborDensity = sortedDensity[cursor];
                float pressureTerm = -params.mass *
                    ((pressureI / (rhoi * rhoi)) +
                     (sortedPressure[cursor] / (neighborDensity * neighborDensity))) *
                    params.pressureKernelGrad * influence * influence;
                float viscosityTerm = params.kinematicViscosity * params.mass *
                    params.viscosityKernelLap * influence / neighborDensity;
                float densityKernelValue = params.densityKernel * (h2 - r2) * (h2 - r2) * (h2 - r2);

                ax += pressureTerm * dx * invR;
                ay += pressureTerm * dy * invR;

                ax += viscosityTerm * (sortedVX[cursor] - vxi);
                ay += viscosityTerm * (sortedVY[cursor] - vyi);

                temperatureRate += params.temperatureDiffusion * params.mass *
                    (sortedTemperature[cursor] - temperatureI) *
                    params.viscosityKernelLap * influence / (rhoi * neighborDensity);

                xsphVX += params.mass * (sortedVX[cursor] - vxi) * densityKernelValue / neighborDensity;
                xsphVY += params.mass * (sortedVY[cursor] - vyi) * densityKernelValue / neighborDensity;
            }
        }
    }

    addBoundaryForceContribution(params, xi, yi, vxi, vyi, rhoi, pressureI, temperatureI, ax, ay);
    addSceneForceContribution(params, xi, yi, vxi, vyi, ax, ay);
    addSpeakerForceContribution(params, xi, yi, vxi, vyi, ax, ay);

    if (params.mouseActive != 0u) {
        float dx = xi - params.mouseX;
        float dy = yi - params.mouseY;
        float r2 = dx * dx + dy * dy;
        if (r2 > 1e-8f && r2 < params.mouseRadiusSquared) {
            float r = sqrt(r2);
            float falloff = 1.0f - (r / params.mouseRadius);
            float impulse = params.mouseStrength * falloff * falloff;
            float invR = 1.0f / r;
            ax += dx * invR * impulse;
            ay += dy * invR * impulse;
        }
    }

    int particleIndex = sortedIndices[slot];
    axBuffer[particleIndex] = ax;
    ayBuffer[particleIndex] = ay;
    temperatureRateBuffer[particleIndex] = temperatureRate;
    xsphVXBuffer[particleIndex] = xsphVX;
    xsphVYBuffer[particleIndex] = xsphVY;
}

kernel void integrateParticles(device float *x [[buffer(0)]],
    device float *y [[buffer(1)]],
    device float *vx [[buffer(2)]],
    device float *vy [[buffer(3)]],
    const device float *ax [[buffer(4)]],
    const device float *ay [[buffer(5)]],
    device float *temperature [[buffer(6)]],
    const device float *temperatureRate [[buffer(7)]],
    const device float *xsphVX [[buffer(8)]],
    const device float *xsphVY [[buffer(9)]],
    constant GpuSimParams &params [[buffer(10)]],
    uint id [[thread_position_in_grid]])
{
    if (id >= params.particleCount) {
        return;
    }

    float vxi = vx[id] + ax[id] * params.substepDt;
    float vyi = vy[id] + ay[id] * params.substepDt;

    float maxSpeed = (params.preset == MATERIAL_GAS)
        ? params.soundSpeed * 5.0f
        : params.soundSpeed * 2.5f;
    float speed2 = vxi * vxi + vyi * vyi;
    if (speed2 > maxSpeed * maxSpeed) {
        float scale = maxSpeed / sqrt(speed2);
        vxi *= scale;
        vyi *= scale;
    }

    float advVx = vxi + xsphBlend(params) * xsphVX[id];
    float advVy = vyi + xsphBlend(params) * xsphVY[id];
    float px = x[id] + advVx * params.substepDt;
    float py = y[id] + advVy * params.substepDt;
    float nextTemperature = clampf(temperature[id] + temperatureRate[id] * params.substepDt, TEMP_MIN, TEMP_MAX);

    if (params.preset != MATERIAL_GAS) {
        vxi *= 0.9992f;
        vyi *= 0.9992f;
    }

    resolveBounds(params, id, px, py, vxi, vyi, nextTemperature);
    resolveSpeaker(params, px, py, vxi, vyi);

    x[id] = px;
    y[id] = py;
    vx[id] = vxi;
    vy[id] = vyi;
    temperature[id] = nextTemperature;
}
