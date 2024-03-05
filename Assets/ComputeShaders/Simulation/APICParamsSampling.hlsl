#ifndef CS_SIMULATION_APIC_PARAMS_SAMPLING_HLSL
#define CS_SIMULATION_APIC_PARAMS_SAMPLING_HLSL

#include "MPMParticle.hlsl"

#include "../GridData.hlsl"
#include "../GridHelper.hlsl"

// sample grid params using spline interpolation
#define S0(S) (0.5f * (0.5f - S) * (0.5f - S))
#define S1(S) (0.75f - S * S)
#define S2(S) (0.5f * (0.5f + S) * (0.5f + S))

#define GET_VALUE_OF_EACH_CELL(SUM, G_POS, C_INDEX, S, WEIGHT, GRID_VELOCITY_BUFFER) {\
const uint c_id = CellIndexToCellID(C_INDEX);\
const float1x4 c_pvec = (float1x4)(CellIndexToWorldPos(C_INDEX) - G_POS, 1.0f);\
SUM += mul((float3x1)GRID_VELOCITY_BUFFER[c_id], c_pvec) * WEIGHT;\
}\

#define INTERPOLATE_X(SUM, G_POS, C_INDEX, S, WEIGHT, GRID_VELOCITY_BUFFER) {\
float3x4 sum_x = 0;\
GET_VALUE_OF_EACH_CELL(sum_x, G_POS, C_INDEX + int3(-1, 0, 0), S, S0(S.x), GRID_VELOCITY_BUFFER)\
GET_VALUE_OF_EACH_CELL(sum_x, G_POS, C_INDEX + int3(0, 0, 0), S, S1(S.x), GRID_VELOCITY_BUFFER)\
GET_VALUE_OF_EACH_CELL(sum_x, G_POS, C_INDEX + int3(1, 0, 0), S, S2(S.x), GRID_VELOCITY_BUFFER)\
sum_x *= WEIGHT;\
SUM += sum_x;\
}\

#define INTERPOLATE_Y(SUM, G_POS, C_INDEX, S, WEIGHT, GRID_VELOCITY_BUFFER) {\
float3x4 sum_y = 0;\
INTERPOLATE_X(sum_y, G_POS, C_INDEX + int3(0, -1, 0), S, S0(S.y), GRID_VELOCITY_BUFFER)\
INTERPOLATE_X(sum_y, G_POS, C_INDEX + int3(0, 0, 0), S, S1(S.y), GRID_VELOCITY_BUFFER)\
INTERPOLATE_X(sum_y, G_POS, C_INDEX + int3(0, 1, 0), S, S2(S.y), GRID_VELOCITY_BUFFER)\
sum_y *= WEIGHT;\
SUM += sum_y;\
}\

#define INTERPOLATE_Z(SUM, G_POS, C_INDEX, S, GRID_VELOCITY_BUFFER) {\
INTERPOLATE_Y(SUM, G_POS, C_INDEX + int3(0, 0, -1), S, S0(S.z), GRID_VELOCITY_BUFFER)\
INTERPOLATE_Y(SUM, G_POS, C_INDEX + int3(0, 0, 0), S, S1(S.z), GRID_VELOCITY_BUFFER)\
INTERPOLATE_Y(SUM, G_POS, C_INDEX + int3(0, 0, 1), S, S2(S.z), GRID_VELOCITY_BUFFER)\
}\

#define INTERPOLATE(B_VELOCITY, G_POS, C_INDEX, S, GRID_VELOCITY_BUFFER) \
INTERPOLATE_Z(B_VELOCITY, G_POS, C_INDEX, S, GRID_VELOCITY_BUFFER)\

#define SAMPLE_APIC_PARAMS(PARTICLE, GRID_VELOCITY_BUFFER) \
float3x4 b_velocity = 0;\
const float3 g_pos = WorldPosToGridPos(PARTICLE.position);\
const int3 c_index = round(g_pos - 0.5f);\
const float3 s = (g_pos - 0.5f) - round(g_pos - 0.5f);\
INTERPOLATE(b_velocity, g_pos, c_index, s, GRID_VELOCITY_BUFFER)\
float3x3 B;\
B._m00_m01_m02 = b_velocity._m00_m01_m02;\
B._m10_m11_m12 = b_velocity._m10_m11_m12;\
B._m20_m21_m22 = b_velocity._m20_m21_m22;\
PARTICLE.C = B * (4.0f * _GridInvSpacing * _GridInvSpacing);\
PARTICLE.velocity = b_velocity._m03_m13_m23;\

inline void SampleAPICParams(inout Particle particle, StructuredBuffer<float3> grid_velocity_buffer)
{
    SAMPLE_APIC_PARAMS(particle, grid_velocity_buffer)
}

inline void SampleAPICParams(inout Particle particle, RWStructuredBuffer<float3> grid_velocity_buffer)
{
    SAMPLE_APIC_PARAMS(particle, grid_velocity_buffer)
}

#endif /* CS_SIMULATION_APIC_PARAMS_SAMPLING_HLSL */