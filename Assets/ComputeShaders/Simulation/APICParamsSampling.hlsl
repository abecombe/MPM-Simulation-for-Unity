#ifndef _CS_PIC_APICPARAMSSAMPLING_HLSL
#define _CS_PIC_APICPARAMSSAMPLING_HLSL

// sample grid params using spline interpolation
#define S0(S) (0.5f * (0.5f - S) * (0.5f - S))
#define S1(S) (0.75f - S * S)
#define S2(S) (0.5f * (0.5f + S) * (0.5f + S))

#define INTERPOLATE_X(SUM, C_INDEX, P_INDEX, S, AXIS) {\
const int3 c_index_0 = C_INDEX + int3(-1, 0, 0);\
const int3 c_index_1 = C_INDEX + int3(0, 0, 0);\
const int3 c_index_2 = C_INDEX + int3(1, 0, 0);\
const uint c_id_0 = CellIndexToCellID(c_index_0);\
const uint c_id_1 = CellIndexToCellID(c_index_1);\
const uint c_id_2 = CellIndexToCellID(c_index_2);\
const float4 c_pvec_0 = float4((c_index_0 - P_INDEX - S) * _GridSpacing, 1.0f);\
const float4 c_pvec_1 = float4((c_index_1 - P_INDEX - S) * _GridSpacing, 1.0f);\
const float4 c_pvec_2 = float4((c_index_2 - P_INDEX - S) * _GridSpacing, 1.0f);\
SUM += _GridVelocityBufferRead[c_id_0].AXIS * S0(S.x) * c_pvec_0;\
SUM += _GridVelocityBufferRead[c_id_1].AXIS * S1(S.x) * c_pvec_1;\
SUM += _GridVelocityBufferRead[c_id_2].AXIS * S2(S.x) * c_pvec_2;\
}\

#define INTERPOLATE_Y(SUM, C_INDEX, P_INDEX, S, AXIS) {\
float4 sum_x_0 = (float4)0;\
float4 sum_x_1 = (float4)0;\
float4 sum_x_2 = (float4)0;\
INTERPOLATE_X(sum_x_0, C_INDEX + int3(0, -1, 0), P_INDEX, S, AXIS)\
INTERPOLATE_X(sum_x_1, C_INDEX + int3(0, 0, 0), P_INDEX, S, AXIS)\
INTERPOLATE_X(sum_x_2, C_INDEX + int3(0, 1, 0), P_INDEX, S, AXIS)\
SUM += sum_x_0 * S0(S.y);\
SUM += sum_x_1 * S1(S.y);\
SUM += sum_x_2 * S2(S.y);\
}\

#define INTERPOLATE_Z(SUM, C_INDEX, P_INDEX, S, AXIS) {\
float4 sum_y_0 = (float4)0;\
float4 sum_y_1 = (float4)0;\
float4 sum_y_2 = (float4)0;\
INTERPOLATE_Y(sum_y_0, C_INDEX + int3(0, 0, -1), P_INDEX, S, AXIS)\
INTERPOLATE_Y(sum_y_1, C_INDEX + int3(0, 0, 0), P_INDEX, S, AXIS)\
INTERPOLATE_Y(sum_y_2, C_INDEX + int3(0, 0, 1), P_INDEX, S, AXIS)\
SUM += sum_y_0 * S0(S.z);\
SUM += sum_y_1 * S1(S.z);\
SUM += sum_y_2 * S2(S.z);\
}\

#define INTERPOLATE(SUM, C_INDEX, S, AXIS) \
INTERPOLATE_Z(SUM, C_INDEX, C_INDEX, S, AXIS)\

#define SAMPLE_APIC_PARAMS(PARAMS, G_POS, AXIS) \
const int3 c_index = round(G_POS - 0.5f);\
const float3 s = (G_POS - 0.5f) - round(G_POS - 0.5f);\
INTERPOLATE(PARAMS, c_index, s, AXIS)\
const float invD = 4.0f / _GridSpacing / _GridSpacing;\
PARAMS.xyz *= invD;\

#define SAMPLE_APIC_PARAMS_X(PARAMS, POS) {\
const float3 g_pos = (POS - _GridMin) / _GridSpacing;\
SAMPLE_APIC_PARAMS(PARAMS, g_pos, x)\
}\

#define SAMPLE_APIC_PARAMS_Y(PARAMS, POS) {\
const float3 g_pos = (POS - _GridMin) / _GridSpacing;\
SAMPLE_APIC_PARAMS(PARAMS, g_pos, y)\
}\

#define SAMPLE_APIC_PARAMS_Z(PARAMS, POS) {\
const float3 g_pos = (POS - _GridMin) / _GridSpacing;\
SAMPLE_APIC_PARAMS(PARAMS, g_pos, z)\
}\

#define SAMPLE_APIC_PARAMS_MASTER(PARTICLE) \
float4 params_x = (float4)0;\
float4 params_y = (float4)0;\
float4 params_z = (float4)0;\
SAMPLE_APIC_PARAMS_X(params_x, PARTICLE.position)\
SAMPLE_APIC_PARAMS_Y(params_y, PARTICLE.position)\
SAMPLE_APIC_PARAMS_Z(params_z, PARTICLE.position)\
PARTICLE.C._m00_m01_m02 = params_x.xyz;\
PARTICLE.C._m10_m11_m12 = params_y.xyz;\
PARTICLE.C._m20_m21_m22 = params_z.xyz;\
PARTICLE.velocity.x = params_x.w;\
PARTICLE.velocity.y = params_y.w;\
PARTICLE.velocity.z = params_z.w;\


#endif /* _CS_PIC_APICPARAMSSAMPLING_HLSL */