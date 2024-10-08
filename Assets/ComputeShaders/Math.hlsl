﻿#ifndef CS_SIMULATION_MATH_HLSL
#define CS_SIMULATION_MATH_HLSL

static const float2x2 Identity2x2 = float2x2(1,0,0,1);
static const float3x3 Identity3x3 = float3x3(1,0,0,0,1,0,0,0,1);

float2x2 inverse(float2x2 m)
{
    const float det = determinant(m);
    return
        det == 0.0f ? Identity2x2 :
        1.0f / det *
            float2x2(
                 m[1][1], -m[0][1],
                -m[1][0],  m[0][0]
            );
}

float3x3 inverse(float3x3 m)
{
    const float det = determinant(m);
    return
        det == 0.0f ? Identity3x3 :
        1.0f / det *
            float3x3(
                  m[1][1] * m[2][2] - m[1][2] * m[2][1],
                -(m[0][1] * m[2][2] - m[0][2] * m[2][1]),
                  m[0][1] * m[1][2] - m[0][2] * m[1][1],

                -(m[1][0] * m[2][2] - m[1][2] * m[2][0]),
                  m[0][0] * m[2][2] - m[0][2] * m[2][0],
                -(m[0][0] * m[1][2] - m[0][2] * m[1][0]),

                  m[1][0] * m[2][1] - m[1][1] * m[2][0],
                -(m[0][0] * m[2][1] - m[0][1] * m[2][0]),
                  m[0][0] * m[1][1] - m[0][1] * m[1][0]
            );
}


#endif /* CS_SIMULATION_MATH_HLSL */