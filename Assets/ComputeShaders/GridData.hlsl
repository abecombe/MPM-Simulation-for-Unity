#ifndef CS_GRID_DATA_HLSL
#define CS_GRID_DATA_HLSL

float3 _GridMin;
float3 _GridMax;
int3 _GridSize;
float _GridSpacing;
float _GridInvSpacing;
float _InvD; // 4.0f / (_GridSpacing * _GridSpacing)


#endif /* CS_GRID_DATA_HLSL */