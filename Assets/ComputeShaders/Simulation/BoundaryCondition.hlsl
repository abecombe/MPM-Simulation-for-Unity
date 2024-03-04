#ifndef CS_SIMULATION_BOUNDARY_HLSL
#define CS_SIMULATION_BOUNDARY_HLSL

static const float POSITION_EPSILON = 1e-4;

inline void ClampPosition(inout float3 position, float3 grid_min, float3 grid_max, float grid_spacing)
{
    position = clamp(position, grid_min + grid_spacing, grid_max - grid_spacing);
}

inline void EnforceBoundaryCondition(inout float3 velocity, int3 c_index, int3 grid_size)
{
    velocity = (c_index >= 2 || c_index <= grid_size - 3) ? velocity : 0.0f;
}


#endif /* CS_SIMULATION_BOUNDARY_HLSL */