#ifndef CS_SIMULATION_MPM_PARTICLE_HLSL
#define CS_SIMULATION_MPM_PARTICLE_HLSL

struct Particle
{
    float3 position;
    float3 velocity;
    float3x3 C;
    float mass;
};


#endif /* CS_SIMULATION_MPM_PARTICLE_HLSL */