#ifndef CS_SIMULATION_MPM_PARTICLE_HLSL
#define CS_SIMULATION_MPM_PARTICLE_HLSL

struct Particle
{
    float3 position;
    float3 velocity;
    float3x3 F; // deformation gradient
    float3x3 C; // affine momentum matrix
    float mass;
    float volume0; // initial volume
};


#endif /* CS_SIMULATION_MPM_PARTICLE_HLSL */