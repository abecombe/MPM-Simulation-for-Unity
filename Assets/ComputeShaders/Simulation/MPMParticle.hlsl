#ifndef CS_SIMULATION_MPM_PARTICLE_HLSL
#define CS_SIMULATION_MPM_PARTICLE_HLSL

#include "../Bit.hlsl"

static const uint ParticleTypeMask      = 0x00000007;
static const uint ParticleTypeMaskShift = 0;

static const uint PT_NONE    = 0;
static const uint PT_FLUID   = 1;
static const uint PT_ELASTIC = 2;

struct Particle
{
    float3 position;
    float3 velocity;
    float3x3 F; // deformation gradient
    float3x3 C; // affine momentum matrix
    float mass;
    float volume0; // initial volume
    uint flags;

    uint Type()
    {
        return GET_VALUE(flags, ParticleTypeMask, ParticleTypeMaskShift);
    }
    void SetType(uint type)
    {
        SET_VALUE(flags, type, ParticleTypeMask, ParticleTypeMaskShift);
    }
};


#endif /* CS_SIMULATION_MPM_PARTICLE_HLSL */