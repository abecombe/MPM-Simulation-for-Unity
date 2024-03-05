using System;
using Abecombe.FPSUtil;
using Abecombe.GPUUtil;
using RosettaUI;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.VFX;

public class MPMSimulation : MonoBehaviour, IDisposable
{
    #region Structs & Enums
    private struct Particle
    {
        public float3 Position;
        public float3 Velocity;
        public float3x3 C; // affine momentum matrix
        public float Mass;
    }

    private enum Quality
    {
        Low,
        Medium,
        High,
        Ultra
    }
    #endregion

    #region Properties
    private const float DeltaTime = 1f / 60f;

    private const float NumParticleInACell = 8f;
    private int NumParticles => (int)(ParticleInitGridSize.x * ParticleInitGridSize.y * ParticleInitGridSize.z * NumParticleInACell);
    private int NumGrids => GridSize.x * GridSize.y * GridSize.z;

    // Quality
    [SerializeField] private Quality _quality = Quality.Medium;
    private readonly float[] _qualityToGridSpacing = { 0.5f, 0.4f, 0.3f, 0.2f };

    // Particle Params
    [SerializeField] private float3 _particleInitRangeMin;
    [SerializeField] private float3 _particleInitRangeMax;
    private float3 ParticleInitRangeMin => _particleInitRangeMin;
    private float3 ParticleInitRangeMax => _particleInitRangeMax;
    private float3 ParticleInitGridSize => (ParticleInitRangeMax - ParticleInitRangeMin) / GridSpacing;

    // Grid Params
    private float3 TempSimulationSize => transform.localScale;
    private float3 SimulationSize => (float3)GridSize * GridSpacing;
    private float3 GridMin => -SimulationSize / 2f;
    private float3 GridMax => SimulationSize / 2f;
    private int3 GridSize => (int3)math.ceil(TempSimulationSize / GridSpacing);
    private float GridSpacing => _qualityToGridSpacing[(int)_quality];
    private float GridInvSpacing => 1f / GridSpacing;

    // Particle Data Buffers
    private GPUDoubleBuffer<Particle> _particleBuffer = new();
    private GPUBuffer<float3x3> _particleStressForceBuffer = new();
    private GPUBuffer<float4> _particleRenderingBuffer = new(); // xyz: position, w: speed

    // Grid Data Buffers
    private GPUBuffer<uint2> _gridParticleIDBuffer = new();
    private GPUBuffer<float3> _gridVelocityBuffer = new();
    private GPUBuffer<float> _gridMassBuffer = new();

    // Compute Shaders
    private GPUComputeShader _particleInitCs;
    private GPUComputeShader _particleToGridCs;
    private GPUComputeShader _externalForceCs;
    private GPUComputeShader _boundaryCs;
    private GPUComputeShader _gridToParticleCs;
    private GPUComputeShader _particleAdvectionCs;
    private GPUComputeShader _renderingCs;

    // Grid Sort Helper
    private GridSortHelper<Particle> _gridSortHelper = new();

    // Simulation Params
    [SerializeField] private float _particleMass = 1.0f;
    [SerializeField] private Vector3 _gravity = Vector3.down * 9.8f;
    [SerializeField] private float _eosStiffness = 10.0f;
    [SerializeField] private float _eosPower = 4.0f;
    [SerializeField] private float _dynamicViscosity = 0.1f;
    [SerializeField] [Range(0f, 5f)] private float _mouseForce = 1.32f;
    [SerializeField] [Range(0f, 5f)] private float _mouseForceRange = 2.25f;

    // RosettaUI
    [SerializeField] private RosettaUIRoot _rosettaUIRoot;
    [SerializeField] private KeyCode _toggleUIKey = KeyCode.Tab;
    [SerializeField] private bool _showFps = true;
    #endregion

    #region Initialize Functions
    private void InitComputeShaders()
    {
        _particleInitCs = new GPUComputeShader("ParticleInitCS");
        _particleToGridCs = new GPUComputeShader("ParticleToGridCS");
        _externalForceCs = new GPUComputeShader("ExternalForceCS");
        _boundaryCs = new GPUComputeShader("BoundaryCS");
        _gridToParticleCs = new GPUComputeShader("GridToParticleCS");
        _particleAdvectionCs = new GPUComputeShader("ParticleAdvectionCS");
        _renderingCs = new GPUComputeShader("RenderingCS");
    }

    private void InitParticleBuffers()
    {
        _particleBuffer.Init(NumParticles);
        _particleStressForceBuffer.Init(NumParticles);
        _particleRenderingBuffer.Init(NumParticles);

        // init particle
        var cs = _particleInitCs;
        var k = cs.FindKernel("InitParticle");
        SetConstants(cs);
        cs.SetFloat("_ParticleMass", _particleMass);
        cs.SetVector("_ParticleInitRangeMin", ParticleInitRangeMin);
        cs.SetVector("_ParticleInitRangeMax", ParticleInitRangeMax);
        k.SetBuffer("_ParticleBufferWrite", _particleBuffer.Read);
        k.Dispatch(NumParticles);

        // init vfx
        var vfx = FindObjectOfType<VisualEffect>();
        vfx.Reinit();
        vfx.SetFloat("NumInstance", NumParticles);
        vfx.SetFloat("Size", GridSpacing * 0.8f);
        vfx.SetGraphicsBuffer("ParticleBuffer", _particleRenderingBuffer);
    }

    private void InitGridBuffers()
    {
        _gridParticleIDBuffer.Init(NumGrids);
        _gridVelocityBuffer.Init(NumGrids);
        _gridMassBuffer.Init(NumGrids);
    }

    private void InitGPUBuffers()
    {
        InitParticleBuffers();
        InitGridBuffers();
    }
    #endregion

    #region Update Functions
    private void SetConstants(GPUComputeShader cs)
    {
        cs.SetFloat("_DeltaTime", DeltaTime);

        cs.SetVector("_GridMin", GridMin);
        cs.SetVector("_GridMax", GridMax);
        cs.SetInts("_GridSize", GridSize);
        cs.SetFloat("_GridSpacing", GridSpacing);
        cs.SetFloat("_GridInvSpacing", GridInvSpacing);
    }

    // transferring velocity from particle to grid
    private void DispatchParticleToGrid()
    {
        _gridSortHelper.Sort(_particleBuffer, _gridParticleIDBuffer, GridMin, GridMax, GridSize, GridSpacing);

        var cs = _particleToGridCs;

        SetConstants(cs);

        var k = cs.FindKernel("ParticleToGrid1");
        k.SetBuffer("_ParticleBufferRead", _particleBuffer.Read);
        k.SetBuffer("_GridParticleIDBufferRead", _gridParticleIDBuffer);
        k.SetBuffer("_GridVelocityBufferWrite", _gridVelocityBuffer);
        k.SetBuffer("_GridMassBufferWrite", _gridMassBuffer);
        k.Dispatch(NumGrids);

        k = cs.FindKernel("ParticleToGrid2");
        cs.SetFloat("_EosStiffness", _eosStiffness);
        cs.SetFloat("_EosPower", _eosPower);
        cs.SetFloat("_InvRestDensity", 1f / (NumParticleInACell * _particleMass / math.pow(GridSpacing, 3)));
        cs.SetFloat("_DynamicViscosity", _dynamicViscosity);
        k.SetBuffer("_ParticleBufferRead", _particleBuffer.Read);
        k.SetBuffer("_GridMassBufferRead", _gridMassBuffer);
        k.SetBuffer("_ParticleStressForceBufferWrite", _particleStressForceBuffer);
        k.Dispatch(NumParticles);

        k = cs.FindKernel("ParticleToGrid3");
        k.SetBuffer("_ParticleBufferRead", _particleBuffer.Read);
        k.SetBuffer("_ParticleStressForceBufferRead", _particleStressForceBuffer);
        k.SetBuffer("_GridParticleIDBufferRead", _gridParticleIDBuffer);
        k.SetBuffer("_GridVelocityBufferRW", _gridVelocityBuffer);
        k.SetBuffer("_GridMassBufferRead", _gridMassBuffer);
        k.Dispatch(NumGrids);
    }

    // external force term with reference to https://github.com/dli/fluid
    private float2 _lastMousePlane = float2.zero;
    private void DispatchExternalForce()
    {
        var cs = _externalForceCs;
        var k = cs.FindKernel("AddExternalForce");

        SetConstants(cs);

        k.SetBuffer("_GridVelocityBufferRW", _gridVelocityBuffer);

        cs.SetVector("_Gravity", _gravity);

        var cam = Camera.main;
        var mouseRay = cam.ScreenPointToRay(Input.mousePosition);
        cs.SetVector("_RayOrigin", mouseRay.origin);
        cs.SetVector("_RayDirection", mouseRay.direction);

        var height = Mathf.Tan(cam.fieldOfView * 0.5f * Mathf.Deg2Rad) * 2f;
        var width = height * Screen.width / Screen.height;
        var mousePlane = ((float3)Input.mousePosition).xy / new float2(Screen.width, Screen.height) - 0.5f;
        mousePlane *= new float2(width, height);
        mousePlane *= cam.GetComponent<OrbitCamera>().Distance;
        var cameraViewMatrix = cam.worldToCameraMatrix;
        var cameraRight = new float3(cameraViewMatrix[0], cameraViewMatrix[4], cameraViewMatrix[8]);
        var cameraUp = new float3(cameraViewMatrix[1], cameraViewMatrix[5], cameraViewMatrix[9]);
        var mouseVelocity = (mousePlane - _lastMousePlane) / DeltaTime;
        if (Input.GetMouseButton(0) || Input.GetMouseButton(1) || Input.GetMouseButton(2) || Time.frameCount <= 1)
            mouseVelocity = float2.zero;
        _lastMousePlane = mousePlane;
        var mouseAxisVelocity = mouseVelocity.x * cameraRight + mouseVelocity.y * cameraUp;
        cs.SetVector("_MouseForceParameter", new float4(mouseAxisVelocity * _mouseForce, _mouseForceRange));

        k.Dispatch(NumGrids);
    }

    // enforcing boundary condition
    private void DispatchBoundaryCondition()
    {
        var cs = _boundaryCs;
        var k = cs.FindKernel("EnforceBoundaryCondition");

        SetConstants(cs);

        k.SetBuffer("_GridVelocityBufferRW", _gridVelocityBuffer);

        k.Dispatch(NumGrids);
    }

    // transferring velocity from grid to particle
    private void DispatchGridToParticle()
    {
        var cs = _gridToParticleCs;
        var k = cs.FindKernel("GridToParticle");

        SetConstants(cs);

        k.SetBuffer("_ParticleBufferRW", _particleBuffer.Read);
        k.SetBuffer("_GridVelocityBufferRead", _gridVelocityBuffer);

        k.Dispatch(NumParticles);
    }

    // particle advection term
    private void DispatchAdvection()
    {
        var cs = _particleAdvectionCs;
        var k = cs.FindKernel("Advect");

        SetConstants(cs);

        k.SetBuffer("_ParticleBufferRW", _particleBuffer.Read);

        k.Dispatch(NumParticles);
    }

    private void RenderParticles()
    {
        var cs = _renderingCs;
        var k = cs.FindKernel("PrepareRendering");

        k.SetBuffer("_ParticleBufferRead", _particleBuffer.Read);
        k.SetBuffer("_ParticleRenderingBufferWrite", _particleRenderingBuffer);

        k.Dispatch(NumParticles);
    }
    #endregion

    #region Release Buffers
    public void Dispose()
    {
        _particleBuffer.Dispose();
        _particleStressForceBuffer.Dispose();
        _particleRenderingBuffer.Dispose();

        _gridParticleIDBuffer.Dispose();
        _gridVelocityBuffer.Dispose();
        _gridMassBuffer.Dispose();

        _gridSortHelper.Dispose();
    }
    #endregion

    #region RosettaUI
    private void InitRosettaUI()
    {
        var window = UI.Window(
            $"Settings ( press {_toggleUIKey.ToString()} to open / close )",
            UI.Indent(UI.Box(UI.Indent(
                UI.Space().SetHeight(5f),
                UI.Field("Quality", () => _quality)
                    .RegisterValueChangeCallback(InitGPUBuffers),
                UI.Indent(
                    UI.FieldReadOnly("Num Particles", () => NumParticles),
                    UI.FieldReadOnly("Num Grids", () => NumGrids)
                ),
                UI.Space().SetHeight(10f),
                UI.Label("Parameter"),
                UI.Indent(
                    UI.Field("Particle Mass", () => _particleMass),
                    UI.Field("Gravity", () => _gravity),
                    UI.Field("EOS Stiffness", () => _eosStiffness),
                    UI.Field("EOS Power", () => _eosPower),
                    UI.Field("Dynamic Viscosity", () => _dynamicViscosity)
                ),
                UI.Space().SetHeight(10f),
                UI.Label("Interaction"),
                UI.Indent(
                    UI.Slider("Mouse Force", () => _mouseForce),
                    UI.Slider("Mouse Force Range", () => _mouseForceRange)
                ),
                UI.Space().SetHeight(10f),
                UI.Field("Show FPS", () => _showFps)
                    .RegisterValueChangeCallback(() =>
                    {
                        FindObjectOfType<FPSCounter>().enabled = _showFps;
                    }),
                UI.Space().SetHeight(10f),
                UI.Button("Restart", InitGPUBuffers),
                UI.Space().SetHeight(5f)
            )))
        );
        window.Closable = false;

        _rosettaUIRoot.Build(window);
    }

    private void UpdateRosettaUI()
    {
        if (Input.GetKeyDown(_toggleUIKey))
            _rosettaUIRoot.enabled = !_rosettaUIRoot.enabled;
    }
    #endregion

    #region MonoBehaviour
    private void OnEnable()
    {
        InitComputeShaders();
        InitGPUBuffers();
    }

    private void Start()
    {
        InitRosettaUI();
    }

    private void Update()
    {
        DispatchParticleToGrid();
        DispatchExternalForce();
        DispatchBoundaryCondition();
        DispatchGridToParticle();
        DispatchAdvection();
        RenderParticles();

        UpdateRosettaUI();
    }

    private void OnDisable()
    {
        Dispose();
    }
    #endregion
}