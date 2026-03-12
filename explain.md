# How This SPH Simulation Works

This project is a 2D weakly compressible SPH solver written in C with Raylib. "SPH" means Smoothed Particle Hydrodynamics: instead of solving fluid on a fixed mesh, we represent the material as particles and estimate density, pressure, and forces from nearby particles with smoothing kernels.

The code uses one particle system for water-like and gas-like behavior. The particles are the same data structure in every mode. What changes is the global parameter set: spacing, smoothing radius, viscosity, gravity, drag, buoyancy, and pressure model.

## 1. The main idea

Each particle stores position, velocity, acceleration, density, pressure, and temperature. During every simulation step, the solver does this:

1. Put particles into a uniform spatial grid.
2. Estimate each particle's density from nearby particles.
3. Convert density into pressure with an equation of state.
4. Compute forces from pressure, viscosity, walls, gravity, drag, heat diffusion, and mouse input.
5. Integrate velocity and position forward in time.
6. Update diagnostics used for coloring and the HUD.

That loop repeats every frame.

## 2. The data layout

The `ParticleSystem` stores particle data in structure-of-arrays form:

- `x`, `y`
- `vx`, `vy`
- `ax`, `ay`
- `density`, `pressure`
- `temperature`, `temperatureRate`
- `xsphVX`, `xsphVY`

This layout is better for CPU cache and parallel loops than a big array of particle structs.

There is also a second set of "sorted" arrays:

- `sortedX`, `sortedY`
- `sortedVX`, `sortedVY`
- `sortedTemperature`
- `sortedDensity`, `sortedPressure`

These are rebuilt in grid order so the hot neighbor loops can read mostly contiguous memory instead of jumping randomly through the original arrays.

## 3. Startup

At program start, `main()` does the following:

1. Creates the window.
2. Allocates all particle and grid arrays.
3. Creates two soft-circle textures:
   one for normal particles and one for smoothing-radius view.
4. Resets the simulation to the default preset.
5. Builds the first grid.
6. Computes initial density and pressure.
7. Computes initial diagnostics.

After that, the program enters the frame loop:

1. Process input.
2. Step the simulation if not paused.
3. Draw particles and HUD.

The app now supports two scene layouts:

- `Tank`: the original closed-box fluid or gas setup
- `Wind tunnel`: a gas-only flow scene with left-to-right recirculating flow and a switchable obstacle

The scene can change how particles are spawned, how boundaries behave, and what extra forces get added during the force pass.

## 4. Resetting a preset

When the sim resets, it chooses one of the material presets:

- Water-like
- Gas-like

Each preset defines the base physical parameters, such as:

- initial particle spacing
- support radius `h`
- visible particle radius
- reference density
- sound speed
- kinematic viscosity
- gravity and buoyancy
- thermal diffusion and thermal expansion
- drag and wall damping
- base timestep

The reset code also scales spacing and support radius to aim for the current target particle count. That is how the same preset can run at different counts without manually changing all constants every time.

Then it fills part of the box with particles:

- water starts as a compact block
- gas starts spread through almost the whole container
- wind tunnel starts with gas filling most of the channel, with a hole left around the active obstacle shape

Initial temperature is set from ambient temperature plus an optional vertical gradient and a small noise term. Gas gets a small random initial velocity. Water starts nearly still.

In wind tunnel mode, gas also starts with a rightward inlet-like velocity profile so the flow already moves across the obstacle from the first frame.

For water, the code then calibrates particle mass so the initial packing produces the intended average density. Gas skips this calibration.

## 5. Spatial hashing and the uniform grid

A naive SPH solver would compare every particle with every other particle. That is `O(n^2)` and becomes unusable quickly.

This sim uses a uniform grid with cell size equal to the support radius. Since the kernels are zero outside that support radius, a particle only needs to inspect its own cell and the 8 surrounding cells.

`BuildGrid()` works in four stages:

1. Count how many particles fall into each cell.
2. Prefix-sum those counts into `cellStarts`.
3. Scatter particle indices into `sortedIndices`.
4. Gather hot particle data into the sorted arrays.

That turns neighbor search from "look at every particle" into "look at a small local block of cells."

## 6. Density estimation

The first physical pass is density estimation.

For each particle, the code sums contributions from neighboring particles with a poly6-style kernel:

```text
rho_i = sum_j m * W_poly6(|x_i - x_j|, h)
```

In code, the kernel contribution is proportional to:

```text
(h^2 - r^2)^3
```

for neighbors with `r < h`.

The density pass only uses nearby particles from the grid. For water, it also adds boundary density support near walls so particles at the edge do not lose all neighbors on one side. Gas skips that density-wall term.

After density is computed, it is clamped to a preset-dependent floor:

- liquids use a higher floor
- gas uses a lower one

This prevents extremely small densities from creating pathological behavior.

## 7. Pressure model

Once density is known, the code computes pressure.

### Liquid

Water uses a Tait-style weakly compressible equation of state:

```text
p = k * ((rho / rho_rest_local)^7 - 1)
```

where `rho_rest_local` is adjusted by temperature through thermal expansion. Negative liquid pressure is clamped away to avoid tensile instability.

### Gas

Gas uses a different model:

```text
p = k_gas * rho * T
```

This is closer to an ideal-gas-style positive pressure law than a liquid compression law. The important point is that gas pressure stays positive instead of becoming an attractive "blob force."

## 8. Force computation

After density and pressure, the solver computes acceleration for every particle.

The force pass includes several terms.

### Pressure force

The symmetric SPH pressure force is:

```text
a_pressure =
- m * sum_j (p_i / rho_i^2 + p_j / rho_j^2) * grad W_spiky
```

This is the main force that pushes compressed regions apart.

### Viscosity force

Viscosity damps velocity differences between neighbors:

```text
a_visc ~ nu * m * sum_j (v_j - v_i) * laplacian(W)
```

This is one of the main terms that makes water resist relative motion more than gas.

### Temperature diffusion

Temperature is diffused between neighbors with a viscosity-kernel Laplacian. That creates a `temperatureRate` rather than an immediate force.

### XSPH transport smoothing

The solver also computes an XSPH correction:

```text
v_advect = v + eps * sum_j ...
```

This smooths transport velocity for liquids so the flow looks less noisy. Gas disables this because it makes gas look too liquid-like.

### Gravity, buoyancy, and drag

Every particle also gets:

- global drag
- gravity
- temperature-based buoyancy

Gas currently uses zero gravity and zero buoyancy in its default preset, so it behaves more like a room-filling compressible medium than rising smoke.

### Wall forces

The wall model is different for liquids and gas.

Liquids use SPH-style wall support:

- density support near boundaries
- pressure and viscosity response from mirrored wall contributions

Gas uses a simpler soft wall repulsion plus damping. That keeps it from forming liquid-like pressure ridges against the container walls.

### Scene forces

In wind tunnel mode, the solver adds two extra effects:

- a left-to-right driving force that keeps gas moving across the box
- a repulsion shell around the obstacle so particles flow around it instead of through it

The wind tunnel also changes the left and right behavior from reflective walls into wrap-around boundaries. Particles leaving on the right reappear on the left, which keeps the tunnel full while maintaining a continuous flow past the obstacle. The current wind-tunnel obstacles are a circle, a simple symmetric airfoil section, and a stylized 2D car profile.

### Mouse interaction

Holding left mouse or touchpad click applies an outward radial impulse around the cursor. It acts like a local pressure explosion.

## 9. Adaptive timestep

After forces are computed, the solver chooses a timestep for that substep.

It does not always use the preset timestep directly. Instead, it applies three stability limits:

- a CFL-like limit from sound speed and current velocity
- an acceleration limit from the largest force magnitude
- a viscosity limit from `h^2 / nu`

The final `dt` is clamped between a fraction of the preset timestep and the preset maximum.

This is important because SPH can become unstable if the same large timestep is used during violent motion and calm motion.

## 10. Integration

The code uses a semi-implicit Euler style update:

```text
v = v + a * dt
x = x + v_advect * dt
```

More exactly, the solver does this in order:

1. Update velocity from acceleration.
2. Clamp speed to a preset-dependent maximum.
3. Build an advection velocity with XSPH smoothing.
4. Advance position with that advection velocity.
5. Advance temperature with `temperatureRate`.
6. Apply a small extra damping term for liquids.
7. Resolve wall collisions.

Wall collision resolution finally clamps a particle back inside the box and reflects the velocity using preset-dependent bounce and friction.

## 11. The per-frame solver loop

`StepSimulation()` uses an accumulator so simulation time can be stepped in multiple substeps inside one render frame.

For each substep, the order is:

1. `BuildGrid()`
2. `ComputeDensityAndPressure()`
3. `ComputeForces()`
4. `ComputeAdaptiveTimeStep()`
5. `Integrate()`

After at least one substep, scalar fields are marked dirty.

The code does not always recompute density and pressure again immediately after stepping. That refresh only happens when it is needed for:

- pressure or density coloring
- periodic diagnostics

This is a performance optimization. The simulation state is still updated every step, but expensive scalar refreshes are skipped when they are not needed for the current view.

## 12. Diagnostics and color modes

`UpdateDiagnostics()` computes:

- min, max, and average density
- min, max, and average pressure
- min, max, and average speed
- min, max, and average temperature

These statistics drive:

- HUD values
- color ramps for temperature, pressure, speed, and density view modes

Material color mode uses fixed preset colors instead.

## 13. Rendering

The sim has two main visualization modes.

### Particle view

This draws each particle as a small soft sprite using `particleTexture`.

### Smoothing-radius view

This draws a larger soft sprite with diameter `2h` using additive blending. Visually, this shows how each particle's influence radius overlaps with nearby particles.

At very high counts, this mode can decimate draws for speed, but it now draws the full set up to the current 50k preset range so the field does not show fake gaps caused by render skipping.

## 14. Input and interaction flow

Input is processed at the start of every frame before stepping the sim. The current code uses Raylib's queued key events for one-shot actions so short key taps are less likely to be lost when the sim is running slowly.

The important controls are:

- `1`, `2` for water and gas
- `O` to cycle the wind-tunnel obstacle
- `4` to `8` for particle-count presets from 10k to 50k
- `M` to toggle water and gas quickly
- `V` to switch visualization mode
- `C` to switch color mode
- `R` to reset
- `Space` to pause
- mouse click to push particles outward

## 15. Why the three presets feel different

The solver itself is shared, but the presets make the same code behave differently.

### Water-like

- strong gravity
- medium viscosity
- strong liquid pressure response
- small bounce at walls
- light XSPH smoothing

This gives a settling liquid with surface motion and splashes.

### Syrup-like

- similar liquid pressure model
- much higher viscosity
- stronger damping

This makes the fluid resist shear and flow slowly.

### Gas-like

- positive pressure proportional to density and temperature
- no XSPH smoothing
- very low viscosity
- no default gravity or buoyancy
- soft wall repulsion instead of liquid wall pressure

This makes gas spread and fill the container more evenly instead of behaving like a falling liquid blob.

## 16. Performance design

The biggest performance ideas in this implementation are:

- uniform-grid neighbor lookup instead of all-pairs comparison
- structure-of-arrays particle storage
- grid-sorted hot arrays for better cache locality
- parallel density, force, and integration passes with `dispatch_apply`
- deferred diagnostics so the expensive scalar refresh is not forced every frame

Those choices are what let the sim handle tens of thousands of particles on CPU.

## 17. What this sim is and is not

This is a real SPH-style simulation, but it is still a practical classroom/demo solver, not a research-grade CFD package.

It is good for:

- showing how WCSPH works
- comparing liquid-like and gas-like parameter regimes
- visualizing density, pressure, speed, and temperature
- experimenting with stability, viscosity, and external forces

It is not a strict incompressible solver. It does not use methods like PCISPH, IISPH, or DFSPH, and it does not use full boundary-particle solids. So it is physically motivated and mechanically correct in structure, but still simplified for speed and clarity.
