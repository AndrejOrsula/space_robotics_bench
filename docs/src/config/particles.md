# Environment Configuration — Particles

Space Robotics Bench can simulate liquid and granular materials like regolith, sand, and dust using particle-based physics. This is particularly relevant for space applications where interaction with loose granular material is common.

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/Pap-QDMsHk4?si=xJTKjGKLHRyMz4rO&mute=1&autoplay=1&loop=1&playlist=Pap-QDMsHk4" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

## Enabling Particles

You can enable particles in any environment by setting the `particles` parameter to `true`:

```bash
srb agent teleop -e _manipulation env.particles=true env.robot=+scoop
```

## Particle Configuration Parameters

| Parameter             | Description                 | Default |
| --------------------- | --------------------------- | ------- |
| `env.particles`       | Scatter particles           | `false` |
| `env.particles_size`  | Particle diameter (meters)  | `0.025` |
| `env.particles_ratio` | Particle density multiplier | `0.001` |

```bash
srb agent teleop -e _manipulation env.particles=true env.particles_size=0.01 env.particles_ratio=0.1
```

## Particle Behavior

By default, the particle system uses a pyramid distribution to create natural-looking piles of granular material with higher density at the center. Particles interact with other objects through physical collision and settle over time due to gravity. Robots can push, scoop, or otherwise interact with particles.

> **Note:** When particles are enabled, Fabric is disabled via `env.sim.use_fabric=false`.
