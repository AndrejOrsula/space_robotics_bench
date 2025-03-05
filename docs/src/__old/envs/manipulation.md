# Robotic Manipulation Environments

## Tasks

### Sample Collection

#### Scenario: Moon, Objects: Procedural

```bash
.docker/run.bash -e SRB_SCENARIO=moon -e SRB_ASSETS_OBJECT_VARIANT=procedural scripts/teleop.py --env sample_collection
```

![](../../_images/envs/sample_collection_moon.jpg)

#### Scenario: Mars, Objects: Dataset

```bash
.docker/run.bash -e SRB_SCENARIO=mars -e SRB_ASSETS_OBJECT_VARIANT=dataset scripts/teleop.py --env sample_collection
```

![](../../_images/envs/sample_collection_mars.jpg)

#### Other Examples

```bash
# Scenario: Moon, Objects: Primitive
.docker/run.bash -e SRB_SCENARIO=moon -e SRB_ASSETS_OBJECT_VARIANT=primitive scripts/teleop.py --env sample_collection

# Scenario: Mars, Objects: Procedural
.docker/run.bash -e SRB_SCENARIO=mars -e SRB_ASSETS_OBJECT_VARIANT=procedural scripts/teleop.py --env sample_collection

# Scenario: Moon, Objects: Multi + Dataset
.docker/run.bash -e SRB_SCENARIO=orbit -e SRB_ASSETS_OBJECT_VARIANT=dataset scripts/teleop.py --env sample_collection_multi
```

### Debris Capture

#### Scenario: Orbit, Objects: Dataset

```bash
.docker/run.bash -e SRB_SCENARIO=orbit -e SRB_ASSETS_OBJECT_VARIANT=dataset scripts/teleop.py --env debris_capture
```

![](../../_images/envs/debris_capture_orbit.jpg)

#### Other Examples

```bash
# Scenario: Orbit, Objects: Procedural
.docker/run.bash -e SRB_SCENARIO=orbit -e SRB_ASSETS_OBJECT_VARIANT=procedural scripts/teleop.py --env debris_capture
```

### Peg-in-Hole

#### Scenario: Moon, Objects: Dataset

```bash
.docker/run.bash -e SRB_SCENARIO=moon -e SRB_ASSETS_OBJECT_VARIANT=dataset scripts/teleop.py --env peg_in_hole
```

![](../../_images/envs/peg_in_hole_moon.jpg)

#### Scenario: Orbit, Objects: Dataset

```bash
.docker/run.bash -e SRB_SCENARIO=orbit -e SRB_ASSETS_OBJECT_VARIANT=dataset scripts/teleop.py --env peg_in_hole
```

![](../../_images/envs/peg_in_hole_orbit.jpg)

#### Other Examples

```bash
# Scenario: Moon, Objects: Prodecural
.docker/run.bash -e SRB_SCENARIO=moon -e SRB_ASSETS_OBJECT_VARIANT=procedural scripts/teleop.py --env peg_in_hole

# Scenario: Mars, Objects: Dataset
.docker/run.bash -e SRB_SCENARIO=mars -e SRB_ASSETS_OBJECT_VARIANT=dataset scripts/teleop.py --env peg_in_hole

# Scenario: Moon, Objects: Multi + Dataset
.docker/run.bash -e SRB_SCENARIO=mars -e SRB_ASSETS_OBJECT_VARIANT=dataset scripts/teleop.py --env peg_in_hole_multi

# Scenario: Mars, Objects: Multi + Procedural
.docker/run.bash -e SRB_SCENARIO=mars -e SRB_ASSETS_OBJECT_VARIANT=procedural scripts/teleop.py --env peg_in_hole_multi

# Scenario: Orbit, Objects: Multi + Dataset
.docker/run.bash -e SRB_SCENARIO=orbit -e SRB_ASSETS_OBJECT_VARIANT=dataset scripts/teleop.py --env peg_in_hole_multi
```

### Solar Panel Assembly

#### Scenario: Moon

```bash
.docker/run.bash -e SRB_SCENARIO=moon scripts/teleop.py --env solar_panel_assembly
```

![](../../_images/envs/solar_panel_assembly_moon.jpg)

#### Other Examples

```bash
# Scenario: Mars
.docker/run.bash -e SRB_SCENARIO=mars scripts/teleop.py --env solar_panel_assembly

# Scenario: Orbit
.docker/run.bash -e SRB_SCENARIO=orbit scripts/teleop.py --env solar_panel_assembly
```

## Demos

### Gateway

```bash
.docker/run.bash -e SRB_SCENARIO=orbit scripts/teleop.py --env gateway
```

![](../../_images/envs/gateway.jpg)
