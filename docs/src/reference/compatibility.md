# Compatibility Matrix

The matrix shows supported robot categories for each task. A ✓ marks a supported combination; a blank cell means unavailable. Tasks prefixed with `_` (for example, `_manipulation`) are reusable templates, not preconfigured goal scenarios; they expose a generic environment and require robot and task-specific goal configuration.

<!-- PRESERVE:scope_compatibility -->

> **Scope.** This matrix is about **robot categories only**. A ✓ says a task accepts at least one robot of that category, not every robot in it: the three `terrain_landscaping` tasks, for example, accept only `RaphRover` with the `RaphShovel` payload and raise `TypeError` for any other. It does not say that the task is solved, that any learner has been validated on it, or that it has been run on physical hardware. See the note below the table for the landscaping tasks.

<!-- /PRESERVE:scope_compatibility -->

| Task | **Manipulator** | **Mobile Manipulator** | **Mobile Robot** |
|---|:---: | :---: | :---:|
| `lunalab` | | ✓ | |
| `orbital_lunalab` | | | ✓ |
| `zerog_lab` | | ✓ | |
| `_manipulation` | ✓ | | |
| `debris_capture` | ✓ | | |
| `excavation` | ✓ | | |
| `excavation_demo` | ✓ | | |
| `modular_assembly` | ✓ | | |
| `multi_peg_in_hole` | ✓ | | |
| `multi_sample_collection` | ✓ | | |
| `peg_in_hole` | ✓ | | |
| `sample_collection` | ✓ | | |
| `screwdriving` | ✓ | | |
| `solar_panel_assembly` | ✓ | | |
| `_aerial` | | | ✓ |
| `_ground` | | | ✓ |
| `_orbital` | | | ✓ |
| `aerial_navigation` | | | ✓ |
| `beam_transport` | | | ✓ |
| `flight` | | | ✓ |
| `formation_following` | | | ✓ |
| `formation_following_decentralized` | | | ✓ |
| `formation_following_orbital` | | | ✓ |
| `landing` | | | ✓ |
| `locomotion_velocity_tracking` | | ✓ | ✓ |
| `locomotion_waypoint_navigation` | | ✓ | ✓ |
| `moonfall` | | | ✓ |
| `orbital_evasion` | | | ✓ |
| `orbital_waypoint_navigation` | | | ✓ |
| `rendezvous` | | | ✓ |
| `terrain_landscaping` | | | ✓ |
| `terrain_landscaping_crater` | | | ✓ |
| `terrain_landscaping_mound` | | | ✓ |
| `traversal` | | | ✓ |
| `velocity_tracking` | | | ✓ |
| `waypoint_navigation` | | | ✓ |
| `_aerial_manipulation` | | ✓ | |
| `_ground_manipulation` | | ✓ | |
| `_orbital_manipulation` | | ✓ | |
| `mobile_debris_capture` | | ✓ | |
| `spacewalk` | | ✓ | |
| `spacewalk_eva` | | ✓ | |
| `spacewalk_iva` | | ✓ | |

<!-- PRESERVE:notes_compatibility -->

## What this table does not say

A row in this matrix is **not** a support claim for any learner or for physical hardware.
For the measured state of the terrain landscaping tasks, see [Terrain Landscaping Training](../workflows/terrain_landscaping_training.md#results-and-limitations).

<!-- /PRESERVE:notes_compatibility -->
