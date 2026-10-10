# Contributing — New Assets

Contribute SRB static assets through [srb_assets](https://github.com/AndrejOrsula/srb_assets). Isaac Lab also supplies assets.

## Choose an asset type

SRB assets fall into three types:

| Asset Type | Description |
| ----------- | -------------------------------------------------- |
| **Scenery** | Terrain and structures |
| **Objects** | Interactive objects, tools, and payloads |
| **Robots** | Controllable systems |

## Add a static asset

1. Identify its family and interface:

   - Robots: joints and sensors.
   - Terrain: source metadata.
   - Objects: spawn configuration.

1. Record the source, author, and license in the applicable metadata catalog. For assets in SRB's catalog, edit `docs/data/assets.toml`.

1. Follow the family format and validation rules. The rock pipeline checksum-verifies, decodes, bounds, and cleans terrain rocks. Robot and scenery checks differ.

1. Check dependencies, payload paths, and references against family conventions.

1. Add the asset to your [srb_assets fork](https://github.com/AndrejOrsula/srb_assets/fork).

1. Submit a pull request.

## Create procedural assets with SimForge

> Reference: [SimForge](https://AndrejOrsula.github.io/simforge)

[SimForge](https://github.com/AndrejOrsula/simforge) supplies procedural environments and assets, including robot structures such as the Cubesat. Examples:

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/XgXYHmEIvSM?si=IZGp8lwmlxWIsO4h&mute=1&autoplay=1&loop=1&playlist=XgXYHmEIvSM" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/jd7IIaL5Vgg?si=yCifBNo3eu6J22Vy&mute=1&autoplay=1&loop=1&playlist=jd7IIaL5Vgg" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

<iframe style="width:100%;aspect-ratio:16/9" src="https://www.youtube.com/embed/DbnE086w-sY?si=WXsuxjkrSW6Oxy1A&mute=1&autoplay=1&loop=1&playlist=DbnE086w-sY" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

See the [SimForge documentation](https://AndrejOrsula.github.io/simforge), especially:

- [New Assets](https://AndrejOrsula.github.io/simforge/development/assets.html)
  - [Blender Generator: New Assets](https://andrejorsula.github.io/simforge/generators/blender.html#new-assets)

Next: [Contribute a new task](new_tasks.md)
