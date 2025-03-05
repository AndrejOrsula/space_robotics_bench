All assets used by the Space Robotics Bench are separated into the [`srb_assets`](https://github.com/AndrejOrsula/srb_assets) repository to encourage their reuse. Both static assets (datasets) and procedural pipelines in Blender are supported.

# Procedural Assets with Blender

## Motivation

Procedural generation is a powerful technique for creating diverse and realistic environments without relying on static, disk-consuming datasets. This approach allows for the generation of an infinite number of unique environments, a feature that has been underutilized in the fields of robotics and space exploration. The Space Robotics Bench seeks to address this gap by offering a versatile framework for procedurally generating 3D assets, which can be combined to create complex environments suitable for the development, training, and validation of space robotic systems.

## Approach

The package utilizes [Blender](https://www.blender.org) to procedurally generate both the geometry and materials (PBR textures) of 3D assets.

Geometry generation is achieved using Blender's [Geometry Nodes](https://docs.blender.org/manual/en/latest/modeling/geometry_nodes/introduction.html), a robust node-based system that allows for the creation, manipulation, and modification of arbitrary geometry and data types. First introduced in Blender 2.92 (2021), Geometry Nodes have evolved significantly, supporting the creation of intricate geometries through a series of interconnected node trees. Each node system can consist of multiple node trees that handle different aspects of the geometry. By applying randomness and variation within these node trees, a wide range of unique assets can be produced simply by adjusting the seed value.

Blender's [Shader Nodes](https://docs.blender.org/manual/en/latest/render/shader_nodes/introduction.html), which have a longer history, are used to define the appearance of objects through material properties. Like Geometry Nodes, Shader Nodes are also node-based and allow for the creation of complex materials. Blender provides several procedural textures and maps (e.g., Perlin noise, Voronoi, Wave), which can be adjusted and combined to form more sophisticated materials. By integrating randomness into the shader nodes, each procedurally generated asset can have a unique appearance, even with the same underlying geometry.

## Workflow

The [`srb_assets` repository](https://github.com/AndrejOrsula/srb_assets) includes a [`procgen_assets.py`](<(https://github.com/AndrejOrsula/srb_assets/blob/main/scripts/blender/procgen_assets.py)>) script that automates the entire procedural generation process, including node construction, modifier application, seeding, texture baking, and model export. This script is fully standalone and interacts with Blender's Python API (`bpy`) through its binary executable. Although Blender can be used as a Python module via [bpy](https://pypi.org/project/bpy), it is often linked to a specific Python version and has longer release cycles. The standalone script offers more flexibility, allowing it to be used with any Blender version.

Node trees can be generated from Python source files provided as input to the script. The [Node To Python addon](https://extensions.blender.org/add-ons/node-to-python) simplifies the creation of such source code. This addon enables users to design node trees in Blender's graphical interface and convert them into Python code that can be integrated into the [`procgen_assets.py`](<(https://github.com/AndrejOrsula/srb_assets/blob/main/scripts/blender/procgen_assets.py)>) script. This method allows users to prototype assets interactively within Blender's GUI and then export them into code.
