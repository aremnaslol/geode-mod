# GD Neural Pilot (demo.gd-neural-pilot)

Look-ahead autoplayer for Geometry Dash 2.2091, Geode 6.0.0-alpha.2, Windows 64-bit.

STATUS: written against the real 2.2091 bindings and the v6.0.0-alpha.2 SDK, but NOT compiled or run
(the authoring sandbox had no MSVC/Windows SDK). Expect to fix a build error or two and to tune the
physics constants in src/Pilot.cpp (`namespace phys`).

## Build the .geode (easiest: GitHub Actions, no local setup)
1. Create a GitHub repo, push this folder.
2. Actions tab -> "Build GD Neural Pilot (Windows)" -> run.
3. Download the artifact; the file is `demo.gd-neural-pilot.geode`. Put it in `<GD folder>\geode\mods`.

## Build locally (Windows, Visual Studio 2022 + CMake + Geode CLI)
    geode sdk install
    geode sdk install-binaries
    set GEODE_SDK=<your sdk path>
    geode build

## Use
F7 menu, F6 autopilot on/off (rebindable in the mod's settings). Menu has the toggle,
reaction time (ms) and prediction distance (units).

## What it does
- Cube/robot: simulates jump/no-jump ahead (spikes, saws, blocks, pads, orbs, gravity portals).
- Ship/UFO/swing/wave: steers to the middle of the nearest free vertical gap.
- Ball/spider: flips when a hazard enters its lane.
Not handled: slopes, speed/size/teleport portals mid-prediction, moving/triggered objects,
platformer mode, dual-mode second player. It will die on those.
