# Making a rover model GodotWings can drive

`GWRover` can take a vehicle's geometry from its visual model instead of from
numbers in a config: how many wheels it has, where their hubs are, how big they
are, where the hull is and where the centre of mass sits. It then animates the
wheels (steering, rolling, suspension travel) as the physics runs. All it needs
is a glTF whose nodes follow a small naming standard. This page is the
step-by-step for Blender; the standard itself is tool-agnostic.

## The standard

```
RoverModel                  root — any name
├── Hull                    body mesh(es). Name prefix "Hull". Their combined
│                           bounding box is the collision hull and the inertia box
├── CG                      optional Empty at the centre of mass. Without it the
│                           centre of the hull box is used
├── Wheel_FL                one node per wheel, name prefix "Wheel" (any suffix,
├── Wheel_FR                3 or more, may live inside a group called "Wheels").
├── Wheel_RL                Node origin = hub centre. The wheel mesh (the node
└── Wheel_RR                itself or its children) is a disc: its thinnest axis
                            is the axle, half its diameter is the radius
```

- Names are matched case-insensitively on the prefix, so `Hull.001`,
  `Wheel_L2`, `wheel_rear_right` all count. Godot's importer turns `.` into
  `_`, which changes nothing here.
- Authoring frame: standard glTF, **+Y up, −Z forward**. Onshape/CAD Z-up
  exports are accepted via `GWRover.model_orientation`.
- Units are metres. The model must be at real size; `GWRover.model_scale`
  scales the geometry read from it as well as the picture.
- The wheels furthest forward become the steered axle, the furthest back the
  rear axle (see `GWRoverConfig.steer_front` / `steer_rear` / `drive_layout`).
  Left and right are decided by position.
- Below three `Wheel*` nodes the model is drawn but the config's
  `wheelbase` / `track` / `wheel_radius` / `cg_height` drive the physics, and a
  warning says so.

## In Blender, step by step

1. **Units.** Scene Properties → Units → Metric, Unit Scale 1.0, Length in
   metres.
2. **Orientation.** Nose along Blender **+Y**, up along **+Z**. The glTF
   exporter maps that onto Godot's −Z forward / +Y up. If it comes out
   tail-first, `model_yaw_offset_deg = 180` fixes it without a re-export.
3. **Rest pose.** Model it sitting on its wheels with the bottom of the tyres on
   Z = 0. The suspension is preloaded around this pose.
4. **Hull.** One or more meshes named `Hull…`. Keep antennas, mirrors, sensor
   masts in separate meshes with other names so they don't inflate the hull box.
5. **Centre of mass.** Add an Empty named `CG` where the mass really is, usually
   low and between the axles at battery height. This matters more than anything
   else on this list: too high and the vehicle tips in corners it should hold.
6. **Wheels.** One object per wheel, named `Wheel_…`:
   - Set each object's origin to the hub centre (Object → Set Origin → Origin to
     Geometry on a cylinder does it).
   - Keep the cylinder's axis along one of the object's local axes. Rotate the
     *object* 90° to lay it along the vehicle's X; don't free-rotate the mesh
     data, or the bounding box no longer tells the axle from the rim.
   - A rim and a tyre may be children of the wheel object. Wheel objects may sit
     under a group Empty named `Wheels`. Don't nest a wheel inside another wheel.
7. **Export.** File → Export → glTF 2.0, format **glTF Binary (.glb)** so
   textures travel with it, **+Y Up** ticked (the default), Apply Modifiers on,
   no animation. Do not join everything into one mesh: the node names *are* the
   standard.

Quick check in the Outliner before exporting: `Hull…`, `CG`, N `Wheel…`
objects; each wheel's origin dot on its hub; tyres touching Z = 0; nose
pointing +Y.

## What comes from the model, what stays in the config

| From the **model** (3+ wheel nodes) | From **GWRoverConfig** |
|---|---|
| Wheel count, hub positions → wheelbase, track | Mass |
| Wheel radius and width, per wheel | Suspension frequency, damping ratio, travel |
| CG position (the `CG` empty, else the hull box centre) | Motor torque, top speed, motor lag, neutral brake |
| Hull box → collision hull, inertia estimate | Steering mode, lock, servo lag, which axles steer / drive |
| | Tyre grip, slip stiffness, rolling resistance, drag, rollover angle |

The inertia tensor is estimated as a solid box of the config's mass with the
hull's dimensions unless `Ixx` / `Iyy` / `Izz` are set explicitly.

## Checking it in Godot

Drop a `GWRover` in a scene, set `model_scene` to the imported glTF, run it
with `control_source = Manual` and drive with ↑/↓ and ←/→. In the remote
inspector `wheels` lists what was read (name, `hub_rest` in metres FRD from
the CG, `radius`), and `geometry_source` says `"model"` when the scan
succeeded. `tests/test_rover_model.gd` shows the same scan run against a
hand-built six-wheel scene.
