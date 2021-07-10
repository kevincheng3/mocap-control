# mocap-control
By creating a mocap body and fixing it to the end point of the robotic arm, Realize inverse kinematics in mujoco through mocap control.

creating a mocap body:
```c
    <body mocap="true" name="mocap" pos="0.08229997 0.10921554 1.871059">
        <!-- <geom conaffinity="0" contype="0" pos="0 0 0" rgba="0 0.5 0 0.7" size="0.005 0.005 0.005" type="box"></geom>
        <geom conaffinity="0" contype="0" pos="1 0 0" rgba="1 0 0 0.1" size="1 0.005 0.005" type="box"></geom>
        <geom conaffinity="0" contype="0" pos="0 1 0" rgba="0 1 0 0.1" size="0.005 1 0.001" type="box"></geom>
        <geom conaffinity="0" contype="0" pos="0 0 1" rgba="0 0 1 0.1" size="0.005 0.005 1" type="box"></geom> -->
    </body>
```c

fixing it to the end point of the robotic arm:

```c
    <equality>
        <weld body1="mocap" body2="ee_link" solimp="0.9 0.95 0.001" solref="0.02 1"></weld>
    </equality>
```

By setting setting the mocap position and quaternion, you can compelting tcp control
```c
sim.data.set_mocap_pos("mocap", pos)
self.sim.data.set_mocap_quat("mocap", quat)
```

```c

"Go up/down/left/right",     "[up]/[down]/[left]/[right] arrow",
"Go forwarf/backward",       "[F]/[B]"
"ROT_X",                     "[Q]/[W]"
"ROT_Y",                     "[A]/[S]"
"ROT_Z",                     "[Z]/[X]"
"Slow down/Speed up",        "[-]/[=]"
```

![videos](https://user-images.githubusercontent.com/43990826/125153758-8fd78480-e188-11eb-9871-388e9073612d.gif)



            
