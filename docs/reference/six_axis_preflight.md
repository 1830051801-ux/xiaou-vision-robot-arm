# Six-axis offline verification and real-motion prerequisites

The desktop pipeline is perception and planning only:

`voice/text -> YOLO -> 2D homography -> fixed object Z -> ROS2/MoveIt plan`

The real-motion gate remains locked until measured CAN interface/bitrate, six
joint IDs, encoder zero and direction, position/velocity/acceleration limits,
feedback, emergency stop, and the vendor protocol are verified one joint at a
time at low speed with a person holding the stop control.

Run offline checks from the project root:

```powershell
python -m unittest discover -s tests -v
python -m compileall -q robot_ai tests tools
python tools/verify_six_axis_stack.py
```

No Python demo in this checkout sends an actuator command. ROS2/MoveIt remains
the only future execution boundary.
