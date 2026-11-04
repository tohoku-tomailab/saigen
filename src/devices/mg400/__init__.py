"""
Dobot MG400 over TCP/IP: two human-run tools (observation, a small jog) and the control
core the flow robot type ``mg400`` runs on.

- ``protocol.py`` -- the six read-only commands, the control commands (ResetRobot,
  EnableRobot, DisableRobot, EmergencyStop, JointMovJ of the jog tool, MovL of the flow
  robot type; refused unless the connection was built with ``control=True``), all built,
  never free text; the reply parser
- ``feedback.py`` -- the 1440-byte real-time feedback packet (port 30004)
- ``observe.py``  -- human-run tool that records replies, feedback and the controller's
  kinematics without changing the arm's state (it never arms a connection)
- ``control.py``  -- the control core shared by the jog tool and the robot type: connections,
  the phase-based interrupt model, the stop sequence, the recorder, and ``Controller``, the
  one-thread worker the flow wrapper ``src.devices.safety.mg400_robot`` talks to
- ``jog.py``      -- human-run tool that enables the arm, moves J1 / J4 by 1..10 deg and
  back, disables it; its stop sequence is described in its docstring and README
- ``fake.py``     -- loopback fake controller for ``--mock`` and the tests

As robot type ``mg400`` (``config.yaml`` ``robots.<id>.type``) a flow may use ``move_xyz``,
``move_z``, ``go_home`` and ``wait`` on it through ``src.devices.safety.mg400_robot``
(``docs/setup.md`` section 13). See ``README.md``.
"""
