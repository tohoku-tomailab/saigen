"""
Dobot MG400 over TCP/IP: two human-run tools, observation and a small jog.

- ``protocol.py`` -- the six read-only commands, the five control commands of the jog tool
  (ResetRobot, EnableRobot, DisableRobot, EmergencyStop, JointMovJ; refused unless the
  connection was built with ``control=True``), all built, never free text; the reply parser
- ``feedback.py`` -- the 1440-byte real-time feedback packet (port 30004)
- ``observe.py``  -- human-run tool that records replies, feedback and the controller's
  kinematics without changing the arm's state (it never arms a connection)
- ``jog.py``      -- human-run tool that enables the arm, moves J1 / J4 by 1..10 deg and
  back, disables it; its stop sequence is described in its docstring and README
- ``fake.py``     -- loopback fake controller for ``--mock`` and the tests

The MG400 is not a SAIGEN robot type yet: no flow can use it. See ``README.md``.
"""
