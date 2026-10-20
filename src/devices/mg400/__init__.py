"""
Dobot MG400 over TCP/IP: observation only, for now.

- ``protocol.py`` -- the six read-only commands (built, never free text) and the reply parser
- ``feedback.py`` -- the 1440-byte real-time feedback packet (port 30004)
- ``observe.py``  -- human-run tool that records replies, feedback and the controller's
  kinematics without changing the arm's state
- ``fake.py``     -- loopback fake controller for ``--mock`` and the tests

The MG400 is not a SAIGEN robot type yet: no flow can use it. See ``README.md``.
"""
