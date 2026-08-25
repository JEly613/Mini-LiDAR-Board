"""Host-side tools for the Mini-LiDAR-Board.

Milestone 1 covers the on-board ICM-42688-P only: read orientation packets over
the board's USB CDC link and display the board's live 3D attitude.

There is deliberately no LiDAR support here yet; see ``protocol.py`` for the
reserved packet-type range that a future LiDAR stream will use.
"""

__version__ = "1.0.0"
