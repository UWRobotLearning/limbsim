""" A convenience dataclass for configuring the simulator (pybullet).
"""
from dataclasses import dataclass
from typing import Any
from typing import Tuple

import pybullet


@dataclass
class SimulatorConf:
    """Simulator configuration dataclass.
    connection_mode:
        `None` connects to an existing simulation or, if fails, creates a
          new headless simulation,
        `pybullet.GUI` creates a new simulation with a GUI,
        `pybullet.DIRECT` creates a headless simulation,
        `pybullet.SHARED_MEMORY` connects to an existing simulation.
    """

    connection_mode: Any = pybullet.GUI
    timestep: float = 0.002
    action_repeat: int = 1
    reset_time: float = 3
    num_solver_iterations: int = 30
    # TODO: Determine whether init_position should be passed to the simulator
    # TODO(yxyang): Add to docs what this is, e.g. vector in R^3
    init_position: Tuple[float, float, float] = (0.0, 0.0, 0.32)
    init_rack_position: Tuple[float, float, float] = (0.0, 0.0, 1)
    on_rack: bool = False
