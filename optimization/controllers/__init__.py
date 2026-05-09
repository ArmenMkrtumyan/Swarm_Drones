"""
Swarm coverage controllers.

Each controller is a callable matching the policy contract used by
`tools/demo.py` and `visualize.animate`:

    policy_fn(env: CoverageEnv) -> np.ndarray of shape (n_drones, 3)

Returning [ax, ay, alpha_yaw] per drone in world frame. Shape (n_drones, 2)
is also accepted by env.step (treated as alpha_yaw = 0).

Four families live here:
    - PotentialFieldsController (potential_fields.py) — Track 3 (control)
    - ConsensusController       (consensus.py)        — Track 3 (control)
    - MARLController            (marl.py)             — Track 3 (learning, PPO)
    - PSOController             (pso.py)              — Track 2 (metaheuristic)
"""

from .potential_fields import PotentialFieldsController, PFConfig
from .consensus import ConsensusController, ConsensusConfig
from .pso import PSOController, PSOConfig
from .ga import GAController, GAConfig
from .aco import ACOController, ACOConfig
from .sa import SAController, SAConfig
from .gwo import GWOController, GWOConfig
from .boustrophedon import BoustrophedonController, BoustrophedonConfig
from .spiral import SpiralController, SpiralConfig
from .voronoi_partition import VoronoiPartitionController, VoronoiPartitionConfig
from .grid_decomposition import GridDecompositionController, GridDecompositionConfig
from .stc import STCController, STCConfig

# MARL is gated on torch + stable-baselines3 being installed. Import
# lazily so the package still works for PF / Consensus users without
# the heavier ML stack.
def _try_import_marl():
    try:
        from .marl import MARLController
        return MARLController
    except ImportError:
        return None

MARLController = _try_import_marl()

__all__ = [
    "PotentialFieldsController",
    "PFConfig",
    "ConsensusController",
    "ConsensusConfig",
    "MARLController",
    "PSOController",
    "PSOConfig",
    "GAController",
    "GAConfig",
    "ACOController",
    "ACOConfig",
    "SAController",
    "SAConfig",
    "GWOController",
    "GWOConfig",
    "BoustrophedonController",
    "BoustrophedonConfig",
    "SpiralController",
    "SpiralConfig",
    "VoronoiPartitionController",
    "VoronoiPartitionConfig",
    "GridDecompositionController",
    "GridDecompositionConfig",
    "STCController",
    "STCConfig",
]
