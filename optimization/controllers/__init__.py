"""
Swarm coverage controllers.

Each controller is a callable matching the policy contract used by
`tools/demo.py` and `visualize.animate`:

    policy_fn(env: CoverageEnv) -> np.ndarray of shape (n_drones, 3)

Returning [ax, ay, alpha_yaw] per drone in world frame. Shape (n_drones, 2)
is also accepted by env.step (treated as alpha_yaw = 0).

Three families live here, in increasing complexity:
    - PotentialFieldsController (potential_fields.py)
    - ConsensusController       (consensus.py)
    - MARLController            (marl.py — wraps a trained PPO policy)
"""

from .potential_fields import PotentialFieldsController, PFConfig
from .consensus import ConsensusController, ConsensusConfig

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
]
