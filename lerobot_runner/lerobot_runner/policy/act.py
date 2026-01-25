"""ACT (Action Chunking Transformer) policy implementation."""
import time
from typing import Any, Dict, Optional
import numpy as np

from .base import BasePolicy, PolicyConfig
from .registry import register_policy

# Optional imports
try:
    import torch
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False
    torch = None

try:
    from lerobot.common.policies.act.modeling_act import ACTPolicy as LeRobotACT
    HAS_LEROBOT_ACT = True
except ImportError:
    HAS_LEROBOT_ACT = False
    LeRobotACT = None


@register_policy('act')
class ACTPolicy(BasePolicy):
    """
    ACT (Action Chunking Transformer) policy implementation.

    Uses the LeRobot ACT model for action prediction with temporal chunking.
    """

    def __init__(self, config: PolicyConfig, logger=None):
        super().__init__(config, logger)
        self._action_buffer = []
        self._action_index = 0

    @property
    def policy_type(self) -> str:
        return 'act'

    def load(self, checkpoint_path: str, device: str) -> bool:
        """Load ACT model from checkpoint."""
        if not HAS_TORCH:
            self._log_error("PyTorch not available")
            return False

        if not HAS_LEROBOT_ACT:
            self._log_error("LeRobot ACT not available. Install with: pip install lerobot")
            return False

        try:
            start_time = time.time()
            self._log_info(f"Loading ACT from {checkpoint_path}")

            self._model = LeRobotACT.from_pretrained(
                checkpoint_path,
                local_files_only=True,
            )
            self._model.to(device)
            self._model.eval()

            self._device = device
            self._is_loaded = True

            load_time = (time.time() - start_time) * 1000
            self._log_info(f"ACT loaded in {load_time:.0f}ms on {device}")
            return True

        except Exception as e:
            self._log_error(f"Failed to load ACT: {e}")
            self._is_loaded = False
            return False

    def predict(self, observation: Dict[str, Any]) -> Optional[np.ndarray]:
        """
        Run ACT inference with action chunking.

        ACT predicts a sequence of actions. This method manages the action buffer
        and returns one action at a time.

        Args:
            observation: Dict with 'images' and 'state'

        Returns:
            Action array or None
        """
        if not self._is_loaded or self._model is None:
            return None

        # If we have buffered actions, return next one
        if self._action_index < len(self._action_buffer):
            action = self._action_buffer[self._action_index]
            self._action_index += 1
            return action

        # Need to run new inference
        try:
            images = observation.get('images', {})
            state = observation.get('state', np.array([]))

            obs_dict = {}

            # Process images
            for cam_name, img in images.items():
                img_tensor = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).float() / 255.0
                obs_dict[f'observation.images.{cam_name}'] = img_tensor.to(self._device)

            # Process state
            state_tensor = torch.from_numpy(state).unsqueeze(0).float().to(self._device)
            obs_dict['observation.state'] = state_tensor

            # Run inference - ACT returns action chunk
            with torch.no_grad():
                actions = self._model.select_action(obs_dict)

            # Convert to numpy
            if isinstance(actions, torch.Tensor):
                actions = actions.cpu().numpy()

            # Store action chunk in buffer
            if actions.ndim == 1:
                self._action_buffer = [actions]
            else:
                self._action_buffer = list(actions)

            self._action_index = 1  # Already returning first one
            return self._action_buffer[0]

        except Exception as e:
            self._log_error(f"ACT inference failed: {e}")
            return None

    def reset(self):
        """Reset ACT policy state and action buffer."""
        self._action_buffer = []
        self._action_index = 0
        if self._model is not None and hasattr(self._model, 'reset'):
            self._model.reset()
