"""SmolVLA policy implementation."""
import time
from typing import Any, Dict, Optional
import numpy as np

from .base import BasePolicy, PolicyConfig
from .registry import register_policy

# Optional imports - graceful fallback if not available
try:
    import torch
    HAS_TORCH = True
except ImportError:
    HAS_TORCH = False
    torch = None

try:
    from lerobot.common.policies.smolvla.modeling_smolvla import SmolVLAPolicy as LeRobotSmolVLA
    from lerobot.common.policies.factory import make_policy
    HAS_LEROBOT = True
except ImportError:
    HAS_LEROBOT = False
    LeRobotSmolVLA = None
    make_policy = None


@register_policy('smolvla')
class SmolVLAPolicy(BasePolicy):
    """
    SmolVLA (Small Vision-Language-Action) policy implementation.

    Uses the LeRobot SmolVLA model for vision-language conditioned action prediction.
    """

    @property
    def policy_type(self) -> str:
        return 'smolvla'

    def load(self, checkpoint_path: str, device: str) -> bool:
        """Load SmolVLA model from checkpoint."""
        if not HAS_TORCH:
            self._log_error("PyTorch not available")
            return False

        if not HAS_LEROBOT:
            self._log_error("LeRobot not available. Install with: pip install lerobot")
            return False

        try:
            start_time = time.time()
            self._log_info(f"Loading SmolVLA from {checkpoint_path}")

            # Load the model using LeRobot's factory
            self._model = LeRobotSmolVLA.from_pretrained(
                checkpoint_path,
                local_files_only=True,
            )
            self._model.to(device)
            self._model.eval()

            self._device = device
            self._is_loaded = True

            load_time = (time.time() - start_time) * 1000
            self._log_info(f"SmolVLA loaded in {load_time:.0f}ms on {device}")
            return True

        except Exception as e:
            self._log_error(f"Failed to load SmolVLA: {e}")
            self._is_loaded = False
            return False

    def predict(self, observation: Dict[str, Any]) -> Optional[np.ndarray]:
        """
        Run SmolVLA inference.

        Args:
            observation: Dict with 'images', 'state', and optionally 'task'

        Returns:
            Action array or None
        """
        if not self._is_loaded or self._model is None:
            return None

        try:
            # Prepare observation for LeRobot format
            images = observation.get('images', {})
            state = observation.get('state', np.array([]))
            task = observation.get('task', self.config.task)

            # Convert to torch tensors
            obs_dict = {}

            # Process images
            for cam_name, img in images.items():
                # img: (H, W, C) uint8 -> (1, C, H, W) float32 normalized
                img_tensor = torch.from_numpy(img).permute(2, 0, 1).unsqueeze(0).float() / 255.0
                obs_dict[f'observation.images.{cam_name}'] = img_tensor.to(self._device)

            # Process state
            state_tensor = torch.from_numpy(state).unsqueeze(0).float().to(self._device)
            obs_dict['observation.state'] = state_tensor

            # Add task as language condition
            if task:
                obs_dict['task'] = task

            # Run inference
            with torch.no_grad():
                action = self._model.select_action(obs_dict)

            # Convert back to numpy
            if isinstance(action, torch.Tensor):
                action = action.cpu().numpy()

            # Return first action if action chunk
            if action.ndim > 1:
                action = action[0]

            return action

        except Exception as e:
            self._log_error(f"SmolVLA inference failed: {e}")
            return None

    def reset(self):
        """Reset SmolVLA policy state."""
        if self._model is not None and hasattr(self._model, 'reset'):
            self._model.reset()
