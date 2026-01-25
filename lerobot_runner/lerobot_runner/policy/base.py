"""Base policy class and configuration."""
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional
import numpy as np


@dataclass
class PolicyConfig:
    """Configuration for a policy."""
    checkpoint_path: str = ""
    device: str = "cuda:0"
    task: str = ""

    # Inference settings
    inference_rate: float = 1.0  # Hz
    publish_rate: float = 25.0   # Hz

    # Robot configuration
    joint_names: List[str] = field(default_factory=list)
    arm_namespaces: List[str] = field(default_factory=list)

    # Camera configuration
    camera_names: List[str] = field(default_factory=list)
    camera_topics: Dict[str, str] = field(default_factory=dict)

    # Filtering
    lpf_enable: bool = True
    lpf_alpha: float = 0.1


class BasePolicy(ABC):
    """
    Abstract base class for all policies.

    Subclasses must implement:
    - load(): Load model from checkpoint
    - predict(): Run inference on observation
    - reset(): Reset policy state
    - policy_type: Property returning policy type name
    """

    def __init__(self, config: PolicyConfig, logger=None):
        self.config = config
        self.logger = logger
        self._is_loaded = False
        self._model = None

    @property
    @abstractmethod
    def policy_type(self) -> str:
        """Return the policy type name (e.g., 'smolvla', 'act')."""
        pass

    @abstractmethod
    def load(self, checkpoint_path: str, device: str) -> bool:
        """
        Load model from checkpoint.

        Args:
            checkpoint_path: Path to model checkpoint directory
            device: Device to load model on (e.g., 'cuda:0', 'cpu')

        Returns:
            True if successful, False otherwise
        """
        pass

    @abstractmethod
    def predict(self, observation: Dict[str, Any]) -> Optional[np.ndarray]:
        """
        Run inference on observation.

        Args:
            observation: Dictionary containing:
                - 'images': Dict[camera_name, np.ndarray (H, W, C)]
                - 'state': np.ndarray of joint positions
                - 'task': str task description (optional, uses config.task if not provided)

        Returns:
            np.ndarray of action (joint positions) or None if inference fails
        """
        pass

    @abstractmethod
    def reset(self):
        """Reset policy state (e.g., action buffer, hidden states)."""
        pass

    def unload(self):
        """Unload model and free resources."""
        self._model = None
        self._is_loaded = False

    @property
    def is_loaded(self) -> bool:
        """Check if model is loaded."""
        return self._is_loaded

    def set_task(self, task: str):
        """Update the task description."""
        self.config.task = task

    def _log_info(self, msg: str):
        if self.logger:
            self.logger.info(msg)

    def _log_warn(self, msg: str):
        if self.logger:
            self.logger.warn(msg)

    def _log_error(self, msg: str):
        if self.logger:
            self.logger.error(msg)
