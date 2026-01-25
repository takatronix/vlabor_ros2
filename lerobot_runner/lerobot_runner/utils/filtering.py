"""Filtering utilities for smooth action output."""
from typing import Optional
import numpy as np


class LowPassFilter:
    """
    Simple exponential moving average low-pass filter.

    Smooths action output to reduce jerkiness.
    """

    def __init__(self, alpha: float = 0.1, num_joints: int = 6):
        """
        Initialize low-pass filter.

        Args:
            alpha: Smoothing factor (0-1). Lower = smoother but more lag.
            num_joints: Number of joints to filter
        """
        self._alpha = alpha
        self._prev_value: Optional[np.ndarray] = None
        self._num_joints = num_joints

    def filter(self, value: np.ndarray) -> np.ndarray:
        """
        Apply low-pass filter to value.

        Args:
            value: Input value (joint positions)

        Returns:
            Filtered value
        """
        if self._prev_value is None:
            self._prev_value = value.copy()
            return value

        # Exponential moving average
        filtered = self._alpha * value + (1 - self._alpha) * self._prev_value
        self._prev_value = filtered.copy()
        return filtered

    def reset(self, initial_value: Optional[np.ndarray] = None):
        """
        Reset filter state.

        Args:
            initial_value: Optional initial value to set
        """
        self._prev_value = initial_value.copy() if initial_value is not None else None

    @property
    def alpha(self) -> float:
        """Get current alpha value."""
        return self._alpha

    @alpha.setter
    def alpha(self, value: float):
        """Set alpha value (clamped to 0-1)."""
        self._alpha = max(0.0, min(1.0, value))
