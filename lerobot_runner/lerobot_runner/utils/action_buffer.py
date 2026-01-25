"""Action buffer for managing action chunks with delay compensation."""
from collections import deque
from typing import List, Optional
import numpy as np
import threading


class ActionBuffer:
    """
    Thread-safe action buffer for managing action chunks.

    Handles:
    - Storing action chunks from inference
    - Delay compensation (skip initial actions)
    - Retrieving actions at publish rate
    """

    def __init__(self, max_size: int = 100):
        """
        Initialize action buffer.

        Args:
            max_size: Maximum number of actions to buffer
        """
        self._buffer: deque = deque(maxlen=max_size)
        self._lock = threading.Lock()

    def set(self, actions: List[np.ndarray], skip: int = 0):
        """
        Set new action chunk, optionally skipping initial actions.

        Args:
            actions: List of action arrays
            skip: Number of initial actions to skip (for delay compensation)
        """
        with self._lock:
            self._buffer.clear()
            for i, action in enumerate(actions):
                if i >= skip:
                    self._buffer.append(np.array(action))

    def get(self) -> Optional[np.ndarray]:
        """
        Get next action from buffer.

        Returns:
            Action array or None if buffer is empty
        """
        with self._lock:
            if len(self._buffer) > 0:
                return self._buffer.popleft()
            return None

    def peek(self) -> Optional[np.ndarray]:
        """
        Peek at next action without removing it.

        Returns:
            Action array or None if buffer is empty
        """
        with self._lock:
            if len(self._buffer) > 0:
                return self._buffer[0].copy()
            return None

    def clear(self):
        """Clear all actions from buffer."""
        with self._lock:
            self._buffer.clear()

    def __len__(self) -> int:
        """Return number of actions in buffer."""
        with self._lock:
            return len(self._buffer)

    @property
    def is_empty(self) -> bool:
        """Check if buffer is empty."""
        with self._lock:
            return len(self._buffer) == 0
