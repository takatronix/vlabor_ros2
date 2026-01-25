"""Policy registry for dynamic policy loading."""
from typing import Dict, Type, TYPE_CHECKING

if TYPE_CHECKING:
    from .base import BasePolicy

# Global registry of policy classes
POLICY_REGISTRY: Dict[str, Type['BasePolicy']] = {}


def register_policy(name: str):
    """
    Decorator to register a policy class.

    Usage:
        @register_policy('smolvla')
        class SmolVLAPolicy(BasePolicy):
            ...
    """
    def decorator(cls):
        POLICY_REGISTRY[name] = cls
        return cls
    return decorator


def get_policy_class(name: str) -> Type['BasePolicy']:
    """Get a policy class by name."""
    if name not in POLICY_REGISTRY:
        available = list(POLICY_REGISTRY.keys())
        raise ValueError(f"Unknown policy: {name}. Available: {available}")
    return POLICY_REGISTRY[name]


def list_policies() -> list:
    """List all registered policy names."""
    return list(POLICY_REGISTRY.keys())
