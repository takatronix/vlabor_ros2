"""ROS2 message conversion utilities."""
from typing import List, Optional
import numpy as np

from sensor_msgs.msg import JointState, Image

try:
    from cv_bridge import CvBridge
    _cv_bridge = CvBridge()
    HAS_CV_BRIDGE = True
except ImportError:
    _cv_bridge = None
    HAS_CV_BRIDGE = False


def ros_image_to_numpy(msg: Image, encoding: str = 'rgb8') -> Optional[np.ndarray]:
    """
    Convert ROS Image message to numpy array.

    Args:
        msg: ROS Image message
        encoding: Desired encoding ('rgb8', 'bgr8', etc.)

    Returns:
        numpy array (H, W, C) or None if conversion fails
    """
    if not HAS_CV_BRIDGE:
        # Fallback without cv_bridge
        try:
            if msg.encoding in ('rgb8', 'bgr8'):
                channels = 3
            elif msg.encoding == 'mono8':
                channels = 1
            else:
                return None

            arr = np.frombuffer(msg.data, dtype=np.uint8)
            arr = arr.reshape(msg.height, msg.width, channels)

            # Convert BGR to RGB if needed
            if msg.encoding == 'bgr8' and encoding == 'rgb8':
                arr = arr[:, :, ::-1].copy()

            return arr
        except Exception:
            return None

    try:
        return _cv_bridge.imgmsg_to_cv2(msg, desired_encoding=encoding)
    except Exception:
        return None


def ros_jointstate_to_array(
    msg: JointState,
    joint_names: List[str],
    default_value: float = 0.0
) -> np.ndarray:
    """
    Convert ROS JointState message to numpy array in specified order.

    Args:
        msg: ROS JointState message
        joint_names: List of joint names in desired order
        default_value: Value to use for missing joints

    Returns:
        numpy array of joint positions
    """
    name_to_idx = {name: i for i, name in enumerate(msg.name)}
    positions = []

    for name in joint_names:
        if name in name_to_idx:
            idx = name_to_idx[name]
            if idx < len(msg.position):
                positions.append(msg.position[idx])
            else:
                positions.append(default_value)
        else:
            positions.append(default_value)

    return np.array(positions, dtype=np.float32)


def array_to_ros_jointstate(
    positions: np.ndarray,
    joint_names: List[str],
    stamp=None
) -> JointState:
    """
    Convert numpy array to ROS JointState message.

    Args:
        positions: numpy array of joint positions
        joint_names: List of joint names
        stamp: Optional timestamp (builtin_interfaces/Time)

    Returns:
        ROS JointState message
    """
    msg = JointState()
    if stamp is not None:
        msg.header.stamp = stamp
    msg.name = list(joint_names)
    msg.position = positions.tolist()
    return msg
