"""
device_mapping.py - デバイス割り当て設定の管理

プロファイル YAML の devices セクション読み込みと、
デバイス割り当て設定の永続化・自動照合を行う。
"""
from __future__ import annotations

import os
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml

from .device_scanner import DetectedDevice, scan_all_devices


# vlabor_launch ソースツリーの固定パス
# symlink-install ではソースを直接参照するが、
# フォールバックとして ament share ディレクトリも検索する
_VLABOR_LAUNCH_SRC = Path("/home/ros2/ros2_ws/src/vlabor_ros2/vlabor_launch")


def _get_config_dir() -> Path:
    """config ディレクトリを返す（ソースツリー優先、share フォールバック）"""
    src_config = _VLABOR_LAUNCH_SRC / "config"
    if src_config.exists():
        return src_config
    # ament share からの検索
    try:
        from ament_index_python.packages import get_package_share_directory
        share_dir = Path(get_package_share_directory('vlabor_launch'))
        share_config = share_dir / "config"
        if share_config.exists():
            return share_config
    except (ImportError, Exception):
        pass
    return src_config


def _get_profiles_dir() -> Path:
    """プロファイル YAML のディレクトリ"""
    return _get_config_dir() / "profiles"


def _get_assignments_dir() -> Path:
    """デバイス割り当てファイルの保存ディレクトリ"""
    d = _get_config_dir() / "device_assignments"
    d.mkdir(parents=True, exist_ok=True)
    return d


def load_profile_device_slots(profile_name: str) -> List[Dict[str, Any]]:
    """プロファイル YAML の devices セクションを読み込む"""
    profiles_dir = _get_profiles_dir()
    profile_path = profiles_dir / f"{profile_name}.yaml"
    if not profile_path.exists():
        return []
    try:
        with open(profile_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return []
    profile_data = data.get("profile", data)
    return profile_data.get("devices", []) or []


def load_assignments(profile_name: str) -> Dict[str, Any]:
    """保存済みデバイス割り当てを読み込む"""
    path = _get_assignments_dir() / f"{profile_name}.yaml"
    if not path.exists():
        return {}
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    except (OSError, yaml.YAMLError):
        return {}
    return data.get("assignments", {})


def save_assignments(
    profile_name: str,
    assignments: Dict[str, Dict[str, Any]],
) -> str:
    """デバイス割り当てを保存。保存先パスを返す。"""
    path = _get_assignments_dir() / f"{profile_name}.yaml"
    data = {
        "assignments": assignments,
        "_metadata": {
            "updated_at": datetime.now().isoformat(),
            "profile": profile_name,
        },
    }
    with open(path, "w", encoding="utf-8") as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
    return str(path)


def resolve_to_overrides(profile_name: str) -> Dict[str, str]:
    """
    保存済みデバイス割り当てを launch override 形式に変換する。
    戻り値例: {"left_serial_port": "/dev/ttyACM2", "right_serial_port": "/dev/ttyACM1"}
    """
    assignments = load_assignments(profile_name)
    overrides: Dict[str, str] = {}
    for key, entry in assignments.items():
        device_path = entry.get("device_path")
        if device_path:
            overrides[key] = device_path
    return overrides


def auto_assign(
    profile_name: str,
    available_devices: Optional[List[DetectedDevice]] = None,
) -> Dict[str, Dict[str, Any]]:
    """
    保存済みの match 情報を使い、現在接続されているデバイスに自動割り当てを試みる。
    照合優先度: serial_number(100) > vendor+product(50) > device_path(10)
    """
    if available_devices is None:
        available_devices = scan_all_devices()

    saved = load_assignments(profile_name)
    slots = load_profile_device_slots(profile_name)
    result: Dict[str, Dict[str, Any]] = {}
    used_paths: set = set()

    for slot in slots:
        key = slot.get("key", "")
        slot_type = slot.get("type", "serial")
        slot_label = slot.get("label", key)
        slot_default = slot.get("default", "")
        saved_entry = saved.get(key, {})
        match_info = saved_entry.get("match", {})

        best_device: Optional[DetectedDevice] = None
        best_score = -1

        candidates = [
            d for d in available_devices
            if d.device_type == slot_type and d.path not in used_paths
        ]

        for dev in candidates:
            score = 0
            # シリアルナンバー一致 = 最高スコア
            if (match_info.get("serial_number")
                    and dev.serial_number == match_info["serial_number"]):
                score = 100
            # vendor+product 一致
            elif (match_info.get("vendor_id") and match_info.get("product_id")
                  and dev.vendor_id == match_info["vendor_id"]
                  and dev.product_id == match_info["product_id"]):
                score = 50
            # パス一致 (フォールバック)
            if dev.path == saved_entry.get("device_path", slot_default):
                score = max(score, 10)

            if score > best_score:
                best_score = score
                best_device = dev

        if best_device:
            used_paths.add(best_device.path)
            result[key] = {
                "label": slot_label,
                "device_path": best_device.path,
                "match": {
                    "serial_number": best_device.serial_number,
                    "vendor_id": best_device.vendor_id,
                    "product_id": best_device.product_id,
                },
                "auto_assigned": True,
                "confidence": (
                    "high" if best_score >= 100
                    else "medium" if best_score >= 50
                    else "low"
                ),
            }
        else:
            # 候補なし: 保存済みパスまたはデフォルトをそのまま使用
            result[key] = {
                "label": slot_label,
                "device_path": saved_entry.get("device_path", slot_default),
                "match": match_info,
                "auto_assigned": False,
                "confidence": "none",
            }

    return result
