"""
device_scanner.py - 接続済み物理デバイスの検出

sysfs を読み取り、シリアルポートやカメラデバイスの情報を収集する。
外部依存なし（pyudev 等不要）。
"""
from __future__ import annotations

import glob
import os
import re
from dataclasses import dataclass, field
from typing import List, Optional


@dataclass
class DetectedDevice:
    """検出されたデバイス情報"""
    path: str                                   # /dev/ttyACM0
    device_type: str                            # "serial" | "camera"
    subsystem: str                              # "ttyACM" | "ttyUSB" | "video"
    vendor_id: Optional[str] = None             # USB vendor ID
    product_id: Optional[str] = None            # USB product ID
    serial_number: Optional[str] = None         # USB シリアルナンバー
    manufacturer: Optional[str] = None          # USB manufacturer
    product_name: Optional[str] = None          # USB product name
    symlinks: List[str] = field(default_factory=list)

    @property
    def display_name(self) -> str:
        """UI 表示用の名前"""
        parts = [self.path]
        if self.product_name:
            parts.append(f"({self.product_name})")
        elif self.manufacturer:
            parts.append(f"({self.manufacturer})")
        if self.serial_number:
            parts.append(f"[S/N: {self.serial_number}]")
        return " ".join(parts)

    def to_dict(self) -> dict:
        """JSON シリアライズ用"""
        return {
            'path': self.path,
            'device_type': self.device_type,
            'subsystem': self.subsystem,
            'vendor_id': self.vendor_id,
            'product_id': self.product_id,
            'serial_number': self.serial_number,
            'manufacturer': self.manufacturer,
            'product_name': self.product_name,
            'display_name': self.display_name,
            'symlinks': self.symlinks,
        }


def scan_serial_devices() -> List[DetectedDevice]:
    """シリアルデバイス (/dev/ttyACM*, /dev/ttyUSB*) を検出"""
    devices: List[DetectedDevice] = []
    for pattern in ["/dev/ttyACM*", "/dev/ttyUSB*"]:
        for path in sorted(glob.glob(pattern)):
            subsystem = "ttyACM" if "ttyACM" in path else "ttyUSB"
            dev = DetectedDevice(
                path=path,
                device_type="serial",
                subsystem=subsystem,
            )
            _enrich_from_sysfs(dev)
            devices.append(dev)
    return devices


def scan_camera_devices() -> List[DetectedDevice]:
    """カメラデバイス (/dev/video*) を検出（capture デバイスのみ）"""
    devices: List[DetectedDevice] = []
    for path in sorted(glob.glob("/dev/video*")):
        if not _is_capture_device(path):
            continue
        dev = DetectedDevice(
            path=path,
            device_type="camera",
            subsystem="video",
        )
        _enrich_from_sysfs(dev)
        devices.append(dev)
    return devices


def scan_all_devices() -> List[DetectedDevice]:
    """全デバイスを検出"""
    return scan_serial_devices() + scan_camera_devices()


def _enrich_from_sysfs(dev: DetectedDevice) -> None:
    """sysfs からデバイスの USB 詳細情報を取得"""
    basename = os.path.basename(dev.path)
    if dev.device_type == "serial":
        sysfs_base = f"/sys/class/tty/{basename}/device"
    else:
        sysfs_base = f"/sys/class/video4linux/{basename}/device"

    # USB 親デバイスをたどって情報取得
    usb_device_path = _find_usb_parent(sysfs_base)
    if usb_device_path:
        dev.vendor_id = _read_sysfs(usb_device_path, "idVendor")
        dev.product_id = _read_sysfs(usb_device_path, "idProduct")
        dev.serial_number = _read_sysfs(usb_device_path, "serial")
        dev.manufacturer = _read_sysfs(usb_device_path, "manufacturer")
        dev.product_name = _read_sysfs(usb_device_path, "product")

    # by-id / by-path シンボリックリンクの逆引き
    for link_dir in ["/dev/serial/by-id", "/dev/serial/by-path"]:
        if os.path.isdir(link_dir):
            try:
                for link in os.listdir(link_dir):
                    link_path = os.path.join(link_dir, link)
                    if os.path.realpath(link_path) == os.path.realpath(dev.path):
                        dev.symlinks.append(link_path)
            except OSError:
                pass


def _find_usb_parent(sysfs_path: str) -> Optional[str]:
    """sysfs パスから USB 親デバイスを検索"""
    try:
        current = os.path.realpath(sysfs_path)
    except OSError:
        return None
    for _ in range(10):
        if os.path.exists(os.path.join(current, "idVendor")):
            return current
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent
    return None


def _read_sysfs(base: str, attr: str) -> Optional[str]:
    """sysfs ファイルを読み取る"""
    path = os.path.join(base, attr)
    try:
        with open(path, "r") as f:
            return f.read().strip()
    except (OSError, IOError):
        return None


def _is_capture_device(path: str) -> bool:
    """V4L2 キャプチャデバイスかどうかを判定（metadata デバイスを除外）"""
    basename = os.path.basename(path)
    name_path = f"/sys/class/video4linux/{basename}/name"
    if os.path.exists(name_path):
        try:
            with open(name_path) as f:
                name = f.read().strip()
            if "metadata" in name.lower():
                return False
        except OSError:
            pass
    return True
