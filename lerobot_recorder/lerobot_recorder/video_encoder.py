"""
Video Encoder - PNG sequence to MP4 conversion
"""
import subprocess
import shutil
from pathlib import Path
from typing import Optional
import logging

logger = logging.getLogger(__name__)


def check_ffmpeg() -> bool:
    """Check if ffmpeg is available."""
    return shutil.which('ffmpeg') is not None


def encode_video(
    image_dir: Path,
    output_path: Path,
    fps: int = 30,
    codec: str = 'libx264',
    pix_fmt: str = 'yuv420p',
    crf: int = 23,
    pattern: str = 'frame_%06d.png',
) -> bool:
    """
    Encode PNG sequence to MP4 video.

    Args:
        image_dir: Directory containing PNG images
        output_path: Output MP4 file path
        fps: Frame rate
        codec: Video codec (default: libx264)
        pix_fmt: Pixel format (default: yuv420p)
        crf: Constant Rate Factor (quality, lower=better, default: 23)
        pattern: Input file pattern (default: frame_%06d.png)

    Returns:
        True if successful, False otherwise
    """
    if not check_ffmpeg():
        logger.error('ffmpeg not found. Please install ffmpeg.')
        return False

    if not image_dir.exists():
        logger.error(f'Image directory not found: {image_dir}')
        return False

    # Ensure output directory exists
    output_path.parent.mkdir(parents=True, exist_ok=True)

    cmd = [
        'ffmpeg',
        '-y',  # Overwrite output
        '-framerate', str(fps),
        '-i', str(image_dir / pattern),
        '-c:v', codec,
        '-pix_fmt', pix_fmt,
        '-crf', str(crf),
        str(output_path)
    ]

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            check=True
        )
        logger.info(f'Video encoded: {output_path}')
        return True
    except subprocess.CalledProcessError as e:
        logger.error(f'ffmpeg failed: {e.stderr}')
        return False
    except Exception as e:
        logger.error(f'Video encoding failed: {e}')
        return False


def get_video_info(video_path: Path) -> Optional[dict]:
    """
    Get video information using ffprobe.

    Returns:
        Dictionary with video info or None if failed
    """
    if not video_path.exists():
        return None

    ffprobe = shutil.which('ffprobe')
    if not ffprobe:
        return None

    cmd = [
        'ffprobe',
        '-v', 'error',
        '-select_streams', 'v:0',
        '-show_entries', 'stream=width,height,r_frame_rate,nb_frames,duration',
        '-of', 'json',
        str(video_path)
    ]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, check=True)
        import json
        data = json.loads(result.stdout)
        if 'streams' in data and len(data['streams']) > 0:
            stream = data['streams'][0]
            # Parse frame rate (e.g., "30/1" -> 30.0)
            fps_str = stream.get('r_frame_rate', '30/1')
            if '/' in fps_str:
                num, den = fps_str.split('/')
                fps = float(num) / float(den)
            else:
                fps = float(fps_str)

            return {
                'width': stream.get('width'),
                'height': stream.get('height'),
                'fps': fps,
                'nb_frames': int(stream.get('nb_frames', 0)),
                'duration': float(stream.get('duration', 0)),
            }
    except Exception as e:
        logger.warning(f'Failed to get video info: {e}')

    return None


class VideoEncodingManager:
    """Manager for batch video encoding."""

    def __init__(self, fps: int = 30):
        self.fps = fps
        self._pending_encodings: list[tuple[Path, Path]] = []

    def add_pending(self, image_dir: Path, output_path: Path):
        """Add a pending encoding task."""
        self._pending_encodings.append((image_dir, output_path))

    def encode_all(self, cleanup: bool = True) -> list[Path]:
        """
        Encode all pending videos.

        Args:
            cleanup: If True, delete source PNG directories after encoding

        Returns:
            List of successfully encoded video paths
        """
        encoded = []
        for image_dir, output_path in self._pending_encodings:
            if encode_video(image_dir, output_path, fps=self.fps):
                encoded.append(output_path)
                if cleanup and image_dir.exists():
                    shutil.rmtree(image_dir)
                    logger.info(f'Cleaned up: {image_dir}')

        self._pending_encodings.clear()
        return encoded

    def clear(self):
        """Clear pending encodings without processing."""
        self._pending_encodings.clear()
