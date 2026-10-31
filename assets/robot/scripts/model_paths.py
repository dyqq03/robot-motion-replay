"""Single active URDF source for model conversion and calibration tools."""
from pathlib import Path
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
SOURCE_URDF = ROOT / "URDF_90kg_with_hands/urdf/humanoid_with_hands.urdf"


def model_reference_point(xyz):
    """Translate historical camera/IK points with the Base visual geometry."""
    origin = ET.parse(SOURCE_URDF).find("link[@name='Base']/visual/origin")
    offset = [float(v) for v in origin.get("xyz", "0 0 0").split()] if origin is not None else [0., 0., 0.]
    return [float(v) + delta for v, delta in zip(xyz, offset)]
