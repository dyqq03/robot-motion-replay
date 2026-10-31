"""Read-only FT encoder/joint coordinates for the local shadow model.

Use the very same HandCalibration class as the hardware UI. Never construct a
HandBus, open a serial port, or treat an encoder radian as a model joint radian.
These files define encoder zero, not an independently measured mesh alignment.
"""
import copy
import hashlib
import json
import math
from pathlib import Path
import sys

from joint_calibration import validate_hand_sources

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "unified_ui/app"))
from hand_io import load_specs


class CalibratedHandPose:
    def __init__(self, root=ROOT, mapping=None):
        self.root = Path(root)
        self.mapping = mapping or json.loads((self.root / "configs/hand_mapping.json").read_text())
        validate_hand_sources(self.mapping, self.root)
        self.settings, _, self.models = load_specs(self.root)
        self.specs = {s["source_joint"]: s for s in self.mapping["joints"]}
        self.revision = self.settings["calibration_revision"]
        expected = {side + "/" + n for side in ("left", "right") for n in self.models[side + "_hand"].names}
        if len(self.specs) != 32 or set(self.specs) != expected:
            raise ValueError("Hand mapping must cover the 32 calibrated joints")
        for side in ("left", "right"):
            model = self.models[side + "_hand"]
            for name, mid, inverted in zip(model.names, model.ids, model.inverted):
                s = self.specs[side + "/" + name]
                if s.get("motor_id") != mid or s.get("motor_direction") != (-1 if inverted else 1):
                    raise ValueError("Hand mapping motor identity differs from calibration: " + side + "/" + name)

    def from_joint_degrees(self, positions):
        """Already calibrated degrees: apply only joint-to-model sign/offset."""
        if set(positions) - set(self.specs):
            raise ValueError("Unknown hand joint names")
        result = {}
        for name, raw in positions.items():
            if isinstance(raw, bool) or not isinstance(raw, (int, float)) or not math.isfinite(raw):
                raise ValueError("Invalid hand joint angle: " + name)
            s = self.specs[name]
            value = math.radians(raw)
            lo, hi = s["real_limits_rad"]
            if not lo - 1e-6 <= value <= hi + 1e-6:
                raise ValueError("Hand joint outside calibrated limits: " + name)
            result[s["model_joint"]] = s["sign"] * value + s["offset_rad"]
        return result

    def from_encoders(self, payload):
        """Side -> {unit:rad, motor_ids, position[, wrap_offsets_rad, serial]}.

        Inputs are motor radians ordered by the explicit IDs, not joint names.
        An explicit wrap array preserves the driver's selected encoder branch;
        otherwise initialize the branch exactly as FT_core does at connection.
        """
        if not payload or set(payload) - {"left", "right"}:
            raise ValueError("Encoder input must contain left and/or right")
        joint_degrees = {}
        for side, data in payload.items():
            if data.get("unit") != "rad":
                raise ValueError("Encoder inputs must use motor radians")
            bus = self.settings["hands"][side]
            if data.get("serial", bus["serial"]) != bus["serial"]:
                raise ValueError("Wrong hand USB identity: " + side)
            if data.get("calibration_revision", self.revision) != self.revision:
                raise ValueError("Wrong hand calibration revision: " + side)
            ids, values = data["motor_ids"], data["position"]
            if (len(ids) != 16 or len(values) != 16 or any(type(i) is not int for i in ids)
                    or set(ids) != set(range(1, 17))):
                raise ValueError("Exactly 16 distinct motor IDs 1-16 are required")
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
                raise ValueError("Encoder positions must be finite motor radians")
            model = copy.deepcopy(self.models[side + "_hand"])
            by_id = dict(zip(ids, values))
            ordered = [by_id[mid] for mid in model.ids]
            wraps = data.get("wrap_offsets_rad")
            if wraps is None:
                model.initialize_wrap(ordered)
            else:
                if len(wraps) != 16 or any(isinstance(v, bool) or not isinstance(v, (int, float))
                        or not math.isfinite(v) or min(abs(v - k * 2 * math.pi) for k in (-1, 0, 1)) > 1e-9 for v in wraps):
                    raise ValueError("Wrap offsets must be 0 or +/-2pi motor radians")
                by_id = dict(zip(ids, wraps))
                model.wrap = [by_id[mid] for mid in model.ids]
            joint_degrees.update({side + "/" + n: q for n, q in zip(model.names, model.to_joint(ordered))})
        return self.from_joint_degrees(joint_degrees), joint_degrees

    def encoder_zero(self):
        """Derive encoder positions at calibrated joint 0 from both YAML files."""
        payload = {}
        for side in ("left", "right"):
            model = copy.deepcopy(self.models[side + "_hand"])
            # The calibration file's own unwrapped branch, with no serial reads.
            model.wrap = [0.0] * 16
            payload[side] = dict(unit="rad", motor_ids=model.ids,
                                 position=model.to_motor([0.0] * 16),
                                 wrap_offsets_rad=[0.0] * 16,
                                 serial=self.settings["hands"][side]["serial"],
                                 calibration_revision=self.revision)
        return payload

    def zero_model_pose(self):
        pose, decoded = self.from_encoders(self.encoder_zero())
        if any(abs(v) > 1e-9 for v in decoded.values()):
            raise ValueError("Encoder zero failed to map to calibrated joint zero")
        # Suppress floating-point cancellation at zero; preserve real offsets.
        return {n: 0.0 if abs(q) < 1e-12 else q for n, q in pose.items()}

    def report(self):
        files = {}
        for side, bus in self.settings["hands"].items():
            for filename in ("config.yaml", "calibration.yaml"):
                path = Path(bus["model_path"]) / filename
                path = path if path.is_absolute() else self.root / path
                files[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
        return dict(calibration_revision=self.revision, files_sha256=files,
                    encoder_zero=self.encoder_zero(), model_zero=self.zero_model_pose(),
                    zero_source="FT joint ROM, motor limits, ratios and motor direction",
                    encoder_formula="q_deg=q_min+(motor-wrap-m_min)/ratio; inverted: q_max-(motor-wrap-m_min)/ratio",
                    calibrated_joint_input="degrees; do not apply motor conversion a second time",
                    physical_geometry_zero_verified=bool(self.mapping.get("physical_zero_verified")),
                    robot_connected=False)


def calibrated_hand_zero_pose(mapping=None):
    return CalibratedHandPose(mapping=mapping).zero_model_pose()
