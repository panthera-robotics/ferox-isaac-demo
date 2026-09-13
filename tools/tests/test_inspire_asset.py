"""CPU admission regressions; fixtures are synthetic, not CAD qualification."""
import importlib.util
import math
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET

SPEC = importlib.util.spec_from_file_location("inspire_asset", Path(__file__).parents[1]/"inspire_asset.py")
asset = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(asset)


def synthetic_hand():
    root = ET.Element("robot", name="synthetic_FTP_contract_fixture")
    ET.SubElement(root, "link", name="right_wrist_yaw_link")
    def link(name, mass):
        value = ET.SubElement(root, "link", name=name)
        inertia = ET.SubElement(value, "inertial")
        ET.SubElement(inertia, "mass", value=str(mass))
        ET.SubElement(inertia, "inertia", ixx="0.001", iyy="0.001", izz="0.001", ixy="0", ixz="0", iyz="0")
    def joint(name, parent, child, upper=1.4381, mimic=None):
        value = ET.SubElement(root, "joint", name=name, type="revolute")
        ET.SubElement(value, "parent", link=parent); ET.SubElement(value, "child", link=child)
        ET.SubElement(value, "axis", xyz="0 0 1")
        ET.SubElement(value, "limit", lower="0", upper=str(upper), effort="10", velocity="1")
        if mimic:
            ET.SubElement(value, "mimic", joint=mimic[0], multiplier=str(mimic[1]), offset="0")
    link("right_base_link", .7583)
    fixed = ET.SubElement(root, "joint", name="right_base_joint", type="fixed")
    ET.SubElement(fixed, "parent", link="right_wrist_yaw_link")
    ET.SubElement(fixed, "child", link="right_base_link")
    ET.SubElement(fixed, "origin", xyz="0.0415 0 0", rpy="0 1.5707963267 0")
    for digit in ("little", "ring", "middle", "index"):
        for index in (1, 2):
            child = f"right_{digit}_{index}"; link(child, .01)
            joint(child+"_joint", "right_base_link" if index == 1 else f"right_{digit}_1", child,
                  upper=1.4381 if index == 1 else 3.14,
                  mimic=None if index == 1 else (f"right_{digit}_1_joint", 1.0843))
    for index in range(1, 5):
        child = f"right_thumb_{index}"; link(child, .01)
        mimic = None if index <= 2 else (f"right_thumb_{index-1}_joint", .8024 if index == 3 else .9487)
        joint(child+"_joint", "right_base_link" if index == 1 else f"right_thumb_{index-1}", child,
              upper={1:1.1641, 2:.5864, 3:.5, 4:3.14}[index], mimic=mimic)
    return root


class InspireAssetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)/"hand.urdf"
        self.root = synthetic_hand()
        self.save()

    def save(self):
        ET.ElementTree(self.root).write(self.path)

    def test_counts_order_chirality_and_mass_gap_are_explicit(self):
        audit = asset.audit_urdf(self.path)
        summary = asset.hand_summary(audit, "right")
        self.assertEqual(audit["authored_revolute_count"], 12)
        self.assertEqual(audit["independent_command_count"], 6)
        self.assertEqual(list(summary["semantic_to_donor_joint"]), list(asset.SEMANTICS))
        self.assertEqual(summary["semantic_to_donor_joint"]["thumb_bend"], "right_thumb_2_joint")
        self.assertAlmostEqual(summary["authored_mass_kg"], .8783)
        self.assertFalse(summary["within_target_nominal_hand_mass"])
        self.assertFalse(summary["exact_RH56E2_equivalence"])
        with self.assertRaises(ValueError):
            asset.hand_summary(audit, "left")

    def test_coupling_chain_and_per_joint_limits(self):
        audit = asset.audit_urdf(self.path)
        targets = {name: .1 for name in audit["independent_joint_names"]}
        q = asset.coupled_positions(audit, targets)
        self.assertAlmostEqual(q["right_thumb_4_joint"], .1*.8024*.9487)
        self.assertAlmostEqual(q["right_little_2_joint"], .10843)
        for value in (2, math.inf, math.nan, True):
            targets["right_thumb_2_joint"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                asset.coupled_positions(audit, targets)
        with self.assertRaises(ValueError):
            asset.coupled_positions(audit, {})

    def test_rejects_cyclic_and_missing_mimic_targets(self):
        mimic = self.root.find("joint[@name='right_thumb_3_joint']/mimic")
        for name in ("absent", "right_thumb_4_joint"):
            mimic.set("joint", name); self.save()
            with self.subTest(name=name), self.assertRaises(ValueError):
                asset.audit_urdf(self.path)

    def test_nonpositive_or_nonfinite_physical_parameters_fail(self):
        for tag, field, value in (("mass", "value", "-1"), ("mass", "value", "nan"),
                                  ("inertia", "ixx", "-0.001"), ("inertia", "ixy", "1"),
                                  ("inertia", "izz", "inf")):
            self.root = synthetic_hand()
            self.root.find(f"link[@name='right_base_link']/inertial/{tag}").set(field, value)
            self.save()
            with self.subTest(tag=tag, field=field, value=value), self.assertRaises(ValueError):
                asset.audit_urdf(self.path)

    def test_bench_removes_only_empty_fixture_preserves_mass_and_flange(self):
        before = asset.audit_urdf(self.path)
        output = self.path.parent/"bench.urdf"
        info = asset.bench_hand(self.path, output, "right")
        after = info["audit"]
        self.assertEqual(after["root_link"], "right_base_link")
        self.assertEqual(after["link_masses_kg"], before["link_masses_kg"])
        self.assertEqual(after["physical_joint_names"], before["physical_joint_names"])
        self.assertEqual(info["retained_flange_transform"]["xyz"], (.0415, 0., 0.))
        self.assertEqual(after["links_without_authored_inertia"], [])
        self.assertTrue(info["requires_explicit_fixed_base_fixture_in_simulator"])
        ET.SubElement(self.root.find("link[@name='right_wrist_yaw_link']"), "visual")
        self.save()
        with self.assertRaises(ValueError):
            asset.bench_hand(self.path, output, "right")

    def test_duplicate_or_disconnected_kinematic_topology_rejected(self):
        ET.SubElement(self.root, "link", name="orphan"); self.save()
        with self.assertRaises(ValueError):
            asset.audit_urdf(self.path)
        self.root = synthetic_hand()
        self.root.find("joint[@name='right_little_1_joint']/child").set("link", "right_thumb_1")
        self.save()
        with self.assertRaises(ValueError):
            asset.audit_urdf(self.path)


if __name__ == "__main__":
    unittest.main()
