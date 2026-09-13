"""Validate source selection before Kit imports; no simulation is started."""
import ast
from pathlib import Path
import unittest


SOURCE = Path(__file__).resolve().parents[1] / "probes/moving_hand.py"
TREE = ast.parse(SOURCE.read_text())
FUNCTION = next(node for node in TREE.body if isinstance(node, ast.FunctionDef)
                and node.name == "hand_probe_configuration")
NAMESPACE = {}
exec(compile(ast.Module(body=[FUNCTION], type_ignores=[]), str(SOURCE), "exec"), NAMESPACE)
configure = NAMESPACE["hand_probe_configuration"]


class HandSourceSelectionTests(unittest.TestCase):
    def test_historical_right_defaults_and_solver_variants_preserved(self):
        result = configure({}, "component-palm-sweeps")
        self.assertEqual(result, {"hand_side": "right", "palm_candidate_id": "ftp_palm_components_v1",
            "solver_velocity_iterations": 8, "source_filename": "FTP_right_hand_bench.urdf",
            "imported_root": "/Rhand", "usd_filename": "ftp_right_bench.usd"})
        for iterations in (8, 16, 32):
            for mode in ("component-palm-sweeps", "component-palm-blocked", "component-palm-wrist"):
                value = configure({"palm_candidate_id": "ftp_palm_yz_slabs_v2", "solver_velocity_iterations": iterations}, mode)
                self.assertEqual(value["hand_side"], "right")
                self.assertEqual(value["solver_velocity_iterations"], iterations)

    def test_left_selection_keeps_its_own_geometry_source_and_artifacts(self):
        result = configure({"hand_side": "left"}, "component-palm-blocked")
        self.assertEqual(result["source_filename"], "FTP_left_hand_bench.urdf")
        self.assertEqual(result["imported_root"], "/Lhand")
        self.assertEqual(result["palm_candidate_id"], "ftp_left_palm_yz_slabs_v1")
        self.assertEqual(result["usd_filename"], "ftp_left_bench.usd")
        self.assertEqual(result["solver_velocity_iterations"], 8)

    def test_wrong_side_candidates_and_unsupported_fixture_fail_before_import(self):
        for config, mode in [({"hand_side": "left"}, "component-palm-wrist"),
            ({"hand_side": "left", "palm_candidate_id": "ftp_palm_yz_slabs_v2"}, "component-palm-sweeps"),
            ({"palm_candidate_id": "ftp_left_palm_yz_slabs_v1"}, "component-palm-sweeps"),
            ({"hand_side": "mirrored"}, "component-palm-sweeps"),
            ({"solver_velocity_iterations": True}, "component-palm-sweeps"),
            ({"solver_velocity_iterations": 64}, "component-palm-sweeps"),
            ({"disable_collisions": True}, "component-palm-sweeps")]:
            with self.subTest(config=config, mode=mode), self.assertRaises(ValueError):
                configure(config, mode)

    def test_source_resolution_precedes_audit_call(self):
        assignments = {node.targets[0].id: i for i, node in enumerate(TREE.body)
            if isinstance(node, ast.Assign) and isinstance(node.targets[0], ast.Name)}
        self.assertLess(assignments["selection"], assignments["source"])
        self.assertLess(assignments["source"], assignments["facts"])


if __name__ == "__main__": unittest.main()
