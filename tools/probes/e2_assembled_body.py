"""Build the ROBOT_USD_DIR-compatible assembled-body directories of the PUBLIC exact-E2 prior asset (lane A, hdR-e2usd-import).

No physics stepping, no policy, no hardware: the Kit URDF importer runs once per declared variant (probe-config `variants`:
name + URDF file under /source-assets) with the donor's importer settings (inspire_e2_asset.import_body_e2, floating base like
the donor-assembled-body variant), and each result is laid out as <name>/g1.usd + configuration/ + PROVENANCE_assembled_asset.json
+ SHA256SUMS. Receipt: probe.json (PASS when every variant imported and its layout verified).
"""
import hashlib, json, os, shutil, sys, time
from pathlib import Path
assert os.environ.get('PANTHERA_SIM_AUTHORIZED') == '1'
assert sorted(p.name for p in Path('/sys/class/net').iterdir()) == ['lo']
assert os.environ.get('PANTHERA_PROBE_MODE') == 'e2-assembled-body-import'
config = json.loads(Path(os.environ['PANTHERA_PROBE_CONFIG']).read_text())
assert config['schema_version'] == 1 and config['hardware_authorized'] is False and config['variants']
out = Path('/evidence'); sys.path.insert(0, '/workspace/ferox_tools')
t0 = time.time()
from isaacsim import SimulationApp
app = SimulationApp({'headless': True, 'renderer': 'RaytracedLighting'})
from pxr import Usd, UsdPhysics
from inspire_e2_asset import import_body_e2

def sha(p):
    return hashlib.sha256(Path(p).read_bytes()).hexdigest()

results = {}; ok_all = True
for v in config['variants']:
    name = v['name']; src = Path('/source-assets') / v['urdf']; work = out / ('import_' + name); work.mkdir()
    rec = {'urdf': v['urdf'], 'urdf_sha256': sha(src), 'expected_urdf_sha256': v.get('urdf_sha256'), 'status': 'FAIL'}
    try:
        if v.get('urdf_sha256') and v['urdf_sha256'] != rec['urdf_sha256']:
            raise RuntimeError('mounted URDF differs from the declared hash')
        asset, facts = import_body_e2(src, work, fixed_base=bool(config.get('fixed_base', False)))
        stage = Usd.Stage.Open(str(asset)); n_joints = len([p for p in stage.Traverse() if p.IsA(UsdPhysics.RevoluteJoint)])
        arts = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        # ROBOT_USD_DIR layout: g1.usd + configuration/ (the importer's own layers) + provenance + hashes
        dest = out / name; dest.mkdir(); shutil.copyfile(asset, dest / 'g1.usd')
        cfg_dir = work / 'configuration'
        if cfg_dir.is_dir(): shutil.copytree(cfg_dir, dest / 'configuration')
        prov = dict(facts); prov.update(variant=name, label=v.get('label'), description=v.get('description'), importer='isaacsim.asset.importer.urdf (URDFParseAndImportFile)',
                    import_settings={'distance_scale': 1.0, 'merge_fixed_joints': False, 'fix_base': bool(config.get('fixed_base', False)), 'import_inertia_tensor': True, 'convex_decomp': True, 'self_collision': True, 'parse_mimic': True, 'default_drive_type': 'JOINT_DRIVE_NONE'},
                    usd_revolute_joints=n_joints, articulation_roots=arts, run_id=os.environ.get('PANTHERA_SIM_RUN_ID'), hardware_authorized=False, gpu_job='hdR-e2usd-import')
        (dest / 'PROVENANCE_assembled_asset.json').write_text(json.dumps(prov, indent=2, allow_nan=False))
        lines = []
        for f in sorted(p for p in dest.rglob('*') if p.is_file() and p.name != 'SHA256SUMS'):
            lines.append('%s  %s' % (sha(f), f.relative_to(dest)))
        (dest / 'SHA256SUMS').write_text('\n'.join(lines) + '\n')
        rec.update(status='PASS', g1_usd_sha256=sha(dest / 'g1.usd'), configuration_files=sorted(p.name for p in (dest / 'configuration').iterdir()) if (dest / 'configuration').is_dir() else [],
                   usd_revolute_joints=n_joints, articulation_roots=arts, imported_physical_mass_kg=facts['imported_physical_mass_kg'], hand_independent=facts['hand_independent_names'], folded_flange_frames=facts['folded_flange_frames'])
        assert n_joints == 53 and len(arts) == 1
    except Exception as e:   # the receipt records the failure; the launcher keeps the evidence
        rec['error'] = repr(e); ok_all = False
    results[name] = rec
(out / 'metrics.json').write_text(json.dumps({'variants': results, 'wall_s': round(time.time() - t0, 1), 'config': config}, indent=2))
artifacts = [str(p.relative_to(out)) for p in out.rglob('*') if p.is_file() and p.name not in ['run.json', 'probe.json', 'console.log', 'executed_probe.py', 'executed_launcher.py', 'uncommitted.patch']]
(out / 'probe.json').write_text(json.dumps({'status': 'PASS' if ok_all else 'FAIL', 'scope': 'e2_assembled_body_usd_import_no_physics', 'metrics': 'metrics.json', 'artifacts': artifacts}))
app.close()
