"""Execute the inherited asset/sensor regression suite without hiding its exit."""
from pathlib import Path
import runpy
import shutil
import os
import sys

interfaces = sorted(p.name for p in Path('/sys/class/net').iterdir())
assert interfaces == ['lo'], interfaces
Path('/evidence/routes.txt').write_text(Path('/proc/net/route').read_text())
Path('/evidence/interfaces.txt').write_text('\n'.join(interfaces))
os.environ['TWIN_TEST_REPORT'] = '/evidence/results.txt'
os.environ['TWIN_TEST_RECEIPT'] = '/evidence/probe.json'
asset_root = Path('/source-assets')
assert (asset_root / 'Isaac/Sensors/NVIDIA/Example_Rotary.usda').is_file(), 'Pinned offline RTX sensor source required'
sys.argv.append('--/persistent/isaac/asset_root/default=' + str(asset_root))
try:
    runpy.run_path('/workspace/ferox_tools/tests/test_twin_isaac.py', run_name='__main__')
finally:
    result = Path('/tmp/twin_isaac_tests.txt')
    if result.exists():
        shutil.copyfile(result, '/evidence/results.txt')
