"""README.md tables must equal what the recorded evaluation files produce."""
import os
import subprocess
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")


def test_readme_tables_match_recorded_data():
    out = subprocess.run([sys.executable, "render_readme_tables.py", "--check"], cwd=ROOT, capture_output=True, text=True)
    assert out.returncode == 0, out.stdout + out.stderr
