"""src/main.py must be exactly what tools/build_main.py generates from src/stages/ (no drift)."""
import os, subprocess, sys
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
r = subprocess.run([sys.executable, os.path.join(ROOT, "tools", "build_main.py"), "--check"], capture_output=True, text=True)
print("PASS" if r.returncode == 0 else "FAIL: " + (r.stderr or r.stdout).strip())
sys.exit(r.returncode)
