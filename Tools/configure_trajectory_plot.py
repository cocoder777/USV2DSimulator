"""Remember the active Python for Unity's plot exporter; local configuration stays in Logs."""
from pathlib import Path
import sys

import matplotlib  # Fail here if this interpreter cannot generate plots.

if __name__ == "__main__":
    config = Path(__file__).resolve().parent.parent / "Logs" / "trajectory_python.txt"
    config.parent.mkdir(parents=True, exist_ok=True)
    config.write_text(sys.executable + "\n", encoding="utf-8")
    print(f"Configured plotting Python: {sys.executable} (matplotlib {matplotlib.__version__})")
