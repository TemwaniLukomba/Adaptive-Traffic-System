"""Locate SUMO programs (sumo, sumo-gui, netconvert) on this computer."""
import os
import shutil
from pathlib import Path

HELP = (
    "Could not find '{name}'. Install SUMO first:\n"
    "  pip install eclipse-sumo traci sumolib\n"
    "or install it from https://sumo.dlr.de and add its bin folder to PATH\n"
    "(or set SUMO_HOME). Check with:  {name} --version"
)


def find_binary(name):
    """Return the full path of a SUMO program, or exit with install help."""
    found = shutil.which(name)
    if found:
        return found

    bin_dirs = []
    if os.environ.get("SUMO_HOME"):
        bin_dirs.append(Path(os.environ["SUMO_HOME"]) / "bin")
    try:                                   # the pip package 'eclipse-sumo'
        import sumo                        # type: ignore
        bin_dirs.append(Path(sumo.SUMO_HOME) / "bin")
    except Exception:
        pass

    for folder in bin_dirs:
        for suffix in ("", ".exe"):
            candidate = folder / (name + suffix)
            if candidate.exists():
                return str(candidate)
    raise SystemExit(HELP.format(name=name))
