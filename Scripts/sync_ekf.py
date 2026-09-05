"""Copy the MATLAB Coder output for filter_entry into the firmware tree.

The generated sources live under Simulation/Codegen, which is where MATLAB
writes them and where they are version controlled. The firmware build compiles
whatever is in Firmware/Core, so this copies them across.

Putting them in Core/Src and Core/Inc rather than a folder of their own is
deliberate: both the command-line build and STM32CubeIDE already treat those two
directories as the project's source and include paths, so the generated code
needs no build configuration at all. Run this after regenerating from MATLAB.
"""
import filecmp
import pathlib
import shutil
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
CODEGEN = ROOT / "Simulation" / "Codegen" / "codegen" / "lib" / "filter_entry"
SRC_DEST = ROOT / "Firmware" / "Core" / "Src"
INC_DEST = ROOT / "Firmware" / "Core" / "Inc"


def main():
    if not CODEGEN.is_dir():
        print("No codegen output at " + str(CODEGEN), file=sys.stderr)
        return 1

    copied = []
    unchanged = []
    for pattern, dest in (("*.c", SRC_DEST), ("*.h", INC_DEST)):
        for src in sorted(CODEGEN.glob(pattern)):
            target = dest / src.name
            if target.exists() and filecmp.cmp(src, target, shallow=False):
                unchanged.append(src.name)
                continue
            shutil.copy2(src, target)
            copied.append(src.name)

    if copied:
        print("updated: " + ", ".join(copied))
    print("%d generated files in sync (%d unchanged)"
          % (len(copied) + len(unchanged), len(unchanged)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
