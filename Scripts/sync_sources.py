"""Keep the generated build files in step with the sources in Core/Src.

STM32CubeIDE regenerates subdir.mk and objects.list from the project whenever it
builds, so it picks up new source files on its own. Command-line builds do not
get that, and both files are marked "do not edit", so rather than hand-editing
them we rewrite their Core/Src entries from whatever is actually on disk. The
result is what CubeIDE would have produced anyway.
"""
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
SRC_DIR = ROOT / "Firmware" / "Core" / "Src"
SUBDIR_MK = ROOT / "Firmware" / "Debug" / "Core" / "Src" / "subdir.mk"
OBJECTS_LIST = ROOT / "Firmware" / "Debug" / "objects.list"

LISTS = {
    "C_SRCS": "../Core/Src/{}.c",
    "OBJS": "./Core/Src/{}.o",
    "C_DEPS": "./Core/Src/{}.d",
}

CORE_OBJ_PREFIX = "./Core/Src/"

# MATLAB Coder output, compiled at a higher optimisation level than the rest of
# the project. At -O0 the generated filter is about 117 kB of code and overflows
# the 128 kB part on its own; the hand-written firmware stays at -O0 so it is
# still comfortable to step through in a debugger.
CODEGEN_DIR = ROOT / "Simulation" / "Codegen" / "codegen" / "lib" / "filter_entry"
CODEGEN_OPT = "-O2"
BEGIN_MARK = "# BEGIN generated-source rules (managed by Scripts/sync_sources.py)"
END_MARK = "# END generated-source rules"


def generated_stems():
    if not CODEGEN_DIR.is_dir():
        return set()
    return {p.stem for p in CODEGEN_DIR.glob("*.c")}


def build_opt_rules(lines, stems):
    """Explicit per-file rules that override the -O0 pattern rule.

    The recipe is copied from the pattern rule already in the file and only the
    optimisation flag is swapped, so if CubeIDE ever changes the other flags
    these rules follow along instead of drifting.
    """
    recipe = None
    for i, line in enumerate(lines):
        if line.startswith("Core/Src/%.o") and i + 1 < len(lines):
            recipe = lines[i + 1]
            break
    if recipe is None or "-O0" not in recipe:
        return []

    out = [BEGIN_MARK]
    for stem in sorted(stems):
        out.append("Core/Src/%s.o Core/Src/%s.su Core/Src/%s.cyclo: "
                   "../Core/Src/%s.c Core/Src/subdir.mk" % (stem, stem, stem, stem))
        out.append(recipe.replace("-O0", CODEGEN_OPT, 1))
    out.append(END_MARK)
    return out


def sync_subdir_mk(stems):
    # CubeIDE emits a mix of CRLF and LF, so work line-wise rather than trying
    # to match newlines in a pattern.
    lines = SUBDIR_MK.read_text(encoding="utf-8").splitlines()

    out = []
    i = 0
    replaced = set()
    while i < len(lines):
        line = lines[i].rstrip()
        var = line.split("+=")[0].strip() if line.endswith("+= \\") else None
        if var in LISTS:
            # Skip the old list: its continuation lines, then one final line.
            i += 1
            while i < len(lines) and lines[i].rstrip().endswith("\\"):
                i += 1
            i += 1
            entries = [LISTS[var].format(s) for s in stems]
            out.append(var + " += \\")
            out.extend(e + " \\" for e in entries[:-1])
            out.append(entries[-1] + " ")
            replaced.add(var)
            continue
        out.append(lines[i])
        i += 1

    # Drop any previously injected block before adding a fresh one.
    if BEGIN_MARK in out:
        begin = out.index(BEGIN_MARK)
        end = out.index(END_MARK)
        del out[begin:end + 1]

    gen = generated_stems() & set(stems)
    if gen:
        rules = build_opt_rules(out, gen)
        if rules:
            anchor = next(i for i, l in enumerate(out) if l.startswith("clean:"))
            out[anchor:anchor] = rules + [""]

    missing = set(LISTS) - replaced
    if missing:
        print("Could not locate lists in subdir.mk: " + ", ".join(sorted(missing)),
              file=sys.stderr)
        return False

    SUBDIR_MK.write_text("\n".join(out) + "\n", encoding="utf-8", newline="")
    return True


def sync_objects_list(stems):
    """objects.list is what the linker actually reads, and is generated too."""
    lines = OBJECTS_LIST.read_text(encoding="utf-8").splitlines()
    kept = [ln for ln in lines if not ln.strip().strip('"').startswith(CORE_OBJ_PREFIX)]
    entries = ['"' + CORE_OBJ_PREFIX + s + '.o"' for s in stems]
    # Core/Src objects come first in CubeIDE's own output; keep that order.
    OBJECTS_LIST.write_text("\n".join(entries + kept) + "\n",
                            encoding="utf-8", newline="")


def main():
    stems = sorted(p.stem for p in SRC_DIR.glob("*.c"))
    if not stems:
        print("No sources found in " + str(SRC_DIR), file=sys.stderr)
        return 1

    if not sync_subdir_mk(stems):
        return 1
    sync_objects_list(stems)

    print("build files: %d sources (%s)" % (len(stems), ", ".join(stems)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
