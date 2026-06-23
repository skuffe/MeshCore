# Stamp a short, self-identifying build id into FIRMWARE_BUILD_DATE (shown by `ver`
# and the OLED). It is the firmware repo's short commit SHA plus a "-dirty" flag when
# the tree has uncommitted changes, so builds are traceable to a commit and an
# uncommitted build is obvious. Re-evaluated every build.
#
# LENGTH IS LOAD-BEARING: UITask renders "<version> (<FIRMWARE_BUILD_DATE>)" into a
# fixed char _version_info[32] with an UNBOUNDED sprintf (upstream examples/.../UITask).
# A long value (e.g. a full `git describe` + timestamp) overflows that buffer and the
# node hard-faults at boot. So keep this SHORT and capped — do not reintroduce a
# timestamp or the tag/commits-ahead prefix here.
import subprocess

Import("env")  # noqa: F821  (injected by PlatformIO/SCons)

proj = env["PROJECT_DIR"]

def git(*args):
    return subprocess.check_output(["git", *args], cwd=proj, stderr=subprocess.DEVNULL).decode().strip()

try:
    rev = git("rev-parse", "--short=8", "HEAD")
    # --quiet exits non-zero when tracked files differ from HEAD => dirty tree.
    dirty = subprocess.call(["git", "diff", "--quiet", "HEAD", "--"], cwd=proj) != 0
    build_id = rev + ("-dirty" if dirty else "")
except Exception:
    build_id = "nogit"

# Hard cap so "<version> (<build_id>)" always fits UITask's char[32] (see note above).
build_id = build_id[:18]

# Drop any FIRMWARE_BUILD_DATE inherited from build_flags so there's no -D clash.
env["CPPDEFINES"] = [
    d for d in env.get("CPPDEFINES", [])
    if not (isinstance(d, (list, tuple)) and d and d[0] == "FIRMWARE_BUILD_DATE")
    and d != "FIRMWARE_BUILD_DATE"
]
env.Append(CPPDEFINES=[("FIRMWARE_BUILD_DATE", env.StringifyMacro(build_id))])
print("build id: " + build_id)
