#!/usr/bin/env bash
#
# Run a scan launch file and capture everything about the run into
# dump/<timestamp>/ so nothing has to be selected out of a terminal by hand.
#
#   ./src/arm_scan_demo/scripts/run_scan.sh
#   ./src/arm_scan_demo/scripts/run_scan.sh test.launch.py
#   ./src/arm_scan_demo/scripts/run_scan.sh test.launch.py scan_start_delay:=45.0
#
# Each run directory holds:
#   console.log      raw terminal output, escape codes and all
#   console-clean.log same, ANSI stripped and \r progress lines split apart
#   summary.txt      scan tally, waypoint outcomes, collisions, errors
#   environment.txt  package prefixes, git state, ROS env vars
#   ros_log/         everything ROS 2 normally buries in ~/.ros/log/

set -uo pipefail

LAUNCH_FILE="ur10e_sim_demo.launch.py"
KILL_STALE=0
LAUNCH_ARGS=()

for arg in "$@"; do
    case "$arg" in
        --kill-stale)
            KILL_STALE=1
            ;;
        *.launch.py)
            LAUNCH_FILE="$arg"
            ;;
        *)
            LAUNCH_ARGS+=("$arg")
            ;;
    esac
done

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
RUN_DIR="$REPO_ROOT/dump/$(date +%Y%m%d-%H%M%S)"

# ----------------------------------------------------------------------------
# Refuse to start on top of a stale Gazebo.
#
# A gzserver left over from a previous run keeps serving /spawn_entity, so the
# new gzserver exits 255, spawn_entity reports "Entity [ur] already exists",
# the controllers fail to configure, and the scan then runs against the *old*
# robot. That failure mode is quiet and it invalidates the whole run.
# ----------------------------------------------------------------------------
STALE_PIDS="$(pgrep -f 'gzserver|gzclient' || true)"

if [ -n "$STALE_PIDS" ]; then
    echo "ERROR: Gazebo is already running (pids: $(echo "$STALE_PIDS" | tr '\n' ' '))" >&2
    echo "" >&2
    echo "Starting now would silently reuse the old robot: the new gzserver dies" >&2
    echo "with exit code 255, spawn_entity reports 'Entity [ur] already exists'," >&2
    echo "and the scan executes against whatever was loaded before." >&2
    echo "" >&2

    if [ "$KILL_STALE" -eq 0 ]; then
        echo "Re-run with --kill-stale, or clear it yourself:" >&2
        echo "    pkill -9 -f 'gzserver|gzclient'" >&2
        exit 1
    fi

    echo "--kill-stale given; killing them." >&2
    pkill -9 -f 'gzserver|gzclient' || true
    sleep 2
fi

mkdir -p "$RUN_DIR/ros_log"

# Keep ROS 2's own per-node logs with the run instead of in ~/.ros/log/.
export ROS_LOG_DIR="$RUN_DIR/ros_log"

# ----------------------------------------------------------------------------
# Environment snapshot, so a log can be matched to the tree that produced it.
# ----------------------------------------------------------------------------
{
    echo "date:        $(date -Is)"
    echo "launch file: $LAUNCH_FILE"
    echo "launch args: ${LAUNCH_ARGS[*]:-<none>}"
    echo "host:        $(uname -a)"
    echo ""
    echo "== git =="
    git -C "$REPO_ROOT" log -1 --oneline 2>&1
    git -C "$REPO_ROOT" status --short 2>&1
    echo ""
    echo "== package prefixes =="
    for pkg in arm_scan_demo ur10e_tool_description ur_moveit_config \
               ur_simulation_gazebo ur_description pymoveit2; do
        echo "$pkg: $(ros2 pkg prefix "$pkg" 2>&1)"
    done
    echo ""
    echo "== ros env =="
    printenv | grep -E '^(ROS_|AMENT_|COLCON_|GAZEBO_|LD_LIBRARY)' | sort
} > "$RUN_DIR/environment.txt" 2>&1

echo "Logging this run to: $RUN_DIR"
echo ""

# ----------------------------------------------------------------------------
# Run it.
# ----------------------------------------------------------------------------
ros2 launch arm_scan_demo "$LAUNCH_FILE" "${LAUNCH_ARGS[@]}" 2>&1 \
    | tee "$RUN_DIR/console.log"

LAUNCH_STATUS="${PIPESTATUS[0]}"

# ----------------------------------------------------------------------------
# Readable copy: drop ANSI colour, and turn the progress reporter's \r
# overwrites into real lines so grep and editors can cope with them.
# ----------------------------------------------------------------------------
sed -e 's/\x1b\[[0-9;?]*[a-zA-Z]//g' \
    -e 's/\r/\n/g' \
    "$RUN_DIR/console.log" > "$RUN_DIR/console-clean.log"

CLEAN="$RUN_DIR/console-clean.log"

# ----------------------------------------------------------------------------
# Summary. Ordered so the things that invalidate a run come first.
# ----------------------------------------------------------------------------
{
    echo "launch exit status: $LAUNCH_STATUS"
    echo ""

    echo "== run-invalidating failures =="
    grep -nE "process has died|Failed to configure controller|Spawn service failed|already exists|Semantic description is not specified" "$CLEAN" \
        || echo "(none)"
    echo ""

    echo "== scan result =="
    grep -E "Sphere scan complete|Total scan duration|Starting scan with" "$CLEAN" \
        || echo "(scan never reported a result)"
    echo ""

    echo "== waypoint outcomes =="
    grep -oE "Waypoint [0-9]+/[0-9]+: (SUCCESS|FAILED) in [0-9.]+s" "$CLEAN" \
        || echo "(none)"
    echo ""

    echo "== collision pairs (most frequent first) =="
    grep -oE "contact between '[^']+' \(type '[^']+'\) and '[^']+'" "$CLEAN" \
        | sort | uniq -c | sort -rn \
        || echo "(none)"
    echo ""

    echo "== python tracebacks =="
    grep -A 20 "Traceback (most recent call last)" "$CLEAN" || echo "(none)"
    echo ""

    echo "== distinct errors =="
    grep -E "\[ERROR\]" "$CLEAN" \
        | sed -E 's/\[[0-9]+\.[0-9]+\]//g; s/pid [0-9]+/pid N/g' \
        | sort | uniq -c | sort -rn | head -40 \
        || echo "(none)"
} > "$RUN_DIR/summary.txt" 2>&1

echo ""
echo "============================================================"
cat "$RUN_DIR/summary.txt"
echo "============================================================"
echo "Full logs: $RUN_DIR"

exit "$LAUNCH_STATUS"
