#!/usr/bin/env bash
#
# NIX Wi-Fi DEBUG smoke test.
#
# Function:
#   Temporarily connect this computer to the robot Wi-Fi, point LCM multicast to
#   that Wi-Fi interface, verify DEBUG status/feedback, and optionally send one
#   short WAIST[0] hold command using the current feedback position.
#
# Usage:
#   bash scripts/nix_wifi_debug_smoke.sh
#   bash scripts/nix_wifi_debug_smoke.sh --sn 005 --wifi-iface wlp3s0
#   bash scripts/nix_wifi_debug_smoke.sh --diagnose-only
#   bash scripts/nix_wifi_debug_smoke.sh --control-waist-hold
#   bash scripts/nix_wifi_debug_smoke.sh --no-restore-wifi
#
# Safety:
#   By default this script does not publish joint commands. Add
#   --control-waist-hold only after the robot is supported, emergency stop is
#   available, and the operator accepts a short DEBUG hold command.

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SDK_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

SN="005"
ROBOT_SSID=""
ROBOT_PASSWORD=""
WIFI_IFACE=""
RESTORE_WIFI=1
CONTROL_WAIST_HOLD=0
DIAGNOSE_ONLY=0
SKIP_PASSIVE_CHECK=0
ENTER_TIMEOUT=25
STAND_SETTLE=2
FEEDBACK_TIMEOUT_MS=5000
PASSIVE_CHECK_SECONDS=4
RESTORE_CONN=""
PYTHON_BIN=""
ENTERED_DEBUG=0
LEFT_DEBUG=0
LCM_PASSIVE_LOG=""

log() {
  printf '[wifi-debug-smoke] %s\n' "$*"
}

die() {
  log "ERROR: $*"
  exit 1
}

usage() {
  printf '%s\n' \
    'NIX Wi-Fi DEBUG smoke test.' \
    '' \
    'Function:' \
    '  Temporarily connect this computer to the robot Wi-Fi, point LCM multicast to' \
    '  that Wi-Fi interface, verify DEBUG status/feedback, and optionally send one' \
    '  short WAIST[0] hold command using the current feedback position.' \
    '' \
    'Usage:' \
    '  bash scripts/nix_wifi_debug_smoke.sh' \
    '  bash scripts/nix_wifi_debug_smoke.sh --sn 005 --wifi-iface wlp3s0' \
    '  bash scripts/nix_wifi_debug_smoke.sh --diagnose-only' \
    '  bash scripts/nix_wifi_debug_smoke.sh --control-waist-hold' \
    '  bash scripts/nix_wifi_debug_smoke.sh --no-restore-wifi' \
    '' \
    'Safety:' \
    '  By default this script does not publish joint commands. Add' \
    '  --control-waist-hold only after the robot is supported, emergency stop is' \
    '  available, and the operator accepts a short DEBUG hold command.'
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sn)
      SN="${2:-}"
      shift 2
      ;;
    --ssid)
      ROBOT_SSID="${2:-}"
      shift 2
      ;;
    --password)
      ROBOT_PASSWORD="${2:-}"
      shift 2
      ;;
    --wifi-iface)
      WIFI_IFACE="${2:-}"
      shift 2
      ;;
    --restore-connection)
      RESTORE_CONN="${2:-}"
      shift 2
      ;;
    --no-restore-wifi)
      RESTORE_WIFI=0
      shift
      ;;
    --control-waist-hold)
      CONTROL_WAIST_HOLD=1
      shift
      ;;
    --diagnose-only)
      DIAGNOSE_ONLY=1
      shift
      ;;
    --skip-passive-check)
      SKIP_PASSIVE_CHECK=1
      shift
      ;;
    --enter-timeout)
      ENTER_TIMEOUT="${2:-}"
      shift 2
      ;;
    --stand-settle)
      STAND_SETTLE="${2:-}"
      shift 2
      ;;
    --feedback-timeout-ms)
      FEEDBACK_TIMEOUT_MS="${2:-}"
      shift 2
      ;;
    --passive-check-seconds)
      PASSIVE_CHECK_SECONDS="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      die "unknown argument: $1"
      ;;
  esac
done

if [[ -z "$ROBOT_SSID" ]]; then
  ROBOT_SSID="nix_NIX${SN}"
fi
if [[ -z "$ROBOT_PASSWORD" ]]; then
  ROBOT_PASSWORD="${ROBOT_SSID}_pd"
fi

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "missing command: $1"
}

detect_wifi_iface() {
  if [[ -n "$WIFI_IFACE" ]]; then
    return
  fi
  WIFI_IFACE="$(nmcli -t -f DEVICE,TYPE,STATE device status \
    | awk -F: '$2 == "wifi" && $1 !~ /^p2p-/ {print $1; exit}')"
  [[ -n "$WIFI_IFACE" ]] || die "could not detect Wi-Fi interface; pass --wifi-iface"
}

detect_restore_connection() {
  if [[ -n "$RESTORE_CONN" ]]; then
    return
  fi
  RESTORE_CONN="$(nmcli -t -f NAME,DEVICE,TYPE,STATE connection show --active \
    | awk -F: -v iface="$WIFI_IFACE" -v robot="$ROBOT_SSID" \
      '$2 == iface && $3 == "802-11-wireless" && $4 == "activated" && $1 != robot {print $1; exit}')"
}

pick_python() {
  local candidate
  for candidate in "${PYTHON:-}" python3 /home/lumos/miniforge3/bin/python3; do
    [[ -n "$candidate" ]] || continue
    if command -v "$candidate" >/dev/null 2>&1 && "$candidate" -c 'import lcm' >/dev/null 2>&1; then
      PYTHON_BIN="$candidate"
      return
    fi
  done
  die "no Python with importable lcm binding found; install lcm in the active environment"
}

restore_wifi() {
  local status=$?
  trap - EXIT INT TERM
  if [[ "$ENTERED_DEBUG" -eq 1 && "$LEFT_DEBUG" -eq 0 && -n "$PYTHON_BIN" ]]; then
    log "attempting to leave DEBUG before restoring Wi-Fi"
    "$PYTHON_BIN" "$SDK_ROOT/python/nix_debug_state.py" leave \
      --timeout "$ENTER_TIMEOUT" >/dev/null 2>&1 || \
      log "WARNING: failed to leave DEBUG during cleanup"
  fi
  if [[ "$RESTORE_WIFI" -eq 1 && -n "$RESTORE_CONN" ]]; then
    log "restoring Wi-Fi connection: ${RESTORE_CONN}"
    nmcli connection up "$RESTORE_CONN" >/dev/null 2>&1 || \
      log "WARNING: failed to restore Wi-Fi connection: ${RESTORE_CONN}"
  elif [[ "$RESTORE_WIFI" -eq 1 ]]; then
    log "no previous Wi-Fi connection recorded; leaving current Wi-Fi unchanged"
  else
    log "--no-restore-wifi set; leaving current Wi-Fi unchanged"
  fi
  exit "$status"
}

configure_lcm_route() {
  log "configuring LCM multicast route on ${WIFI_IFACE}"
  sudo ifconfig "$WIFI_IFACE" multicast || return 1
  sudo ip route del 224.0.0.0/4 dev lo 2>/dev/null || true
  sudo ip route replace 224.0.0.0/4 dev "$WIFI_IFACE" || return 1
}

print_network_diagnostics() {
  log "diagnostic: active NetworkManager connections"
  nmcli -t -f NAME,DEVICE,TYPE,STATE connection show --active || true

  log "diagnostic: interface ${WIFI_IFACE}"
  ip addr show "$WIFI_IFACE" || true

  log "diagnostic: routes"
  ip route show || true

  log "diagnostic: multicast route lookup"
  ip route get 239.255.76.67 || true

  if [[ -n "$LCM_PASSIVE_LOG" && -f "$LCM_PASSIVE_LOG" ]]; then
    log "diagnostic: passive LCM check log (${LCM_PASSIVE_LOG})"
    tail -n 80 "$LCM_PASSIVE_LOG" || true
  fi
}

passive_lcm_check() {
  if [[ "$SKIP_PASSIVE_CHECK" -eq 1 ]]; then
    log "passive LCM check skipped"
    return 0
  fi
  if [[ ! -x "$SDK_ROOT/build/nix_lcm_sub" ]]; then
    log "WARNING: build/nix_lcm_sub not found; run cmake --build build, or use --skip-passive-check"
    return 0
  fi

  LCM_PASSIVE_LOG="$TMP_DIR/passive_lcm_check.log"
  rm -f "$LCM_PASSIVE_LOG"
  log "checking passive LCM receive for ${PASSIVE_CHECK_SECONDS}s before sending state commands"
  timeout --preserve-status --signal=INT "${PASSIVE_CHECK_SECONDS}" \
    "$SDK_ROOT/build/nix_lcm_sub" >"$LCM_PASSIVE_LOG" 2>&1 || true

  if rg -q 'lcm_imu_data=[1-9][0-9]* msg/s|status=[1-9][0-9]* msg/s|lcm_joint_data=[1-9][0-9]* msg/s' "$LCM_PASSIVE_LOG"; then
    log "passive LCM receive check passed"
    return 0
  fi

  log "passive LCM receive check failed: no IMU/status/joint messages observed"
  print_network_diagnostics
  return 1
}

capture_feedback_csv() {
  local csv_path="$1"
  rm -f "$csv_path"
  "$PYTHON_BIN" "$SDK_ROOT/python/nix_lcm_sub.py" \
    --once \
    --timeout-ms "$FEEDBACK_TIMEOUT_MS" \
    --print-limit 21 \
    --csv "$csv_path"
}

extract_waist_position() {
  local csv_path="$1"
  awk -F, '
    /^#/ {next}
    NR == 2 {
      for (i = 1; i <= NF; i++) {
        if ($i == "JointID") joint_id_col = i
        if ($i == "Position") position_col = i
      }
      next
    }
    joint_id_col && position_col && $joint_id_col == "7:0" {
      print $position_col
      exit
    }
  ' "$csv_path"
}

require_cmd nmcli
require_cmd awk
require_cmd sudo
require_cmd ifconfig
require_cmd ip
require_cmd rg
require_cmd timeout

detect_wifi_iface
detect_restore_connection
pick_python

log "robot Wi-Fi SSID: ${ROBOT_SSID}"
log "Wi-Fi interface: ${WIFI_IFACE}"
log "restore connection: ${RESTORE_CONN:-<none>}"
log "Python: ${PYTHON_BIN}"

trap restore_wifi EXIT INT TERM

log "connecting to robot Wi-Fi"
nmcli dev wifi connect "$ROBOT_SSID" password "$ROBOT_PASSWORD" ifname "$WIFI_IFACE" \
  || die "failed to connect robot Wi-Fi: ${ROBOT_SSID}"

configure_lcm_route || die "failed to configure LCM multicast route"

TMP_DIR="$SDK_ROOT/build/wifi_debug_smoke"
mkdir -p "$TMP_DIR"
FEEDBACK_BEFORE="$TMP_DIR/feedback_before_debug.csv"
FEEDBACK_DEBUG="$TMP_DIR/feedback_debug.csv"

print_network_diagnostics
passive_lcm_check || die "LCM traffic is not visible on robot Wi-Fi; not sending DEBUG command"

if [[ "$DIAGNOSE_ONLY" -eq 1 ]]; then
  log "--diagnose-only set; stopping before state commands"
  exit 0
fi

log "entering DEBUG through lumos_sdk"
"$PYTHON_BIN" "$SDK_ROOT/python/nix_debug_state.py" enter \
  --timeout "$ENTER_TIMEOUT" \
  --stand-settle "$STAND_SETTLE" \
  || die "failed to enter DEBUG"
ENTERED_DEBUG=1

log "checking joint feedback in DEBUG"
capture_feedback_csv "$FEEDBACK_BEFORE" || die "failed to receive joint feedback in DEBUG"

WAIST_POS="$(extract_waist_position "$FEEDBACK_BEFORE")"
[[ -n "$WAIST_POS" ]] || die "could not find WAIST[0] feedback row JointID=7:0"
log "WAIST[0] current position from feedback: ${WAIST_POS}"

if [[ "$CONTROL_WAIST_HOLD" -eq 1 ]]; then
  log "publishing short WAIST[0] hold command at current position"
  "$PYTHON_BIN" "$SDK_ROOT/python/nix_joint_cmd.py" single \
    --component WAIST \
    --joint-id 0 \
    --pos "$WAIST_POS" \
    --kp 60 \
    --kd 2 \
    --duration 0.2 \
    --rate-hz 20 \
    || die "failed to publish WAIST[0] hold command"

  log "checking feedback after hold command"
  capture_feedback_csv "$FEEDBACK_DEBUG" || die "failed to receive feedback after hold command"
else
  log "joint command skipped; add --control-waist-hold to publish the short hold command"
fi

log "leaving DEBUG through lumos_sdk"
"$PYTHON_BIN" "$SDK_ROOT/python/nix_debug_state.py" leave \
  --timeout "$ENTER_TIMEOUT" \
  || die "failed to leave DEBUG"
LEFT_DEBUG=1

log "Wi-Fi DEBUG smoke test completed"
