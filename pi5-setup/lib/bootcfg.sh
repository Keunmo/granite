# Shared functions for the Raspberry Pi 5 setup scripts. Source this file.
#
# The functions edit /boot/firmware/config.txt and cmdline.txt of Ubuntu 24.04
# on a Raspberry Pi 5. Set BOOT_DIR to test the functions on a copy.

BOOT_DIR="${BOOT_DIR:-/boot/firmware}"
CONFIG_TXT="$BOOT_DIR/config.txt"
CMDLINE_TXT="$BOOT_DIR/cmdline.txt"

# Set to 1 by the functions when they change a boot file.
BOOT_CHANGED=0

log() { printf '  %-8s %s\n' "$1" "$2"; }
die() { echo "Error: $*" >&2; exit 1; }

require_root() {
  [[ ${EUID} -eq 0 ]] || die "run with sudo"
}

require_pi5() {
  local model
  model=$(tr -d '\0' </proc/device-tree/model 2>/dev/null || true)
  [[ $model == *"Raspberry Pi 5"* ]] || die "this is not a Raspberry Pi 5 (model: '${model:-unknown}')"
  [[ -f $CONFIG_TXT ]] || die "$CONFIG_TXT does not exist"
}

# save_backup SRC FILE SUFFIX
# Copy SRC to FILE.SUFFIX. When FILE.SUFFIX exists, keep it (it holds the
# state before the first run) and copy SRC to FILE.SUFFIX.<time>.
save_backup() {
  local src=$1 file=$2 suffix=$3 dst
  dst="$file.$suffix"
  [[ -e $dst ]] && dst="$file.$suffix.$(date +%Y%m%d-%H%M%S)"
  cp -p "$src" "$dst"
  log BACKUP "$dst"
}

# begin_config_edit / end_config_edit SUFFIX
# end_config_edit writes a backup of config.txt only when the edits between
# the two calls changed the file.
begin_config_edit() {
  CONFIG_ORIG=$(mktemp)
  cp -p "$CONFIG_TXT" "$CONFIG_ORIG"
}
end_config_edit() {
  if cmp -s "$CONFIG_ORIG" "$CONFIG_TXT"; then
    log OK "$CONFIG_TXT is already correct"
  else
    save_backup "$CONFIG_ORIG" "$CONFIG_TXT" "$1"
    BOOT_CHANGED=1
  fi
  rm -f "$CONFIG_ORIG"
}

# comment_lines REGEX [KEEP [FILTER]]
# Put '#' at the start of each active config.txt line that matches the
# extended REGEX. The REGEX applies to the line without leading and trailing
# spaces. A line that is equal to KEEP stays active. When FILTER is the name
# of a function, only the lines for which 'FILTER LINE' returns 0 get a '#'.
comment_lines() {
  local regex=$1 keep=${2-} filter=${3-} tmp line trimmed
  tmp=$(mktemp)
  while IFS= read -r line || [[ -n $line ]]; do
    trimmed=${line#"${line%%[![:space:]]*}"}
    trimmed=${trimmed%"${trimmed##*[![:space:]]}"}
    if [[ ! $line =~ ^[[:space:]]*# ]] && [[ $trimmed =~ $regex ]] && [[ $trimmed != "$keep" ]] &&
      { [[ -z $filter ]] || "$filter" "$trimmed"; }; then
      printf '#%s\n' "$line" >>"$tmp"
      log COMMENT "$line"
      BOOT_CHANGED=1
    else
      printf '%s\n' "$line" >>"$tmp"
    fi
  done <"$CONFIG_TXT"
  cat "$tmp" >"$CONFIG_TXT"
  rm -f "$tmp"
}

# has_line_in_last_all LINE
# True when config.txt has LINE as an active line in the last section and
# the last section is [all].
has_line_in_last_all() {
  local want=$1
  awk -v want="$want" '
    /^[[:space:]]*\[/ { sect = $0; gsub(/[[:space:]]/, "", sect); found = 0; next }
    { l = $0; gsub(/^[[:space:]]+|[[:space:]]+$/, "", l); if (l == want) found = 1 }
    END { exit !(sect == "[all]" && found) }
  ' "$CONFIG_TXT"
}

# append_to_all LINE
# Add LINE at the end of config.txt. Add an [all] section first when the last
# section is not [all], so that LINE applies to all boards.
append_to_all() {
  local line=$1 last
  last=$(grep -E '^[[:space:]]*\[' "$CONFIG_TXT" | tail -n 1 | tr -d '[:space:]' || true)
  # Make sure that the file ends with a newline.
  [[ -s $CONFIG_TXT && $(tail -c 1 "$CONFIG_TXT") != "" ]] && echo >>"$CONFIG_TXT"
  if [[ $last != "[all]" ]]; then
    printf '\n[all]\n' >>"$CONFIG_TXT"
  fi
  printf '%s\n' "$line" >>"$CONFIG_TXT"
  log ADD "$line"
  BOOT_CHANGED=1
}

# set_key KEY VALUE
# Set each active KEY=... line to KEY=VALUE. Add KEY=VALUE to [all] when no
# active line has the key.
set_key() {
  local key=$1 value=$2
  if grep -qE "^[[:space:]]*${key}=" "$CONFIG_TXT"; then
    if grep -E "^[[:space:]]*${key}=" "$CONFIG_TXT" | grep -vE "^[[:space:]]*${key}=${value}[[:space:]]*$" >/dev/null; then
      sed -i -E "s/^([[:space:]]*)${key}=.*/\1${key}=${value}/" "$CONFIG_TXT"
      log SET "${key}=${value}"
      BOOT_CHANGED=1
    else
      log OK "${key}=${value}"
    fi
  else
    append_to_all "${key}=${value}"
  fi
}

# remove_cmdline_arg REGEX
# Remove each kernel argument of cmdline.txt that matches the extended REGEX.
remove_cmdline_arg() {
  local regex=$1 arg kept=() removed=0
  [[ -f $CMDLINE_TXT ]] || return 0
  for arg in $(cat "$CMDLINE_TXT"); do
    if [[ $arg =~ $regex ]]; then
      log REMOVE "cmdline.txt: $arg"
      removed=1
    else
      kept+=("$arg")
    fi
  done
  if [[ $removed -eq 1 ]]; then
    save_backup "$CMDLINE_TXT" "$CMDLINE_TXT" bak-uart0
    printf '%s\n' "${kept[*]}" >"$CMDLINE_TXT"
    BOOT_CHANGED=1
  fi
}
