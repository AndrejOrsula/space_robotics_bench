#!/usr/bin/env bash
### Install SRB CLI tools and configure shell completions
### Usage: setup_cli.bash [destination_directory] [shell1 shell2 ...]
set -e

## Config
DEST_DIR="${DEST_DIR:-"$HOME/.local/bin"}"
SPECIFIED_SHELLS=()
COMPLETIONS_ONLY=false
EXECUTABLES=(
    "simforge"
    "srb"
    "space_robotics_bench"
)

## Process arguments
for arg in "$@"; do
    if [[ "$arg" == "--completions-only" ]]; then
        COMPLETIONS_ONLY=true
    elif [[ "$arg" == /* || "$arg" == ./* || "$arg" == ~/* || "$arg" == ../* ]]; then
        DEST_DIR="$arg"
    elif [[ "$arg" == "bash" || "$arg" == "fish" || "$arg" == "zsh" ]]; then
        SPECIFIED_SHELLS+=("$arg")
    else
        echo >&2 -e "\033[1;31m[ERROR]\033[0m Unrecognized argument: $arg"
        echo >&2 "Usage: $0 [--completions-only] [destination_directory] [bash|fish|zsh]"
        exit 1
    fi
done

## Check if a shell is requested
is_shell_requested() {
    local shell="$1"
    # No selection means configure every detected shell.
    [[ ${#SPECIFIED_SHELLS[@]} -eq 0 ]] && return 0
    # Otherwise, configure only selected shells.
    for s in "${SPECIFIED_SHELLS[@]}"; do
        [[ "$s" == "$shell" ]] && return 0
    done
    return 1
}

echo "[INFO] Executables: [$(IFS=", "; echo "${EXECUTABLES[*]}")]"
echo "[INFO] Destination path: ${DEST_DIR}"
if [[ ${#SPECIFIED_SHELLS[@]} -gt 0 ]]; then
    echo "[INFO] Configuring shells: [$(IFS=", "; echo "${SPECIFIED_SHELLS[*]}")]"
else
    echo "[INFO] Configuring all detected shells"
fi

if [[ -z "${ISAACSIM_PYTHON}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR]\033[0m ISAACSIM_PYTHON is not set. Please set it to the path of the Python executable in the Isaac Sim environment."
    exit 1
fi
if [[ ! -x "${ISAACSIM_PYTHON}" ]]; then
    echo >&2 -e "\033[1;31m[ERROR]\033[0m ISAACSIM_PYTHON is not a valid Python executable. Please set it to the path of the Python executable in the Isaac Sim environment."
    exit 1
fi

ISAACSIM_PATH="${ISAACSIM_PATH:-"$(dirname "${ISAACSIM_PYTHON}")"}"
ISAACSIM_BIN_PATH="${ISAACSIM_PATH}/kit/python/bin"

echo "[INFO] ISAACSIM_PATH: ${ISAACSIM_PATH}"
echo "[INFO] ISAACSIM_PYTHON: ${ISAACSIM_PYTHON}"
echo -e "\033[1;90m[TRACE]\033[0m ISAACSIM_BIN_PATH: ${ISAACSIM_BIN_PATH}"

if [[ "${COMPLETIONS_ONLY}" == true ]]; then
    echo "[INFO] --completions-only: skipping executable installation"
else
    # Ensure destination directory exists
    if [[ ! -d "${DEST_DIR}" ]]; then
        echo "[INFO] Creating destination directory ${DEST_DIR}"
        mkdir -p "${DEST_DIR}"
    fi

    # Make sure the destination directory is in PATH
    if [[ ! ":$PATH:" == *":${DEST_DIR}:"* ]]; then
        echo >&2 -e "\033[1;31m[ERROR]\033[0m ${DEST_DIR} is not in PATH. Add it to your PATH or specify a different destination directory via \`setup_cli.bash [destination_directory]\`."
        exit 1
    fi

    echo "[INFO] Installing executables..."

    WITH_SUDO=""
    if [[ ! -w "${DEST_DIR}" ]]; then
        echo "[INFO] Destination directory requires elevated permissions. Using sudo for file operations."
        WITH_SUDO="sudo"
        sudo -v
    fi
    for exe in "${EXECUTABLES[@]}"; do
        exe_path="${ISAACSIM_BIN_PATH}/${exe}"

        if [[ -f "${exe_path}" ]]; then
            echo "[INFO] Copying ${exe} to ${DEST_DIR}"
            ${WITH_SUDO} cp -f "${exe_path}" "${DEST_DIR}/"
            echo -e "\033[1;90m[TRACE]\033[0m Updating shebang of ${DEST_DIR}/${exe}"
            ${WITH_SUDO} sed -i "1s|.*|#!${ISAACSIM_PYTHON}|" "${DEST_DIR}/${exe}"
            ${WITH_SUDO} chmod +x "${DEST_DIR}/${exe}"
        else
            echo >&2 -e "\033[1;33m[WARNING]\033[0m Executable ${exe} not found at ${exe_path}."
        fi
    done
fi

echo "[INFO] Configuring argcomplete..."

# Find register-python-argcomplete in Isaac Sim
REGISTER_ARGCOMPLETE=()
if [[ -x "${ISAACSIM_BIN_PATH}/register-python-argcomplete" ]]; then
    REGISTER_ARGCOMPLETE=("${ISAACSIM_BIN_PATH}/register-python-argcomplete")
    echo -e "\033[1;90m[TRACE]\033[0m Found register-python-argcomplete at ${REGISTER_ARGCOMPLETE[*]}"
fi

if [[ ${#REGISTER_ARGCOMPLETE[@]} -eq 0 ]]; then
    echo >&2 -e "\033[1;33m[WARNING]\033[0m register-python-argcomplete not found in Isaac Sim, looking in system PATH"

    if command -v register-python-argcomplete &>/dev/null; then
        REGISTER_ARGCOMPLETE=("$(command -v register-python-argcomplete)")
        echo -e "\033[1;90m[TRACE]\033[0m Using system register-python-argcomplete at ${REGISTER_ARGCOMPLETE[*]}"
    fi
fi

if [[ ${#REGISTER_ARGCOMPLETE[@]} -eq 0 ]] && "${ISAACSIM_PYTHON}" -c 'from argcomplete.scripts.register_python_argcomplete import main' &>/dev/null; then
    # A valid CLI environment may contain the Python package without exposing
    # its console script (common for embedded Isaac Python installations).
    # Invoke the same entry point in-process instead of disabling completion.
    REGISTER_ARGCOMPLETE=(
        "${ISAACSIM_PYTHON}"
        "-c"
        'from argcomplete.scripts.register_python_argcomplete import main; main()'
    )
    echo -e "\033[1;90m[TRACE]\033[0m Using argcomplete from ${ISAACSIM_PYTHON}"
fi

## Resolve the namespaced cache directory for Hydra override completions.
## Prefer the same interpreter that owns the installed CLI; a system ``python3``
## often cannot import an editable or wheel-only SRB installation. The path
## module applies ``SRB_CACHE_ROOT`` and the active cache identity.
SRB_CACHE_DIR="$("${ISAACSIM_PYTHON}" -c 'from srb.utils.path import SRB_CACHE_NAMESPACE_PATH; print(SRB_CACHE_NAMESPACE_PATH)' 2>/dev/null || true)"
if [[ -z "${SRB_CACHE_DIR}" ]]; then
    SRB_CACHE_DIR="$(python3 -c 'from srb.utils.path import SRB_CACHE_NAMESPACE_PATH; print(SRB_CACHE_NAMESPACE_PATH)' 2>/dev/null || true)"
fi
if [[ -n "${SRB_CACHE_DIR}" && -d "${SRB_CACHE_DIR}" ]]; then
    echo "[INFO] SRB cache directory: ${SRB_CACHE_DIR}"
else
    echo >&2 -e "\033[1;33m[WARN]\033[0m SRB cache directory not found. Hydra override completions will be non-functional until cache is generated."
fi
# Escape shell and sed metacharacters so generated snippets support valid
# cache paths containing quotes, substitutions, delimiters, or backslashes.
SRB_CACHE_DIR_SHELL="${SRB_CACHE_DIR//\\/\\\\}"
SRB_CACHE_DIR_SHELL="${SRB_CACHE_DIR_SHELL//\"/\\\"}"
SRB_CACHE_DIR_SHELL="${SRB_CACHE_DIR_SHELL//\$/\\\$}"
SRB_CACHE_DIR_SHELL="${SRB_CACHE_DIR_SHELL//\`/\\\`}"
SRB_CACHE_DIR_SHELL="${SRB_CACHE_DIR_SHELL//!/\\!}"
SRB_CACHE_DIR_SED="${SRB_CACHE_DIR_SHELL//\\/\\\\}"
SRB_CACHE_DIR_SED="${SRB_CACHE_DIR_SED//|/\\|}"
SRB_CACHE_DIR_SED="${SRB_CACHE_DIR_SED//&/\\&}"
SRB_CACHE_DIR_FISH="${SRB_CACHE_DIR//\\/\\\\}"
SRB_CACHE_DIR_FISH="${SRB_CACHE_DIR_FISH//\"/\\\"}"
SRB_CACHE_DIR_FISH="${SRB_CACHE_DIR_FISH//\$/\\\$}"
SRB_CACHE_DIR_FISH_SED="${SRB_CACHE_DIR_FISH//\\/\\\\}"
SRB_CACHE_DIR_FISH_SED="${SRB_CACHE_DIR_FISH_SED//|/\\|}"
SRB_CACHE_DIR_FISH_SED="${SRB_CACHE_DIR_FISH_SED//&/\\&}"

## Options after which a Hydra override may start a new word (flags without a
## value, or variadic ones whose consumed overrides are forwarded to Hydra).
## Read them from the installed parsers so a reinstall tracks new flags.
HYDRA_FREE_FLAGS_CODE='from srb.cli.commands.completions import hydra_free_flags; print(" ".join(hydra_free_flags()))'
SRB_HYDRA_FREE_FLAGS="$("${ISAACSIM_PYTHON}" -c "${HYDRA_FREE_FLAGS_CODE}" 2>/dev/null | tail -n 1 || true)"
if [[ ! "${SRB_HYDRA_FREE_FLAGS}" =~ ^[-A-Za-z0-9_\ ]+$ ]]; then
    SRB_HYDRA_FREE_FLAGS="$(python3 -c "${HYDRA_FREE_FLAGS_CODE}" 2>/dev/null | tail -n 1 || true)"
fi
if [[ ! "${SRB_HYDRA_FREE_FLAGS}" =~ ^[-A-Za-z0-9_\ ]+$ ]]; then
    echo >&2 -e "\033[1;33m[WARN]\033[0m Cannot read SRB CLI flags; Hydra override completions use a built-in flag list."
    SRB_HYDRA_FREE_FLAGS="--continue --continue_training --enable_cameras --hardware --headless --hide_ui --hw --interface --normalize_actions --perf --record-events --record_events --success_only --teleop_device --video --xr"
fi

## Check for jq (required by Hydra override completions)
if ! command -v jq &>/dev/null; then
    echo -e "\033[1;33m[WARN]\033[0m jq is required for Hydra override completions. Install with: apt install jq"
fi

if [[ ${#REGISTER_ARGCOMPLETE[@]} -eq 0 ]]; then
    echo >&2 -e "\033[1;33m[WARNING]\033[0m register-python-argcomplete not found; base CLI completions are unavailable."
fi

CONFIGURED_SHELLS=()

# Keep shell setup grouped. Fish Hydra overrides remain useful without
# argcomplete, so that requested shell can enter through the fallback path.
if [[ ${#REGISTER_ARGCOMPLETE[@]} -gt 0 ]] || is_shell_requested "fish"; then
    # Setup Fish completions
    if command -v fish &>/dev/null && is_shell_requested "fish"; then
        FISH_COMPLETION_DIR="${HOME}/.config/fish/completions"
        mkdir -p "${FISH_COMPLETION_DIR}"
        echo "[INFO] Setting up Fish completions in ${FISH_COMPLETION_DIR}"

        if [[ ${#REGISTER_ARGCOMPLETE[@]} -gt 0 ]]; then
            for exe in "${EXECUTABLES[@]}"; do
                "${REGISTER_ARGCOMPLETE[@]}" --shell fish "${exe}" > "${FISH_COMPLETION_DIR}/${exe}.fish"
                echo -e "\033[1;90m[TRACE]\033[0m Created Fish completion for ${exe}"
            done
        else
            : > "${FISH_COMPLETION_DIR}/srb.fish"
            echo -e "\033[1;90m[TRACE]\033[0m Creating Hydra-only Fish completion for srb"
        fi

        # Append Hydra override completions for srb
        echo "[INFO] Appending Hydra override completions to Fish"
        cat >> "${FISH_COMPLETION_DIR}/srb.fish" << 'FISH_HYDRA_EOF'

# Hydra override completions (env.domain=..., +env.key=..., agent.key=...) for
# `srb agent` and `srb real_agent`, plus `srb agent tune ... --pin KEY=VALUE`.
set -g __srb_hydra_free_flags __SRB_HYDRA_FREE_FLAGS__

function __srb_cache_dir
    set -l root "__SRB_CACHE_DIR_FISH__"
    set -l current "$root/offline/current.json"
    if test -e "$current"
        if test -r "$current"; and command -q jq
            set -l generation (jq -r 'select(.schema == "srb-publication/1") | .generation_id // empty' "$current" 2>/dev/null)
            if string match -rq '^[0-9a-f]{32}$' -- "$generation"
                set -l candidate "$root/offline/generations/$generation"
                if test -d "$candidate"; and not test -L "$candidate"
                    printf '%s\n' "$candidate"
                    return 0
                end
            end
        end
        # A present but invalid generation pointer must not make completion
        # read stale legacy files from the cache root.
        return 0
    end
    printf '%s\n' "$root"
end

# Query one cache member: __srb_jq DIR MEMBER FILTER JQ_ARGS...
function __srb_jq
    jq -r $argv[4..-1] "if has(\"__srb_cache__\") then .payload else . end | $argv[3]" "$argv[1]/$argv[2].json" 2>/dev/null
end

function __srb_hydra_candidates
    set -l cache_dir (__srb_cache_dir)
    test -n "$cache_dir"; and test -d "$cache_dir"; or return
    command -q jq; or return

    set -l words (commandline -opc)
    # Raw current token without its opening quote or backslash escapes.
    set -l cur (commandline -ct | string replace -r '^[\'"]' '' | string replace -a '\\' '')
    set -q cur[1]; or set cur ""

    # Locate the command, action, task, algorithm, and an open `--pin` list.
    set -l subcmd ""
    set -l action ""
    set -l task ""
    set -l algo ""
    set -l in_pin 0
    set -l n (count $words)
    for i in (seq 2 $n)
        set -l w $words[$i]
        set -l next ""
        test $i -lt $n; and set next $words[(math $i + 1)]
        switch $w
            case -e --env --task --tasks
                test -z "$task"; and set task $next
            case '--env=*' '--task=*' '--tasks=*'
                test -z "$task"; and set task (string split -m1 = -- $w)[2]
            case --algo
                set algo $next
            case '--algo=*'
                set algo (string split -m1 = -- $w)[2]
        end
        if test "$w" = --pin; or string match -q -- '--pin=*' $w
            set in_pin 1
        else if string match -q -- '-*' $w
            set in_pin 0
        else if test -z "$subcmd"
            set subcmd $w
        else if test -z "$action"
            set action $w
        end
    end
    contains -- "$subcmd" agent real_agent; or return
    test -n "$action"; or return
    set -l prev $words[-1]
    if test "$subcmd" = agent -a "$action" = tune
        # Tune trials take overrides only through `--pin KEY=VALUE`.
        test $in_pin -eq 1; or return
    else if string match -q -- '-*' $prev; and not string match -q -- '*=*' $prev
        # The previous option takes this word as its value.
        test "$prev" = --; or contains -- $prev $__srb_hydra_free_flags; or return
    end
    # Cache keys use bare task ids and canonical lowercase algorithm labels.
    set task (string replace -r '.*/' '' -- $task)
    set algo (string lower -- $algo)

    # Keep Hydra's `+`/`++`/`~` prefix while looking up the plain key.
    set -l hprefix (string match -r '^(?:\+\+|\+|~)' -- $cur)
    set -l body (string replace -r '^(?:\+\+|\+|~)' '' -- $cur)

    if not string match -q -- '*=*' $body
        printf '%s\t%s\n' "$hprefix"env. "Environment config overrides"
        test -n "$algo"; and printf '%s\t%s\n' "$hprefix"agent. "Algorithm config overrides"
        if string match -q -- 'env.*' $body
            for k in (__srb_jq $cache_dir config_keys '.[$t] // .__common__ // [] | .[]' --arg t "$task")
                printf '%s%s\n' "$hprefix" $k
            end
        else if string match -q -- 'agent.*' $body; and test -n "$algo"
            for k in (__srb_jq $cache_dir agent_config_keys '.[$a] // .__common__ // [] | .[]' --arg a "$algo")
                printf '%s%s\n' "$hprefix" $k
            end
        end
        return
    end

    set -l key (string split -m1 = -- $body)[1]
    set -l typed (string split -m1 = -- $body)[2]
    set -l values

    # `<base>+<TAB>` adds a tool to a manipulator or a payload to a mobile
    # base; `*.robot` keys look up the base category in robot.json.
    set -l suffix_type ""
    if string match -q -- '*+*' $typed
        set -l base (string split -m1 + -- $typed)[1]
        switch $key
            case '*.manipulator'
                set suffix_type tool
            case '*.mobile_base'
                set suffix_type payload
            case '*.robot'
                set -l is_manip (__srb_jq $cache_dir robot '[.manipulator[]?[]] | index($b) // empty' --arg b "$base")
                set -l is_mobile (__srb_jq $cache_dir robot '[.mobile_robot[]?[]] | index($b) // empty' --arg b "$base")
                if test -n "$is_manip"
                    set suffix_type tool
                else if test -n "$is_mobile"
                    set suffix_type payload
                end
        end
        if test -n "$suffix_type"
            for v in (__srb_jq $cache_dir object '.[$k] // [] | .[]' --arg k "$suffix_type")
                set -a values "$base+$v"
            end
        end
    end

    if test -z "$suffix_type"; and string match -q -- '*.action_mode' $key
        # Modes of the asset selected earlier on the line, else the task default.
        set -l asset_key (string replace -r '\.action_mode$' '' -- $key)
        set -l selected ""
        for tok in $words
            set -l pair (string split -m1 = -- (string replace -r '^(?:\+\+|\+)' '' -- $tok))
            if test (count $pair) -eq 2; and contains -- $pair[1] $asset_key $asset_key.name
                set selected $pair[2]
            end
        end
        if test -z "$selected"; and test -n "$task"
            set -l sub_key (string replace -r '^env\.robot\.?' '' -- $asset_key)
            test -z "$sub_key"; and set sub_key robot
            set selected (__srb_jq $cache_dir task_defaults '.[$t][$s] // empty' --arg t "$task" --arg s "$sub_key")
        end
        if test -n "$selected"
            set values (__srb_jq $cache_dir action_modes '.[$n][]? // empty' --arg n "$selected")
        end
    else if test -z "$suffix_type"
        set -l vals (__srb_jq $cache_dir enum_values '.[$t][$k] // .__common__[$k] // [] | .[]' --arg t "$task" --arg k "$key")
        if test (count $vals) -eq 0; and string match -q -- 'agent.*' $key
            set vals (__srb_jq $cache_dir agent_enum_values '.[$k] // [] | .[]' --arg k "$key")
        end
        set -l open ""
        switch "$vals[1]"
            case __set__ __list__
                set open '['
                set -e vals[1]
            case __dict__
                set open '{'
                set -e vals[1]
        end
        if test -n "$open"
            # Hydra containers need brackets: complete the element after the
            # last comma and skip elements already listed.
            set -l content (string replace -r '^[\[{]' '' -- $typed)
            set -l selected (string match -r '^.*,' -- $content)
            set -l listed (string split , -- $selected)
            for v in $vals
                contains -- $v $listed; and continue
                set -a values "$open$selected$v"
            end
        else
            set values $vals
        end
    end

    for v in $values
        printf '%s%s=%s\n' "$hprefix" $key $v
    end
end

# Keep candidates extending the current token. Fish would hide the others too,
# but they must not suppress the path fallback below.
function __srb_hydra_complete
    set -l cur (commandline -ct | string replace -r '^[\'"]' '' | string replace -a '\\' '')
    set -l n (string length -- "$cur")
    set -l lcur (string lower -- "$cur")
    __srb_hydra_candidates | while read -l candidate
        set -l name (string split -m1 \t -- $candidate)[1]
        test "$(string lower -- "$(string sub -l $n -- "$name")")" = "$lcur"; and printf '%s\n' $candidate
    end
end

# Compose argcomplete's candidates with Hydra overrides in one registration,
# and fall back to paths when neither matches (as Bash's `-o default`).
function __srb_complete
    set -l results
    if functions -q __fish_srb_complete
        set results (__fish_srb_complete)
    end
    set -a results (__srb_hydra_complete)
    if test (count $results) -eq 0
        __fish_complete_path (commandline -ct)
        return
    end
    printf '%s\n' $results
end

complete -c srb -e
complete -c srb -f -a '(__srb_complete)'
FISH_HYDRA_EOF
        # Replace placeholders with the cache directory and generated flags
        sed -i -e "s|__SRB_CACHE_DIR_FISH__|${SRB_CACHE_DIR_FISH_SED}|g" \
            -e "s|__SRB_HYDRA_FREE_FLAGS__|${SRB_HYDRA_FREE_FLAGS}|g" \
            "${FISH_COMPLETION_DIR}/srb.fish"
        echo -e "\033[1;90m[TRACE]\033[0m Appended Hydra completions to ${FISH_COMPLETION_DIR}/srb.fish"

        CONFIGURED_SHELLS+=("fish")
        echo "[INFO] Fish completions installed"
    fi

    # Setup Bash completions
    if [[ ${#REGISTER_ARGCOMPLETE[@]} -gt 0 ]] && command -v bash &>/dev/null && is_shell_requested "bash"; then
        BASH_COMPLETION_DIR="${HOME}/.bash_completion.d"
        mkdir -p "${BASH_COMPLETION_DIR}"
        echo "[INFO] Setting up Bash completions in ${BASH_COMPLETION_DIR}"

        for exe in "${EXECUTABLES[@]}"; do
            "${REGISTER_ARGCOMPLETE[@]}" "${exe}" > "${BASH_COMPLETION_DIR}/${exe}"
            echo -e "\033[1;90m[TRACE]\033[0m Created Bash completion for ${exe}"
        done

        # Append Hydra override completions for srb
        echo "[INFO] Appending Hydra override completions to Bash"
        cat >> "${BASH_COMPLETION_DIR}/srb" << 'BASH_HYDRA_EOF'

# Hydra override completions (env.domain=..., +env.key=..., agent.key=...) for
# `srb agent` and `srb real_agent`, plus `srb agent tune ... --pin KEY=VALUE`.
_SRB_HYDRA_FREE_FLAGS=" __SRB_HYDRA_FREE_FLAGS__ "

_srb_cache_dir() {
    local root="__SRB_CACHE_DIR__"
    local current="$root/offline/current.json"
    if [[ -e "$current" ]]; then
        if [[ -r "$current" ]] && command -v jq &>/dev/null; then
            local generation
            generation=$(jq -r 'select(.schema == "srb-publication/1") | .generation_id // empty' "$current" 2>/dev/null) || generation=""
            if [[ "$generation" =~ ^[0-9a-f]{32}$ ]] && [[ -d "$root/offline/generations/$generation" ]] && [[ ! -L "$root/offline/generations/$generation" ]]; then
                printf '%s\n' "$root/offline/generations/$generation"
                return 0
            fi
        fi
        # A present but invalid generation pointer must not make completion
        # read stale legacy files from the cache root.
        return 0
    fi
    printf '%s\n' "$root"
}

# Query one cache member: _srb_jq DIR MEMBER FILTER JQ_ARGS...
_srb_jq() {
    local file="$1/$2.json" filter="$3"
    shift 3
    jq -r "$@" "if has(\"__srb_cache__\") then .payload else . end | $filter" "$file" 2>/dev/null
}

_srb_hydra_complete() {
    local cache_dir
    cache_dir="$(_srb_cache_dir)"
    [[ -n "$cache_dir" && -d "$cache_dir" ]] || return
    command -v jq &>/dev/null || return

    # COMP_WORDS splits `key=value` at COMP_WORDBREAKS (`=`, `:`), hiding the
    # override key, so rebuild whitespace-separated words up to the cursor.
    local line="${COMP_LINE:0:COMP_POINT}"
    local -a words=()
    read -ra words <<< "$line"
    [[ -z "$line" || "$line" == *[[:space:]] ]] && words+=("")
    local n=${#words[@]} i w
    local raw_cur="${words[n-1]}"
    for ((i=0; i<n; i++)); do
        w="${words[i]}"
        w="${w#[\"\']}"
        ((i < n-1)) && w="${w%[\"\']}"
        words[i]="$w"
    done
    local cur="${words[n-1]}"

    # Locate the command, action, task, algorithm, and an open `--pin` list.
    local subcmd="" action="" task="" algo="" in_pin=false
    for ((i=1; i<n-1; i++)); do
        w="${words[i]}"
        case "$w" in
            -e|--env|--task|--tasks) [[ -z "$task" ]] && task="${words[i+1]}" ;;
            --env=*|--task=*|--tasks=*) [[ -z "$task" ]] && task="${w#*=}" ;;
            --algo) algo="${words[i+1]}" ;;
            --algo=*) algo="${w#*=}" ;;
        esac
        if [[ "$w" == --pin || "$w" == --pin=* ]]; then
            in_pin=true
        elif [[ "$w" == -* ]]; then
            in_pin=false
        elif [[ -z "$subcmd" ]]; then
            subcmd="$w"
        elif [[ -z "$action" ]]; then
            action="$w"
        fi
    done
    [[ "$subcmd" == agent || "$subcmd" == real_agent ]] && [[ -n "$action" ]] || return
    local prev="${words[n-2]}"
    if [[ "$subcmd" == agent && "$action" == tune ]]; then
        # Tune trials take overrides only through `--pin KEY=VALUE`.
        $in_pin || return
    elif [[ "$prev" == -* && "$prev" != *=* && "$prev" != -- && "$_SRB_HYDRA_FREE_FLAGS" != *" $prev "* ]]; then
        return # The previous option takes this word as its value.
    fi
    # Cache keys use bare task ids and canonical lowercase algorithm labels.
    task="${task##*/}"
    algo="${algo,,}"

    # Keep Hydra's `+`/`++`/`~` prefix while looking up the plain key.
    local hprefix=""
    case "$cur" in
        ++*) hprefix="++" ;;
        +*) hprefix="+" ;;
        \~*) hprefix="~" ;;
    esac
    local body="${cur#"$hprefix"}"
    local -a cands=() values=() vals=()
    local v

    if [[ "$body" != *=* ]]; then
        cands+=("${hprefix}env.")
        [[ -n "$algo" ]] && cands+=("${hprefix}agent.")
        if [[ "$body" == env.* ]]; then
            mapfile -t values < <(_srb_jq "$cache_dir" config_keys '.[$t] // .__common__ // [] | .[]' --arg t "$task")
        elif [[ "$body" == agent.* && -n "$algo" ]]; then
            mapfile -t values < <(_srb_jq "$cache_dir" agent_config_keys '.[$a] // .__common__ // [] | .[]' --arg a "$algo")
        fi
        for v in "${values[@]}"; do
            cands+=("${hprefix}${v}")
        done
    else
        local key="${body%%=*}" typed="${body#*=}"

        # `<base>+<TAB>` adds a tool to a manipulator or a payload to a mobile
        # base; `*.robot` keys look up the base category in robot.json.
        local suffix_type=""
        if [[ "$typed" == *+* ]]; then
            local base="${typed%%+*}"
            case "$key" in
                *.manipulator) suffix_type="tool" ;;
                *.mobile_base) suffix_type="payload" ;;
                *.robot)
                    if [[ -n "$(_srb_jq "$cache_dir" robot '[.manipulator[]?[]] | index($b) // empty' --arg b "$base")" ]]; then
                        suffix_type="tool"
                    elif [[ -n "$(_srb_jq "$cache_dir" robot '[.mobile_robot[]?[]] | index($b) // empty' --arg b "$base")" ]]; then
                        suffix_type="payload"
                    fi
                    ;;
            esac
            if [[ -n "$suffix_type" ]]; then
                mapfile -t vals < <(_srb_jq "$cache_dir" object '.[$k] // [] | .[]' --arg k "$suffix_type")
                for v in "${vals[@]}"; do
                    values+=("${base}+${v}")
                done
            fi
        fi

        if [[ -z "$suffix_type" && "$key" == *.action_mode ]]; then
            # Modes of the asset selected earlier on the line, else the task default.
            local asset_key="${key%.action_mode}" selected="" tok
            for tok in "${words[@]:0:n-1}"; do
                tok="${tok#+}"
                tok="${tok#+}"
                [[ "$tok" == *=* ]] || continue
                if [[ "${tok%%=*}" == "$asset_key" || "${tok%%=*}" == "$asset_key.name" ]]; then
                    selected="${tok#*=}"
                fi
            done
            if [[ -z "$selected" && -n "$task" ]]; then
                local sub_key="${asset_key#env.robot}"
                sub_key="${sub_key#.}"
                [[ -z "$sub_key" ]] && sub_key="robot"
                selected=$(_srb_jq "$cache_dir" task_defaults '.[$t][$s] // empty' --arg t "$task" --arg s "$sub_key")
            fi
            if [[ -n "$selected" ]]; then
                mapfile -t values < <(_srb_jq "$cache_dir" action_modes '.[$n][]? // empty' --arg n "$selected")
            fi
        elif [[ -z "$suffix_type" ]]; then
            mapfile -t vals < <(_srb_jq "$cache_dir" enum_values '.[$t][$k] // .__common__[$k] // [] | .[]' --arg t "$task" --arg k "$key")
            if [[ ${#vals[@]} -eq 0 && "$key" == agent.* ]]; then
                mapfile -t vals < <(_srb_jq "$cache_dir" agent_enum_values '.[$k] // [] | .[]' --arg k "$key")
            fi
            local open=""
            case "${vals[0]-}" in
                __set__|__list__) open="[" ;;
                __dict__) open="{" ;;
            esac
            if [[ -n "$open" ]]; then
                # Hydra containers need brackets: complete the element after
                # the last comma and skip elements already listed.
                local content="${typed#[\[\{]}" listed=""
                [[ "$content" == *,* ]] && listed="${content%,*},"
                for v in "${vals[@]:1}"; do
                    [[ ",$listed" == *",$v,"* ]] && continue
                    values+=("${open}${listed}${v}")
                done
            else
                values=("${vals[@]}")
            fi
        fi
        for v in "${values[@]}"; do
            cands+=("${hprefix}${key}=${v}")
        done
    fi

    # Readline replaces only the text after the last word-break character of
    # an unquoted word; quoted words are not split. Match case-insensitively
    # since Hydra accepts enum names in any case.
    local strip=0
    if [[ "$raw_cur" != [\"\']* ]]; then
        for ((i=${#cur}-1; i>=0; i--)); do
            if [[ "$COMP_WORDBREAKS" == *"${cur:i:1}"* ]]; then
                strip=$((i+1))
                break
            fi
        done
    fi
    local c
    for c in "${cands[@]}"; do
        [[ "${c,,}" == "${cur,,}"* ]] && COMPREPLY+=("${c:strip}")
    done
}

# Compose argcomplete's CLI candidates with SRB's Hydra override candidates.
# A later `complete` registration replaces an earlier one in Bash, so binding
# `_srb_hydra_complete` directly would silently discard all regular CLI
# subcommands and options generated by register-python-argcomplete.
_srb_complete() {
    local -a hydra_replies=()

    COMPREPLY=()
    _srb_hydra_complete
    hydra_replies=("${COMPREPLY[@]}")

    COMPREPLY=()
    _python_argcomplete "$@"
    COMPREPLY+=("${hydra_replies[@]}")
    # Argcomplete re-enables trailing spaces; a partial override needs none.
    [[ ${#hydra_replies[@]} -gt 0 ]] && compopt -o nospace 2>/dev/null
    return 0
}
complete -F _srb_complete -o nospace -o default -o bashdefault srb
BASH_HYDRA_EOF
        # Replace placeholders with the cache directory and generated flags
        sed -i -e "s|__SRB_CACHE_DIR__|${SRB_CACHE_DIR_SED}|g" \
            -e "s|__SRB_HYDRA_FREE_FLAGS__|${SRB_HYDRA_FREE_FLAGS}|g" \
            "${BASH_COMPLETION_DIR}/srb"
        echo -e "\033[1;90m[TRACE]\033[0m Appended Hydra completions to ${BASH_COMPLETION_DIR}/srb"

        # Add source command to bashrc if not already there
        if ! grep -q "bash_completion.d" "${HOME}/.bashrc" 2>/dev/null; then
            echo "[INFO] Adding Bash completion source to ~/.bashrc"
            cat >> "${HOME}/.bashrc" << 'EOF'

# Source bash completions
if [ -d "$HOME/.bash_completion.d" ]; then
  for f in "$HOME/.bash_completion.d"/*; do
    [ -f "$f" ] && source "$f"
  done
fi
EOF
        else
            echo -e "\033[1;90m[TRACE]\033[0m Bash completion source already in ~/.bashrc"
        fi
        CONFIGURED_SHELLS+=("bash")
        echo "[INFO] Bash completions installed"
    fi

    # Setup Zsh completions
    if [[ ${#REGISTER_ARGCOMPLETE[@]} -gt 0 ]] && command -v zsh &>/dev/null && is_shell_requested "zsh"; then
        ZSH_COMPLETION_DIR="${HOME}/.zsh/completion"
        mkdir -p "${ZSH_COMPLETION_DIR}"
        echo "[INFO] Setting up Zsh completions in ${ZSH_COMPLETION_DIR}"

        for exe in "${EXECUTABLES[@]}"; do
            "${REGISTER_ARGCOMPLETE[@]}" "${exe}" > "${ZSH_COMPLETION_DIR}/_${exe}"
            echo -e "\033[1;90m[TRACE]\033[0m Created Zsh completion for ${exe}"
        done

        # Append Hydra override completions for srb
        echo "[INFO] Appending Hydra override completions to Zsh"
        cat >> "${ZSH_COMPLETION_DIR}/_srb" << 'ZSH_HYDRA_EOF'

# Hydra override completions (env.domain=..., +env.key=..., agent.key=...) for
# `srb agent` and `srb real_agent`, plus `srb agent tune ... --pin KEY=VALUE`.
# Mirrors the Bash logic; zsh keeps `key=value` in one word.
_SRB_HYDRA_FREE_FLAGS=" __SRB_HYDRA_FREE_FLAGS__ "

_srb_cache_dir() {
    local root="__SRB_CACHE_DIR__"
    local current="$root/offline/current.json"
    if [[ -e "$current" ]]; then
        if [[ -r "$current" ]] && command -v jq &>/dev/null; then
            local generation
            generation=$(jq -r 'select(.schema == "srb-publication/1") | .generation_id // empty' "$current" 2>/dev/null) || generation=""
            if [[ "$generation" =~ ^[0-9a-f]{32}$ ]] && [[ -d "$root/offline/generations/$generation" ]] && [[ ! -L "$root/offline/generations/$generation" ]]; then
                print -r -- "$root/offline/generations/$generation"
                return 0
            fi
        fi
        # A present but invalid generation pointer must not make completion
        # read stale legacy files from the cache root.
        return 0
    fi
    print -r -- "$root"
}

# Query one cache member: _srb_jq DIR MEMBER FILTER JQ_ARGS...
_srb_jq() {
    local file="$1/$2.json" filter="$3"
    shift 3
    jq -r "$@" "if has(\"__srb_cache__\") then .payload else . end | $filter" "$file" 2>/dev/null
}

_srb_hydra_complete() {
    local cache_dir
    cache_dir="$(_srb_cache_dir)"
    [[ -n "$cache_dir" && -d "$cache_dir" ]] || return
    command -v jq &>/dev/null || return

    local cur="$PREFIX" i w

    # Locate the command, action, task, algorithm, and an open `--pin` list.
    local subcmd="" action="" task="" algo="" in_pin=false
    for ((i=2; i<CURRENT; i++)); do
        w="${(Q)words[i]}"
        case "$w" in
            -e|--env|--task|--tasks) [[ -z "$task" ]] && task="${(Q)words[i+1]}" ;;
            --env=*|--task=*|--tasks=*) [[ -z "$task" ]] && task="${w#*=}" ;;
            --algo) algo="${(Q)words[i+1]}" ;;
            --algo=*) algo="${w#*=}" ;;
        esac
        if [[ "$w" == --pin || "$w" == --pin=* ]]; then
            in_pin=true
        elif [[ "$w" == -* ]]; then
            in_pin=false
        elif [[ -z "$subcmd" ]]; then
            subcmd="$w"
        elif [[ -z "$action" ]]; then
            action="$w"
        fi
    done
    [[ "$subcmd" == agent || "$subcmd" == real_agent ]] && [[ -n "$action" ]] || return
    local prev="${(Q)words[CURRENT-1]}"
    if [[ "$subcmd" == agent && "$action" == tune ]]; then
        # Tune trials take overrides only through `--pin KEY=VALUE`.
        $in_pin || return
    elif [[ "$prev" == -* && "$prev" != *=* && "$prev" != -- && "$_SRB_HYDRA_FREE_FLAGS" != *" $prev "* ]]; then
        return # The previous option takes this word as its value.
    fi
    # Cache keys use bare task ids and canonical lowercase algorithm labels.
    task="${task##*/}"
    algo="${(L)algo}"

    # Keep Hydra's `+`/`++`/`~` prefix while looking up the plain key.
    local hprefix=""
    case "$cur" in
        ++*) hprefix="++" ;;
        +*) hprefix="+" ;;
        \~*) hprefix="~" ;;
    esac
    local body="${cur#$hprefix}"
    local -a cands values vals
    local v

    if [[ "$body" != *=* ]]; then
        cands+=("${hprefix}env.")
        [[ -n "$algo" ]] && cands+=("${hprefix}agent.")
        if [[ "$body" == env.* ]]; then
            values=(${(f)"$(_srb_jq "$cache_dir" config_keys '.[$t] // .__common__ // [] | .[]' --arg t "$task")"})
        elif [[ "$body" == agent.* && -n "$algo" ]]; then
            values=(${(f)"$(_srb_jq "$cache_dir" agent_config_keys '.[$a] // .__common__ // [] | .[]' --arg a "$algo")"})
        fi
        for v in "${values[@]}"; do
            cands+=("${hprefix}${v}")
        done
    else
        local key="${body%%=*}" typed="${body#*=}"

        # `<base>+<TAB>` adds a tool to a manipulator or a payload to a mobile
        # base; `*.robot` keys look up the base category in robot.json.
        local suffix_type=""
        if [[ "$typed" == *+* ]]; then
            local base="${typed%%+*}"
            case "$key" in
                *.manipulator) suffix_type="tool" ;;
                *.mobile_base) suffix_type="payload" ;;
                *.robot)
                    if [[ -n "$(_srb_jq "$cache_dir" robot '[.manipulator[]?[]] | index($b) // empty' --arg b "$base")" ]]; then
                        suffix_type="tool"
                    elif [[ -n "$(_srb_jq "$cache_dir" robot '[.mobile_robot[]?[]] | index($b) // empty' --arg b "$base")" ]]; then
                        suffix_type="payload"
                    fi
                    ;;
            esac
            if [[ -n "$suffix_type" ]]; then
                vals=(${(f)"$(_srb_jq "$cache_dir" object '.[$k] // [] | .[]' --arg k "$suffix_type")"})
                for v in "${vals[@]}"; do
                    values+=("${base}+${v}")
                done
            fi
        fi

        if [[ -z "$suffix_type" && "$key" == *.action_mode ]]; then
            # Modes of the asset selected earlier on the line, else the task default.
            local asset_key="${key%.action_mode}" selected="" tok
            for tok in "${(Q)words[@]:1:CURRENT-2}"; do
                tok="${tok#+}"
                tok="${tok#+}"
                [[ "$tok" == *=* ]] || continue
                if [[ "${tok%%=*}" == "$asset_key" || "${tok%%=*}" == "$asset_key.name" ]]; then
                    selected="${tok#*=}"
                fi
            done
            if [[ -z "$selected" && -n "$task" ]]; then
                local sub_key="${asset_key#env.robot}"
                sub_key="${sub_key#.}"
                [[ -z "$sub_key" ]] && sub_key="robot"
                selected=$(_srb_jq "$cache_dir" task_defaults '.[$t][$s] // empty' --arg t "$task" --arg s "$sub_key")
            fi
            if [[ -n "$selected" ]]; then
                values=(${(f)"$(_srb_jq "$cache_dir" action_modes '.[$n][]? // empty' --arg n "$selected")"})
            fi
        elif [[ -z "$suffix_type" ]]; then
            vals=(${(f)"$(_srb_jq "$cache_dir" enum_values '.[$t][$k] // .__common__[$k] // [] | .[]' --arg t "$task" --arg k "$key")"})
            if [[ ${#vals[@]} -eq 0 && "$key" == agent.* ]]; then
                vals=(${(f)"$(_srb_jq "$cache_dir" agent_enum_values '.[$k] // [] | .[]' --arg k "$key")"})
            fi
            local open=""
            case "${vals[1]-}" in
                __set__|__list__) open="[" ;;
                __dict__) open="{" ;;
            esac
            if [[ -n "$open" ]]; then
                # Hydra containers need brackets: complete the element after
                # the last comma and skip elements already listed.
                local content="${typed#[\[\{]}" listed=""
                [[ "$content" == *,* ]] && listed="${content%,*},"
                for v in "${vals[@]:1}"; do
                    [[ ",$listed" == *",$v,"* ]] && continue
                    values+=("${open}${listed}${v}")
                done
            else
                values=("${vals[@]}")
            fi
        fi
        for v in "${values[@]}"; do
            cands+=("${hprefix}${key}=${v}")
        done
    fi

    # Match case-insensitively since Hydra accepts enum names in any case.
    local -a matches
    local c
    for c in "${cands[@]}"; do
        [[ "${(L)c}" == "${(L)cur}"* ]] && matches+=("$c")
    done
    (( ${#matches[@]} )) && compadd -U -S '' -- "${matches[@]}"
}

# Compose both completion providers. As with Bash, a later `compdef` replaces
# argcomplete's registration instead of extending it.
_srb_complete() {
    _python_argcomplete "$@"
    _srb_hydra_complete "$@"
}
compdef _srb_complete srb
# On autoload the argcomplete shim above already ran for this completion.
if [[ $zsh_eval_context == *func ]]; then _srb_hydra_complete "$@"; fi
ZSH_HYDRA_EOF
        # Replace placeholders with the cache directory and generated flags
        sed -i -e "s|__SRB_CACHE_DIR__|${SRB_CACHE_DIR_SED}|g" \
            -e "s|__SRB_HYDRA_FREE_FLAGS__|${SRB_HYDRA_FREE_FLAGS}|g" \
            "${ZSH_COMPLETION_DIR}/_srb"
        echo -e "\033[1;90m[TRACE]\033[0m Appended Hydra completions to ${ZSH_COMPLETION_DIR}/_srb"

        # Add fpath and compinit to zshrc if not already there
        if ! grep -q "fpath=.*${ZSH_COMPLETION_DIR}" "${HOME}/.zshrc" 2>/dev/null; then
            echo "[INFO] Adding Zsh completion configuration to ~/.zshrc"
            printf -v ZSH_COMPLETION_DIR_QUOTED '%q' "$ZSH_COMPLETION_DIR"
            cat >> "${HOME}/.zshrc" << EOF

# Add custom completions directory
fpath=(${ZSH_COMPLETION_DIR_QUOTED} \$fpath)
autoload -U compinit && compinit
EOF
        else
            echo -e "\033[1;90m[TRACE]\033[0m Zsh completion configuration already in ~/.zshrc"
        fi
        CONFIGURED_SHELLS+=("zsh")
        echo "[INFO] Zsh completions installed"
    fi
fi

# Report requested shells that could not be configured, and fail when no shell
# was configured at all instead of claiming success.
for shell in "${SPECIFIED_SHELLS[@]}"; do
    if [[ " ${CONFIGURED_SHELLS[*]} " != *" ${shell} "* ]]; then
        echo >&2 -e "\033[1;33m[WARNING]\033[0m ${shell} completions were not installed (shell or register-python-argcomplete not found)"
    fi
done
if [[ ${#CONFIGURED_SHELLS[@]} -eq 0 ]]; then
    echo >&2 -e "\033[1;31m[ERROR]\033[0m No shell completions were installed; install fish, or argcomplete for Bash/Zsh."
    exit 1
fi

echo "[INFO] Installation completed successfully (completions: ${CONFIGURED_SHELLS[*]})"
echo "[INFO] Start a new shell (or run: exec \"\$SHELL\" -l) to activate completions."
