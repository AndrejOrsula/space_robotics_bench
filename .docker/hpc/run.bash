#!/usr/bin/env bash
### Run a command inside the Singularity container
### Usage: run.bash [CMD]
### Env: SRB_REQUIRE_GPU/VULKAN/RTX/DISPLAY=true for fail-closed preflights
### Env: SRB_DOCKER_PROFILE=ngc with IMAGE_PATH=<sif> (needs <sif>.ngc-source provenance)
set -euo pipefail


run_owned_session() {
    if (( $# == 0 )); then
        return 64
    fi

    local -a owned_command=("$@")
    local stdout_fd stderr_fd supervisor_pid control_read_fd control_source_fd
    local control_write_fd control_char=""
    local anchor_pid="" anchor_start_time="" anchor_state="" anchor_pgid=""
    local anchor_sid="" anchor_observed_start_time=""
    local anchor_observed_pgid="" anchor_observed_sid=""
    local workload_status="" line control_partial="" supervisor_status=0 read_status=0
    local requested_signal="" signal_forwarded=0 signal_failed=0
    local protocol_failed=0 control_eof=0 signal_ambiguous=0
    local ambiguity_checks=0 ambiguity_check_limit=60 ambiguity_timed_out=0
    local waiting_for_supervisor=0 wait_interrupted=0
    local caller_term_definition caller_int_definition
    local caller_term_action="" caller_int_action=""
    local caller_term_default=0 caller_int_default=0
    local decoded_trap_action="" decoded_trap_default=0 return_status=125

    decode_caller_trap_definition() {
        local definition="${1}" signal_name="${2}" canonical_signal="${3}"
        local prefix="trap -- '" suffix="' ${canonical_signal}"
        local encoded quote_escape

        decoded_trap_action=""
        decoded_trap_default=0
        if [[ -z "${definition}" ]]; then
            decoded_trap_default=1
            return 0
        fi
        if [[ "${definition}" != "${prefix}"*"${suffix}" ]]; then
            return 1
        fi
        encoded="${definition#"${prefix}"}"
        encoded="${encoded%"${suffix}"}"
        quote_escape="'\\''"
        decoded_trap_action="${encoded//"${quote_escape}"/\'}"

        (
            trap -- "${decoded_trap_action}" "${signal_name}"
            [[ "$(trap -p "${signal_name}")" == "${definition}" ]]
        )
    }

    restore_caller_traps() {
        local restore_failed=0
        if (( caller_term_default )); then
            trap - TERM || restore_failed=1
        else
            trap -- "${caller_term_action}" TERM || restore_failed=1
        fi
        if (( caller_int_default )); then
            trap - INT || restore_failed=1
        else
            trap -- "${caller_int_action}" INT || restore_failed=1
        fi
        (( restore_failed == 0 ))
    }

    caller_term_definition="$(trap -p TERM)"
    caller_int_definition="$(trap -p INT)"
    if ! decode_caller_trap_definition "${caller_term_definition}" TERM SIGTERM; then
        return 125
    fi
    caller_term_action="${decoded_trap_action}"
    caller_term_default="${decoded_trap_default}"
    if ! decode_caller_trap_definition "${caller_int_definition}" INT SIGINT; then
        return 125
    fi
    caller_int_action="${decoded_trap_action}"
    caller_int_default="${decoded_trap_default}"

    exec {stdout_fd}>&1
    exec {stderr_fd}>&2

    read_anchor_fields() {
        local stat state process_group_id session_id start_time stat_path

        stat_path="/proc/${anchor_pid}/stat"
        if [[ ! -e "${stat_path}" ]]; then
            return 1
        fi
        if [[ ! -r "${stat_path}" ]]; then
            [[ -e "${stat_path}" ]] && return 2
            return 1
        fi
        if ! IFS= read -r stat 2>/dev/null <"${stat_path}"; then
            [[ -e "${stat_path}" ]] && return 2
            return 1
        fi
        stat="${stat##*) }"
        read -r state _ process_group_id session_id _ _ _ _ _ _ _ _ _ _ _ \
            _ _ _ _ start_time _ <<<"${stat}" || return 2
        [[ "${process_group_id}" =~ ^[1-9][0-9]*$ ]] || return 2
        [[ "${session_id}" =~ ^[1-9][0-9]*$ ]] || return 2
        [[ "${start_time}" =~ ^[0-9]+$ ]] || return 2
        anchor_state="${state}"
        anchor_observed_pgid="${process_group_id}"
        anchor_observed_sid="${session_id}"
        anchor_observed_start_time="${start_time}"
    }

    anchor_identity_status() {
        local read_status=0
        if read_anchor_fields; then
            :
        else
            read_status=$?
            return "${read_status}"
        fi
        [[ "${anchor_state}" != Z ]] || return 1
        if [[ "${anchor_observed_pgid}" != "${anchor_pgid}" ]]; then
            return 3
        fi
        if [[ "${anchor_observed_sid}" != "${anchor_sid}" ]]; then
            return 3
        fi
        if [[ "${anchor_observed_start_time}" != "${anchor_start_time}" ]]; then
            return 3
        fi
        return 0
    }

    handle_control_line() {
        if [[ "${1}" =~ ^OWNER[[:space:]]([1-9][0-9]*)[[:space:]]([0-9]+)[[:space:]]([1-9][0-9]*)[[:space:]]([1-9][0-9]*)$ ]]; then
            if [[ -n "${anchor_pid}" || -n "${workload_status}" ]]; then
                return 1
            fi
            local owner_pid="${BASH_REMATCH[1]}"
            local owner_start_time="${BASH_REMATCH[2]}"
            local owner_pgid="${BASH_REMATCH[3]}"
            local owner_sid="${BASH_REMATCH[4]}"
            if [[ "${owner_pid}" != "${owner_pgid}" || "${owner_pid}" != "${owner_sid}" ]]; then
                return 1
            fi
            anchor_pid="${owner_pid}"
            anchor_start_time="${owner_start_time}"
            anchor_pgid="${owner_pgid}"
            anchor_sid="${owner_sid}"
            return 0
        fi
        if [[ "${1}" =~ ^STATUS[[:space:]]([0-9]|[1-9][0-9]|[1-9][0-9][0-9])$ ]]; then
            if [[ -z "${anchor_pid}" || -n "${workload_status}" ]]; then
                return 1
            fi
            workload_status="${BASH_REMATCH[1]}"
            return 0
        fi
        return 1
    }

    request_term() {
        if [[ -z "${requested_signal}" ]]; then
            requested_signal=TERM
        fi
        if (( waiting_for_supervisor )); then
            wait_interrupted=1
        fi
    }

    request_int() {
        if [[ -z "${requested_signal}" ]]; then
            requested_signal=INT
        fi
        if (( waiting_for_supervisor )); then
            wait_interrupted=1
        fi
    }

    request_private_cleanup() {
        if ! printf 'SIGNAL %s\n' "${requested_signal}" >&"${control_write_fd}"; then
            protocol_failed=1
        fi
        signal_failed=1
        signal_ambiguous=1
    }

    forward_signal() {
        if (( signal_forwarded )) || [[ -z "${requested_signal}" ]]; then
            return
        fi
        if [[ -n "${workload_status}" ]]; then
            signal_forwarded=1
            return
        fi
        if [[ -z "${anchor_pid}" ]]; then
            return
        fi
        local identity_status=0
        if anchor_identity_status; then
            :
        else
            identity_status=$?
            if (( identity_status == 1 )); then
                signal_forwarded=1
                return
            fi
            request_private_cleanup
            signal_forwarded=1
            return
        fi
        if ! kill "-${requested_signal}" -- "${anchor_pid}" 2>/dev/null; then
            if anchor_identity_status; then
                signal_failed=1
            else
                identity_status=$?
                if (( identity_status != 1 )); then
                    request_private_cleanup
                fi
            fi
        fi
        signal_forwarded=1
    }

    trap request_term TERM
    trap request_int INT

    coproc SRB_OWNED_SESSION {
        exec setsid --wait bash -c '
            set +e
            requested_signal=""
            control_relay_pid=""
            control_relay_start_time=""
            control_relay_reaped=0
            stdout_relay_pid=""
            stderr_relay_pid=""
            stdout_relay_start_time=""
            stderr_relay_start_time=""
            cleanup_failed=0
            # Keep grace timing independent of polling overhead.
            cleanup_grace_s=30
            if [[ "${SRB_HPC_TERM_GRACE_S:-}" =~ ^[0-9]+$ ]] &&
                (( SRB_HPC_TERM_GRACE_S > 0 )); then
                cleanup_grace_s=${SRB_HPC_TERM_GRACE_S}
            fi
            srb_hpc_term_grace_max_s=45
            if (( cleanup_grace_s > srb_hpc_term_grace_max_s )); then
                printf "srb hpc run: SRB_HPC_TERM_GRACE_S=%s exceeds the %ss safety window (SBATCH --signal=B:SIGTERM@60); clamping to %ss\n" \
                    "${cleanup_grace_s}" "${srb_hpc_term_grace_max_s}" \
                    "${srb_hpc_term_grace_max_s}" >&2
                cleanup_grace_s=${srb_hpc_term_grace_max_s}
            fi
            relay_grace_checks=20
            waiting_for_child=0
            child_wait_interrupted=0
            exec 6<&0

            request_term() {
                if [[ -z "${requested_signal}" ]]; then
                    requested_signal=TERM
                fi
                if (( waiting_for_child )); then
                    child_wait_interrupted=1
                fi
            }

            request_int() {
                if [[ -z "${requested_signal}" ]]; then
                    requested_signal=INT
                fi
                if (( waiting_for_child )); then
                    child_wait_interrupted=1
                fi
            }

            trap request_term TERM
            trap request_int INT

            relay_control_request() {
                local owner_group_id="${1}" control_line="" relay_signal=TERM
                trap - TERM INT
                if IFS= read -r control_line <&6; then
                    case "${control_line}" in
                        "SIGNAL TERM") relay_signal=TERM ;;
                        "SIGNAL INT") relay_signal=INT ;;
                        *) relay_signal=TERM ;;
                    esac
                fi
                kill "-${relay_signal}" -- "-${owner_group_id}" 2>/dev/null || :
            }

            read_process_identity_once() {
                local process_pid="${1}" stat stat_path
                local -a fields
                stat_path="/proc/${process_pid}/stat"
                if [[ ! -e "${stat_path}" ]]; then
                    return 1
                fi
                if [[ ! -r "${stat_path}" ]]; then
                    [[ -e "${stat_path}" ]] && return 2
                    return 1
                fi
                if ! IFS= read -r stat 2>/dev/null <"${stat_path}"; then
                    [[ -e "${stat_path}" ]] && return 2
                    return 1
                fi
                stat="${stat##*) }"
                read -r -a fields <<<"${stat}" || return 2
                if (( ${#fields[@]} < 20 )); then
                    return 2
                fi
                observed_state="${fields[0]}"
                observed_parent_pid="${fields[1]}"
                observed_pgid="${fields[2]}"
                observed_sid="${fields[3]}"
                observed_start_time="${fields[19]}"
                [[ "${observed_pgid}" =~ ^[1-9][0-9]*$ ]] || return 2
                [[ "${observed_sid}" =~ ^[1-9][0-9]*$ ]] || return 2
                [[ "${observed_start_time}" =~ ^[0-9]+$ ]] || return 2
            }

            read_process_identity() {
                local process_pid="${1}" identity_status=0 remaining=3
                while (( remaining > 0 )); do
                    if read_process_identity_once "${process_pid}"; then
                        return 0
                    else
                        identity_status=$?
                    fi
                    (( identity_status == 1 )) && return 1
                    remaining=$((remaining - 1))
                    (( remaining > 0 )) || break
                    sleep 0.01
                done
                return 2
            }

            bound_process_is_live() {
                local process_pid="${1}" expected_start_time="${2}" identity_status=0
                if read_process_identity "${process_pid}"; then
                    :
                else
                    identity_status=$?
                    if (( identity_status == 1 )); then
                        return 1
                    fi
                    cleanup_failed=1
                    return 0
                fi
                [[ "${observed_state}" != Z ]] || return 1
                [[ "${observed_start_time}" == "${expected_start_time}" ]]
            }

            signal_bound_process() {
                local process_pid="${1}" expected_start_time="${2}"
                local expected_pgid="${3}" expected_sid="${4}" signal_name="${5}"
                local identity_status=0
                if read_process_identity "${process_pid}"; then
                    :
                else
                    identity_status=$?
                    (( identity_status == 1 )) && return 0
                    return 1
                fi
                [[ "${observed_state}" != Z ]] || return 0
                [[ "${observed_start_time}" == "${expected_start_time}" ]] || return 1
                [[ "${observed_pgid}" == "${expected_pgid}" ]] || return 1
                [[ "${observed_sid}" == "${expected_sid}" ]] || return 1
                kill "-${signal_name}" -- "${process_pid}" 2>/dev/null || :
            }

            control_relay_identity_status() {
                local identity_status=0
                if read_process_identity "${control_relay_pid}"; then
                    if [[ "${observed_state}" == Z ]]; then
                        return 1
                    fi
                    if [[ "${observed_start_time}" != "${control_relay_start_time}" \
                        || "${observed_pgid}" != "$$" \
                        || "${observed_sid}" != "$$" ]]; then
                        return 2
                    fi
                    return 0
                else
                    identity_status=$?
                fi
                (( identity_status == 1 )) && return 1
                return 2
            }

            reap_stopped_control_relay() {
                wait "${control_relay_pid}" 2>/dev/null || :
                control_relay_reaped=1
            }

            stop_control_relay() {
                local identity_status=0 remaining=20
                (( control_relay_reaped == 0 )) || return 0

                control_relay_identity_status
                identity_status=$?
                if (( identity_status == 1 )); then
                    reap_stopped_control_relay
                    return 0
                fi
                if (( identity_status != 0 )); then
                    cleanup_failed=1
                    return 1
                fi
                if ! kill -TERM -- "${control_relay_pid}" 2>/dev/null; then
                    control_relay_identity_status
                    identity_status=$?
                    if (( identity_status == 1 )); then
                        reap_stopped_control_relay
                        return 0
                    fi
                    cleanup_failed=1
                    return 1
                fi

                while (( remaining > 0 )); do
                    control_relay_identity_status
                    identity_status=$?
                    if (( identity_status == 1 )); then
                        reap_stopped_control_relay
                        return 0
                    fi
                    if (( identity_status != 0 )); then
                        cleanup_failed=1
                        return 1
                    fi
                    sleep 0.01
                    remaining=$((remaining - 1))
                done

                if ! kill -KILL -- "${control_relay_pid}" 2>/dev/null; then
                    control_relay_identity_status
                    identity_status=$?
                    if (( identity_status == 1 )); then
                        reap_stopped_control_relay
                        return 0
                    fi
                    cleanup_failed=1
                    return 1
                fi

                remaining=20
                while (( remaining > 0 )); do
                    control_relay_identity_status
                    identity_status=$?
                    if (( identity_status == 1 )); then
                        reap_stopped_control_relay
                        return 0
                    fi
                    if (( identity_status != 0 )); then
                        cleanup_failed=1
                        return 1
                    fi
                    sleep 0.01
                    remaining=$((remaining - 1))
                done
                cleanup_failed=1
                return 1
            }

            list_owned_session_pids() {
                local session_output remaining=3
                session_pids=()
                while (( remaining > 0 )); do
                    if session_output="$(command ps -o pid= --sid "$$" 2>/dev/null)"; then
                        if [[ -n "${session_output}" ]]; then
                            mapfile -t session_pids <<<"${session_output}"
                        fi
                        return 0
                    fi
                    remaining=$((remaining - 1))
                    (( remaining > 0 )) || return 1
                    sleep 0.01
                done
                return 1
            }

            owned_session_has_live_members() {
                local process_pid identity_status=0
                local -a session_pids=()
                if ! list_owned_session_pids; then
                    cleanup_failed=1
                    return 0
                fi
                for process_pid in "${session_pids[@]}"; do
                    process_pid="${process_pid//[[:space:]]/}"
                    [[ "${process_pid}" =~ ^[1-9][0-9]*$ ]] || continue
                    if [[ "${process_pid}" == "$$" \
                        || "${process_pid}" == "${control_relay_pid}" \
                        || "${process_pid}" == "${stdout_relay_pid}" \
                        || "${process_pid}" == "${stderr_relay_pid}" ]]; then
                        continue
                    fi
                    if read_process_identity "${process_pid}"; then
                        :
                    else
                        identity_status=$?
                        if (( identity_status == 1 )); then
                            continue
                        fi
                        cleanup_failed=1
                        return 0
                    fi
                    if [[ "${observed_state}" != Z && "${observed_sid}" == "$$" ]]; then
                        return 0
                    fi
                done
                return 1
            }

            signal_owned_session_members() {
                local signal_name="${1}" process_pid expected_start_time identity_status=0
                local expected_pgid expected_sid
                local -a session_pids=()
                if ! list_owned_session_pids; then
                    cleanup_failed=1
                    return 1
                fi
                for process_pid in "${session_pids[@]}"; do
                    process_pid="${process_pid//[[:space:]]/}"
                    [[ "${process_pid}" =~ ^[1-9][0-9]*$ ]] || continue
                    if [[ "${process_pid}" == "$$" \
                        || "${process_pid}" == "${control_relay_pid}" \
                        || "${process_pid}" == "${stdout_relay_pid}" \
                        || "${process_pid}" == "${stderr_relay_pid}" ]]; then
                        continue
                    fi
                    if read_process_identity "${process_pid}"; then
                        :
                    else
                        identity_status=$?
                        if (( identity_status == 1 )); then
                            continue
                        fi
                        cleanup_failed=1
                        continue
                    fi
                    if [[ "${observed_state}" == Z || "${observed_sid}" != "$$" ]]; then
                        continue
                    fi
                    expected_start_time="${observed_start_time}"
                    expected_pgid="${observed_pgid}"
                    expected_sid="${observed_sid}"
                    if ! signal_bound_process "${process_pid}" \
                        "${expected_start_time}" "${expected_pgid}" \
                        "${expected_sid}" "${signal_name}"; then
                        cleanup_failed=1
                    fi
                done
            }

            terminate_owned_session_members() {
                local first_signal="${1}" grace_deadline
                signal_owned_session_members "${first_signal}"
                grace_deadline=$(( SECONDS + cleanup_grace_s ))
                while (( SECONDS < grace_deadline )); do
                    if ! owned_session_has_live_members; then
                        return 0
                    fi
                    sleep 0.05
                done
                if owned_session_has_live_members; then
                    signal_owned_session_members KILL
                fi
                sleep 0.05
                if owned_session_has_live_members; then
                    signal_owned_session_members KILL
                    sleep 0.05
                fi
                ! owned_session_has_live_members
            }

            if ! read_process_identity "$$"; then
                exit 125
            fi
            if [[ "${observed_pgid}" != "$$" || "${observed_sid}" != "$$" ]]; then
                exit 125
            fi
            if ! command -v ps >/dev/null 2>&1; then
                exit 125
            fi
            owner_start_time="${observed_start_time}"
            printf "OWNER %s %s %s %s\n" "$$" "${owner_start_time}" \
                "$$" "$$" >&3

            relay_dir="$(mktemp -d "${TMPDIR:-/tmp}/srb-hpc-supervisor.XXXXXXXX")" || {
                printf "STATUS 125\n" >&3
                exit 125
            }
            stdout_fifo="${relay_dir}/stdout"
            stderr_fifo="${relay_dir}/stderr"
            if ! mkfifo -- "${stdout_fifo}" "${stderr_fifo}"; then
                rmdir -- "${relay_dir}" 2>/dev/null || :
                printf "STATUS 125\n" >&3
                exit 125
            fi

            exec 4>&1 5>&2
            command cat "${stdout_fifo}" 3>&- 5>&- >&4 &
            stdout_relay_pid=$!
            if ! read_process_identity "${stdout_relay_pid}"; then
                kill -KILL -- "${stdout_relay_pid}" 2>/dev/null || :
                wait "${stdout_relay_pid}" 2>/dev/null || :
                rm -f -- "${stdout_fifo}" "${stderr_fifo}"
                rmdir -- "${relay_dir}" 2>/dev/null || :
                printf "STATUS 125\n" >&3
                exit 125
            fi
            stdout_relay_start_time="${observed_start_time}"

            command cat "${stderr_fifo}" 3>&- 4>&- >&5 &
            stderr_relay_pid=$!
            if ! read_process_identity "${stderr_relay_pid}"; then
                signal_bound_process "${stdout_relay_pid}" \
                    "${stdout_relay_start_time}" "$$" "$$" KILL || :
                kill -KILL -- "${stderr_relay_pid}" 2>/dev/null || :
                wait "${stdout_relay_pid}" 2>/dev/null || :
                wait "${stderr_relay_pid}" 2>/dev/null || :
                rm -f -- "${stdout_fifo}" "${stderr_fifo}"
                rmdir -- "${relay_dir}" 2>/dev/null || :
                printf "STATUS 125\n" >&3
                exit 125
            fi
            stderr_relay_start_time="${observed_start_time}"

            relay_control_request "$$" 3>&- 4>&- 5>&- \
                >/dev/null 2>&1 &
            control_relay_pid=$!
            control_relay_identity_valid=0
            if read_process_identity "${control_relay_pid}"; then
                if [[ "${observed_state}" != Z \
                    && "${observed_parent_pid}" == "$$" \
                    && "${observed_pgid}" == "$$" \
                    && "${observed_sid}" == "$$" ]]; then
                    control_relay_identity_valid=1
                    control_relay_start_time="${observed_start_time}"
                fi
            fi
            if (( control_relay_identity_valid == 0 )); then
                signal_bound_process "${stdout_relay_pid}" \
                    "${stdout_relay_start_time}" "$$" "$$" KILL || :
                signal_bound_process "${stderr_relay_pid}" \
                    "${stderr_relay_start_time}" "$$" "$$" KILL || :
                wait "${stdout_relay_pid}" 2>/dev/null || :
                wait "${stderr_relay_pid}" 2>/dev/null || :
                rm -f -- "${stdout_fifo}" "${stderr_fifo}"
                rmdir -- "${relay_dir}" 2>/dev/null || :
                printf "STATUS 125\n" >&3
                exec 3>&- 4>&- 5>&- 6>&-
                exit 125
            fi

            "$@" 3>&- 4>&- 5>&- 6>&- </dev/null >"${stdout_fifo}" 2>"${stderr_fifo}" &
            child_pid=$!
            child_status=0
            external_signal_handled=0
            while :; do
                child_wait_interrupted=0
                waiting_for_child=1
                wait "${child_pid}"
                child_wait_status=$?
                waiting_for_child=0

                cleanup_succeeded=1
                if [[ -n "${requested_signal}" \
                    && "${external_signal_handled}" -eq 0 ]]; then
                    if ! terminate_owned_session_members "${requested_signal}"; then
                        cleanup_failed=1
                        cleanup_succeeded=0
                    fi
                    external_signal_handled=1
                fi

                interrupted_status=0
                if [[ "${requested_signal}" == TERM ]]; then
                    interrupted_status=143
                elif [[ "${requested_signal}" == INT ]]; then
                    interrupted_status=130
                fi
                if (( child_wait_interrupted \
                    && child_wait_status == interrupted_status \
                    && cleanup_succeeded )); then
                    continue
                fi

                child_status="${child_wait_status}"
                break
            done

            if (( external_signal_handled == 0 )); then
                if ! terminate_owned_session_members TERM; then
                    cleanup_failed=1
                fi
            fi
            stop_control_relay || cleanup_failed=1

            remaining="${relay_grace_checks}"
            while (( remaining > 0 )); do
                stdout_relay_live=0
                stderr_relay_live=0
                if bound_process_is_live "${stdout_relay_pid}" \
                    "${stdout_relay_start_time}"; then
                    stdout_relay_live=1
                fi
                if bound_process_is_live "${stderr_relay_pid}" \
                    "${stderr_relay_start_time}"; then
                    stderr_relay_live=1
                fi
                if (( stdout_relay_live == 0 && stderr_relay_live == 0 )); then
                    break
                fi
                sleep 0.05
                remaining=$((remaining - 1))
            done

            if bound_process_is_live "${stdout_relay_pid}" \
                "${stdout_relay_start_time}"; then
                cleanup_failed=1
                signal_bound_process "${stdout_relay_pid}" \
                    "${stdout_relay_start_time}" "$$" "$$" TERM || cleanup_failed=1
            fi
            if bound_process_is_live "${stderr_relay_pid}" \
                "${stderr_relay_start_time}"; then
                cleanup_failed=1
                signal_bound_process "${stderr_relay_pid}" \
                    "${stderr_relay_start_time}" "$$" "$$" TERM || cleanup_failed=1
            fi
            sleep 0.05
            if bound_process_is_live "${stdout_relay_pid}" \
                "${stdout_relay_start_time}"; then
                signal_bound_process "${stdout_relay_pid}" \
                    "${stdout_relay_start_time}" "$$" "$$" KILL || cleanup_failed=1
            fi
            if bound_process_is_live "${stderr_relay_pid}" \
                "${stderr_relay_start_time}"; then
                signal_bound_process "${stderr_relay_pid}" \
                    "${stderr_relay_start_time}" "$$" "$$" KILL || cleanup_failed=1
            fi

            wait "${stdout_relay_pid}" 2>/dev/null || :
            wait "${stderr_relay_pid}" 2>/dev/null || :
            rm -f -- "${stdout_fifo}" "${stderr_fifo}"
            rmdir -- "${relay_dir}" 2>/dev/null || cleanup_failed=1
            exec 4>&- 5>&-

            final_status="${child_status}"
            if (( cleanup_failed )); then
                final_status=125
            fi
            printf "STATUS %s\n" "${final_status}" >&3
            exec 3>&- 6>&-
            exit "${final_status}"
        ' -- "${owned_command[@]}" 3>&1 1>&"${stdout_fd}" 2>&"${stderr_fd}"
    }
    supervisor_pid="${SRB_OWNED_SESSION_PID}"
    control_source_fd="${SRB_OWNED_SESSION[0]}"
    control_write_fd="${SRB_OWNED_SESSION[1]}"
    exec {control_read_fd}<&"${control_source_fd}"
    exec {control_source_fd}<&-

    while (( control_eof == 0 )); do
        if [[ -n "${requested_signal}" ]] && (( signal_forwarded == 0 )); then
            forward_signal
        fi
        control_char=""
        # Timed reads can consume input on timeout; read one byte to preserve frames.
        if IFS= read -r -N 1 -t 0.05 control_char <&"${control_read_fd}"; then
            read_status=0
        else
            read_status=$?
        fi
        if [[ -n "${control_char}" ]]; then
            if [[ "${control_char}" == $'\n' ]]; then
                line="${control_partial}"
                control_partial=""
                if ! handle_control_line "${line}"; then
                    protocol_failed=1
                fi
            else
                control_partial+="${control_char}"
                if (( ${#control_partial} > 128 )); then
                    protocol_failed=1
                    control_eof=1
                fi
            fi
        elif (( read_status == 1 )); then
            control_eof=1
        elif (( read_status > 128 )); then
            :
        else
            protocol_failed=1
            control_eof=1
        fi
        if (( protocol_failed )) && [[ -z "${requested_signal}" ]]; then
            requested_signal=TERM
        fi
        if (( signal_ambiguous )); then
            ambiguity_checks=$((ambiguity_checks + 1))
            if (( ambiguity_checks >= ambiguity_check_limit )); then
                protocol_failed=1
                ambiguity_timed_out=1
                control_eof=1
            fi
        fi
    done

    if [[ -n "${requested_signal}" ]] && (( signal_forwarded == 0 )); then
        forward_signal
    fi
    exec {control_write_fd}>&-

    if (( ambiguity_timed_out == 0 )); then
        while :; do
            wait_interrupted=0
            waiting_for_supervisor=1
            if wait "${supervisor_pid}"; then
                supervisor_status=0
                waiting_for_supervisor=0
                break
            else
                supervisor_status=$?
            fi
            waiting_for_supervisor=0
            if (( wait_interrupted )); then
                if [[ -n "${requested_signal}" ]] && (( signal_forwarded == 0 )); then
                    forward_signal
                fi
                continue
            fi
            break
        done
    else
        supervisor_status=125
    fi

    if (( ambiguity_timed_out == 0 )); then
        while IFS= read -r line <&"${control_read_fd}"; do
            line="${control_partial}${line}"
            control_partial=""
            if ! handle_control_line "${line}"; then
                protocol_failed=1
            fi
        done
    fi
    if [[ -n "${control_partial}" || -z "${anchor_pid}" || -z "${workload_status}" ]]; then
        protocol_failed=1
    fi
    exec {control_read_fd}<&-
    exec {stdout_fd}>&-
    exec {stderr_fd}>&-

    if (( protocol_failed || signal_failed )); then
        return_status=125
    elif [[ -n "${workload_status}" ]]; then
        return_status="${workload_status}"
    else
        return_status="${supervisor_status}"
    fi
    if ! restore_caller_traps; then
        return 125
    fi
    return "${return_status}"
}

# Keep in step with srb_vulkan_rt_device in .docker/run.bash.
srb_vulkan_rt_device() {
    awk '
        function flush() {
            if (in_dev && nvidia && accel && rtp && !gtx && found == "") found = name
        }
        /^GPU[0-9]+:?[ \t]*$/ {
            flush(); in_dev = 1; nvidia = 0; accel = 0; rtp = 0; gtx = 0; name = ""
            next
        }
        !in_dev { next }
        /^[ \t]*vendorID[ \t]*=/ { if ($0 ~ /0x10[dD][eE]/ || $0 ~ /=[ \t]*4318/) nvidia = 1 }
        /^[ \t]*deviceName[ \t]*=/ {
            name = $0; sub(/^[ \t]*deviceName[ \t]*=[ \t]*/, "", name)
            if (name ~ /GTX/) gtx = 1
        }
        /^[ \t]*VK_KHR_acceleration_structure[ \t]*:/ { accel = 1 }
        /^[ \t]*VK_KHR_ray_tracing_pipeline[ \t]*:/ { rtp = 1 }
        END { flush(); if (found == "") exit 1; print found }
    '
}

run_container() {
    local script_dir dot_docker_dir repository_dir images_dir project_name
    local image_path with_gpu with_gui require_gpu require_vulkan require_rtx
    local require_display ngc_enabled ngc_mode ngc_platform_input ngc_platform ngc_arch
    local ngc_image_digest ngc_expected_digest ngc_cache_root ngc_workspace
    local display_number volumes_root volume host_dir protected_dir rtx_device
    local ngc_expected_ref ngc_record ngc_record_source ngc_record_sif_sha256
    local ngc_record_key ngc_record_value ngc_sif_sha256
    local vulkan_report vulkan_report_status
    local -a singularity_exec_opts_array singularity_volumes
    local -a singularity_bind_args singularity_exec_cmd cmd

    if (( $# == 0 )); then
        echo >&2 -e "\033[1;31m[ERROR] No command provided.\033[0m"
        return 1
    fi
    cmd=("$@")

    script_dir="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" &>/dev/null && pwd)"
    dot_docker_dir="$(dirname "${script_dir}")"
    repository_dir="$(dirname "${dot_docker_dir}")"
    images_dir="${script_dir}/images"
    project_name="$(basename "${repository_dir}")"

    image_path="${IMAGE_PATH:-"${images_dir}/${project_name}.sif"}"
    if [[ -n "${SINGULARITY_EXEC_OPTS:-}" ]]; then
        read -r -a singularity_exec_opts_array <<<"${SINGULARITY_EXEC_OPTS}"
    else
        singularity_exec_opts_array=(--no-home)
    fi
    with_gpu="${WITH_GPU:-true}"
    with_gui="${WITH_GUI:-false}"
    require_gpu="${SRB_REQUIRE_GPU:-false}"
    require_vulkan="${SRB_REQUIRE_VULKAN:-false}"
    require_rtx="${SRB_REQUIRE_RTX:-false}"
    require_display="${SRB_REQUIRE_DISPLAY:-false}"
    ngc_enabled=false
    case "${SRB_DOCKER_PROFILE:-}" in
        ngc|ngc-headless|ngc-webrtc|ngc-gui) ngc_enabled=true ;;
    esac
    if [[ "${SRB_DOCKER_NGC:-false}" = true ]]; then
        ngc_enabled=true
    fi
    if [[ "${ngc_enabled}" = true ]]; then
        require_gpu=true
        require_vulkan=true
        require_rtx=true
        ngc_mode="${SRB_NGC_MODE:-headless}"
        if [[ "${ngc_mode,,}" = gui || "${SRB_DOCKER_PROFILE:-}" = ngc-gui ]]; then
            require_display=true
            with_gui="${WITH_GUI:-true}"
        elif [[ "${with_gui,,}" = true ]]; then
            echo >&2 "srb hpc run: NGC headless/WebRTC mode cannot use WITH_GUI=true"
            return 2
        fi
        ngc_platform_input="${SRB_NGC_PLATFORM:-${SRB_DOCKER_PLATFORM:-${DOCKER_DEFAULT_PLATFORM:-}}}"
        if [[ -z "${ngc_platform_input}" ]]; then
            ngc_platform_input="$(uname -m)"
        fi
        case "${ngc_platform_input,,}" in
            linux/amd64|amd64|x86_64)
                ngc_arch=amd64
                ngc_platform=linux/amd64
                ngc_image_digest="sha256:0c16dd67d09a70ea474f2c809a13c4d09bd23c738184cf1bf28af487ecd39080"
                ;;
            linux/arm64|arm64|aarch64)
                ngc_arch=arm64
                ngc_platform=linux/arm64
                ngc_image_digest="sha256:dcf1a6e81c56f644ab7ac69c4251291c51ecb234aa0792d12d0d9c7e4e4e7cf0"
                ;;
            *)
                echo >&2 "srb hpc run: unsupported NGC architecture/platform '${ngc_platform_input}'"
                return 2
                ;;
        esac
        ngc_expected_digest="${ngc_image_digest}"
        ngc_expected_ref="nvcr.io/nvidia/isaac-sim@${ngc_expected_digest}"
        if [[ -n "${SRB_NGC_IMAGE_DIGEST:-}" && "${SRB_NGC_IMAGE_DIGEST}" != "${ngc_expected_digest}" ]]; then
            echo >&2 "srb hpc run: NGC image digest does not match ${ngc_platform} Isaac Sim 6.1 digest"
            return 2
        fi
        if [[ -n "${SRB_NGC_IMAGE_REFERENCE:-}" && "${SRB_NGC_IMAGE_REFERENCE}" != "nvcr.io/nvidia/isaac-sim@${ngc_expected_digest}" ]]; then
            echo >&2 "srb hpc run: NGC image reference must be immutable and architecture-specific"
            return 2
        fi
    fi
    volumes_root="${SCRATCH:-${HOME}}/volumes/${project_name}"
    if [[ "${ngc_enabled}" = true ]]; then
        ngc_cache_root="${SRB_NGC_CACHE_DIR:-${volumes_root}/ngc/isaac-sim-6.1/${ngc_arch}}"
        if [[ -z "${ngc_cache_root}" || "${ngc_cache_root}" != /* ]]; then
            echo >&2 "srb hpc run: SRB_NGC_CACHE_DIR must be an absolute path to a dedicated cache directory"
            return 2
        fi
        ngc_cache_root="$(readlink -m -- "${ngc_cache_root}")"
        for protected_dir in / "${HOME:-}" "${repository_dir}"; do
            [[ -n "${protected_dir}" ]] || continue
            protected_dir="$(readlink -m -- "${protected_dir}")"
            if [[ "${ngc_cache_root}" = "${protected_dir}" || "${protected_dir}" = "${ngc_cache_root%/}"/* ]]; then
                echo >&2 "srb hpc run: SRB_NGC_CACHE_DIR must name a dedicated cache directory; '${ngc_cache_root}' is '${protected_dir}' or contains it"
                return 2
            fi
        done
        ngc_workspace="${SRB_NGC_WORKSPACE:-/workspace}"
        singularity_volumes=(
            "${ngc_cache_root}/cache:/isaac-sim/.cache:rw"
            "${ngc_cache_root}/compute:/isaac-sim/.nv/ComputeCache:rw"
            "${ngc_cache_root}/kit:/isaac-sim/kit/cache:rw"
            "${ngc_cache_root}/logs:/isaac-sim/.nvidia-omniverse/logs:rw"
            "${ngc_cache_root}/config:/isaac-sim/.nvidia-omniverse/config:rw"
            "${ngc_cache_root}/data:/isaac-sim/.local/share/ov/data:rw"
            "${ngc_cache_root}/pkg:/isaac-sim/.local/share/ov/pkg:rw"
            "${ngc_cache_root}/hub:/var/cache/hub:rw"
            "${repository_dir}:${ngc_workspace}:rw"
        )
        singularity_exec_opts_array+=(
            --env "ACCEPT_EULA=Y"
            --env "PRIVACY_CONSENT=Y"
            --env "ISAACSIM_PATH=/isaac-sim"
            --env "SRB_NGC_UID=$(id -u)"
            --env "SRB_IMAGE_ID=${ngc_expected_ref}"
            --env "SRB_NGC_ARCH=${ngc_arch}"
            --env "SRB_NGC_PLATFORM=${ngc_platform}"
            --env "PYTHONPATH=${ngc_workspace}"
        )
    else
        singularity_volumes=(
            "${volumes_root}/.nvidia-omniverse/data/ov:/root/.local/share/ov/data:rw"
            "${volumes_root}/.nvidia-omniverse/data/isaac-sim:/root/isaac-sim/kit/data:rw"
            "${volumes_root}/.cache/isaac-sim:/root/isaac-sim/kit/cache:rw"
            "${volumes_root}/.cache/nvidia/GLCache:/root/.cache/nvidia/GLCache:rw"
            "${volumes_root}/.cache/ov:/root/.cache/ov:rw"
            "${volumes_root}/.nv/ComputeCache:/root/.nv/ComputeCache:rw"
            "${volumes_root}/.nvidia-omniverse/logs:/root/.nvidia-omniverse/logs:rw"
            "${volumes_root}/.nvidia-omniverse/logs/isaac-sim:/root/isaac-sim/kit/logs:rw"
            "${volumes_root}/.cache/simforge:/root/.cache/simforge:rw"
            "${repository_dir}:/root/ws:rw"
            "${volumes_root}/home/users:/home/users:rw"
        )
    fi

    if [[ "${ngc_enabled}" = true && -z "${IMAGE_PATH:-}" ]]; then
        echo >&2 "srb hpc run: the NGC lane needs IMAGE_PATH naming a SIF built from ${ngc_expected_ref}; the default ${image_path} is the SRB source image"
        return 2
    fi
    if [ ! -f "${image_path}" ]; then
        echo >&2 -e "\033[1;31m[ERROR] Singularity image '${image_path}' is missing; set IMAGE_PATH to an existing SIF image and retry.\033[0m"
        return 1
    fi
    if [[ "${ngc_enabled}" = true ]]; then
        ngc_record="${image_path}.ngc-source"
        if [[ ! -f "${ngc_record}" ]]; then
            echo >&2 "srb hpc run: NGC image provenance record ${ngc_record} is missing; build the SIF from ${ngc_expected_ref} and record source= and sif_sha256="
            return 2
        fi
        ngc_record_source=""
        ngc_record_sif_sha256=""
        while IFS='=' read -r ngc_record_key ngc_record_value || [[ -n "${ngc_record_key}" ]]; do
            case "${ngc_record_key}" in
                source) ngc_record_source="${ngc_record_value}" ;;
                sif_sha256) ngc_record_sif_sha256="${ngc_record_value}" ;;
            esac
        done <"${ngc_record}"
        if [[ "${ngc_record_source}" != "${ngc_expected_ref}" ]]; then
            echo >&2 "srb hpc run: NGC image ${image_path} was built from '${ngc_record_source:-unknown}', not the ${ngc_platform} Isaac Sim 6.1 image ${ngc_expected_ref}"
            return 2
        fi
        if [[ ! "${ngc_record_sif_sha256}" =~ ^[0-9a-f]{64}$ ]]; then
            echo >&2 "srb hpc run: NGC image provenance record ${ngc_record} has no valid sif_sha256"
            return 2
        fi
        ngc_sif_sha256="$(sha256sum -- "${image_path}")"
        ngc_sif_sha256="${ngc_sif_sha256%% *}"
        if [[ "${ngc_sif_sha256}" != "${ngc_record_sif_sha256}" ]]; then
            echo >&2 "srb hpc run: NGC image ${image_path} (sha256 ${ngc_sif_sha256}) does not match its provenance record (sha256 ${ngc_record_sif_sha256})"
            return 2
        fi
    fi

    for volume in "${singularity_volumes[@]}"; do
        if [[ "${volume}" =~ ^([^:]+):([^:]+).*$ ]]; then
            host_dir="${BASH_REMATCH[1]}"
            if [ ! -d "${host_dir}" ]; then
                mkdir -p "${host_dir}"
                echo -e "[INFO] Created directory ${host_dir}"
            fi
        fi
    done
    if [[ "${ngc_enabled}" = true ]]; then
        case "${SRB_NGC_CACHE_OWNERSHIP:-user}" in
            user)
                for volume in "${singularity_volumes[@]}"; do
                    host_dir="${volume%%:*}"
                    [[ "${host_dir}" == "${ngc_cache_root}"/* ]] || continue
                    if [[ ! -w "${host_dir}" ]]; then
                        echo >&2 "srb hpc run: NGC cache directory ${host_dir} is not writable by the invoking user"
                        return 2
                    fi
                done
                ;;
            skip) ;;
            chown)
                echo >&2 "srb hpc run: SRB_NGC_CACHE_OWNERSHIP=chown does not apply here; Singularity runs the image as the invoking user, who must own the caches (use user or skip)"
                return 2
                ;;
            *)
                echo >&2 "srb hpc run: SRB_NGC_CACHE_OWNERSHIP must be user or skip"
                return 2
                ;;
        esac
    fi

    if [[ "${with_gpu,,}" = true ]]; then
        if [[ "${require_gpu,,}" = true && -n "${WITH_GPU_FORCE_NVIDIA:-}" ]]; then
            echo >&2 "srb hpc run: required GPU preflight rejects WITH_GPU_FORCE_NVIDIA"
            return 2
        fi
        if [[ "${require_gpu,,}" = true ]]; then
            if ! command -v nvidia-smi >/dev/null 2>&1 || ! nvidia-smi -L >/dev/null 2>&1; then
                echo >&2 "srb hpc run: required NVIDIA GPU preflight failed"
                return 2
            fi
        fi
        singularity_exec_opts_array+=(--nv)
    elif [[ "${require_gpu,,}" = true ]]; then
        echo >&2 "srb hpc run: required GPU preflight needs WITH_GPU=true"
        return 2
    fi
    if [[ "${require_vulkan,,}" = true ]]; then
        if ! command -v vulkaninfo >/dev/null 2>&1 || ! vulkaninfo --summary >/dev/null 2>&1; then
            echo >&2 "srb hpc run: required Vulkan preflight failed"
            return 2
        fi
    fi
    if [[ "${require_rtx,,}" = true ]]; then
        if ! command -v vulkaninfo >/dev/null 2>&1; then
            echo >&2 "srb hpc run: required RTX preflight could not read the Vulkan report: vulkaninfo is not installed"
            return 2
        fi
        vulkan_report_status=0
        vulkan_report="$(vulkaninfo 2>/dev/null)" || vulkan_report_status=$?
        if ! rtx_device="$(srb_vulkan_rt_device <<<"${vulkan_report}")"; then
            if grep -Eq '^GPU[0-9]+:?[[:space:]]*$' <<<"${vulkan_report}"; then
                echo >&2 "srb hpc run: required RTX preflight found no NVIDIA Vulkan device with hardware ray tracing (VK_KHR_ray_tracing_pipeline + VK_KHR_acceleration_structure)"
            elif [[ "${vulkan_report_status}" -ne 0 ]]; then
                echo >&2 "srb hpc run: required RTX preflight could not read the Vulkan report: vulkaninfo failed (exit ${vulkan_report_status})"
            else
                echo >&2 "srb hpc run: required RTX preflight could not read the Vulkan report: vulkaninfo listed no Vulkan device"
            fi
            return 2
        fi
        echo "srb hpc run: RTX preflight: hardware ray tracing on '${rtx_device}'"
    fi

    if [[ "${with_gui,,}" = true ]]; then
        display_number="${DISPLAY:-}"
        display_number="${display_number##*:}"
        display_number="${display_number%%.*}"
        if [[ -z "${DISPLAY:-}" || ! -e "/tmp/.X11-unix/X${display_number}" ]]; then
            if [[ "${require_display,,}" = true ]]; then
                echo >&2 "srb hpc run: required display preflight needs DISPLAY and /tmp/.X11-unix socket"
                return 2
            fi
            echo >&2 "srb hpc run: DISPLAY/socket unavailable; GUI disabled"
            with_gui=false
        elif [[ -z "${XAUTHORITY:-}" || ! -r "${XAUTHORITY}" ]]; then
            if [[ "${require_display,,}" = true ]]; then
                echo >&2 "srb hpc run: required display preflight needs readable XAUTHORITY"
                return 2
            fi
            echo >&2 "srb hpc run: XAUTHORITY unavailable; GUI disabled"
            with_gui=false
        else
            singularity_volumes+=(
                "/tmp/.X11-unix:/tmp/.X11-unix:ro"
                "${XAUTHORITY}:${XAUTHORITY}:ro"
            )
            singularity_exec_opts_array+=(
                --env "DISPLAY=${DISPLAY}"
                --env "XAUTHORITY=${XAUTHORITY}"
            )
        fi
    elif [[ "${require_display,,}" = true ]]; then
        echo >&2 "srb hpc run: required display preflight needs WITH_GUI=true"
        return 2
    fi

    if ! command -v module >/dev/null 2>&1; then
        echo >&2 -e "\033[1;31m[ERROR] The 'module' command is not available. Please run this script on a compute node.\033[0m"
        return 1
    fi
    module purge
    module load tools/Singularity

    singularity_bind_args=()
    for volume in "${singularity_volumes[@]}"; do
        singularity_bind_args+=(--bind "${volume}")
    done

    singularity_exec_cmd=(
        singularity exec
        "${singularity_exec_opts_array[@]}"
        "${singularity_bind_args[@]}"
        "${image_path}"
        bash --noprofile --norc -c 'if [[ -f /entrypoint.bash ]]; then source /entrypoint.bash -- || exit; fi; exec "$@"' srb-hpc
        "${cmd[@]}"
    )
    printf "\033[1;90m[TRACE] singularity exec image=%q opts=%d binds=%d command_args=%d\033[0m\n" \
        "${image_path}" "${#singularity_exec_opts_array[@]}" \
        "${#singularity_bind_args[@]}" "${#cmd[@]}"
    run_owned_session "${singularity_exec_cmd[@]}"
}

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    return 0
fi

run_container "$@"
