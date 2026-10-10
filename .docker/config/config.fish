if set -q FISH_THEME
    fish_config theme choose "$FISH_THEME"
end

function fish_prompt
    set -l last_status $status
    set_color brblue
    printf docker
    set_color brwhite
    printf "|"
    set_color -o bryellow
    printf SRB
    set_color brwhite
    printf "@"
    set_color brgreen
    printf "%s" (hostname -s)
    set_color brcyan
    printf " %s" (prompt_pwd)
    if test $last_status -ne 0
        set_color brred
        printf " [%s]" $last_status
    end
    set_color normal
    printf " ❯ "
end

function fish_right_prompt
    if set -q _cmd_was_run
        set -l d "$CMD_DURATION"
        if test -n "$d" -a "$d" -gt 50
            set_color 555
            if test $d -lt 1000
                printf "%sms" "$d"
            else
                printf "%ss" (math -s1 $d / 1000)
            end
            set_color normal
        end
        set -e _cmd_was_run
    end
end

function stamp_time --on-event fish_preexec
    set -g _cmd_was_run 1
    set -l now (date "+%H:%M:%S")
    set_color 555
    printf "\033[F\033[%dG%s\n" (math $COLUMNS - (string length "$now") + 1) "$now"
    set_color normal
end


function fish_greeting
    function _v_clean
        printf "%s" "$argv[1]" | sed -E 's/([^0-9.]).*//' | awk '{print $1}'
    end

    function _get_uv_v
        set -l raw (uv pip show $argv[1] 2>/dev/null | awk '/Version:/ {print $2}')
        _v_clean "$raw"
    end

    set -l v_srb (_get_uv_v "srb")
    if test -z "$v_srb"
        set v_srb N/A
    end

    set -l v_isaac_sim N/A
    if test -f /root/isaac-sim/VERSION
        set v_isaac_sim (_v_clean (head -n1 "/root/isaac-sim/VERSION"))
    end

    set -l v_isaac_lab (_get_uv_v "isaaclab")
    if test -z "$v_isaac_lab" -o "$v_isaac_lab" = N/A
        if test -f /root/isaaclab/VERSION
            set v_isaac_lab (_v_clean (head -n1 "/root/isaaclab/VERSION"))
        else
            set v_isaac_lab N/A
        end
    end

    set -l v_simforge (_get_uv_v "simforge")
    if test -z "$v_simforge"
        set v_simforge N/A
    end

    set -l v_simforge_foundry (_get_uv_v "simforge-foundry")
    if test -z "$v_simforge_foundry"
        set v_simforge_foundry N/A
    end

    set -l v_blender N/A
    if command -v blender >/dev/null
        set v_blender (_v_clean (blender --version | head -n1 | awk '{print $2}'))
    end

    set -l v_ros_distro "$ROS_DISTRO"
    if test -z "$v_ros_distro"
        set v_ros_distro N/A
    end

    set -l v_ros_domain_id "$ROS_DOMAIN_ID"
    if test -z "$v_ros_domain_id"
        set v_ros_domain_id 0
    end

    set -l v_rmw_impl "$RMW_IMPLEMENTATION"
    if test -z "$v_rmw_impl"
        set v_rmw_impl \<unset\>
    end

    set -l v_nvidia N/A
    if test -f /proc/driver/nvidia/version
        set v_nvidia (head -n 1 /proc/driver/nvidia/version | awk '{print $8}' | cut -d. -f1,2)
    end

    set -l v_display "$DISPLAY"
    if test -z "$v_display"
        set v_display N/A
    end

    set -l c_lbl (set_color brcyan)
    set -l c_lbl_srb (set_color bryellow)
    set -l c_lbl_env (set_color brgreen)
    set -l c_lbl_system (set_color brmagenta)
    set -l c_val (set_color brwhite)
    set -l c_sep (set_color 555)
    set -l c_rst (set_color normal)

    printf "%s%-9s%s %-6s %s| %s%-9s%s %-6s %s| %s%-9s%s %-7s %s| %s%-7s%s %s\n%s%-9s%s %-6s %s| %s%-9s%s %-6s %s| %s%-9s%s %-7s %s| %s%-7s%s %s\n%s%-9s%s %-6s %s| %s%-9s%s %-6s %s| %s%-9s%s %s %s\n" \
        $c_lbl_srb SRB $c_val "$v_srb" $c_sep \
        $c_lbl SimForge $c_val "$v_simforge" $c_sep \
        $c_lbl_env "ROS 2" $c_val "$v_ros_distro" $c_sep \
        $c_lbl_env DISPLAY $c_val "$v_display" \
        $c_lbl "Isaac Sim" $c_val "$v_isaac_sim" $c_sep \
        $c_lbl "⤷ Foundry" $c_val "$v_simforge_foundry" $c_sep \
        $c_lbl_env "Domain ID" $c_val "$v_ros_domain_id" $c_sep \
        $c_lbl_system NVIDIA $c_val "$v_nvidia" \
        $c_lbl "Isaac Lab" $c_val "$v_isaac_lab" $c_sep \
        $c_lbl Blender $c_val "$v_blender" $c_sep \
        $c_lbl_env "RMW Impl." $c_val "$v_rmw_impl" $c_sep
    set_color normal
end
