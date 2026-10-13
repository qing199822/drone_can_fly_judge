#!/bin/bash

ensure_graphics_environment() {
    local session_environment="${1:-}"
    local desktop_pid=""
    local process_name
    local name
    local value

    if [ -z "${DISPLAY:-}" ] ||
       { [ -n "${XAUTHORITY:-}" ] && [ ! -r "$XAUTHORITY" ]; }; then
        if [ -z "$session_environment" ]; then
            for process_name in gnome-shell plasmashell; do
                desktop_pid="$(pgrep -n -u "$(id -u)" "$process_name" 2>/dev/null || true)"
                if [ -n "$desktop_pid" ]; then
                    session_environment="/proc/$desktop_pid/environ"
                    break
                fi
            done
        fi

        if [ -z "$session_environment" ] || [ ! -r "$session_environment" ]; then
            echo "错误：无法读取当前桌面会话，Gazebo 相机需要可用的图形显示。" >&2
            return 1
        fi

        while IFS='=' read -r name value; do
            case "$name" in
                DISPLAY)
                    if [ -z "${DISPLAY:-}" ]; then
                        export DISPLAY="$value"
                    fi
                    ;;
                XAUTHORITY)
                    if [ -z "${XAUTHORITY:-}" ] || [ ! -r "$XAUTHORITY" ]; then
                        export XAUTHORITY="$value"
                    fi
                    ;;
                XDG_RUNTIME_DIR|WAYLAND_DISPLAY)
                    if [ -z "${!name:-}" ]; then
                        export "$name=$value"
                    fi
                    ;;
            esac
        done < <(tr '\0' '\n' < "$session_environment")
    fi

    if [ -z "${DISPLAY:-}" ]; then
        echo "错误：未找到 DISPLAY，Gazebo 无法启用相机渲染。" >&2
        return 1
    fi

    if [ -n "${XAUTHORITY:-}" ] && [ ! -r "$XAUTHORITY" ]; then
        echo "错误：XAUTHORITY 不可读：$XAUTHORITY" >&2
        return 1
    fi
}
