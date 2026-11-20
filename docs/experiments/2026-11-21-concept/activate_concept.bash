#!/usr/bin/env bash
# Source this file from Bash. The build and the installed desktop launcher share it.
_concept_activate_1121() {
    local ws="$HOME/concept_1121_ws"
    local venv="$ws/venv" site preload lib
    source /opt/ros/foxy/setup.bash || return 1
    source /home/workspace/install/setup.bash || return 1
    if [[ -d "$venv" ]]; then
        [[ -x "$venv/bin/python3" ]] || { echo "Broken vehicle Python environment: $venv" >&2; return 1; }
        site=$("$venv/bin/python3" -c 'import sysconfig; print(sysconfig.get_path("purelib"))') || return 1
        export VIRTUAL_ENV="$venv" PYTHONNOUSERSITE=1
        case ":${PATH}:" in *":$venv/bin:"*) ;; *) export PATH="$venv/bin:$PATH" ;; esac
        # Foxy console scripts can use /usr/bin/python3 via their shebang.
        case ":${PYTHONPATH:-}:" in *":$site:"*) ;; *) export PYTHONPATH="$site${PYTHONPATH:+:$PYTHONPATH}" ;; esac
    fi
    if [[ $(uname -m) == aarch64 ]]; then
        preload=${LD_PRELOAD:-}
        for lib in /lib/aarch64-linux-gnu/libGLdispatch.so.0 /lib/aarch64-linux-gnu/libgomp.so.1; do
            [[ -r "$lib" ]] || { echo "Missing 11/19 Jetson runtime library: $lib" >&2; return 1; }
            case ":${preload}:" in *":$lib:"*) ;; *) preload="$lib${preload:+:$preload}" ;; esac
        done
        export LD_PRELOAD="$preload"
    fi
    if [[ ${1:-} != --base-for-build ]]; then
        source "$ws/install/local_setup.bash" || return 1
    fi
}
_concept_activate_1121 "${1:-}" || { unset -f _concept_activate_1121; return 1; }
unset -f _concept_activate_1121
