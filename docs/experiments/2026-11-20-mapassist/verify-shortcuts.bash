# Source this file in the operator's new interactive Bash terminal.
(
    if [[ $- != *i* ]]; then
        echo 'Open a new interactive Bash terminal and source verify-shortcuts.bash.' >&2
        exit 2
    fi
    here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd) || exit 1
    python3 "$here/shortcuts.py" --check || exit 1
    hash -r
    rc=0
    for cmd in allnodes yolop mapassist lya; do
        type -a "$cmd"
        if [[ $(type -t "$cmd") != file || $(command -v "$cmd") != "$HOME/.local/bin/$cmd" ]]; then
            echo "Wrong terminal shortcut for $cmd; inspect alias, function and PATH above." >&2
            rc=1
        fi
        while IFS= read -r found; do
            if [[ ! "$found" -ef "$HOME/.local/bin/$cmd" ]]; then
                echo "Duplicate terminal command: $found" >&2
                rc=1
            fi
        done < <(type -ap "$cmd")
    done
    for cmd in yolop-launch oa_2 oa-avoidance semantic mapassist-live; do
        if type -a "$cmd" 2>/dev/null; then
            echo "Obsolete terminal shortcut still resolves: $cmd" >&2
            rc=1
        fi
    done
    exit "$rc"
)
