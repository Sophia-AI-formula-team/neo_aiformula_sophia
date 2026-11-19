# Source from the operator's interactive Bash to include aliases and functions.
(
    if [[ $- != *i* ]]; then
        echo 'Open an interactive Bash terminal and source concept/verify-shortcuts.bash.' >&2
        exit 2
    fi
    here=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd) || exit 1
    python3 "$here/shortcuts.py" --check || exit 1
    hash -r
    type -a concept-live
    expected="$HOME/.local/bin/concept-live"
    if [[ $(type -t concept-live) != file || $(command -v concept-live) != "$expected" ]]; then
        echo "concept-live resolves to another command. Inspect type -a concept-live and PATH." >&2
        exit 1
    fi
    while IFS= read -r found; do
        if [[ ! "$found" -ef "$expected" ]]; then
            echo "Another concept-live is on PATH: $found" >&2
            exit 1
        fi
    done < <(type -ap concept-live)
)
