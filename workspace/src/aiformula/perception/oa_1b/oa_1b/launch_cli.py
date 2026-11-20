"""Let Foxy's regular launch shutdown run when the MapAssist B terminal closes."""
import os
import signal

from ros2cli.cli import main


if __name__ == "__main__":
    signal.signal(signal.SIGHUP, lambda _signum, _frame: os.kill(os.getpid(), signal.SIGINT))
    raise SystemExit(main())
