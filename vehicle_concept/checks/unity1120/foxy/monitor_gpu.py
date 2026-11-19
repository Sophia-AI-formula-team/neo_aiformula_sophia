"""Run a test command and sample host-wide GPU load in a CSV beside its output."""
import argparse
import csv
import subprocess
import time
from datetime import datetime, timezone


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output')
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    cmd = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not cmd:
        parser.error('a test command is required')
    fields = ('index', 'utilization.gpu', 'memory.used', 'clocks.sm', 'pstate')
    with open(args.output, 'w', newline='') as trace:
        writer = csv.writer(trace)
        writer.writerow(['utc', *fields])
        proc = subprocess.Popen(cmd)
        try:
            while proc.poll() is None:
                utc = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
                sample = subprocess.run(['nvidia-smi', '--query-gpu=' + ','.join(fields),
                                         '--format=csv,noheader,nounits'], capture_output=True, text=True,
                                        check=True, timeout=5)
                for line in sample.stdout.splitlines():
                    writer.writerow([utc, *(x.strip() for x in line.split(','))])
                trace.flush()
                time.sleep(.25)
        finally:
            result = proc.wait()
    raise SystemExit(result)


if __name__ == '__main__':
    main()
