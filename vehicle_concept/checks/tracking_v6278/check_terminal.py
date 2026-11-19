"""Exercise the installed Foxy terminal menu against the replay driver (Linux)."""
import os
from pathlib import Path
import pty
import select
import signal
import subprocess
import time


def run_choice(key, expected):
    master, slave = pty.openpty()
    command = ['ros2', 'run', 'concept', 'live', 'mode:=replay', 'panel:=false', 'rviz:=false',
               'device:=cpu', 'use_sim_time:=true']
    process = subprocess.Popen(command, stdin=slave, stdout=slave, stderr=slave, start_new_session=True)
    os.close(slave)
    transcript = b''
    try:
        deadline = time.monotonic() + 15
        while b'Select 1 or 2' not in transcript and time.monotonic() < deadline:
            if select.select([master], [], [], .25)[0]:
                transcript += os.read(master, 8192)
        if b'Select 1 or 2' not in transcript:
            raise RuntimeError('Menu did not open: ' + transcript.decode(errors='replace'))
        os.write(master, key + b'\n')
        deadline = time.monotonic() + 30
        while time.monotonic() < deadline:
            if select.select([master], [], [], .25)[0]:
                transcript += os.read(master, 16384)
            if b'[drive-' not in transcript:
                continue
            result = subprocess.run(['ros2', 'param', 'get', '/concept/drive', 'tracking_controller'],
                                    capture_output=True, text=True, timeout=7)
            if result.returncode == 0 and result.stdout.strip().endswith(': ' + expected):
                print('Terminal choice', key.decode(), '=>', result.stdout.strip(), flush=True)
                return
        raise RuntimeError('Driver did not confirm the choice: ' + transcript.decode(errors='replace')[-5000:])
    finally:
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        os.close(master)


if __name__ == '__main__':
    from ament_index_python.packages import get_package_share_directory

    os.environ.setdefault('FASTRTPS_DEFAULT_PROFILES_FILE', str(
        Path(get_package_share_directory('concept')) / 'config' / 'foxy-fastdds.xml'))
    run_choice(b'1', 'pure_pursuit')
    run_choice(b'2', 'lya')
