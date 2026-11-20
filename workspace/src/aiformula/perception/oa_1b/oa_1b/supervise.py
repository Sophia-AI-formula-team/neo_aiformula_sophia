"""Own the four foreground vehicle stages in one Linux ROS-domain session."""
import array
import ctypes
import fcntl
import json
import os
from pathlib import Path
import select
import signal
import socket
import struct
import subprocess
import sys
import time
from contextlib import ExitStack


STAGES = ("allnodes", "yolop", "oa1b", "lya")
PACKET_SIZE = 192 * 1024


def session_address(domain):
    # Linux abstract sockets disappear on close, including after a crash.
    return "\0aiformula-oa1b-{}-{}".format(os.getuid(), domain)


def verify_peer(sock):
    _, uid, _ = struct.unpack("3i", sock.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED, 12))
    if uid != os.getuid():
        raise PermissionError("The OA1B session belongs to another user")


def lock_vehicle(stack, domain):
    home = Path.home()
    # The original launcher can be invoked from terminals with or without XDG_RUNTIME_DIR.
    roots = {home, Path(os.environ.get("XDG_RUNTIME_DIR") or home)}
    login_runtime = Path("/run/user") / str(os.getuid())
    if login_runtime.is_dir() and login_runtime.stat().st_uid == os.getuid():
        roots.add(login_runtime)
    paths = {root / f"aiformula-1120-{domain}-{name}.lock" for root in roots
             for name in ("allnodes", "mapassist", "lya")}
    paths.add(home / "aiformula_runtime/2026-11-19-yolop-selection" / f"selector-{domain}.lock")
    for path in sorted(paths):
        handle = stack.enter_context(path.open("a+b"))
        try:
            fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError(f"Vehicle experiment already owns {path}") from error


def receive_request(conn):
    descriptor_size = array.array("i").itemsize
    payload, ancillary, flags, _ = conn.recvmsg(PACKET_SIZE, socket.CMSG_SPACE(3 * descriptor_size))
    descriptors = array.array("i")
    for level, kind, data in ancillary:
        if level == socket.SOL_SOCKET and kind == socket.SCM_RIGHTS:
            descriptors.frombytes(data[:len(data) - len(data) % descriptor_size])
    try:
        if not payload or flags & (socket.MSG_TRUNC | socket.MSG_CTRUNC):
            raise ValueError("Incomplete stage request")
        request = json.loads(payload)
        stage = request["stage"]
        if stage not in (*STAGES[1:], "stop"):
            raise ValueError("Unknown stage")
        if stage != "stop" and (len(descriptors) != 3 or not request["argv"]):
            raise ValueError("Stage has no command or terminal")
        return request, descriptors
    except Exception:
        for fd in descriptors:
            os.close(fd)
        raise


def descendants():
    # PR_SET_CHILD_SUBREAPER retains orphans even if a launch grandchild calls setsid().
    # Read ancestry rather than matching executable names or ROS process group names.
    children = {}
    for path in Path("/proc").glob("[0-9]*/stat"):
        try:
            pid = int(path.parent.name)
            fields = path.read_bytes().rsplit(b") ", 1)[1].split()
            children.setdefault(int(fields[1]), []).append((pid, fields[19], fields[0]))
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
    pending = [os.getpid()]
    result = []
    while pending:
        for child in children.get(pending.pop(), ()):
            result.append(child)
            pending.append(child[0])
    return result


def stop_children():
    began = time.monotonic()
    last_signal = {}
    while True:
        try:
            while os.waitpid(-1, os.WNOHANG)[0]:
                pass
        except ChildProcessError:
            return
        age = time.monotonic() - began
        sig = signal.SIGINT if age < 3 else signal.SIGTERM if age < 5 else signal.SIGKILL
        for pid, start, state in descendants():
            identity = (pid, start)
            if state == b"Z" or last_signal.get(identity) == sig:
                continue
            try:
                # A PID seen in /proc can have exited between enumeration and signalling.
                stat = Path(f"/proc/{pid}/stat").read_bytes().rsplit(b") ", 1)[1].split()
                if stat[19] == start:
                    os.kill(pid, sig)
                    last_signal[identity] = sig
            except (FileNotFoundError, ProcessLookupError):
                pass
        time.sleep(.05)


def report(conn, **message):
    try:
        conn.sendall(json.dumps(message).encode())
    except OSError:
        pass


def run_session(argv, domain, interrupted, primary):
    children, peers = [], [primary]
    result = 0
    with ExitStack() as stack:
        lock_vehicle(stack, domain)
        listener = stack.enter_context(socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET))
        listener.bind(session_address(domain))
        listener.listen(8)
        if ctypes.CDLL(None, use_errno=True).prctl(36, 1, 0, 0, 0) != 0:  # PR_SET_CHILD_SUBREAPER
            raise OSError(ctypes.get_errno(), "Cannot supervise orphaned children")
        try:
            children.append(subprocess.Popen(argv, start_new_session=True))
            while not interrupted:
                ended = next((p.returncode for p in children if p.poll() is not None), None)
                if ended is not None:
                    result = ended if ended >= 0 else 128 - ended
                    break
                readable, _, _ = select.select([listener, *peers], [], [], .08)
                close_requests = [peer for peer in peers if peer in readable]
                if close_requests:
                    for peer in close_requests:
                        peer.recv(PACKET_SIZE)
                    break  # A stage terminal closed or explicitly requested shutdown.
                if listener not in readable:
                    continue
                conn, _ = listener.accept()
                fds = []
                try:
                    verify_peer(conn)
                    conn.settimeout(1)
                    request, fds = receive_request(conn)
                    stage = request["stage"]
                    if stage == "stop":
                        peers.append(conn)
                        break
                    index = len(children)
                    if index >= len(STAGES) or stage != STAGES[index]:
                        raise ValueError("Expected " + (STAGES[index] if index < len(STAGES) else "no additional stages"))
                    child = subprocess.Popen(request["argv"], env=request["env"], cwd=request["cwd"],
                                             stdin=fds[0], stdout=fds[1], stderr=fds[2], start_new_session=True)
                    children.append(child)
                    peers.append(conn)
                    report(conn, started=stage)
                except (OSError, ValueError, KeyError) as error:
                    report(conn, error=str(error))
                    conn.close()
                    if isinstance(error, (FileNotFoundError, PermissionError)) and fds:
                        result = 1
                        break
                finally:
                    for fd in fds:
                        os.close(fd)
        except BaseException:
            result = 1
            raise
        finally:
            # Hold the listener and four shared locks until the last orphan was reaped.
            stop_children()
            while select.select([listener], [], [], 0)[0]:
                late, _ = listener.accept()
                with late:
                    try:
                        verify_peer(late)
                        late.settimeout(.25)
                        request, fds = receive_request(late)
                        for fd in fds:
                            os.close(fd)
                        if request["stage"] == "stop":
                            report(late, done=result)
                        else:
                            report(late, error="OA1B session is stopping")
                    except (OSError, ValueError, KeyError):
                        pass
            listener.close()
            stack.close()
            for peer in peers:
                # A closing SOCK_SEQPACKET socket with an unread stop packet may
                # reset the caller before it receives the completion receipt.
                try:
                    if select.select([peer], [], [], 0)[0]:
                        peer.recv(PACKET_SIZE)
                except OSError:
                    pass
                report(peer, done=result)
                peer.close()
    return result


def wait_session(conn, interrupted, requested_stop=False):
    while True:
        if interrupted and not requested_stop:
            conn.sendall(b"stop")
            requested_stop = True
        readable, _, _ = select.select([conn], [], [], .08)
        if not readable:
            continue
        packet = conn.recv(PACKET_SIZE)
        if not packet:
            raise RuntimeError("OA1B supervisor disconnected before completing cleanup")
        answer = json.loads(packet)
        if "error" in answer:
            raise RuntimeError(answer["error"])
        if "done" in answer:
            return int(answer["done"])


def primary_terminal(argv, domain, interrupted):
    # Keeping ownership outside the terminal lets kernel EOF stop the ROS tree even
    # if the terminal forcibly kills its foreground client without delivering SIGHUP.
    terminal, owner = socket.socketpair(socket.AF_UNIX, socket.SOCK_SEQPACKET)
    child = os.fork()
    if child == 0:
        terminal.close()
        code = 1
        try:
            os.setsid()
            code = run_session(argv, domain, interrupted, owner)
        except (RuntimeError, OSError) as error:
            report(owner, error=str(error))
            print(f"OA1B: {error}", file=sys.stderr, flush=True)
        finally:
            owner.close()
            os._exit(code)
    owner.close()
    try:
        return wait_session(terminal, interrupted)
    finally:
        terminal.close()
        os.waitpid(child, 0)


def join_session(stage, argv, domain, interrupted):
    with socket.socket(socket.AF_UNIX, socket.SOCK_SEQPACKET) as conn:
        try:
            conn.connect(session_address(domain))
        except ConnectionRefusedError:
            if stage == "stop":
                return 0
            raise RuntimeError("Start OA1B allnodes first")
        verify_peer(conn)
        request = dict(stage=stage, argv=argv, cwd=os.getcwd(), env=dict(os.environ))
        rights = [] if stage == "stop" else [(socket.SOL_SOCKET, socket.SCM_RIGHTS, array.array("i", [0, 1, 2]))]
        conn.sendmsg([json.dumps(request).encode()], rights)
        return wait_session(conn, interrupted, requested_stop=(stage == "stop"))


def main(args=None):
    args = list(sys.argv[1:] if args is None else args)
    domain = os.environ.get("ROS_DOMAIN_ID") or "0"
    if not domain.isdecimal() or int(domain) > 232 or str(int(domain)) != domain:
        raise SystemExit("Invalid ROS_DOMAIN_ID")
    if args != ["stop"] and (len(args) < 3 or args[0] not in STAGES or args[1] != "--"):
        raise SystemExit("Usage: supervise.py <allnodes|yolop|oa1b|lya> -- command ... | stop")
    interrupted = []
    for sig in (signal.SIGINT, signal.SIGHUP, signal.SIGTERM):
        signal.signal(sig, lambda number, _frame: interrupted.append(number))
    try:
        if args[0] == STAGES[0]:
            return primary_terminal(args[2:], domain, interrupted)
        return join_session(args[0], args[2:], domain, interrupted)
    except (RuntimeError, OSError) as error:
        print(f"OA1B: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
