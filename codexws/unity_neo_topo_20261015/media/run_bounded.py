"""One Windows-owned media attempt, explicit deadline, no retry or broad kill.

The builder performs full decode and source validation. This supervisor additionally
seals inputs before/after, records decimal-string process birth tokens, assigns a
suspended launcher to a kill-on-close Job, and verifies that Job is empty on exit.
"""
from __future__ import annotations

import argparse
import ctypes
from ctypes import wintypes
import datetime
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

JOB_MODULE_SHA = "aa6f98f52029502a5ad4452acafc7829f99d1a31e55a7d1b518b21446f98fd24"

BOOTSTRAP = r'''
import ctypes, sys
from ctypes import wintypes
k=ctypes.WinDLL('kernel32',use_last_error=True)
k.OpenEventW.argtypes=[wintypes.DWORD,wintypes.BOOL,wintypes.LPCWSTR]
k.OpenEventW.restype=wintypes.HANDLE
k.WaitForSingleObject.argtypes=[wintypes.HANDLE,wintypes.DWORD]
k.WaitForSingleObject.restype=wintypes.DWORD
k.CloseHandle.argtypes=[wintypes.HANDLE]
k.CloseHandle.restype=wintypes.BOOL
h=k.OpenEventW(0x00100000,False,sys.argv[1])
if not h: raise SystemExit(125)
try: ready=k.WaitForSingleObject(h,15000)==0
finally: k.CloseHandle(h)
if not ready: raise SystemExit(125)
import os,json,pathlib,datetime
k.GetCurrentProcess.argtypes=[]
k.GetCurrentProcess.restype=wintypes.HANDLE
k.GetProcessTimes.argtypes=[wintypes.HANDLE]+[ctypes.POINTER(wintypes.FILETIME)]*4
k.GetProcessTimes.restype=wintypes.BOOL
k.GetModuleFileNameW.argtypes=[wintypes.HANDLE,wintypes.LPWSTR,wintypes.DWORD]
k.GetModuleFileNameW.restype=wintypes.DWORD
k.IsProcessInJob.argtypes=[wintypes.HANDLE,wintypes.HANDLE,ctypes.POINTER(wintypes.BOOL)]
k.IsProcessInJob.restype=wintypes.BOOL
creation,exit_time,kernel,user=(wintypes.FILETIME() for _ in range(4))
current=k.GetCurrentProcess()
if not k.GetProcessTimes(current,ctypes.byref(creation),ctypes.byref(exit_time),ctypes.byref(kernel),ctypes.byref(user)):
    raise ctypes.WinError(ctypes.get_last_error())
ticks=(creation.dwHighDateTime<<32)|creation.dwLowDateTime
module=ctypes.create_unicode_buffer(32768)
size=k.GetModuleFileNameW(None,module,len(module))
if not size or size>=len(module): raise ctypes.WinError(ctypes.get_last_error())
in_job=wintypes.BOOL()
if not k.IsProcessInJob(current,None,ctypes.byref(in_job)):
    raise ctypes.WinError(ctypes.get_last_error())
identity=dict(pid=os.getpid(),creation_filetime_100ns=str(ticks),
    creation_utc=datetime.datetime.fromtimestamp(ticks/10000000-11644473600,datetime.timezone.utc).isoformat(),
    executable=module.value,kernel_in_any_job=bool(in_job.value),recorded_before_site_main=True)
with pathlib.Path(sys.argv[4]).open('x',encoding='utf-8') as stream:
    json.dump(identity,stream,indent=2)
import site
site.main()
import hashlib,runpy
script=pathlib.Path(sys.argv[2])
if hashlib.sha256(script.read_bytes()).hexdigest()!=sys.argv[3]: raise SystemExit(126)
plan=sys.argv[5]
sys.path.insert(0,str(script.parent))
sys.argv=[str(script),'--plan',plan,'--permit-encoding']
runpy.run_path(str(script),run_name='__main__')
'''


def sha(path):
    with Path(path).open("rb") as stream:
        return hashlib.file_digest(stream, "sha256").hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def input_hashes(plan_path, plan, builder):
    source = Path(plan["source"]).resolve()
    expected = {}
    for name, value in plan["source_sha256"].items():
        path = (source / name).resolve()
        path.relative_to(source)
        expected[str(path)] = value
    for name, value in plan["scripts_sha256"].items():
        path = (builder.parent / name).resolve()
        path.relative_to(builder.parent)
        expected[str(path)] = value
    for key in ("renderer", "ffmpeg", "policy", "versions"):
        path = plan[key + "_file"] if key in ("policy", "versions") else plan[key]
        expected[str(Path(path).resolve())] = plan[key + "_sha256"]
    expected[str(plan_path)] = sha(plan_path)
    expected[str(Path(__file__).resolve())] = sha(__file__)
    expected[str(Path(__file__).with_name("windows_owned_job.py").resolve())] = JOB_MODULE_SHA
    actual = {path: sha(path) for path in expected}
    mismatches = [path for path in expected if actual[path] != expected[path]]
    if mismatches:
        raise RuntimeError("Pinned input mismatch: " + repr(mismatches))
    return actual


class ProcessAudit:
    def __init__(self, job):
        self.job, self.k = job, job.kernel
        signatures = {
            "OpenProcess": ([wintypes.DWORD, wintypes.BOOL, wintypes.DWORD], wintypes.HANDLE),
            "GetExitCodeProcess": ([wintypes.HANDLE, ctypes.POINTER(wintypes.DWORD)], wintypes.BOOL),
            "QueryFullProcessImageNameW": ([wintypes.HANDLE, wintypes.DWORD, wintypes.LPWSTR, ctypes.POINTER(wintypes.DWORD)], wintypes.BOOL),
            "IsProcessInJob": ([wintypes.HANDLE, wintypes.HANDLE, ctypes.POINTER(wintypes.BOOL)], wintypes.BOOL),
            "WaitForSingleObject": ([wintypes.HANDLE, wintypes.DWORD], wintypes.DWORD),
        }
        for name, (arguments, result) in signatures.items():
            function = getattr(self.k, name)
            function.argtypes, function.restype = arguments, result

    def identity(self, pid, expected_birth=None):
        handle = self.k.OpenProcess(0x00101000, False, pid)  # SYNCHRONIZE | QUERY_LIMITED_INFORMATION
        if not handle:
            error = ctypes.get_last_error()
            if error == 87:
                return {"pid": pid, "state": "absent", "expected_birth": expected_birth}
            return {"pid": pid, "state": "query_failed", "error": error, "expected_birth": expected_birth}
        try:
            creation, exited, kernel, user = (wintypes.FILETIME() for _ in range(4))
            if not self.k.GetProcessTimes(handle, ctypes.byref(creation), ctypes.byref(exited), ctypes.byref(kernel), ctypes.byref(user)):
                raise ctypes.WinError(ctypes.get_last_error())
            ticks = (creation.dwHighDateTime << 32) | creation.dwLowDateTime
            token = str(ticks)
            signaled = self.k.WaitForSingleObject(handle, 0)
            if signaled not in (0, 258):
                raise ctypes.WinError(ctypes.get_last_error())
            buffer, length = ctypes.create_unicode_buffer(32768), wintypes.DWORD(32768)
            image = buffer.value if self.k.QueryFullProcessImageNameW(handle, 0, buffer, ctypes.byref(length)) else None
            member = wintypes.BOOL()
            if not self.k.IsProcessInJob(handle, self.job.handle, ctypes.byref(member)):
                raise ctypes.WinError(ctypes.get_last_error())
            state = "exited" if signaled == 0 else "live"
            if expected_birth is not None and token != expected_birth:
                state = "pid_reused_original_absent"
            return {"pid": pid, "creation_filetime_100ns": token,
                    "creation_utc": datetime.datetime.fromtimestamp(ticks / 10000000 - 11644473600, datetime.timezone.utc).isoformat(),
                    "executable": image, "state": state, "in_owned_job": bool(member.value), "expected_birth": expected_birth,
                    "query": "independent OpenProcess/GetProcessTimes/WaitForSingleObject/IsProcessInJob"}
        finally:
            self.k.CloseHandle(handle)

    def member_ids(self):
        capacity = 64
        while capacity <= 4096:
            data = ctypes.create_string_buffer(8 + ctypes.sizeof(ctypes.c_size_t) * capacity)
            if self.k.QueryInformationJobObject(self.job.handle, 3, data, len(data), None):
                count = ctypes.cast(data, ctypes.POINTER(wintypes.DWORD))[1]
                if count > capacity:
                    capacity *= 2
                    continue
                pointer = ctypes.cast(ctypes.byref(data, 8), ctypes.POINTER(ctypes.c_size_t))
                return [int(pointer[index]) for index in range(count)]
            if ctypes.get_last_error() != 234:
                raise ctypes.WinError(ctypes.get_last_error())
            capacity *= 2
        raise RuntimeError("Unexpected owned Job process count")


def main():
    if os.name != "nt":
        raise RuntimeError("This supervisor requires Windows Job Objects; no uncontained fallback")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--builder", type=Path, default=Path(__file__).with_name("build_video.py"))
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument("--receipt-directory", type=Path, required=True)
    parser.add_argument("--max-wall-sec", type=float, required=True)
    args = parser.parse_args()
    if not (0 < args.max_wall_sec <= 3600):
        parser.error("Explicit primary bound must be >0 and <=3600 seconds")
    module = Path(__file__).with_name("windows_owned_job.py")
    if sha(module) != JOB_MODULE_SHA:
        raise RuntimeError("Reviewed Windows Job helper changed")
    from windows_owned_job import WindowsOwnedJob
    plan_path, builder = args.plan.resolve(), args.builder.resolve()
    plan = read(plan_path)
    output = Path(plan["output_directory"])
    if output.exists():
        raise FileExistsError("Preserve previous media attempt; output directory already exists")
    receipt_dir = args.receipt_directory.resolve()
    receipt_dir.mkdir(parents=True, exist_ok=False)
    before = input_hashes(plan_path, plan, builder)
    write(receipt_dir / "source_hashes_before.json", before)
    write(receipt_dir / "attempt.json", {"schema": "neo_media_owned_attempt/v1", "plan": str(plan_path),
          "plan_sha256": sha(plan_path), "builder": str(builder), "builder_sha256": sha(builder),
          "driver_sha256": sha(__file__), "job_helper_sha256": JOB_MODULE_SHA, "max_primary_sec": args.max_wall_sec,
          "max_stop_sec": 5, "automatic_retry": False, "utc": datetime.datetime.now(datetime.timezone.utc).isoformat()})
    receipt = {"schema": "neo_media_owned_receipt/v1", "status": "starting", "exit_code": 1,
               "actual_interpreter_identity": None, "known_processes": [], "visual_qa": "not performed"}
    job, process, thread, event, audit = None, None, None, None, None
    known = {}
    primary_start = time.monotonic()
    actual_path = receipt_dir / "actual_interpreter_identity.json"
    try:
        job = WindowsOwnedJob()
        audit = ProcessAudit(job)
        receipt["parent"] = audit.identity(os.getpid())
        event_name = "Local\\AIFormulaNeoMedia_" + uuid.uuid4().hex
        event = job.kernel.CreateEventW(None, True, False, event_name)
        if not event:
            raise ctypes.WinError(ctypes.get_last_error())
        command = [str(args.python.resolve()), "-B", "-S", "-c", BOOTSTRAP, event_name,
                   str(builder), sha(builder), str(actual_path), str(plan_path)]
        receipt["command_without_bootstrap"] = [str(args.python.resolve()), "-B", "-S", "<pinned bootstrap>", event_name, str(builder), sha(builder), str(actual_path), str(plan_path)]
        environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", AIFORMULA_NEO_MEDIA_BOUNDED="1")
        with (receipt_dir / "stdout.log").open("xb") as stdout, (receipt_dir / "stderr.log").open("xb") as stderr:
            process = subprocess.Popen(command, stdout=stdout, stderr=stderr, env=environment,
                                       creationflags=0x00000004 | subprocess.CREATE_NO_WINDOW)  # CREATE_SUSPENDED
            birth = job.birth(process)
            birth["filetime_100ns"] = str(birth["filetime_100ns"])
            receipt["launcher"] = {"pid": process.pid, **birth}
            job.assign(process)
            thread, thread_identity = job.open_initial_thread(process)
            thread_identity["creation_filetime_100ns"] = str(thread_identity["creation_filetime_100ns"])
            receipt["initial_thread"] = thread_identity
            previous_suspend = job.kernel.ResumeThread(thread)
            if previous_suspend != 1:
                raise RuntimeError("Unexpected initial suspend count: " + str(previous_suspend))
            job.kernel.CloseHandle(thread)
            thread = None
            if not job.kernel.SetEvent(event):
                raise ctypes.WinError(ctypes.get_last_error())
            receipt["isolation"] = "CREATE_SUSPENDED -> owned Job assignment -> unique initial thread verified -> ResumeThread(1) -> event"
            while True:
                elapsed = time.monotonic() - primary_start
                if elapsed >= args.max_wall_sec:
                    raise subprocess.TimeoutExpired(str(builder), args.max_wall_sec)
                for pid in audit.member_ids():
                    identity = audit.identity(pid)
                    token = identity.get("creation_filetime_100ns")
                    if token:
                        known[(pid, token)] = identity
                code = process.poll()
                if code is not None:
                    if time.monotonic() - primary_start >= args.max_wall_sec:
                        raise subprocess.TimeoutExpired(str(builder), args.max_wall_sec)
                    receipt["exit_code"] = code
                    receipt["status"] = "builder_finished" if code == 0 else "builder_failed_no_retry"
                    break
                time.sleep(min(0.05, max(0.001, args.max_wall_sec - elapsed)))
    except subprocess.TimeoutExpired:
        receipt.update(status="primary_deadline_exceeded_no_retry", exit_code=124)
    except Exception as error:
        receipt.update(status="failed_no_retry", exit_code=1, error=type(error).__name__ + ": " + str(error))
    finally:
        receipt["primary_elapsed_sec"] = time.monotonic() - primary_start
        cleanup_start = time.monotonic()
        try:
            if job:
                receipt["job_active_before_stop"] = job.active()
                job.terminate()
                while job.active() and time.monotonic() - cleanup_start < 4.8:
                    time.sleep(.025)
                receipt["job_active_after_stop"] = job.active()
                if receipt["job_active_after_stop"] != 0:
                    raise RuntimeError("Owned Job still active after bounded stop")
            if process and process.poll() is None:
                process.kill()  # owned suspended launcher only, if Job assignment failed
                process.wait(timeout=max(.001, 5 - (time.monotonic() - cleanup_start)))
            if process:
                receipt["launcher_exit_code"] = process.poll()
                process._handle.Close()
            if actual_path.exists():
                receipt["actual_interpreter_identity"] = read(actual_path)
                item = receipt["actual_interpreter_identity"]
                known.setdefault((item["pid"], item["creation_filetime_100ns"]), item)
            if receipt.get("launcher"):
                item = receipt["launcher"]
                known.setdefault((item["pid"], item["filetime_100ns"]), item)
            if audit:
                receipt["independent_process_checks"] = [audit.identity(pid, token) for pid, token in known]
                if any(item["state"] not in ("absent", "exited", "pid_reused_original_absent") for item in receipt["independent_process_checks"]):
                    raise RuntimeError("A known process birth token remains live or unverified")
        except Exception as error:
            receipt.update(status="cleanup_unverified_do_not_return_CPU", exit_code=1,
                           cleanup_error=type(error).__name__ + ": " + str(error))
        finally:
            if thread and job:
                job.kernel.CloseHandle(thread)
            if event and job:
                job.kernel.CloseHandle(event)
            if job:
                job.close()
        receipt["cleanup_elapsed_sec"] = time.monotonic() - cleanup_start
        receipt["cleanup_verified"] = (receipt.get("job_active_after_stop") == 0
                                        and "cleanup_error" not in receipt
                                        and receipt["cleanup_elapsed_sec"] <= 5)
        if receipt["cleanup_elapsed_sec"] > 5:
            receipt.update(status="cleanup_bound_exceeded", exit_code=1)
        receipt["known_processes"] = list(known.values())
        receipt["process_sampling_scope"] = "Known Job members sampled every 50ms, plus launcher and interpreter bootstrap; short-lived PIDs may be unsampled. Job ActiveProcesses=0 verifies complete owned tree."
        try:
            after = input_hashes(plan_path, plan, builder)
            write(receipt_dir / "source_hashes_after.json", after)
            receipt["source_hashes_before_after_equal"] = before == after
            if before != after:
                raise RuntimeError("Input sources changed during media attempt")
            verification_path = output / "verification.json"
            if receipt["exit_code"] == 0:
                if not receipt["cleanup_verified"] or not receipt.get("actual_interpreter_identity"):
                    raise RuntimeError("Owned tree cleanup or actual interpreter identity unconfirmed")
                verification = read(verification_path)
                if verification.get("source_hashes_before_after_equal") is not True or verification["full_decode"].get("every_frame_index_and_pts_checked") is not True:
                    raise RuntimeError("Builder did not confirm full decode/source integrity")
                if sha(verification["video"]) != verification["video_sha256"]:
                    raise RuntimeError("Encoded video hash differs from builder receipt")
                receipt.update(status="machine_verification_complete_visual_QA_pending",
                               verification_sha256=sha(verification_path), video=verification["video"],
                               video_sha256=verification["video_sha256"], frames=verification["frames"])
        except Exception as error:
            receipt.update(status="post_verification_failed_no_retry", exit_code=1,
                           verification_error=type(error).__name__ + ": " + str(error))
        write(receipt_dir / "receipt.json", receipt)
    print(json.dumps({key: receipt.get(key) for key in ("status", "exit_code", "primary_elapsed_sec", "cleanup_elapsed_sec", "job_active_after_stop", "source_hashes_before_after_equal", "video", "frames")}, indent=2))
    return receipt["exit_code"]


if __name__ == "__main__":
    raise SystemExit(main())
