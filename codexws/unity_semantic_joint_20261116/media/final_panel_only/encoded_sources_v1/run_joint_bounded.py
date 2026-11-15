"""Run one joint renderer in a Windows-owned Job: 300 s primary, 5 s cleanup.

No retry, shell, broad process kill, existing-directory write, or policy import.
"""
from __future__ import annotations

import argparse
import ctypes
from ctypes import wintypes
import datetime
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import time

JOB_SHA = "aa6f98f52029502a5ad4452acafc7829f99d1a31e55a7d1b518b21446f98fd24"
PRIMARY_SEC, CLEANUP_SEC = 300, 5
MUTEX_NAME = r"Local\AIFormulaSemanticJointMediaOneEncoder"

BOOTSTRAP = r'''
import ctypes,datetime,hashlib,json,os,pathlib,sys
from ctypes import wintypes
k=ctypes.WinDLL('kernel32',use_last_error=True)
k.GetCurrentProcess.argtypes=[];k.GetCurrentProcess.restype=wintypes.HANDLE
k.GetProcessTimes.argtypes=[wintypes.HANDLE]+[ctypes.POINTER(wintypes.FILETIME)]*4;k.GetProcessTimes.restype=wintypes.BOOL
k.GetModuleFileNameW.argtypes=[wintypes.HANDLE,wintypes.LPWSTR,wintypes.DWORD];k.GetModuleFileNameW.restype=wintypes.DWORD
k.IsProcessInJob.argtypes=[wintypes.HANDLE,wintypes.HANDLE,ctypes.POINTER(wintypes.BOOL)];k.IsProcessInJob.restype=wintypes.BOOL
current=k.GetCurrentProcess();creation,exit_time,kernel,user=(wintypes.FILETIME() for _ in range(4))
if not k.GetProcessTimes(current,ctypes.byref(creation),ctypes.byref(exit_time),ctypes.byref(kernel),ctypes.byref(user)):
 raise ctypes.WinError(ctypes.get_last_error())
ticks=(creation.dwHighDateTime<<32)|creation.dwLowDateTime
module=ctypes.create_unicode_buffer(32768);size=k.GetModuleFileNameW(None,module,len(module))
if not size or size>=len(module):raise ctypes.WinError(ctypes.get_last_error())
member=wintypes.BOOL()
if not k.IsProcessInJob(current,None,ctypes.byref(member)):raise ctypes.WinError(ctypes.get_last_error())
if not member.value:raise RuntimeError('Interpreter is not in a Job')
identity=dict(pid=os.getpid(),creation_filetime_100ns=str(ticks),executable=module.value,
 kernel_in_any_job=True,recorded_before_site_main=True)
with pathlib.Path(sys.argv[3]).open('x',encoding='utf-8') as stream:json.dump(identity,stream,indent=2)
import site
site.main()
import runpy
script=pathlib.Path(sys.argv[1])
if hashlib.sha256(script.read_bytes()).hexdigest()!=sys.argv[2]:raise RuntimeError('Renderer hash changed')
sys.path.insert(0,str(script.parent))
sys.argv=[str(script),'render','--plan',sys.argv[4],'--output',sys.argv[5],'--permit-encoding']
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


def signatures(kernel):
    for name, arguments, result in (
        ("CreateMutexW", [ctypes.c_void_p, wintypes.BOOL, wintypes.LPCWSTR], wintypes.HANDLE),
        ("ReleaseMutex", [wintypes.HANDLE], wintypes.BOOL),
        ("WaitForSingleObject", [wintypes.HANDLE, wintypes.DWORD], wintypes.DWORD),
        ("CloseHandle", [wintypes.HANDLE], wintypes.BOOL),
        ("OpenProcess", [wintypes.DWORD, wintypes.BOOL, wintypes.DWORD], wintypes.HANDLE),
        ("IsProcessInJob", [wintypes.HANDLE, wintypes.HANDLE, ctypes.POINTER(wintypes.BOOL)], wintypes.BOOL),
    ):
        function = getattr(kernel, name)
        function.argtypes, function.restype = arguments, result


def identity(job, pid, expected_birth=None):
    k = job.kernel
    handle = k.OpenProcess(0x00101000, False, pid)  # SYNCHRONIZE | QUERY_LIMITED_INFORMATION
    if not handle:
        error = ctypes.get_last_error()
        return dict(pid=pid, state="absent" if error == 87 else "query_failed", error=error)
    try:
        creation, exited, kernel, user = (wintypes.FILETIME() for _ in range(4))
        if not k.GetProcessTimes(handle, ctypes.byref(creation), ctypes.byref(exited), ctypes.byref(kernel), ctypes.byref(user)):
            raise ctypes.WinError(ctypes.get_last_error())
        token = str((creation.dwHighDateTime << 32) | creation.dwLowDateTime)
        state = k.WaitForSingleObject(handle, 0)
        if state not in (0, 258):
            raise ctypes.WinError(ctypes.get_last_error())
        member = wintypes.BOOL()
        if not k.IsProcessInJob(handle, job.handle, ctypes.byref(member)):
            raise ctypes.WinError(ctypes.get_last_error())
        return dict(pid=pid, creation_filetime_100ns=token, in_owned_job=bool(member.value),
                    state="pid_reused_original_absent" if expected_birth is not None and token != expected_birth
                    else "exited" if state == 0 else "live")
    finally:
        k.CloseHandle(handle)


def member_ids(job):
    capacity = 64
    while capacity <= 4096:
        data = ctypes.create_string_buffer(8 + ctypes.sizeof(ctypes.c_size_t)*capacity)
        if job.kernel.QueryInformationJobObject(job.handle, 3, data, len(data), None):
            count = ctypes.cast(data, ctypes.POINTER(wintypes.DWORD))[1]
            if count <= capacity:
                pointer = ctypes.cast(ctypes.byref(data, 8), ctypes.POINTER(ctypes.c_size_t))
                return [int(pointer[i]) for i in range(count)]
        elif ctypes.get_last_error() != 234:
            raise ctypes.WinError(ctypes.get_last_error())
        capacity *= 2
    raise RuntimeError("Unexpectedly large owned process tree")


def input_hashes(plan_path, plan, renderer, helper, python):
    single = "aiformula_semantic_joint_media/v1"
    combined = "aiformula_semantic_joint_combined_media/v1"
    if plan.get("schema") not in (single, combined):
        raise ValueError("A prepared joint media plan is required")
    expected = {}

    def put(path, value):
        name = str(Path(path).resolve())
        if name in expected and expected[name] != value:
            raise ValueError("Sealed plans disagree about a shared source/tool hash")
        expected[name] = value

    plans = [plan]
    if plan["schema"] == combined:
        if len(plan.get("cases", [])) != 2:
            raise ValueError("Combined plan must contain exactly two cases")
        for case in plan["cases"]:
            put(case["plan_file"], case["plan_sha256"])
            child = read(case["plan_file"])
            if child.get("schema") != single:
                raise ValueError("Nested combined plans are not supported")
            plans.append(child)
    for item in plans:
        if item["schema"] == single:
            root = Path(item["source"]).resolve()
            for name, value in item["source_sha256"].items():
                path = (root/name).resolve()
                path.relative_to(root)
                put(path, value)
        put(renderer, item["script_sha256"])
        backend = item.get("encoder_backend") or {"kind": "ffmpeg"}
        if backend["kind"] == "pyav":
            for path, value in backend["files_sha256"].items():
                put(path, value)
        elif backend["kind"] == "ffmpeg" and item.get("ffmpeg") and item["schema"] == single:
            put(item["ffmpeg"], item["ffmpeg_sha256"])
        else:
            raise ValueError("Plan must pin its installed PyAV or FFmpeg encoder")
        for key in ("evaluation_surface", "render_transition_receipt"):
            if item.get(key):
                put(item[key], item[key+"_sha256"])
    put(helper, JOB_SHA)
    for path in (plan_path, Path(__file__).resolve(), python):
        expected[str(path)] = sha(path)
    actual = {name: sha(name) for name in expected}
    if actual != expected:
        raise ValueError("A pinned recording/tool hash differs")
    return actual


def attempt(args, WindowsOwnedJob, mutex_state):
    directory = args.attempt.resolve()
    media_root = Path(__file__).resolve().parents[2]/"media"
    directory.relative_to(media_root)
    if directory == media_root:
        raise ValueError("Use a new attempt subdirectory within this task's media directory")
    directory.mkdir(parents=True, exist_ok=False)
    output = directory/"render"
    plan_path, python = args.plan.resolve(), args.python.resolve()
    renderer = Path(__file__).with_name("joint_media.py").resolve()
    helper = Path(__file__).with_name("windows_owned_job.py").resolve()
    receipt = dict(schema="joint_media_owned_receipt/v1", status="starting", exit_code=1,
                   primary_limit_sec=PRIMARY_SEC, cleanup_limit_sec=CLEANUP_SEC, automatic_retry=False,
                   mutex_name=MUTEX_NAME, previous_mutex_abandoned=mutex_state == 0x80,
                   attempt_directory=str(directory), output_directory=str(output), visual_qa="not performed")
    job = process = thread = None
    before, plan, known = None, None, {}
    start = time.monotonic()
    actual_path = directory/"actual_interpreter_identity.json"
    try:
        plan = read(plan_path)
        before = input_hashes(plan_path, plan, renderer, helper, python)
        write(directory/"source_hashes_before.json", before)
        write(directory/"attempt.json", dict(plan=str(plan_path), renderer=str(renderer), python=str(python),
              renderer_sha256=sha(renderer), plan_sha256=sha(plan_path), helper_sha256=JOB_SHA,
              driver_sha256=sha(__file__), bootstrap_sha256=hashlib.sha256(BOOTSTRAP.encode()).hexdigest(),
              started_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
              primary_limit_sec=PRIMARY_SEC, output_directory=str(output)))
        if time.monotonic()-start >= PRIMARY_SEC:
            raise subprocess.TimeoutExpired("input validation", PRIMARY_SEC)
        job = WindowsOwnedJob()
        signatures(job.kernel)
        command = [str(python), "-B", "-S", "-c", BOOTSTRAP, str(renderer), sha(renderer),
                   str(actual_path), str(plan_path), str(output)]
        environment = dict(os.environ, PYTHONDONTWRITEBYTECODE="1", AIFORMULA_JOINT_MEDIA_BOUNDED="1")
        with (directory/"stdout.log").open("xb") as stdout, (directory/"stderr.log").open("xb") as stderr:
            process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr,
                      cwd=str(directory), env=environment,
                      creationflags=0x00000004 | subprocess.CREATE_NO_WINDOW)  # CREATE_SUSPENDED
            birth = job.birth(process)
            birth["filetime_100ns"] = str(birth["filetime_100ns"])
            receipt["launcher"] = dict(pid=process.pid, **birth)
            job.assign(process)
            thread, thread_identity = job.open_initial_thread(process)
            thread_identity["creation_filetime_100ns"] = str(thread_identity["creation_filetime_100ns"])
            receipt["initial_thread"] = thread_identity
            if job.kernel.ResumeThread(thread) != 1:
                raise RuntimeError("Unexpected initial suspend count")
            job.kernel.CloseHandle(thread)
            thread = None
            receipt["containment"] = "CREATE_SUSPENDED -> assign owned KILL_ON_JOB_CLOSE Job -> verify unique initial thread -> ResumeThread(1)"
            while True:
                if time.monotonic()-start >= PRIMARY_SEC:
                    raise subprocess.TimeoutExpired("joint render", PRIMARY_SEC)
                for pid in member_ids(job):
                    item = identity(job, pid)
                    if item.get("creation_filetime_100ns"):
                        known[(pid, item["creation_filetime_100ns"])] = item
                if actual_path.exists() and "actual_interpreter_identity" not in receipt:
                    try:
                        actual = read(actual_path)
                    except json.JSONDecodeError:
                        actual = None  # one small identity file may still be finishing its write
                    if actual:
                        observed = identity(job, actual["pid"], actual["creation_filetime_100ns"])
                        if observed.get("state") == "live" and not observed.get("in_owned_job"):
                            raise RuntimeError("Actual Python interpreter is outside the owned Job")
                        receipt["actual_interpreter_identity"] = actual
                        receipt["actual_interpreter_independent_check"] = observed
                        known.setdefault((actual["pid"], actual["creation_filetime_100ns"]), observed)
                code = process.poll()
                if code is not None:
                    if time.monotonic()-start >= PRIMARY_SEC:
                        raise subprocess.TimeoutExpired("joint render", PRIMARY_SEC)
                    receipt.update(exit_code=code, status="renderer_finished" if code == 0 else "renderer_failed_no_retry")
                    break
                time.sleep(.05)
    except subprocess.TimeoutExpired:
        receipt.update(status="deadline_exceeded_no_retry", exit_code=124)
    except Exception as error:
        receipt.update(status="failed_no_retry", exit_code=1, error=type(error).__name__+": "+str(error))
    finally:
        receipt["primary_elapsed_sec"] = time.monotonic()-start
        stop = time.monotonic()
        try:
            if job:
                receipt["job_active_before_stop"] = job.active()
                if receipt["job_active_before_stop"]:
                    job.terminate()
                    while job.active() and time.monotonic()-stop < 4.8:
                        time.sleep(.025)
                receipt["job_active_after_stop"] = job.active()
                if receipt["job_active_after_stop"]:
                    raise RuntimeError("Owned Job did not empty within cleanup bound")
            if process and process.poll() is None:
                process.kill()  # exact owned Popen handle; only if Job assignment failed
                process.wait(timeout=max(.001, CLEANUP_SEC-(time.monotonic()-stop)))
            if process:
                receipt["launcher_exit_code"] = process.poll()
            if job:
                receipt["independent_final_checks"] = [identity(job, pid, token) for pid, token in known]
                if any(item["state"] not in ("absent", "exited", "pid_reused_original_absent")
                       for item in receipt["independent_final_checks"]):
                    raise RuntimeError("A known process remains live or unverified")
        except Exception as error:
            receipt.update(status="cleanup_unverified_do_not_return_CPU", exit_code=1,
                           cleanup_error=type(error).__name__+": "+str(error))
        finally:
            if thread and job:
                job.kernel.CloseHandle(thread)
            if job:
                job.close()  # closes only this Job; KILL_ON_JOB_CLOSE is the final tree backstop
        receipt["cleanup_elapsed_sec"] = time.monotonic()-stop
        receipt["cleanup_verified"] = bool(job and receipt.get("job_active_after_stop") == 0
                  and "cleanup_error" not in receipt and receipt["cleanup_elapsed_sec"] <= CLEANUP_SEC)
        if job and not receipt["cleanup_verified"]:
            receipt.update(status="cleanup_unverified_do_not_return_CPU", exit_code=1)
        receipt["known_processes"] = list(known.values())
        receipt["process_scope"] = "Job member birth tokens sampled at 50 ms; short-lived processes may be unsampled. ActiveProcesses=0 verifies whole owned tree."
        try:
            if before is not None:
                after = input_hashes(plan_path, plan, renderer, helper, python)
                write(directory/"source_hashes_after.json", after)
                receipt["source_hashes_before_after_equal"] = before == after
                if before != after:
                    raise RuntimeError("Source/tool hashes changed during attempt")
            if receipt["exit_code"] == 0:
                if not receipt["cleanup_verified"] or not receipt.get("actual_interpreter_identity"):
                    raise RuntimeError("Interpreter identity or complete cleanup unconfirmed")
                if receipt.get("actual_interpreter_independent_check", {}).get("in_owned_job") is not True:
                    raise RuntimeError("Independent actual-interpreter Job membership was not observed")
                verification = read(output/"receipt.json")
                if verification.get("complete_decode") is not True or verification.get("source_hashes_before_after") is not True:
                    raise RuntimeError("Renderer did not confirm decode/source integrity")
                if plan["schema"] == "aiformula_semantic_joint_combined_media/v1":
                    if (verification.get("case_boundaries_readback_checked") is not True
                            or sha(output/"case_boundaries.json") != verification.get("case_boundaries_sha256")):
                        raise RuntimeError("Combined case boundary/readback verification is missing")
                    receipt["case_boundaries_sha256"] = verification["case_boundaries_sha256"]
                    receipt["cases"] = plan["cases"]
                video = Path(verification["video"]).resolve()
                video.relative_to(output.resolve())
                if sha(video) != verification["video_sha256"]:
                    raise RuntimeError("Video hash differs from renderer receipt")
                receipt.update(status="machine_checks_complete_visual_QA_pending", video=str(video),
                               video_sha256=verification["video_sha256"], frames=verification["output_frames"])
        except Exception as error:
            receipt.update(status="post_verification_failed_no_retry", exit_code=1,
                           verification_error=type(error).__name__+": "+str(error))
        write(directory/"receipt.json", receipt)
    print(json.dumps({key: receipt.get(key) for key in ("status", "exit_code", "primary_elapsed_sec",
                     "cleanup_elapsed_sec", "cleanup_verified", "job_active_after_stop", "video")}, indent=2))
    return receipt["exit_code"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", type=Path, required=True)
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument("--attempt", type=Path, required=True, help="New owned directory; renderer writes only its render subdirectory")
    args = parser.parse_args()
    if os.name != "nt":
        parser.error("Windows Job Objects required; there is no uncontained fallback")
    helper = Path(__file__).with_name("windows_owned_job.py")
    if sha(helper) != JOB_SHA:
        raise RuntimeError("Reviewed Windows Job helper changed")
    sys.dont_write_bytecode = True
    spec = importlib.util.spec_from_file_location("joint_media_owned_job", helper)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    kernel = ctypes.WinDLL("kernel32", use_last_error=True)
    signatures(kernel)
    mutex = kernel.CreateMutexW(None, False, MUTEX_NAME)
    if not mutex:
        raise ctypes.WinError(ctypes.get_last_error())
    acquired = False
    try:
        state = kernel.WaitForSingleObject(mutex, 0)
        if state == 258:
            print("Another joint media attempt owns the encoder mutex; nothing launched.", file=sys.stderr)
            return 75
        if state not in (0, 0x80):
            raise ctypes.WinError(ctypes.get_last_error())
        acquired = True
        return attempt(args, module.WindowsOwnedJob, state)
    finally:
        if acquired:
            kernel.ReleaseMutex(mutex)
        kernel.CloseHandle(mutex)


if __name__ == "__main__":
    raise SystemExit(main())
