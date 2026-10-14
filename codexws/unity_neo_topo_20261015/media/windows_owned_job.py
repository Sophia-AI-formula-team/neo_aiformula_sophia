"""Verbatim WindowsOwnedJob class from reviewed fixture driver03; no task paths."""
import ctypes
from ctypes import wintypes
import datetime

class WindowsOwnedJob:
    def __init__(self):
        self.kernel=ctypes.WinDLL('kernel32',use_last_error=True)
        class BasicLimits(ctypes.Structure):
            _fields_=[('ProcessTime',ctypes.c_longlong),('JobTime',ctypes.c_longlong),
                ('LimitFlags',wintypes.DWORD),('MinWorkingSet',ctypes.c_size_t),('MaxWorkingSet',ctypes.c_size_t),
                ('ActiveProcessLimit',wintypes.DWORD),('Affinity',ctypes.c_size_t),
                ('Priority',wintypes.DWORD),('Scheduling',wintypes.DWORD)]
        class IoCounters(ctypes.Structure):
            _fields_=[(name,ctypes.c_ulonglong) for name in ('ReadOps','WriteOps','OtherOps','ReadBytes','WriteBytes','OtherBytes')]
        class ExtendedLimits(ctypes.Structure):
            _fields_=[('Basic',BasicLimits),('Io',IoCounters),('ProcessMemory',ctypes.c_size_t),
                ('JobMemory',ctypes.c_size_t),('PeakProcessMemory',ctypes.c_size_t),('PeakJobMemory',ctypes.c_size_t)]
        class Accounting(ctypes.Structure):
            _fields_=[(name,ctypes.c_longlong) for name in ('UserTime','KernelTime','PeriodUser','PeriodKernel')]+[
                (name,wintypes.DWORD) for name in ('PageFaults','TotalProcesses','ActiveProcesses','TerminatedProcesses')]
        self.Accounting=Accounting
        class ThreadEntry(ctypes.Structure):
            _fields_=[('dwSize',wintypes.DWORD),('cntUsage',wintypes.DWORD),
                ('th32ThreadID',wintypes.DWORD),('th32OwnerProcessID',wintypes.DWORD),
                ('tpBasePri',wintypes.LONG),('tpDeltaPri',wintypes.LONG),('dwFlags',wintypes.DWORD)]
        self.ThreadEntry=ThreadEntry
        signatures={
            'CreateJobObjectW':([ctypes.c_void_p,wintypes.LPCWSTR],wintypes.HANDLE),
            'SetInformationJobObject':([wintypes.HANDLE,ctypes.c_int,ctypes.c_void_p,wintypes.DWORD],wintypes.BOOL),
            'AssignProcessToJobObject':([wintypes.HANDLE,wintypes.HANDLE],wintypes.BOOL),
            'QueryInformationJobObject':([wintypes.HANDLE,ctypes.c_int,ctypes.c_void_p,wintypes.DWORD,ctypes.c_void_p],wintypes.BOOL),
            'TerminateJobObject':([wintypes.HANDLE,wintypes.UINT],wintypes.BOOL),
            'CreateEventW':([ctypes.c_void_p,wintypes.BOOL,wintypes.BOOL,wintypes.LPCWSTR],wintypes.HANDLE),
            'SetEvent':([wintypes.HANDLE],wintypes.BOOL),
            'CloseHandle':([wintypes.HANDLE],wintypes.BOOL),
            'GetProcessTimes':([wintypes.HANDLE]+[ctypes.POINTER(wintypes.FILETIME)]*4,wintypes.BOOL),
            'CreateToolhelp32Snapshot':([wintypes.DWORD,wintypes.DWORD],wintypes.HANDLE),
            'Thread32First':([wintypes.HANDLE,ctypes.POINTER(ThreadEntry)],wintypes.BOOL),
            'Thread32Next':([wintypes.HANDLE,ctypes.POINTER(ThreadEntry)],wintypes.BOOL),
            'OpenThread':([wintypes.DWORD,wintypes.BOOL,wintypes.DWORD],wintypes.HANDLE),
            'GetProcessIdOfThread':([wintypes.HANDLE],wintypes.DWORD),
            'GetThreadId':([wintypes.HANDLE],wintypes.DWORD),
            'GetThreadTimes':([wintypes.HANDLE]+[ctypes.POINTER(wintypes.FILETIME)]*4,wintypes.BOOL),
            'ResumeThread':([wintypes.HANDLE],wintypes.DWORD),
        }
        for name,(args,result) in signatures.items():
            fn=getattr(self.kernel,name);fn.argtypes=args;fn.restype=result
        self.handle=self.kernel.CreateJobObjectW(None,None)
        if not self.handle:
            raise ctypes.WinError(ctypes.get_last_error())
        limits=ExtendedLimits()
        limits.Basic.LimitFlags=0x2000  # KILL_ON_JOB_CLOSE; venv launcher children stay in this Job.
        if not self.kernel.SetInformationJobObject(self.handle,9,ctypes.byref(limits),ctypes.sizeof(limits)):
            error=ctypes.get_last_error();self.close();raise ctypes.WinError(error)

    def assign(self, child):
        if not self.kernel.AssignProcessToJobObject(self.handle,wintypes.HANDLE(int(child._handle))):
            raise ctypes.WinError(ctypes.get_last_error())

    def birth(self, child):
        creation,exit_time,kernel,user=(wintypes.FILETIME() for _ in range(4))
        if not self.kernel.GetProcessTimes(wintypes.HANDLE(int(child._handle)),ctypes.byref(creation),
                ctypes.byref(exit_time),ctypes.byref(kernel),ctypes.byref(user)):
            raise ctypes.WinError(ctypes.get_last_error())
        ticks=(creation.dwHighDateTime<<32)|creation.dwLowDateTime
        utc=datetime.datetime.fromtimestamp(ticks/10000000-11644473600,datetime.timezone.utc).isoformat()
        return dict(filetime_100ns=ticks,utc=utc,source='GetProcessTimes_on_owned_process_handle')

    def active(self):
        value=self.Accounting()
        if not self.kernel.QueryInformationJobObject(self.handle,1,ctypes.byref(value),ctypes.sizeof(value),None):
            raise ctypes.WinError(ctypes.get_last_error())
        return value.ActiveProcesses

    def open_initial_thread(self, child):
        """Public APIs only; launcher cannot spawn while CREATE_SUSPENDED."""
        snapshot=self.kernel.CreateToolhelp32Snapshot(0x00000004,0)  # TH32CS_SNAPTHREAD
        if snapshot==ctypes.c_void_p(-1).value:
            raise ctypes.WinError(ctypes.get_last_error())
        tids=[]
        try:
            entry=self.ThreadEntry()
            entry.dwSize=ctypes.sizeof(entry)
            if not self.kernel.Thread32First(snapshot,ctypes.byref(entry)):
                raise ctypes.WinError(ctypes.get_last_error())
            while True:
                if entry.th32OwnerProcessID==child.pid:
                    tids.append(entry.th32ThreadID)
                entry.dwSize=ctypes.sizeof(entry)
                if not self.kernel.Thread32Next(snapshot,ctypes.byref(entry)):
                    error=ctypes.get_last_error()
                    if error!=18:  # ERROR_NO_MORE_FILES
                        raise ctypes.WinError(error)
                    break
        finally:
            self.kernel.CloseHandle(snapshot)
        if len(tids)!=1:
            raise RuntimeError('Expected exactly one suspended initial thread; observed '+repr(tids))
        # THREAD_QUERY_INFORMATION | THREAD_SUSPEND_RESUME; no thread injection.
        thread=self.kernel.OpenThread(0x0042,False,tids[0])
        if not thread:
            raise ctypes.WinError(ctypes.get_last_error())
        try:
            if (self.kernel.GetProcessIdOfThread(thread)!=child.pid
                    or self.kernel.GetThreadId(thread)!=tids[0]):
                raise RuntimeError('Initial thread ownership/identity changed')
            creation,exit_time,kernel,user=(wintypes.FILETIME() for _ in range(4))
            if not self.kernel.GetThreadTimes(thread,ctypes.byref(creation),ctypes.byref(exit_time),
                    ctypes.byref(kernel),ctypes.byref(user)):
                raise ctypes.WinError(ctypes.get_last_error())
            ticks=(creation.dwHighDateTime<<32)|creation.dwLowDateTime
            identity=dict(tid=tids[0],owner_pid=child.pid,creation_filetime_100ns=ticks,
                enumeration='Toolhelp unique owned thread while initial process suspended',
                query='OpenThread/GetProcessIdOfThread/GetThreadId/GetThreadTimes')
            return thread,identity
        except Exception:
            self.kernel.CloseHandle(thread)
            raise

    def terminate(self):
        if not self.kernel.TerminateJobObject(self.handle,124):
            raise ctypes.WinError(ctypes.get_last_error())

    def close(self):
        if self.handle:
            self.kernel.CloseHandle(self.handle);self.handle=None


