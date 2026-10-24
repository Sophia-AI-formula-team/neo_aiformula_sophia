"""Explicit Windows simulation checkpoint adapter; production sources unchanged.

Unity simulation time stays visible as use_sim_time=True. This opt-in adapter
uses isolated case paths and production checkpoint validation/restore methods.
Windows provides file fsync + atomic replace, not the POSIX directory-fsync
durability guarantee. It is not native ROS checkpoint validation.
"""
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType


def install_windows_fcntl_transport():
    if os.name != 'nt':
        return
    import msvcrt
    shim = ModuleType('fcntl')
    shim.LOCK_EX, shim.LOCK_NB = 2, 4

    def flock(handle, operation):
        if operation != (shim.LOCK_EX | shim.LOCK_NB):
            raise OSError('simulation adapter supports only exclusive nonblocking lock')
        handle.seek(0)
        # The production store opens the sidecar in text append mode. Ensure
        # one byte exists, then lock exactly that byte; close releases the lock.
        if os.fstat(handle.fileno()).st_size == 0:
            handle.write('0')
            handle.flush()
        handle.seek(0)
        msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)

    shim.flock = flock
    sys.modules['fcntl'] = shim


def validate_case_path(root, filename):
    root, filename = Path(root), Path(filename)
    if not root.is_absolute() or not filename.is_absolute():
        raise ValueError('simulation checkpoint paths must be absolute')
    for candidate in (root, filename):
        for parent in (candidate, *candidate.parents):
            if parent.is_symlink():
                raise ValueError('simulation checkpoint path must not cross symlinks')
        if any(part.lower() == '.ros' for part in candidate.parts):
            raise ValueError('vehicle .ros checkpoint paths are forbidden')
    root, filename = root.resolve(), filename.resolve()
    relative = filename.relative_to(root)
    if len(relative.parts) != 2 or relative.name not in ('topo.json', 'checkpoint.json'):
        raise ValueError('checkpoint must be root/case/topo.json or checkpoint.json')
    return root, filename


def build_simulation_map_class(map_class, checkpoint_root, checkpoint_path):
    from semantic_local_planner_ros import topo_checkpoint as store_module
    from semantic_local_planner_ros import topo_state

    root, path = validate_case_path(checkpoint_root, checkpoint_path)

    class SimulationStore(store_module.CheckpointStore):
        def save(self, state):
            if os.name != 'nt':
                return super().save(state)
            store_module.validate_state(state, self.max_samples)
            raw = json.dumps(dict(schema_version=store_module.SCHEMA_VERSION,
                                  fingerprint=self.fingerprint, state=state,
                                  sha256=store_module.digest(state)), allow_nan=False,
                             separators=(',', ':')).encode()
            if len(raw) > store_module.MAX_BYTES:
                raise ValueError('checkpoint_too_large')
            temporary = None
            try:
                with tempfile.NamedTemporaryFile(dir=str(self.path.parent),
                        prefix=self.path.name + '.', suffix='.tmp', delete=False) as stream:
                    temporary = stream.name
                    stream.write(raw)
                    stream.flush()
                    os.fsync(stream.fileno())
                os.replace(temporary, str(self.path))
                temporary = None
            finally:
                if temporary is not None:
                    os.unlink(temporary)

    class SimulationMapAssist(map_class):
        def _init_checkpoint(self):
            # This is the one explicitly different lifecycle entry. Do not
            # temporarily lie about use_sim_time to invoke production's guard.
            self.checkpoint = None
            self.checkpoint_worker = None
            self.checkpoint_future = None
            self.restore_pending = None
            self.restore_blocked = False
            self.checkpoint_dirty = False
            self.last_checkpoint_time = -math.inf
            self.last_topo_log_time = -math.inf
            self.persistence_status = dict(state='disabled', path=str(path), error=None,
                                           saved_samples=0, saved_distance_m=0.0)
            if self.get_parameter('use_sim_time').value is not True:
                raise ValueError('simulation checkpoint requires honest use_sim_time=true')
            if Path(self.p['checkpoint_path']).resolve() != path:
                raise ValueError('checkpoint path differs from explicit isolated case')
            try:
                fingerprint = store_module.digest(dict(
                    map_sha256=hashlib.sha256(Path(self.p['topomap_path']).read_bytes()).hexdigest(),
                    parameters={key: self.p[key] for key in topo_state.MAP_PARAMETERS},
                    reconstruction_version=1))
                self.checkpoint = SimulationStore(str(path), fingerprint, self.p['max_recorded_samples'])
                self.checkpoint_worker = ThreadPoolExecutor(max_workers=1)
                state = self.checkpoint.load()
                if state and state['samples']:
                    self.restore_pending = state
                    self.persistence_status.update(state='waiting_for_odom',
                        saved_samples=len(state['samples']), saved_distance_m=state['recorded_distance'])
                else:
                    self.persistence_status.update(state='new_map')
            except (OSError, ValueError, TypeError, KeyError) as error:
                self.restore_blocked = True
                self.persistence_status.update(state='restore_rejected', error=str(error))
                self.get_logger().warn('Simulation topo checkpoint: ' + str(error))

    SimulationMapAssist.simulation_checkpoint_scope = dict(
        adapter='explicit_simulation_checkpoint_v1', root=str(root), path=str(path),
        use_sim_time=True, production_sim_guard_unchanged=True,
        storage=('windows_byte_lock_file_fsync_atomic_replace_no_directory_fsync'
                 if os.name == 'nt' else 'production_posix_store'))
    return SimulationMapAssist
