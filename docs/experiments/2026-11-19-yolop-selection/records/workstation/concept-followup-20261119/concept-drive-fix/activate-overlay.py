import hashlib
import os
from pathlib import Path
from shutil import copy2
from tempfile import NamedTemporaryFile
home = Path.home()
root = home/'aiformula_runtime/concept-drive-fix-20261119-1603'
script = home/'concept_v6275_ws/activate_concept.bash'
before = script.read_bytes()
expected = '10d86a6e9cdbb8b7566284d96fd6e28e17d8cd0d7a853088b4d1f6ebdf122139'
line = b'source "$HOME/aiformula_runtime/concept-drive-fix-20261119-1603/install/local_setup.bash" || return\n'
if line in before:
    print("Overlay already active")
else:
    assert (root/'install/local_setup.bash').is_file()
    assert hashlib.sha256(before).hexdigest() == expected, 'Existing activation changed; inspect before switching'
    copy2(script,root/'activate_concept.before.bash')
    with NamedTemporaryFile(dir=script.parent,delete=False) as tmp:
        tmp.write(before.rstrip()+b'\n'+line)
        tmp.flush()
        os.fsync(tmp.fileno())
        os.fchmod(tmp.fileno(),script.stat().st_mode & 0o777)
    os.replace(tmp.name,script)
    print('Activated Concept desktop overlay', root)
