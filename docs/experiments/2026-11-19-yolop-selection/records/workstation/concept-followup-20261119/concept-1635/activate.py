import hashlib
import os
from pathlib import Path
from shutil import copy2
from tempfile import NamedTemporaryFile
home = Path.home()
root = home/'aiformula_runtime/concept-field-1635'
script = home/'concept_v6275_ws/activate_concept.bash'
before = script.read_bytes()
expected = 'b0ff8367a0c56a48f264ae89818620e4b92d5148aef99f99fe25ebaeb3d71f15'
line = b'source "$HOME/aiformula_runtime/concept-field-1635/install/local_setup.bash" || return\n'
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
