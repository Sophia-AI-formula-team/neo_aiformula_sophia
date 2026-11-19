import gzip
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile

source, supplement, destination = (Path(v) for v in sys.argv[1:4])
with tarfile.open(source, 'r|gz') as archive:
    for item in archive:
        last = item
    end = last.offset_data + ((last.size + 511) // 512) * 512
with destination.open('wb') as file, gzip.open(source, 'rb') as original:
    compressor = subprocess.Popen(['gzip', '-n', '-1'], stdin=subprocess.PIPE, stdout=file)
    try:
        while end:
            block = original.read(min(end, 1024 * 1024))
            if not block:
                raise EOFError(source)
            compressor.stdin.write(block)
            end -= len(block)
        with supplement.open('rb') as extra:
            shutil.copyfileobj(extra, compressor.stdin, 1024 * 1024)
    finally:
        compressor.stdin.close()
        if compressor.wait() != 0:
            raise RuntimeError('gzip failed')
