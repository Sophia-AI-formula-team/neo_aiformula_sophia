"""Import an authorized local copy and verify its content on every model load."""
import argparse
import hashlib
import os
import tempfile
from contextlib import contextmanager
from zipfile import ZipFile
from pathlib import Path

FILENAME = 'B1.4-road-aligned-research.pth'
SHA256 = '19d6eeed99fee90a46f09761746845afcf330974a8e1aff057055697a02706a6'
SIZE = 15293077


def default_path():
    return Path(os.environ.get('XDG_DATA_HOME', str(Path.home() / '.local/share'))) / 'concept_yolop' / FILENAME


def verify(stream):
    digest = hashlib.sha256()
    for chunk in iter(lambda: stream.read(1024 * 1024), b''):
        digest.update(chunk)
    if digest.hexdigest() != SHA256 or stream.tell() != SIZE:
        raise ValueError('Expected the frozen B1.4 checkpoint, SHA256 ' + SHA256)
    stream.seek(0)


@contextmanager
def _source_stream(path):
    path = Path(path).expanduser()
    if path.suffix.lower() == '.zip':
        # Read only the research checkpoint; never extract archive paths to disk.
        member = 'vision/YOLOP_B1/weights/' + FILENAME
        with ZipFile(path) as archive:
            info = archive.getinfo(member)
            if info.file_size != SIZE:
                raise ValueError('Unexpected checkpoint size in the archive')
            with archive.open(info) as stream:
                yield stream
    else:
        with path.open('rb') as stream:
            yield stream


def import_weights(source, destination):
    destination = Path(destination).expanduser().resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    with _source_stream(source) as incoming:
        with tempfile.NamedTemporaryFile(dir=destination.parent, delete=False) as tmp:
            temp = Path(tmp.name)
        try:
            with temp.open('wb+') as out:
                for chunk in iter(lambda: incoming.read(1024 * 1024), b''):
                    out.write(chunk)
                out.flush()
                os.fsync(out.fileno())
                out.seek(0)
                verify(out)
            os.replace(str(temp), str(destination))
        finally:
            if temp.exists():
                temp.unlink()
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--destination', type=Path, default=default_path())
    args = parser.parse_args()
    print(import_weights(args.source, args.destination))


if __name__ == '__main__':
    main()
