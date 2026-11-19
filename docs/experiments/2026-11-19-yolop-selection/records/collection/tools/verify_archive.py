from collections import Counter
from datetime import datetime
import hashlib
import re
import sqlite3
import sys
from pathlib import Path

root = Path(sys.argv[1])
vehicle = root / 'vehicle'
record = root / 'archive-record'
stamp = datetime.now().astimezone().isoformat(timespec='seconds')
manifests = ('vehicle-files', 'bag-files', 'orinssd-files', 'media-files', 'final-supplement')
expected = set()
for tag in manifests:
    manifest = record / f'{tag}.nul'
    if manifest.is_file():
        expected.update(entry.decode().lstrip('/') for entry in manifest.read_bytes().split(b'\0') if entry)
absent = sorted(name for name in expected if not (vehicle / name).is_file())
scanned_sizes = {}
for source in ('vehicle-media-sizes.tsv', 'orinssd-todays-files.tsv'):
    for row in (record / source).read_text().splitlines():
        size, path = row.split('\t', 1)
        scanned_sizes[path.lstrip('/')] = int(size)
mismatch = [
    (path, size, (vehicle/path).stat().st_size)
    for path, size in sorted(scanned_sizes.items())
    if path in expected and (vehicle/path).is_file() and (vehicle/path).stat().st_size != size
]
bags = sorted(
    path for path in scanned_sizes
    if path.endswith('.db3') and path.startswith((
        'home/control/bags/', 'home/control/rosbags/', 'media/control/OrinSSD2000/'
    ))
)
results = []
for name in bags:
    path = vehicle / name
    if not path.is_file() or path.stat().st_size != scanned_sizes[name]:
        results.append((name, scanned_sizes[name], '', 'pending', ''))
        continue
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b''):
            digest.update(chunk)
    try:
        with sqlite3.connect(path.as_uri() + '?mode=ro&immutable=1', uri=True) as db:
            quick = db.execute('PRAGMA quick_check').fetchone()[0]
            count = db.execute('SELECT COUNT(*) FROM messages').fetchone()[0]
        meta = path.with_name('metadata.yaml')
        metadata_match = re.search(r'^  message_count:\s*(\d+)', meta.read_text(), re.M) if meta.is_file() else None
        status = 'ok' if quick == 'ok' and metadata_match and int(metadata_match[1]) == count else 'verify metadata'
        results.append((name, scanned_sizes[name], count, status, digest.hexdigest()))
    except sqlite3.Error as exc:
        results.append((name, scanned_sizes[name], '', f'sqlite error {exc}', digest.hexdigest()))
with (record / 'bag-verification.tsv').open('w') as dest:
    dest.write('vehicle_path\tbytes\tmessages\tstatus\tsha256\n')
    for row in results:
        dest.write('\t'.join(map(str, row)) + '\n')
(record / 'missing-files.txt').write_text(''.join(name+'\n' for name in absent))
(record / 'size-mismatches.tsv').write_text(''.join('\t'.join(map(str,row))+'\n' for row in mismatch))
media_count = sum(Path(path).suffix.lower() in {'.png','.jpg','.jpeg','.webp','.bmp','.tiff','.heic'} for path in scanned_sizes if (vehicle/path).is_file())
bag_status = Counter(row[3] for row in results)
empty_bags = sum(row[2] == 0 for row in results)
print(f'{stamp}  Files {len(expected)-len(absent)}/{len(expected)}, size mismatches {len(mismatch)}, bags {dict(bag_status)}, zero-message bags {empty_bags}, images {media_count}')
for row in results:
    if row[3] != 'ok':
        print(row[3], row[0])
