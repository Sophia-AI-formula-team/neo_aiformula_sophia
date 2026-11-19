import time, collections, base64, zlib, struct
from pathlib import Path
import numpy as np
import cv2
from concept_yolop import frame_codec as codec
root=Path.home()/'aiformula_runtime/concept-field-20261119/deployed-src/vehicle_concept/src/concept_drive/test/data/moving-lap-start'
a=cv2.imread(str(root/'00000_rgb.png'))
cv2.setNumThreads(1)
head=struct.pack('>IIBBBBB',640,360,8,2,0,0,0)
def chunk(name,data):
    return struct.pack('>I',len(data))+name+data+struct.pack('>I',zlib.crc32(data,zlib.crc32(name)))
start=b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',head)
end=chunk(b'IEND',b'')
def fast(a):
    scan=np.empty((360,1921),np.uint8)
    scan[:,0]=0
    scan[:,1:]=cv2.cvtColor(a,cv2.COLOR_BGR2RGB).reshape(360,-1)
    return base64.b64encode(start+chunk(b'IDAT',zlib.compress(memoryview(scan),0))+end).decode('ascii')
for fn in [fast,codec._png]:
    elapsed=[]
    for _ in range(18):
        t=time.perf_counter(); b=fn(a); t2=time.perf_counter()
        out=codec._image(b,3); t3=time.perf_counter()
        np.testing.assert_array_equal(a,out)
        elapsed.append([(t2-t)*1e3,(t3-t2)*1e3])
    print(fn.__name__, np.median(elapsed,axis=0),len(b),flush=True)
