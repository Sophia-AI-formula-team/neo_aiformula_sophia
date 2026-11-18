"""Runtime source extracted from the accepted V6.26 freeze: experiments/v615_recovery/silhouette615.py. """

import cv2

import numpy as np

def complete_box(box, red, image=None):
    x0, y0, x1, y1 = box
    w, h = x1-x0, y1-y0
    a, b = max(0, int(x0-w)), min(red.shape[1], int(np.ceil(x1+w)))
    c, d = max(0, int(y0-2*h)), min(red.shape[0], int(np.ceil(y1+h)))
    if b <= a or d <= c:
        return None
    roi = red[c:d, a:b]
    # Join white stripes vertically; a narrow horizontal kernel keeps nearby
    # cones with distinct visible bodies separate.
    joined = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, np.ones((max(3, int(w*.6)), 3), np.uint8))
    count, labels, stats, centers = cv2.connectedComponentsWithStats(joined)
    yy0, yy1 = max(0, int(y0)-c), min(d-c, int(y1)+1-c)
    xx0, xx1 = max(0, int(x0)-a), min(b-a, int(x1)+1-a)
    colored = roi[yy0:yy1, xx0:xx1] > 0
    ids, counts = np.unique(labels[yy0:yy1, xx0:xx1][colored], return_counts=True)
    choices = [(int(n), int(i)) for i, n in zip(ids, counts) if i > 0 and n >= 3]
    if not choices:
        return None
    component = max(choices)[1]
    yy, xx = np.nonzero((labels == component) & (roi > 0))
    left, right = a+xx.min(), a+xx.max()
    top, bottom = c+yy.min(), c+yy.max()
    # Ground foot includes the narrow black base beneath the coloured body.
    # Account for its pixel allowance in the existing measurement covariance.
    base = max(1., .05*(bottom-top+1))
    foot = min(red.shape[0], bottom+base)
    foot_source = 'coloured_shell_with_partial_base'
    if image is not None:
        middle = (left+right)*.5
        half = max(2, int((right-left)*.3))
        xa, xb = max(0, int(middle-half)), min(red.shape[1], int(middle+half+1))
        ymax = min(red.shape[0]-1, int(bottom+max(3, .2*(bottom-top+1))))
        dark = np.max(image[bottom:ymax+1, xa:xb], axis=2) < 65
        rows = np.flatnonzero(dark.mean(axis=1) > .4)
        if len(rows) and rows[0] <= max(2, .05*(bottom-top+1)):
            contiguous = np.split(rows, np.flatnonzero(np.diff(rows)>1)+1)[0]
            foot = bottom+int(contiguous[-1])
            foot_source = 'observed_black_base'
    return np.array([max(0., left-1.), float(top), min(red.shape[1], right+1.), float(foot)]), dict(
        colored_pixels=int(len(yy)), support_box=[int(left), int(top), int(right), int(bottom)], base_allowance_pixels=float(base), foot_source=foot_source)
