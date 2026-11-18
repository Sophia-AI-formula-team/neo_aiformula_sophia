# cython: language_level=3, boundscheck=False, wraparound=False
from libc.math cimport fabs, INFINITY


def trace_paint(const unsigned char[:, ::1] pixels, int first, int last,
                double cy, double cx, double height_focal, double focal):
    """Associate paint runs as the scan advances up the camera image."""
    cdef int width = pixels.shape[1]
    if not (0 <= last < first < pixels.shape[0]) or width < 8:
        raise ValueError("Invalid marking scan bounds")
    cdef list tracks = []
    cdef list used, track
    cdef int v, u, begin, end, total, count, best, i, n, j, start
    cdef double z, x, last_z, last_x, prediction, gap, best_gap
    cdef double mean_z, mean_x, dz, xx, zz, slope
    for v in range(first, last, -1):
        z = height_focal / (v - cy)
        used = [False] * len(tracks)
        u = 0
        while u < width:
            if not pixels[v, u]:
                u += 1
                continue
            begin = end = u
            count = 1
            total = u
            u += 1
            # A one-pixel hole belongs to the same strip. Its pixel does
            # not contribute to the centroid or the width confidence check.
            while u < width and u - end <= 2:
                if pixels[v, u]:
                    end = u
                    total += u
                    count += 1
                u += 1
            if count <= 1 or begin <= 2 or end >= width - 3 or count*z/focal >= .45:
                continue
            x = (total / <double>count - cx) * z / focal
            best = -1
            best_gap = INFINITY
            for i in range(len(tracks)):
                if used[i]:
                    continue
                track = tracks[i]
                n = len(track)
                last_z, last_x = track[n-1][:2]
                if z-last_z > 4.:
                    continue
                prediction = last_x
                start = max(0, n-8)
                if n-start > 2 and last_z-<double>track[start][0] > .15:
                    # Slope of the least-squares line through the last eight observations.
                    mean_z = mean_x = 0.
                    for j in range(start, n):
                        mean_z += <double>track[j][0]
                        mean_x += <double>track[j][1]
                    mean_z /= n-start
                    mean_x /= n-start
                    xx = zz = 0.
                    for j in range(start, n):
                        dz = <double>track[j][0] - mean_z
                        zz += dz * dz
                        xx += dz * (<double>track[j][1] - mean_x)
                    slope = min(2., max(-2., xx / zz))
                    prediction += slope * (z - last_z)
                gap = fabs(x - prediction)
                if gap < .20 + .10*(z-last_z) + .012*z and gap < best_gap:
                    best = i
                    best_gap = gap
            if best < 0:
                tracks.append([[z, x, 0.]])
                used.append(False)
            else:
                tracks[best].append([z, x, 0.])
                used[best] = True
    return tracks
