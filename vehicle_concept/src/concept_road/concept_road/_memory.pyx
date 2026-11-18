# cython: language_level=3, boundscheck=False, wraparound=False, cdivision=True
"""Sample the original saved camera observation in the current body grid."""
from libc.math cimport hypot, nearbyint
import numpy as np


def project(const double[:, ::1] grid_z, const double[:, ::1] grid_x,
            const unsigned char[:, ::1] observed, const unsigned char[:, ::1] visible,
            const unsigned char[:, ::1] line, double pose_z, double pose_x,
            double c, double s, double base_error, double turn_error,
            double cx, double cy, double focal, double height_focal):
    cdef Py_ssize_t rows=grid_z.shape[0], cols=grid_z.shape[1]
    cdef Py_ssize_t h=observed.shape[0], w=observed.shape[1]
    if grid_x.shape[0] != rows or grid_x.shape[1] != cols:
        raise ValueError("Body grid dimensions differ")
    if visible.shape[0] != h or visible.shape[1] != w or (line is not None and (line.shape[0] != h or line.shape[1] != w)):
        raise ValueError("Saved observations must share the camera geometry")
    support=np.zeros((rows,cols),np.uint8)
    negative=np.zeros_like(support)
    paint=np.zeros_like(support)
    error=np.full((rows,cols),np.inf)
    cdef unsigned char[:, ::1] supported=support, clear=negative, marked=paint
    cdef double[:, ::1] allowance=error
    cdef Py_ssize_t i,j,u,v
    cdef double dz,dx,z,x,e
    cdef bint has_paint = line is not None
    for i in range(rows):
        for j in range(cols):
            dz=grid_z[i,j]-pose_z
            dx=grid_x[i,j]-pose_x
            z=c*dz+s*dx
            if z < .95 or z > 4.:
                continue
            x=-s*dz+c*dx
            u=<Py_ssize_t>nearbyint(cx+x*focal/z)
            v=<Py_ssize_t>nearbyint(cy+height_focal/z)
            if u < 0 or u >= w or v < 0 or v >= h:
                continue
            supported[i,j]=observed[v,u]
            e=base_error+turn_error*hypot(z,x)
            allowance[i,j]=e
            if e <= .30:
                clear[i,j]=visible[v,u] and not observed[v,u]
                if has_paint:
                    marked[i,j]=line[v,u]
    return support.view(np.bool_), negative.view(np.bool_), paint.view(np.bool_), error
