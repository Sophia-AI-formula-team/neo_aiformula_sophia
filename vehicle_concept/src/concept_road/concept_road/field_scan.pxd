# Types for the same scanline rules used by pure-Python and compiled installations.
import cython

@cython.locals(width=int, last=int, v=int, end=int, lo=int, hi=int,
               total=int, i=int, low_light=double, high_light=double)
cdef int _asphalt_end(int first, int direction, int step,
                      const unsigned char[::1] paint, const unsigned char[::1] brightness,
                      const double[::1] green_sum, const double[::1] blue_sum,
                      double inside_green, double inside_blue, double base_light) except -2


@cython.locals(count=int,end=int,n=int,total=int,j=int)
cdef bint _pavement_strip(int lo, int hi, int step, const unsigned char[::1] paint,
                         const unsigned char[::1] brightness, const double[::1] green_sum,
                         const double[::1] blue_sum, double base_g, double base_b, double light) except -1
