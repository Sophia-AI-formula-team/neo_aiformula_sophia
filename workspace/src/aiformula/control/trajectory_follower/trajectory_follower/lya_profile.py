"""The reference settings used by neo's documented lya_0221 controller.

These are controller settings, not measured vehicle safety limits.  In
particular, v_t is a reference: the LYA error-feedback law can request a
different linear velocity.  Lane teach/repeat imports the same reference.
"""

REFERENCE_SPEED_MPS = 2.0
MAX_YAW_RATE_RPS = 2.0
