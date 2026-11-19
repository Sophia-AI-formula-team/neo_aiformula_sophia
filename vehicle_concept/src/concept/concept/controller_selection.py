"""Resolve the low-level tracking controller once, before starting ROS nodes."""
import sys


MODES = ('pure_pursuit', 'lya')


def select_controller(requested, stdin=None, stdout=None):
    if requested in MODES:
        return requested
    if requested != 'auto':
        raise ValueError('tracking_controller must be pure_pursuit, lya or auto')
    stdin = sys.stdin if stdin is None else stdin
    stdout = sys.stdout if stdout is None else stdout
    if not stdin.isatty():
        return 'pure_pursuit'

    print('Concept tracking controller\n  1  Pure Pursuit (default)\n  2  LYA', file=stdout)
    choices = {'': 'pure_pursuit', '1': 'pure_pursuit', '2': 'lya',
               'pure_pursuit': 'pure_pursuit', 'pure pursuit': 'pure_pursuit', 'lya': 'lya'}
    while True:
        print('Select 1 or 2 [1]: ', file=stdout, end='', flush=True)
        reply = stdin.readline()
        if not reply:
            raise RuntimeError('Terminal closed before a tracking controller was selected')
        selected = choices.get(reply.strip().lower())
        if selected is not None:
            return selected
        print('Enter 1 for Pure Pursuit or 2 for LYA.', file=stdout)
