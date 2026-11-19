import io

import pytest

from concept.controller_selection import select_controller


class Terminal(io.StringIO):
    def isatty(self):
        return True


@pytest.mark.parametrize('reply, expected', [
    ('\n', 'pure_pursuit'), ('1\n', 'pure_pursuit'), ('LYA\n', 'lya'),
    ('wrong\n2\n', 'lya'),
])
def test_terminal_menu(reply, expected):
    output = io.StringIO()
    assert select_controller('auto', Terminal(reply), output) == expected
    assert 'Pure Pursuit' in output.getvalue()


def test_unattended_and_explicit_selection_do_not_read_input():
    pipe = io.StringIO('2\n')
    assert select_controller('auto', pipe) == 'pure_pursuit'
    assert select_controller('lya', pipe) == 'lya'
    assert pipe.tell() == 0


def test_invalid_argument_and_closed_terminal_abort():
    with pytest.raises(ValueError):
        select_controller('lva')
    with pytest.raises(RuntimeError):
        select_controller('auto', Terminal(''), io.StringIO())
