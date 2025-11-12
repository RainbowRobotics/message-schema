from psutil import (
    net_io_counters,
)


def test_utils():
    net_io_counters()
    assert 1 == 1
