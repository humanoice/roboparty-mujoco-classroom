# SPDX-License-Identifier: GPL-3.0
"""Python copy of upstream-patches/common/rpo_sim_shm.hpp.

ctypes uses the same layout rules as the C++ compiler, so both sides see the
same bytes. Keep the two files in sync.
"""
import ctypes
import mmap
import os

SHM_PATH = '/dev/shm/rpo_sim'
MAGIC = 0x52504F53  # "RPOS"
VERSION = 1
MAX_MOTORS = 32

F32xN = ctypes.c_float * MAX_MOTORS


class Shm(ctypes.Structure):
    _fields_ = [
        ('magic', ctypes.c_uint32),
        ('version', ctypes.c_uint32),
        ('heartbeat', ctypes.c_uint64),
        ('num_motors', ctypes.c_uint32),
        ('reserved', ctypes.c_uint32),
        ('cmd_p', F32xN),
        ('cmd_v', F32xN),
        ('cmd_kp', F32xN),
        ('cmd_kd', F32xN),
        ('cmd_tau', F32xN),
        ('enabled', ctypes.c_uint8 * MAX_MOTORS),
        ('q', F32xN),
        ('dq', F32xN),
        ('tau', F32xN),
        ('imu_quat', ctypes.c_float * 4),
        ('imu_gyro', ctypes.c_float * 3),
        ('imu_acc', ctypes.c_float * 3),
    ]


# The uint64 field makes ctypes align the struct to 8 bytes, the same as
# alignas(8) in C++.
SHM_SIZE = ctypes.sizeof(Shm)


def create(num_motors, path=SHM_PATH):
    """Create or reuse the shared memory file, zero it, return (Shm view, mmap).

    Do not truncate an existing file: a running inference_node can still map
    it, and a short file would crash that process with SIGBUS.
    """
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
    try:
        if os.fstat(fd).st_size < SHM_SIZE:
            os.ftruncate(fd, SHM_SIZE)
        buf = mmap.mmap(fd, SHM_SIZE)
    finally:
        os.close(fd)
    shm = Shm.from_buffer(buf)
    shm.magic = 0
    ctypes.memset(ctypes.addressof(shm), 0, SHM_SIZE)
    shm.version = VERSION
    shm.num_motors = num_motors
    shm.imu_quat[0] = 1.0
    shm.magic = MAGIC  # write last: drivers connect only when magic is set
    return shm, buf
