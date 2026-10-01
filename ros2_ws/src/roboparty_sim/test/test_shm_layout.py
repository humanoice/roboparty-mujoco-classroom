from roboparty_sim import shm_layout


def test_layout_matches_cpp_static_asserts():
    assert shm_layout.SHM_SIZE == 1120
    assert shm_layout.Shm.enabled.offset == 664
    assert shm_layout.Shm.q.offset == 696
    assert shm_layout.Shm.imu_quat.offset == 1080


def test_create_sets_header(tmp_path):
    path = str(tmp_path / 'rpo_sim')
    shm, _buf = shm_layout.create(23, path)
    assert shm.magic == shm_layout.MAGIC
    assert shm.version == shm_layout.VERSION
    assert shm.num_motors == 23
    assert list(shm.imu_quat) == [1.0, 0.0, 0.0, 0.0]
