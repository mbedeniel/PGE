from types import SimpleNamespace as P

from mfja_yaskawa_motoros2_sim import trajectory_checks as tc

JOINTS = [f'joint_{i}' for i in range(1, 7)]


def point(positions, velocities=None, t=0.0):
    return P(positions=positions, velocities=[0.0] * 6 if velocities is None else velocities,
             time_from_start=t)


def test_valid_trajectory():
    pts = [point([0.0] * 6, t=0.0), point([0.5, 0.3, 0.4, 0.0, -0.5, 0.0], t=4.0)]
    assert tc.check_trajectory(JOINTS, pts, JOINTS, dict.fromkeys(JOINTS, 0.0))[0] == tc.INIT_TRAJ_OK


def test_rules():
    zero = dict.fromkeys(JOINTS, 0.0)
    assert tc.check_trajectory(JOINTS, [point([0.0] * 6)], JOINTS)[0] == tc.INIT_TRAJ_TOO_SMALL
    many = [point([0.0] * 6, t=i * 0.1) for i in range(201)]
    assert tc.check_trajectory(JOINTS, many, JOINTS)[0] == tc.INIT_TRAJ_TOO_BIG
    bad_names = JOINTS[:5] + ['joint_1_s']
    pts = [point([0.0] * 6, t=0.0), point([0.0] * 6, t=1.0)]
    assert tc.check_trajectory(bad_names, pts, JOINTS)[0] == tc.INIT_TRAJ_INVALID_JOINTNAME
    no_vel = [point([0.0] * 6, [], 0.0), point([0.0] * 6, [], 1.0)]
    assert tc.check_trajectory(JOINTS, no_vel, JOINTS)[0] == tc.INIT_TRAJ_WRONG_NUMBER_OF_VELOCITIES
    back = [point([0.0] * 6, t=1.0), point([0.0] * 6, t=1.0)]
    assert tc.check_trajectory(JOINTS, back, JOINTS)[0] == tc.INIT_TRAJ_BACKWARD_TIME
    moving_end = [point([0.0] * 6, t=0.0), point([0.0] * 6, [0.1] + [0.0] * 5, 1.0)]
    assert tc.check_trajectory(JOINTS, moving_end, JOINTS)[0] == tc.INIT_TRAJ_INVALID_ENDING_VELOCITY
    far_start = [point([0.2] + [0.0] * 5, t=0.0), point([0.0] * 6, t=1.0)]
    assert tc.check_trajectory(JOINTS, far_start, JOINTS, zero)[0] == tc.INIT_TRAJ_INVALID_STARTING_POS


def test_codes_and_io():
    assert tc.motoros2_error_code(tc.INIT_TRAJ_TOO_BIG) == -100202
    assert tc.motoros2_error_code(101) == -100101
    assert tc.universal_output_address(1) == 10010
    assert tc.universal_output_address(9) == 10020
    assert [tc.universal_output_address(n) for n in (100, 101, 102)] == [10133, 10134, 10135]
    assert tc.io_address_writable(10134) and not tc.io_address_writable(10018)
    assert not tc.io_address_writable(20010) and tc.io_address_writable(27010)
