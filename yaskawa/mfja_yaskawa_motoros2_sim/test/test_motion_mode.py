from mfja_yaskawa_motoros2_sim import motion_mode as mm


def test_start_one_mode_at_a_time():
    assert mm.start_result(None, mm.TRAJECTORY, False) == mm.READY
    assert mm.start_result(None, mm.POINT_QUEUE, False) == mm.READY
    assert mm.start_result(mm.TRAJECTORY, mm.TRAJECTORY, False) == mm.READY
    assert mm.start_result(mm.POINT_QUEUE, mm.POINT_QUEUE, False) == mm.READY
    assert mm.start_result(mm.TRAJECTORY, mm.POINT_QUEUE, False) == mm.NOT_READY_OTHER_TRAJ_MODE_ACTIVE
    assert mm.start_result(mm.POINT_QUEUE, mm.TRAJECTORY, False) == mm.NOT_READY_OTHER_TRAJ_MODE_ACTIVE


def test_start_with_error():
    assert mm.start_result(None, mm.POINT_QUEUE, True) == mm.NOT_READY_ERROR
    # The other mode is reported first, as MotoROS2 does.
    assert mm.start_result(mm.TRAJECTORY, mm.POINT_QUEUE, True) == mm.NOT_READY_OTHER_TRAJ_MODE_ACTIVE


def test_stop():
    assert not mm.stop_refused(None, False, False)
    assert not mm.stop_refused(mm.TRAJECTORY, False, True)
    assert mm.stop_refused(mm.TRAJECTORY, True, False)
    assert mm.stop_refused(mm.POINT_QUEUE, False, True)
    assert not mm.stop_refused(mm.POINT_QUEUE, False, False)


def test_goal_refusal():
    assert mm.goal_refusal(None) == mm.NOT_READY_SERVO_OFF
    assert mm.goal_refusal(mm.POINT_QUEUE) == mm.INIT_TRAJ_WRONG_MODE
    assert mm.goal_refusal(mm.TRAJECTORY) is None
