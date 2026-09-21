"""无串口、无Tk窗口的独立调试接口回归测试。"""

import queue
import threading
import unittest
from unittest.mock import Mock, patch

import modbus485_monitor as monitor


def make_app():
    # 不初始化Tk，直接验证通信线程和协议逻辑。
    app = monitor.ModbusMonitorApp.__new__(monitor.ModbusMonitorApp)
    app.client = Mock()
    app.events = queue.Queue()
    app.stop_event = threading.Event()
    app.last_snapshot = monitor.empty_snapshot()
    app.last_snapshot.firmware_regs = [3, 1, 2]
    app.last_snapshot.device_info_regs = [0x0202, 3, monitor.MODBUS_CAP_DEBUG_ACQ]
    app.debug_session_active = False
    app.debug_stop_pending = False
    return app


class ValidityTests(unittest.TestCase):
    def valid(self, status, imu=0xFFFF, joint=3, touch=3):
        return monitor.evaluate_sensor_validity(status, imu, joint, touch, 4)[0]

    def test_normal_requires_pps(self):
        self.assertFalse(self.valid(monitor.SENSOR_SNAPSHOT_STATUS_VALID))

    def test_debug_data_can_be_valid_without_pps_or_utc(self):
        status = monitor.SENSOR_SNAPSHOT_STATUS_VALID | monitor.SENSOR_SNAPSHOT_STATUS_DEBUG_MODE
        self.assertTrue(self.valid(status))

    def test_normal_data_can_be_valid_without_utc(self):
        status = monitor.SENSOR_SNAPSHOT_STATUS_VALID | monitor.SENSOR_SNAPSHOT_STATUS_PPS_PRESENT
        self.assertTrue(self.valid(status))

    def test_debug_still_requires_fresh_complete_sensor_data(self):
        status = monitor.SENSOR_SNAPSHOT_STATUS_VALID | monitor.SENSOR_SNAPSHOT_STATUS_DEBUG_MODE
        for imu, joint, touch in [(0xFFFE, 3, 3), (0xFFFF, 1, 3), (0xFFFF, 3, 1)]:
            with self.subTest(imu=imu, joint=joint, touch=touch):
                self.assertFalse(self.valid(status, imu, joint, touch))
        self.assertFalse(self.valid(monitor.SENSOR_SNAPSHOT_STATUS_DEBUG_MODE))


class CommandTests(unittest.TestCase):
    def test_start_uses_magic_and_next_ack_sequence(self):
        app = make_app()
        app.client.read_holding_registers.side_effect = [[1, 123, 0], [1, 124, 0]]
        app._debug_command(1, 0.2, monitor.CMD_DEBUG_ACQ_START)
        app.client.write_multiple_registers.assert_called_once_with(
            1, monitor.REG_CMD_START, [monitor.CMD_DEBUG_ACQ_START, monitor.CMD_DEBUG_ACQ_MAGIC, 124], 0.2
        )

    def test_stop_parameter_is_zero_and_sequence_wraps(self):
        app = make_app()
        app.client.read_holding_registers.side_effect = [[1, 0xFFFF, 0], [1, 0, 0]]
        app._debug_command(1, 0.2, monitor.CMD_DEBUG_ACQ_STOP)
        app.client.write_multiple_registers.assert_called_once_with(
            1, monitor.REG_CMD_START, [monitor.CMD_DEBUG_ACQ_STOP, 0, 0], 0.2
        )

    def test_renew_uses_magic(self):
        app = make_app()
        app.client.read_holding_registers.side_effect = [[1, 4, 0], [1, 5, 0]]
        app._debug_command(1, 0.2, monitor.CMD_DEBUG_ACQ_RENEW)
        self.assertEqual(app.client.write_multiple_registers.call_args.args[2],
                         [monitor.CMD_DEBUG_ACQ_RENEW, monitor.CMD_DEBUG_ACQ_MAGIC, 5])

    def test_rejected_or_mismatched_ack_is_not_success(self):
        for response in [[2, 5, 0], [1, 4, 0], [1, 5, 1]]:
            with self.subTest(response=response):
                app = make_app()
                app.client.read_holding_registers.side_effect = [[1, 4, 0], response]
                with self.assertRaises(monitor.ModbusError):
                    app._debug_command(1, 0.2, monitor.CMD_DEBUG_ACQ_START)

    def test_stop_confirmation_ignores_already_set_stop_event(self):
        app = make_app()
        app.stop_event.set()
        app.client.read_holding_registers.side_effect = [[2] + [0] * 9, [0] * 10]
        with patch.object(monitor.time, "sleep"):
            self.assertEqual(app._wait_debug_mode(1, 0.2, 0)[0], 0)

    def test_pending_start_can_be_cancelled(self):
        app = make_app()
        app.stop_event.set()
        app.client.read_holding_registers.return_value = [2] + [0] * 9
        with self.assertRaises(monitor.ModbusError):
            app._wait_debug_mode(1, 0.2, 1)


class SessionTests(unittest.TestCase):
    def test_stop_during_normal_poll_is_deferred_not_sent_concurrently(self):
        app = make_app()
        app.worker = Mock()
        app.worker.is_alive.return_value = True
        app.debug_status_var = Mock()
        app.stop_debug_acquisition()
        self.assertTrue(app.stop_event.is_set())
        self.assertTrue(app.debug_stop_pending)
        app.client.write_multiple_registers.assert_not_called()

    def test_deferred_stop_runs_after_worker_exits(self):
        app = make_app()
        app.worker = Mock()
        app.worker.is_alive.return_value = False
        app.debug_stop_pending = True
        app.stop_debug_acquisition = Mock()
        app.after = Mock()
        app._process_events()
        app.stop_debug_acquisition.assert_called_once()
        self.assertFalse(app.debug_stop_pending)

    def test_no_start_or_stop_on_unsupported_firmware(self):
        app = make_app()
        app.client.read_holding_registers.return_value = [0x0201, 3, 0x000F]
        app._debug_command = Mock()
        app._debug_acquisition_worker(1, 0.2)
        app._debug_command.assert_not_called()
        self.assertFalse(app.debug_session_active)

    def test_uncertain_start_always_attempts_stop(self):
        app = make_app()
        app.client.read_holding_registers.return_value = [0x0202, 3, monitor.MODBUS_CAP_DEBUG_ACQ]
        app._debug_command = Mock(side_effect=[monitor.ModbusError("ACK timeout"), None])
        app._wait_debug_mode = Mock()
        with patch.object(monitor, "read_snapshot", return_value=app.last_snapshot):
            app._debug_acquisition_worker(1, 0.2)
        self.assertEqual([call.args[2] for call in app._debug_command.call_args_list],
                         [monitor.CMD_DEBUG_ACQ_START, monitor.CMD_DEBUG_ACQ_STOP])
        self.assertFalse(app.debug_session_active)

    def test_normal_debug_exit_stops_and_confirms_normal_mode(self):
        app = make_app()
        app.client.read_holding_registers.return_value = [0x0202, 3, monitor.MODBUS_CAP_DEBUG_ACQ]
        app._debug_command = Mock()
        app._wait_debug_mode = Mock()
        app._sensor_poll_worker = Mock()
        with patch.object(monitor, "read_snapshot", return_value=app.last_snapshot):
            app._debug_acquisition_worker(1, 0.2)
        app._sensor_poll_worker.assert_called_once_with(1, 0.2, debug_session=True)
        self.assertEqual([call.args[2] for call in app._wait_debug_mode.call_args_list], [1, 0])
        self.assertEqual(app._debug_command.call_args_list[-1].args[2], monitor.CMD_DEBUG_ACQ_STOP)
        self.assertFalse(app.debug_session_active)

    def run_poll(self, app, *, debug, clock):
        with patch.object(monitor.time, "monotonic", side_effect=clock), \
                patch.object(monitor, "enter_sensor_poll_scheduling", return_value=None), \
                patch.object(monitor, "leave_sensor_poll_scheduling") as leave, \
                patch.object(monitor, "wait_sensor_poll_deadline"), \
                patch.object(monitor, "read_sensor_snapshot_poll") as read:
            read.side_effect = lambda *args: (app.stop_event.set() or app.last_snapshot)
            app._sensor_poll_worker(1, 0.2, debug_session=debug)
            leave.assert_called_once()
        app.client.finish_low_latency_poll.assert_called_once()
        return read

    def test_normal_poll_has_no_debug_renewal(self):
        app = make_app()
        app._debug_command = Mock()
        self.run_poll(app, debug=False, clock=[0.0])
        app._debug_command.assert_not_called()

    def test_failed_renewal_exits_without_busy_retry_or_fc41(self):
        app = make_app()
        app._debug_command = Mock(side_effect=monitor.ModbusError("lease denied"))
        read = self.run_poll(app, debug=True, clock=[0.0, 4.0])
        app._debug_command.assert_called_once_with(1, 0.2, monitor.CMD_DEBUG_ACQ_RENEW)
        read.assert_not_called()

    def test_lost_debug_mode_exits_poll_and_reports_error(self):
        app = make_app()
        app.last_snapshot.sensor_snapshot_status = 0
        app._debug_command = Mock()
        self.run_poll(app, debug=True, clock=[0.0, 0.1])
        app._debug_command.assert_not_called()
        events = list(app.events.queue)
        self.assertTrue(any(kind == "error" and "mode ended" in str(value)
                            for kind, value in events))


if __name__ == "__main__":
    unittest.main()
