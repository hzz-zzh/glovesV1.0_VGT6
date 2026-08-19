#!/usr/bin/env python3
"""Small Modbus RTU monitor for the glove RS485 link."""

from __future__ import annotations

import csv
import datetime as dt
import math
import queue
import struct
import threading
import time
import tkinter as tk
from dataclasses import dataclass
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # pragma: no cover - shown in GUI at runtime
    serial = None
    list_ports = None


DEFAULT_BAUD = 3000000
DEFAULT_SLAVE = 1
DEFAULT_TIMEOUT_S = 0.20
DEFAULT_POLL_MS = 500

MAX_READ_REGS = 100
MAX_WRITE_REGS = 100
SENSOR_120HZ_PERIOD_S = 1.0 / 120.0
SENSOR_120HZ_TIMEOUT_S = 0.008
SENSOR_120HZ_READ_SLICE_S = 0.002
SENSOR_120HZ_RETRIES = 1
SENSOR_120HZ_RETRY_GAP_S = 0.001
SENSOR_120HZ_SPIN_GUARD_S = 0.004
MODBUS_INTER_REQUEST_GAP_S = 0.0001

REG_BASIC_STATUS_START = 0x0000
REG_BASIC_STATUS_COUNT = 14
REG_UTC_TIMESTAMP_US = 0x0002
REG_LOCAL_UPTIME_US = 0x0006
REG_TIME_SYNC_UTC_US = 0x000A
MODBUS_ROS_TIME_REG_COUNT = 4
REG_CMD_START = 0x0020
REG_CMD_ACK_START = 0x0023
REG_CMD_ACK_COUNT = 3
CMD_HEALTH_CLEAR_HISTORY = 0x004A
CMD_HEALTH_CLEAR_MAGIC = 0xC1EA
CMD_ACK_OK = 0x0001
REG_SYSTEM_STATUS_START = 0x0040
REG_SYSTEM_STATUS_COUNT = 10
REG_HEALTH_STATUS_START = 0x004A
REG_HEALTH_STATUS_COUNT = 22
REG_POWER_STATUS_START = 0x0060
REG_POWER_STATUS_COUNT = 18
REG_WORK_STATE = 0x0500

REG_IMU_DATA_START = 0x1000
REG_IMU_TIMESTAMP_US = 0x1140
REG_IMU_STATUS_BITS = 0x1144
REG_IMU_CALIB_C_START = 0x1154
REG_IMU_CALIB_M_START = 0x11D4
REG_IMU_CALIB_CTRL_START = 0x1254
REG_IMU_CALIB_MAGIC = 0x1254
REG_IMU_CALIB_COMMAND = 0x1255
REG_IMU_CALIB_SEQ = 0x1256
REG_IMU_CALIB_STATUS = 0x1257
REG_IMU_CALIB_ERROR_INDEX = 0x1258
REG_IMU_CALIB_LAST_APPLIED_SEQ = 0x1259
REG_IMU_CALIB_CTRL_COUNT = 6
MODBUS_IMU_COUNT = 16
MODBUS_IMU_FLOATS_PER_UNIT = 10
MODBUS_IMU_REGS_PER_UNIT = 20
MODBUS_IMU_DATA_REG_COUNT = 320
MODBUS_IMU_CALIB_REGS_PER_UNIT = 8
MODBUS_IMU_CALIB_TABLE_REG_COUNT = MODBUS_IMU_COUNT * MODBUS_IMU_CALIB_REGS_PER_UNIT
IMU_CALIB_MAGIC_VALUE = 0xCA1B
IMU_CALIB_CMD_APPLY = 1
IMU_CALIB_CMD_RESET_IDENTITY = 2

REG_JOINT_DATA_START = 0x1300
REG_JOINT_TIMESTAMP_US = 0x1340
REG_JOINT_STATUS_FLAGS = 0x1344
JOINT_STATUS_SNAPSHOT_VALID = 0x0001
JOINT_STATUS_ALGORITHM_VALID = 0x0002
JOINT_STATUS_IMU_CALIB_APPLIED = 0x0004
MODBUS_JOINT_COUNT = 27
MODBUS_JOINT_DATA_REG_COUNT = MODBUS_JOINT_COUNT * 2

REG_TOUCH_DATA_START = 0x2000
REG_TOUCH_TIMESTAMP_US = 0x2080
REG_TOUCH_STATUS_FLAGS = 0x2084
MODBUS_TOUCH_COUNT = 68
MODBUS_TOUCH_DATA_REG_COUNT = 68

MB_FC_READ_SENSOR_SNAPSHOT = 0x41
SENSOR_SNAPSHOT_METADATA_REG_COUNT = 10
SENSOR_SNAPSHOT_POWER_STATE_INDEX = 6
SENSOR_SNAPSHOT_IMU_STATUS_INDEX = 7
SENSOR_SNAPSHOT_JOINT_STATUS_INDEX = 8
SENSOR_SNAPSHOT_TOUCH_STATUS_INDEX = 9
SENSOR_SNAPSHOT_SENSOR_REG_COUNT = (
    MODBUS_IMU_DATA_REG_COUNT
    + MODBUS_JOINT_DATA_REG_COUNT
    + MODBUS_TOUCH_DATA_REG_COUNT
)
SENSOR_SNAPSHOT_REG_COUNT = (
    SENSOR_SNAPSHOT_METADATA_REG_COUNT + SENSOR_SNAPSHOT_SENSOR_REG_COUNT
)
SENSOR_SNAPSHOT_DATA_SIZE = SENSOR_SNAPSHOT_REG_COUNT * 2

CALIB_STATUS_NAMES = {
    0x0000: "idle",
    0x0001: "applied",
    0x0002: "reset_done",
    0x8001: "bad_magic",
    0x8002: "bad_cmd",
    0x8003: "bad_quat",
}

MODBUS_EXCEPTION_NAMES = {
    0x01: "Illegal Function",
    0x02: "Illegal Data Address",
    0x03: "Illegal Data Value",
}

JOINT_FLAG_NAMES = (
    (JOINT_STATUS_SNAPSHOT_VALID, "snapshot"),
    (JOINT_STATUS_ALGORITHM_VALID, "algorithm"),
    (JOINT_STATUS_IMU_CALIB_APPLIED, "calib"),
)

TOUCH_FLAG_NAMES = (
    (0x0001, "snapshot"),
    (0x0002, "touch"),
)

POWER_STATE_NAMES = {
    0: "INIT",
    1: "ON_NORMAL",
    2: "ON_LOW",
    3: "USER_OFF",
    4: "LOW_BAT_LOCKOUT",
    5: "STOPPING",
    6: "RECOVERING",
    7: "RECOVERY_FAULT",
}

HEALTH_STATE_NAMES = {
    0: "INIT",
    1: "OK",
    2: "WARNING",
    3: "DEGRADED",
    4: "RECOVERING",
    5: "FAULT",
    6: "OFF",
    7: "LOCKOUT",
}

HEALTH_SOURCE_NAMES = {
    0: "none",
    1: "IMU",
    2: "CAN1",
    3: "CAN2",
    4: "touch",
    5: "pipeline",
    6: "power",
    7: "battery",
    8: "charger",
    9: "watchdog",
    10: "RS485",
    11: "time_sync",
    12: "calibration",
    13: "storage",
}

RECOVERY_STAGE_NAMES = {
    0: "none",
    1: "confirming sensor loss",
    2: "configuring IMU node",
    3: "verifying IMU node",
    4: "reinitializing CAN bus",
    5: "configuring CAN bus nodes",
    6: "verifying CAN bus",
    7: "stopping acquisition safely",
    8: "peripheral power-off hold",
    9: "starting peripheral power",
    10: "waiting for sensors",
    11: "verifying complete frames",
    12: "recovery failed",
}

HEALTH_FLAG_NAMES = (
    (1 << 0, "imu_partial"),
    (1 << 1, "imu_all_invalid"),
    (1 << 2, "touch_invalid"),
    (1 << 3, "frame_stale"),
    (1 << 4, "joint_invalid"),
    (1 << 5, "can1_error_passive"),
    (1 << 6, "can1_bus_off"),
    (1 << 7, "can2_error_passive"),
    (1 << 8, "can2_bus_off"),
    (1 << 9, "imu_config_failed"),
    (1 << 10, "can_reinit_failed"),
    (1 << 11, "power_recovery_failed"),
    (1 << 12, "low_battery"),
    (1 << 13, "critical_battery"),
    (1 << 14, "bq_comm"),
    (1 << 15, "gauge_comm"),
    (1 << 16, "voltage_mismatch"),
    (1 << 17, "temperature_limit"),
    (1 << 18, "charge_fault"),
    (1 << 19, "watchdog_warning"),
    (1 << 20, "time_unsynced"),
    (1 << 21, "calibration_error"),
    (1 << 22, "rs485_rx_overwrite"),
    (1 << 23, "rs485_uart_error"),
    (1 << 24, "rs485_tx_failed"),
    (1 << 25, "queue_pressure"),
    (1 << 26, "pool_exhausted"),
    (1 << 27, "sd_error"),
)

READY_FLAG_NAMES = (
    (1 << 0, "all_imus"),
    (1 << 1, "touch"),
    (1 << 2, "full_frame"),
    (1 << 3, "joint"),
    (1 << 4, "power"),
    (1 << 5, "time_sync"),
    (1 << 6, "rs485"),
)

HEALTH_ERROR_INFO = {
    0x0000: ("no error", "No action is required."),
    0x1001: ("IMU node data stale", "Check the indicated IMU node and CAN wiring if recovery does not finish."),
    0x1002: ("IMU node configuration failed", "Check the indicated node, its power, and CAN wiring."),
    0x2001: ("CAN entered error-passive", "Inspect CAN termination, wiring, and bus load."),
    0x2002: ("CAN bus-off", "The device will reinitialize the bus automatically; inspect wiring if it repeats."),
    0x2003: ("CAN reinitialization failed", "Power-cycle the device and inspect the indicated CAN bus."),
    0x2004: ("CAN recovery verification failed", "Inspect all nodes on the indicated CAN bus."),
    0x3001: ("touch sync timeout", "Check acquisition sync and peripheral power."),
    0x3002: ("touch ADC DMA timeout", "Check ADC/DMA operation and peripheral power."),
    0x3003: ("touch ADC DMA error", "Check ADC wiring and DMA configuration."),
    0x4001: ("complete sensor frame stale", "Wait for recovery; then inspect IMU/touch readiness."),
    0x4002: ("IMU/touch timestamps do not match", "Check the shared acquisition sync signal."),
    0x4003: ("data queue full", "Reduce processing load or inspect a stalled consumer."),
    0x4004: ("data pool exhausted", "Inspect unreleased buffers or a stalled consumer."),
    0x4005: ("joint algorithm input invalid", "Restore all required IMU data and verify calibration."),
    0x5001: ("acquisition pause timeout", "A producer did not stop safely; inspect IMU and touch tasks."),
    0x5002: ("acquisition sync start failed", "Restart the device and inspect the sync timer/output."),
    0x5003: ("peripheral recovery timeout", "Inspect peripheral power, all IMUs, touch ADC, and sync wiring."),
    0x6001: ("battery low", "Charge the battery soon."),
    0x6002: ("battery critical", "Charge the battery before enabling peripherals."),
    0x6003: ("charger communication failed", "Inspect the BQ25622 and I2C bus."),
    0x6004: ("fuel-gauge communication failed", "Inspect the MAX17043 and I2C bus."),
    0x6005: ("battery voltage readings disagree", "Inspect battery measurement paths."),
    0x6006: ("charging temperature limit", "Allow the battery to return to a safe temperature."),
    0x6007: ("charging fault", "Disconnect power and inspect the charger and battery."),
    0x7001: ("watchdog configuration warning", "Verify watchdog startup and task heartbeat configuration."),
    0x8001: ("RS485 receive frame overwritten", "Reduce request rate or wait for each response."),
    0x8002: ("RS485 UART error", "Inspect baud rate, grounding, termination, and cabling."),
    0x8003: ("RS485 transmit failed", "Inspect the transceiver and request timing."),
    0x8004: ("time synchronization lost", "Send UTC synchronization again."),
    0x9001: ("calibration rejected", "Correct the indicated calibration entry and apply again."),
    0xA001: ("SD logging error", "Inspect the SD card and filesystem, then retry logging."),
}

SENSOR_READY_POWER_STATES = frozenset((1, 2))
IMU_ALL_VALID_MASK = (1 << MODBUS_IMU_COUNT) - 1

CHARGE_STATE_NAMES = {
    0: "UNKNOWN",
    1: "NO_INPUT",
    2: "IDLE",
    3: "CC",
    4: "CV",
    5: "TOPOFF",
    6: "FULL",
    7: "SUSPENDED",
    8: "FAULT",
}

BQ_DIAG_STAGE_NAMES = {
    0: "none",
    1: "init",
    2: "watchdog",
    3: "input_current",
    4: "external_ilim",
    5: "charge_voltage",
    6: "charge_current",
    7: "termination_current",
    8: "charge_safety",
    9: "adc",
    10: "status_read",
    11: "interrupt_config",
    12: "interrupt_read",
}

GLOVE_STATUS_NAMES = {
    0: "OK",
    1: "ERROR",
    2: "TIMEOUT",
    3: "NO_MEMORY",
    4: "INVALID_PARAM",
    5: "QUEUE_FULL",
    6: "QUEUE_EMPTY",
    7: "NOT_READY",
}

POWER_FLAG_NAMES = (
    (1 << 0, "voltage_valid"),
    (1 << 1, "soc_valid"),
    (1 << 2, "current_valid"),
    (1 << 3, "vbus_present"),
    (1 << 4, "charging"),
    (1 << 5, "low"),
    (1 << 6, "critical"),
    (1 << 7, "lockout"),
    (1 << 8, "peripheral_on"),
    (1 << 9, "bq_comm_fault"),
    (1 << 10, "gauge_comm_fault"),
    (1 << 11, "voltage_mismatch"),
    (1 << 12, "temp_limited"),
    (1 << 13, "charge_fault"),
    (1 << 14, "safety_timer"),
    (1 << 15, "charge_full"),
)

BQ_CHARGER_EVENT_NAMES = (
    (1 << 0, "watchdog"),
    (1 << 1, "safety_timer"),
    (1 << 2, "vindpm"),
    (1 << 3, "iindpm"),
    (1 << 4, "vsys"),
    (1 << 5, "thermal_regulation"),
    (1 << 6, "adc_done"),
    (1 << 8, "vbus_changed"),
    (1 << 11, "charge_changed"),
)

BQ_FAULT_EVENT_NAMES = (
    (1 << 0, "ts_changed"),
    (1 << 3, "thermal_shutdown"),
    (1 << 4, "otg_fault"),
    (1 << 5, "system_fault"),
    (1 << 6, "battery_fault"),
    (1 << 7, "vbus_fault"),
)

RESET_CAUSE_NAMES = (
    (1 << 0, "pin_reset"),
    (1 << 1, "power_reset"),
    (1 << 2, "software_reset"),
    (1 << 3, "iwdg_reset"),
    (1 << 4, "wwdg_reset"),
    (1 << 5, "low_power_reset"),
)

WATCHDOG_STATUS_NAMES = (
    (1 << 0, "running"),
    (1 << 1, "refresh_ok"),
    (1 << 2, "config_warning"),
)

IMU_FIELDS = (
    "acc_x_mps2",
    "acc_y_mps2",
    "acc_z_mps2",
    "gyro_x_radps",
    "gyro_y_radps",
    "gyro_z_radps",
    "quat_w",
    "quat_x",
    "quat_y",
    "quat_z",
)


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(frame: bytes) -> bytes:
    crc = crc16_modbus(frame)
    return frame + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def regs_to_ros_time_le_words(regs: list[int]) -> tuple[int, int]:
    if len(regs) < 4:
        raise ModbusError("ROS Time needs 4 registers")
    sec = (regs[0] & 0xFFFF) | ((regs[1] & 0xFFFF) << 16)
    nsec = (regs[2] & 0xFFFF) | ((regs[3] & 0xFFFF) << 16)
    return sec, nsec


def ros_time_to_regs_le_words(sec: int, nsec: int) -> list[int]:
    if not (0 <= sec <= 0xFFFFFFFF):
        raise ModbusError("ROS Time sec must be 0..0xFFFFFFFF")
    if not (0 <= nsec < 1_000_000_000):
        raise ModbusError("ROS Time nanosec must be 0..999999999")
    return [
        sec & 0xFFFF,
        (sec >> 16) & 0xFFFF,
        nsec & 0xFFFF,
        (nsec >> 16) & 0xFFFF,
    ]


def ros_time_to_ns(regs: list[int]) -> int:
    sec, nsec = regs_to_ros_time_le_words(regs)
    return sec * 1_000_000_000 + nsec


def ros_time_to_us(regs: list[int]) -> int:
    sec, nsec = regs_to_ros_time_le_words(regs)
    return sec * 1_000_000 + nsec // 1_000


def format_ros_time(regs: list[int]) -> str:
    sec, nsec = regs_to_ros_time_le_words(regs)
    suffix = " invalid_nsec" if nsec >= 1_000_000_000 else ""
    try:
        stamp = dt.datetime.fromtimestamp(sec, tz=dt.timezone.utc)
        stamp = stamp.replace(microsecond=nsec // 1_000)
        return f"{stamp:%Y-%m-%d %H:%M:%S}.{stamp.microsecond:06d} UTC{suffix}"
    except (OverflowError, OSError, ValueError):
        return f"{sec * 1_000_000 + nsec // 1_000} us{suffix}"


def format_host_time_ns(host_ns: int) -> str:
    sec = host_ns // 1_000_000_000
    nsec = host_ns % 1_000_000_000
    stamp = dt.datetime.fromtimestamp(sec, tz=dt.timezone.utc)
    stamp = stamp.replace(microsecond=nsec // 1_000)
    return f"{stamp:%Y-%m-%d %H:%M:%S}.{stamp.microsecond:06d} UTC"


def format_delta_ns(delta_ns: int) -> str:
    return f"{delta_ns / 1_000:+.3f} us"


def regs_to_f32_le_words(low_word: int, high_word: int) -> float:
    raw = (low_word & 0xFFFF) | ((high_word & 0xFFFF) << 16)
    return struct.unpack("<f", struct.pack("<I", raw))[0]


def f32_to_regs_le_words(value: float) -> list[int]:
    raw = struct.unpack("<I", struct.pack("<f", float(value)))[0]
    return [raw & 0xFFFF, (raw >> 16) & 0xFFFF]


def parse_u16(text: str, name: str = "value") -> int:
    value = int(text.strip(), 0)
    if not (0 <= value <= 0xFFFF):
        raise ModbusError(f"{name} must be 0..0xFFFF")
    return value


def parse_u16_list(text: str) -> list[int]:
    tokens = text.replace(",", " ").split()
    if not tokens:
        raise ModbusError("enter at least one register value")
    return [parse_u16(token, "register value") for token in tokens]


def parse_hex_bytes(text: str) -> bytes:
    tokens = text.replace(",", " ").split()
    if not tokens:
        raise ModbusError("enter at least one byte")
    values = []
    for token in tokens:
        if token.lower().startswith("0x"):
            value = int(token, 16)
        else:
            value = int(token, 16)
        if not (0 <= value <= 0xFF):
            raise ModbusError("raw byte must be 00..FF")
        values.append(value)
    return bytes(values)


def format_flags(value: int, names: tuple[tuple[int, str], ...]) -> str:
    active = [name for mask, name in names if (value & mask) != 0]
    return "|".join(active) if active else "none"


def calib_status_name(value: int) -> str:
    return CALIB_STATUS_NAMES.get(value, f"unknown_0x{value:04X}")


def modbus_exception_name(value: int) -> str:
    return MODBUS_EXCEPTION_NAMES.get(value, f"Exception 0x{value:02X}")


def validate_calibration_table(table: list[list[float]], table_name: str) -> None:
    if len(table) != MODBUS_IMU_COUNT:
        raise ModbusError(f"{table_name} table must contain 16 rows")
    for imu, quat in enumerate(table):
        if len(quat) != 4:
            raise ModbusError(f"{table_name}{imu} must contain w,x,y,z")
        if not all(math.isfinite(value) for value in quat):
            raise ModbusError(f"{table_name}{imu} contains NaN or Inf")
        norm_sq = sum(value * value for value in quat)
        if not (0.25 <= norm_sq <= 2.25):
            raise ModbusError(
                f"{table_name}{imu} norm^2={norm_sq:.6g}, expected 0.25..2.25"
            )


class ModbusError(RuntimeError):
    pass


@dataclass(frozen=True)
class LowLatencyStats:
    requests: int
    timeouts: int
    retries: int
    max_request_ms: float
    last_timeout_start: int


class ModbusRtuClient:
    def __init__(self) -> None:
        self.port: serial.Serial | None = None
        self.lock = threading.Lock()
        self.connected_port_name = ""
        self.connected_baud = 0
        self.last_tx = b""
        self.last_rx = b""
        self.low_latency_saved_timeout: float | None = None
        self.low_latency_saved_inter_byte_timeout: float | None = None
        self.low_latency_requests = 0
        self.low_latency_timeouts = 0
        self.low_latency_retries = 0
        self.low_latency_max_request_ms = 0.0
        self.low_latency_last_timeout_start = 0

    @staticmethod
    def _wait_inter_request_gap() -> None:
        """Leave a short silent interval before each RTU request."""
        deadline = time.perf_counter() + MODBUS_INTER_REQUEST_GAP_S
        while time.perf_counter() < deadline:
            pass

    def open(self, port_name: str, baud: int, timeout_s: float) -> None:
        if serial is None:
            raise ModbusError("pyserial is not installed")
        self.close()
        self.port = serial.Serial(
            port=port_name,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout_s,
            write_timeout=timeout_s,
            inter_byte_timeout=timeout_s,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.connected_port_name = port_name
        self.connected_baud = baud
        self.port.reset_input_buffer()
        self.port.reset_output_buffer()

    def close(self) -> None:
        if self.port is not None:
            try:
                self.port.close()
            finally:
                self.port = None
                self.connected_port_name = ""
                self.connected_baud = 0
                self.low_latency_saved_timeout = None
                self.low_latency_saved_inter_byte_timeout = None

    @property
    def is_open(self) -> bool:
        return self.port is not None and self.port.is_open

    def prepare_low_latency_poll(self) -> None:
        """Configure bounded reads before starting a continuous high-rate poll."""
        if not self.is_open or self.port is None:
            raise ModbusError("serial port is not open")
        with self.lock:
            if self.low_latency_saved_timeout is None:
                self.low_latency_saved_timeout = self.port.timeout
                self.low_latency_saved_inter_byte_timeout = self.port.inter_byte_timeout
            # 高频读取按2ms分片返回，确保外层8ms事务截止时间能够真正生效。
            self.port.timeout = SENSOR_120HZ_READ_SLICE_S
            self.port.inter_byte_timeout = None
            self.port.reset_input_buffer()
            self.low_latency_requests = 0
            self.low_latency_timeouts = 0
            self.low_latency_retries = 0
            self.low_latency_max_request_ms = 0.0
            self.low_latency_last_timeout_start = 0

    def finish_low_latency_poll(self) -> None:
        """Restore normal serial timeouts after the high-rate poll stops."""
        if not self.is_open or self.port is None:
            return
        with self.lock:
            if self.low_latency_saved_timeout is not None:
                self.port.timeout = self.low_latency_saved_timeout
                self.port.inter_byte_timeout = self.low_latency_saved_inter_byte_timeout
                self.low_latency_saved_timeout = None
                self.low_latency_saved_inter_byte_timeout = None

    def get_low_latency_stats(self) -> LowLatencyStats:
        return LowLatencyStats(
            requests=self.low_latency_requests,
            timeouts=self.low_latency_timeouts,
            retries=self.low_latency_retries,
            max_request_ms=self.low_latency_max_request_ms,
            last_timeout_start=self.low_latency_last_timeout_start,
        )

    def recover_low_latency_poll(self) -> None:
        """Discard an incomplete response after a timeout or protocol error."""
        if not self.is_open or self.port is None:
            return
        with self.lock:
            self.port.reset_input_buffer()

    def read_holding_registers(
        self,
        slave: int,
        start: int,
        count: int,
        timeout_s: float,
        low_latency: bool = False,
    ) -> list[int]:
        if not self.is_open or self.port is None:
            raise ModbusError("serial port is not open")
        if not (1 <= slave <= 247):
            raise ModbusError("slave id must be 1..247")
        if not (1 <= count <= 125):
            raise ModbusError("register count must be 1..125")

        request = append_crc(
            bytes(
                (
                    slave & 0xFF,
                    0x03,
                    (start >> 8) & 0xFF,
                    start & 0xFF,
                    (count >> 8) & 0xFF,
                    count & 0xFF,
                )
            )
        )
        expected_len = 5 + count * 2
        deadline = time.monotonic() + timeout_s
        request_started = time.perf_counter()

        with self.lock:
            # 高频连续轮询只在启动和异常恢复时清空缓冲区，避免每帧产生USB控制开销。
            if not low_latency:
                self.port.reset_input_buffer()
            self.last_tx = request
            self.last_rx = b""
            self._wait_inter_request_gap()
            self.port.write(request)
            if not low_latency:
                self.port.flush()

            buffer = bytearray()
            while time.monotonic() < deadline:
                want = max(1, expected_len - len(buffer))
                if low_latency:
                    # 高频模式直接等待当前响应剩余字节，避免每包查询串口队列引入系统调用延迟。
                    chunk = self.port.read(want)
                else:
                    waiting = self.port.in_waiting
                    chunk = self.port.read(waiting or want)
                if chunk:
                    buffer.extend(chunk)
                    parsed = self._try_parse_response(buffer, slave, count)
                    if parsed is not None:
                        self.last_rx = bytes(buffer)
                        if low_latency:
                            request_ms = (time.perf_counter() - request_started) * 1000.0
                            self.low_latency_requests += 1
                            self.low_latency_max_request_ms = max(
                                self.low_latency_max_request_ms, request_ms
                            )
                        return parsed
                else:
                    if not low_latency:
                        time.sleep(0.002)

        self.last_rx = bytes(buffer)
        if low_latency:
            self.low_latency_timeouts += 1
            self.low_latency_last_timeout_start = start
        hex_rx = " ".join(f"{byte:02X}" for byte in buffer)
        raise ModbusError(
            f"timeout reading 0x{start:04X}+{count}, rx=[{hex_rx}], "
            f"expected slave=0x{slave:02X} func=0x03 bytes={count * 2}"
        )

    @staticmethod
    def _try_parse_response(buffer: bytearray, slave: int, count: int) -> list[int] | None:
        expected_data_len = count * 2
        expected_len = 5 + expected_data_len

        index = 0
        while index < len(buffer):
            if buffer[index] != slave:
                index += 1
                continue
            if index + 5 > len(buffer):
                return None

            func = buffer[index + 1]
            if func == 0x83:
                if index + 5 > len(buffer):
                    return None
                frame = bytes(buffer[index : index + 5])
                if crc16_modbus(frame[:-2]) == int.from_bytes(frame[-2:], "little"):
                    raise ModbusError(
                        f"modbus exception 0x{frame[2]:02X} "
                        f"({modbus_exception_name(frame[2])})"
                    )
                index += 1
                continue

            if func != 0x03:
                index += 1
                continue

            byte_count = buffer[index + 2]
            frame_len = 5 + byte_count
            if byte_count != expected_data_len:
                index += 1
                continue
            if index + frame_len > len(buffer):
                return None

            frame = bytes(buffer[index : index + frame_len])
            if len(frame) != expected_len:
                index += 1
                continue
            if crc16_modbus(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
                index += 1
                continue

            data = frame[3:-2]
            return [
                ((data[pos] << 8) | data[pos + 1])
                for pos in range(0, len(data), 2)
            ]

        return None

    def read_sensor_snapshot_registers(
        self,
        slave: int,
        timeout_s: float,
        retries: int = SENSOR_120HZ_RETRIES,
    ) -> list[int]:
        """Read the complete high-rate sensor snapshot with one RTU transaction."""
        last_error: ModbusError | None = None

        for attempt in range(retries + 1):
            try:
                return self._read_sensor_snapshot_once(slave, timeout_s)
            except ModbusError as exc:
                last_error = exc
                if attempt < retries:
                    self.low_latency_retries += 1
                    self.recover_low_latency_poll()
                    # 给从机留出结束上一笔事务的时间，避免超时重试与迟到响应重叠。
                    time.sleep(SENSOR_120HZ_RETRY_GAP_S)

        if last_error is None:
            raise ModbusError("sensor snapshot failed without error detail")
        raise last_error

    def _read_sensor_snapshot_once(self, slave: int, timeout_s: float) -> list[int]:
        if not self.is_open or self.port is None:
            raise ModbusError("serial port is not open")
        if not (1 <= slave <= 247):
            raise ModbusError("slave id must be 1..247")

        request = append_crc(bytes((slave & 0xFF, MB_FC_READ_SENSOR_SNAPSHOT)))
        expected_len = 4 + SENSOR_SNAPSHOT_DATA_SIZE + 2
        deadline = time.monotonic() + timeout_s
        request_started = time.perf_counter()

        with self.lock:
            self.last_tx = request
            self.last_rx = b""
            self._wait_inter_request_gap()
            self.port.write(request)

            buffer = bytearray()
            while time.monotonic() < deadline:
                chunk = self.port.read(max(1, expected_len - len(buffer)))
                if chunk:
                    buffer.extend(chunk)
                    parsed = self._try_parse_sensor_snapshot_response(buffer, slave)
                    if parsed is not None:
                        self.last_rx = bytes(buffer)
                        request_ms = (time.perf_counter() - request_started) * 1000.0
                        self.low_latency_requests += 1
                        self.low_latency_max_request_ms = max(
                            self.low_latency_max_request_ms, request_ms
                        )
                        return parsed

        self.last_rx = bytes(buffer)
        self.low_latency_timeouts += 1
        self.low_latency_last_timeout_start = MB_FC_READ_SENSOR_SNAPSHOT
        hex_head = " ".join(f"{byte:02X}" for byte in buffer[:16])
        raise ModbusError(
            f"timeout reading sensor snapshot, rx_len={len(buffer)} "
            f"rx_head=[{hex_head}], expected={expected_len}"
        )

    @staticmethod
    def _try_parse_sensor_snapshot_response(
        buffer: bytearray, slave: int
    ) -> list[int] | None:
        expected_len = 4 + SENSOR_SNAPSHOT_DATA_SIZE + 2
        index = 0

        while index < len(buffer):
            if buffer[index] != slave:
                index += 1
                continue
            if index + 4 > len(buffer):
                return None

            func = buffer[index + 1]
            if func == (MB_FC_READ_SENSOR_SNAPSHOT | 0x80):
                if index + 5 > len(buffer):
                    return None
                frame = bytes(buffer[index : index + 5])
                if crc16_modbus(frame[:-2]) == int.from_bytes(frame[-2:], "little"):
                    raise ModbusError(
                        f"sensor snapshot exception 0x{frame[2]:02X} "
                        f"({modbus_exception_name(frame[2])})"
                    )
                index += 1
                continue

            if func != MB_FC_READ_SENSOR_SNAPSHOT:
                index += 1
                continue

            byte_count = (buffer[index + 2] << 8) | buffer[index + 3]
            if byte_count != SENSOR_SNAPSHOT_DATA_SIZE:
                index += 1
                continue
            if index + expected_len > len(buffer):
                return None

            frame = bytes(buffer[index : index + expected_len])
            if crc16_modbus(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
                index += 1
                continue

            data = frame[4:-2]
            return [
                ((data[pos] << 8) | data[pos + 1])
                for pos in range(0, len(data), 2)
            ]

        return None

    def read_block_split(
        self,
        slave: int,
        start: int,
        count: int,
        timeout_s: float,
        max_read_regs: int = MAX_READ_REGS,
        retries: int = 0,
        low_latency: bool = False,
    ) -> list[int]:
        regs: list[int] = []
        offset = 0
        while offset < count:
            chunk = min(max_read_regs, count - offset)
            last_error: Exception | None = None
            for attempt in range(retries + 1):
                try:
                    regs.extend(
                        self.read_holding_registers(
                            slave,
                            start + offset,
                            chunk,
                            timeout_s,
                            low_latency=low_latency,
                        )
                    )
                    last_error = None
                    break
                except ModbusError as exc:
                    last_error = exc
                    if attempt < retries:
                        if low_latency:
                            self.low_latency_retries += 1
                            self.recover_low_latency_poll()
                        else:
                            time.sleep(0.001)
            if last_error is not None:
                raise last_error
            offset += chunk
        return regs

    def write_multiple_registers(
        self, slave: int, start: int, regs: list[int], timeout_s: float
    ) -> None:
        if not self.is_open or self.port is None:
            raise ModbusError("serial port is not open")
        if not (1 <= slave <= 247):
            raise ModbusError("slave id must be 1..247")
        if not (1 <= len(regs) <= 123):
            raise ModbusError("write register count must be 1..123")

        payload = bytearray()
        for reg in regs:
            payload.extend(((reg >> 8) & 0xFF, reg & 0xFF))
        request = append_crc(
            bytes(
                (
                    slave & 0xFF,
                    0x10,
                    (start >> 8) & 0xFF,
                    start & 0xFF,
                    (len(regs) >> 8) & 0xFF,
                    len(regs) & 0xFF,
                    len(payload) & 0xFF,
                )
            )
            + bytes(payload)
        )
        deadline = time.monotonic() + timeout_s

        with self.lock:
            self.port.reset_input_buffer()
            self.last_tx = request
            self.last_rx = b""
            self._wait_inter_request_gap()
            self.port.write(request)
            self.port.flush()

            buffer = bytearray()
            while time.monotonic() < deadline:
                waiting = self.port.in_waiting
                chunk = self.port.read(waiting or max(1, 8 - len(buffer)))
                if chunk:
                    buffer.extend(chunk)
                    if self._try_parse_write_response(buffer, slave, start, len(regs)):
                        self.last_rx = bytes(buffer)
                        return
                else:
                    time.sleep(0.002)

        self.last_rx = bytes(buffer)
        hex_rx = " ".join(f"{byte:02X}" for byte in buffer)
        raise ModbusError(f"timeout writing 0x{start:04X}+{len(regs)}, rx=[{hex_rx}]")

    @staticmethod
    def _try_parse_write_response(
        buffer: bytearray, slave: int, start: int, count: int
    ) -> bool:
        index = 0
        while index < len(buffer):
            if buffer[index] != slave:
                index += 1
                continue
            if index + 5 > len(buffer):
                return False

            func = buffer[index + 1]
            if func == 0x90:
                if index + 5 > len(buffer):
                    return False
                frame = bytes(buffer[index : index + 5])
                if crc16_modbus(frame[:-2]) == int.from_bytes(frame[-2:], "little"):
                    raise ModbusError(
                        f"modbus exception 0x{frame[2]:02X} "
                        f"({modbus_exception_name(frame[2])})"
                    )
                index += 1
                continue

            if func != 0x10:
                index += 1
                continue
            if index + 8 > len(buffer):
                return False

            frame = bytes(buffer[index : index + 8])
            if crc16_modbus(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
                index += 1
                continue

            ack_start = (frame[2] << 8) | frame[3]
            ack_count = (frame[4] << 8) | frame[5]
            if ack_start != start or ack_count != count:
                raise ModbusError(
                    f"write ack mismatch start=0x{ack_start:04X} count={ack_count}"
                )
            return True

        return False

    def write_single_register(
        self, slave: int, reg_addr: int, value: int, timeout_s: float
    ) -> None:
        if not self.is_open or self.port is None:
            raise ModbusError("serial port is not open")
        if not (1 <= slave <= 247):
            raise ModbusError("slave id must be 1..247")
        if not (0 <= reg_addr <= 0xFFFF):
            raise ModbusError("register address must be 0..0xFFFF")
        if not (0 <= value <= 0xFFFF):
            raise ModbusError("register value must be 0..0xFFFF")

        request = append_crc(
            bytes(
                (
                    slave & 0xFF,
                    0x06,
                    (reg_addr >> 8) & 0xFF,
                    reg_addr & 0xFF,
                    (value >> 8) & 0xFF,
                    value & 0xFF,
                )
            )
        )
        deadline = time.monotonic() + timeout_s

        with self.lock:
            self.port.reset_input_buffer()
            self.last_tx = request
            self.last_rx = b""
            self._wait_inter_request_gap()
            self.port.write(request)
            self.port.flush()

            buffer = bytearray()
            while time.monotonic() < deadline:
                waiting = self.port.in_waiting
                chunk = self.port.read(waiting or max(1, 8 - len(buffer)))
                if chunk:
                    buffer.extend(chunk)
                    if self._try_parse_write_single_response(buffer, slave, reg_addr, value):
                        self.last_rx = bytes(buffer)
                        return
                else:
                    time.sleep(0.002)

        self.last_rx = bytes(buffer)
        hex_rx = " ".join(f"{byte:02X}" for byte in buffer)
        raise ModbusError(f"timeout writing single 0x{reg_addr:04X}, rx=[{hex_rx}]")

    @staticmethod
    def _try_parse_write_single_response(
        buffer: bytearray, slave: int, reg_addr: int, value: int
    ) -> bool:
        index = 0
        while index < len(buffer):
            if buffer[index] != slave:
                index += 1
                continue
            if index + 5 > len(buffer):
                return False

            func = buffer[index + 1]
            if func == 0x86:
                if index + 5 > len(buffer):
                    return False
                frame = bytes(buffer[index : index + 5])
                if crc16_modbus(frame[:-2]) == int.from_bytes(frame[-2:], "little"):
                    raise ModbusError(
                        f"modbus exception 0x{frame[2]:02X} "
                        f"({modbus_exception_name(frame[2])})"
                    )
                index += 1
                continue

            if func != 0x06:
                index += 1
                continue
            if index + 8 > len(buffer):
                return False

            frame = bytes(buffer[index : index + 8])
            if crc16_modbus(frame[:-2]) != int.from_bytes(frame[-2:], "little"):
                index += 1
                continue

            ack_addr = (frame[2] << 8) | frame[3]
            ack_value = (frame[4] << 8) | frame[5]
            if ack_addr != reg_addr or ack_value != value:
                raise ModbusError(
                    f"write single ack mismatch addr=0x{ack_addr:04X} "
                    f"value=0x{ack_value:04X}"
                )
            return True

        return False

    def last_exchange_hex(self) -> str:
        tx = " ".join(f"{byte:02X}" for byte in self.last_tx) or "<empty>"
        rx = " ".join(f"{byte:02X}" for byte in self.last_rx) or "<empty>"
        return f"TX {tx}\nRX {rx}"

    def transceive_raw(self, frame: bytes, timeout_s: float) -> bytes:
        if not self.is_open or self.port is None:
            raise ModbusError("serial port is not open")
        if not frame:
            raise ModbusError("raw frame is empty")

        deadline = time.monotonic() + timeout_s
        with self.lock:
            self.port.reset_input_buffer()
            self.last_tx = frame
            self.last_rx = b""
            self._wait_inter_request_gap()
            self.port.write(frame)
            self.port.flush()

            buffer = bytearray()
            last_rx_at: float | None = None
            while time.monotonic() < deadline:
                waiting = self.port.in_waiting
                chunk = self.port.read(waiting or 1)
                if chunk:
                    buffer.extend(chunk)
                    last_rx_at = time.monotonic()
                    continue
                if last_rx_at is not None and time.monotonic() - last_rx_at > 0.03:
                    break
                time.sleep(0.002)

            self.last_rx = bytes(buffer)
            return self.last_rx

    def write_block_split(
        self, slave: int, start: int, regs: list[int], timeout_s: float
    ) -> None:
        offset = 0
        while offset < len(regs):
            chunk = min(MAX_WRITE_REGS, len(regs) - offset)
            self.write_multiple_registers(
                slave, start + offset, regs[offset : offset + chunk], timeout_s
            )
            offset += chunk


@dataclass
class PowerSnapshot:
    battery_voltage_v: float
    battery_current_a: float
    soc_percent: float
    system_state: int
    charge_state: int
    flags: int
    fault_code: int
    vbus_voltage_v: float
    input_current_a: float
    bq_diag_stage: int
    bq_diag_status: int
    bq_charger_events: int
    bq_fault_events: int
    bq_interrupt_count: int


@dataclass
class HealthSnapshot:
    version: int
    state: int
    current_flags: int
    current_error: int
    current_source: int
    current_target: int
    recovery_stage: int
    recovery_attempt: int
    recovery_limit: int
    last_error: int
    last_source: int
    last_target: int
    error_seq: int
    error_count: int
    last_error_uptime_ms: int
    live_imu_mask: int
    ready_flags: int
    snapshot_age_ms: int
    rs485_uart_detail: int


def decode_health(regs: list[int]) -> HealthSnapshot:
    if len(regs) < REG_HEALTH_STATUS_COUNT:
        raise ModbusError(f"health status needs {REG_HEALTH_STATUS_COUNT} registers")
    attempt_word = regs[8]
    return HealthSnapshot(
        version=regs[0],
        state=regs[1],
        current_flags=regs[2] | (regs[3] << 16),
        current_error=regs[4],
        current_source=regs[5],
        current_target=regs[6],
        recovery_stage=regs[7],
        recovery_attempt=attempt_word & 0xFF,
        recovery_limit=(attempt_word >> 8) & 0xFF,
        last_error=regs[9],
        last_source=regs[10],
        last_target=regs[11],
        error_seq=regs[12] | (regs[13] << 16),
        error_count=regs[14] | (regs[15] << 16),
        last_error_uptime_ms=regs[16] | (regs[17] << 16),
        live_imu_mask=regs[18],
        ready_flags=regs[19],
        snapshot_age_ms=regs[20],
        rs485_uart_detail=regs[21],
    )


def health_error_text(error: int) -> tuple[str, str]:
    return HEALTH_ERROR_INFO.get(
        error,
        (f"unknown error 0x{error:04X}", "Record the code and inspect the detailed status."),
    )


def format_uart_error_detail(detail: int) -> str:
    if detail == 0:
        return "none captured"
    uart_error_names = (
        (0x0001, "PE/parity"),
        (0x0002, "NE/noise"),
        (0x0004, "FE/frame"),
        (0x0008, "ORE/overrun"),
        (0x0010, "DMA"),
        (0x0020, "RTO/timeout"),
    )
    active = [name for mask, name in uart_error_names if detail & mask]
    names = "|".join(active) if active else "unknown"
    return f"0x{detail:04X} ({names})"


def format_health_target(source: int, target: int) -> str:
    if target == 0:
        return "not specified"
    if source == 1:
        return f"IMU logical node {target}"
    if source in (2, 3):
        return f"CAN bus {target}"
    if source == 10:
        return f"UART error {format_uart_error_detail(target)}"
    if source == 12:
        return f"calibration entry {target}"
    if source == 13:
        return f"SD detail 0x{target:04X}"
    return str(target)


def decode_power(regs: list[int]) -> PowerSnapshot:
    if len(regs) < REG_POWER_STATUS_COUNT:
        raise ModbusError(f"power status needs {REG_POWER_STATUS_COUNT} registers")
    return PowerSnapshot(
        battery_voltage_v=regs_to_f32_le_words(regs[0], regs[1]),
        battery_current_a=regs_to_f32_le_words(regs[2], regs[3]),
        soc_percent=regs_to_f32_le_words(regs[4], regs[5]),
        system_state=regs[6],
        charge_state=regs[7],
        flags=regs[8],
        fault_code=regs[9],
        vbus_voltage_v=regs_to_f32_le_words(regs[10], regs[11]),
        input_current_a=regs_to_f32_le_words(regs[12], regs[13]),
        bq_diag_stage=(regs[14] >> 8) & 0xFF,
        bq_diag_status=regs[14] & 0xFF,
        bq_charger_events=regs[15],
        bq_fault_events=regs[16],
        bq_interrupt_count=regs[17],
    )


@dataclass
class GloveSnapshot:
    timestamp: float
    basic: list[int]
    system: list[int]
    health_regs: list[int]
    power_regs: list[int]
    work_state: int
    imu_status_regs: list[int]
    calib_ctrl_regs: list[int]
    joint_status_regs: list[int]
    touch_status_regs: list[int]
    imu_regs: list[int]
    joint_regs: list[int]
    touch_regs: list[int]
    poll_mode: str = "full"
    actual_hz: float = 0.0
    sensor_hz: float = 0.0
    sensor_frame_id: int = 0
    sensor_timestamp_us: int = 0
    duplicate_responses: int = 0
    comm_requests: int = 0
    comm_timeouts: int = 0
    comm_retries: int = 0
    comm_max_request_ms: float = 0.0
    comm_last_timeout_start: int = 0
    sensor_data_valid: bool = False
    sensor_invalid_reason: str = "sensor snapshot not received"


def evaluate_sensor_validity(
    power_state: int,
    sensor_timestamp_us: int,
    imu_status: int,
    joint_status: int,
    touch_status: int,
) -> tuple[bool, str]:
    if power_state not in SENSOR_READY_POWER_STATES:
        state_name = POWER_STATE_NAMES.get(power_state, f"UNKNOWN({power_state})")
        return False, f"power state {state_name}"
    if sensor_timestamp_us <= 0:
        return False, "sensor snapshot timestamp is invalid"
    if (imu_status & IMU_ALL_VALID_MASK) != IMU_ALL_VALID_MASK:
        return False, f"IMU valid mask 0x{imu_status:04X} is incomplete"
    if (joint_status & (JOINT_STATUS_SNAPSHOT_VALID | JOINT_STATUS_ALGORITHM_VALID)) != (
        JOINT_STATUS_SNAPSHOT_VALID | JOINT_STATUS_ALGORITHM_VALID
    ):
        return False, f"joint status 0x{joint_status:04X} is invalid"
    if (touch_status & 0x0003) != 0x0003:
        return False, f"touch status 0x{touch_status:04X} is invalid"
    return True, "ready"


def read_snapshot(client: ModbusRtuClient, slave: int, timeout_s: float) -> GloveSnapshot:
    snapshot = GloveSnapshot(
        timestamp=time.time(),
        basic=client.read_holding_registers(
            slave, REG_BASIC_STATUS_START, REG_BASIC_STATUS_COUNT, timeout_s
        ),
        system=client.read_holding_registers(
            slave, REG_SYSTEM_STATUS_START, REG_SYSTEM_STATUS_COUNT, timeout_s
        ),
        health_regs=client.read_holding_registers(
            slave, REG_HEALTH_STATUS_START, REG_HEALTH_STATUS_COUNT, timeout_s
        ),
        power_regs=client.read_holding_registers(
            slave, REG_POWER_STATUS_START, REG_POWER_STATUS_COUNT, timeout_s
        ),
        work_state=client.read_holding_registers(slave, REG_WORK_STATE, 1, timeout_s)[0],
        imu_status_regs=client.read_holding_registers(
            slave, REG_IMU_TIMESTAMP_US, 5, timeout_s
        ),
        calib_ctrl_regs=client.read_holding_registers(
            slave, REG_IMU_CALIB_CTRL_START, REG_IMU_CALIB_CTRL_COUNT, timeout_s
        ),
        joint_status_regs=client.read_holding_registers(
            slave, REG_JOINT_TIMESTAMP_US, 7, timeout_s
        ),
        touch_status_regs=client.read_holding_registers(
            slave, REG_TOUCH_TIMESTAMP_US, 8, timeout_s
        ),
        imu_regs=client.read_block_split(
            slave, REG_IMU_DATA_START, MODBUS_IMU_DATA_REG_COUNT, timeout_s
        ),
        joint_regs=client.read_holding_registers(
            slave, REG_JOINT_DATA_START, MODBUS_JOINT_DATA_REG_COUNT, timeout_s
        ),
        touch_regs=client.read_holding_registers(
            slave, REG_TOUCH_DATA_START, MODBUS_TOUCH_DATA_REG_COUNT, timeout_s
        ),
    )
    power_state = decode_power(snapshot.power_regs).system_state
    sensor_timestamp_us = ros_time_to_us(snapshot.imu_status_regs[0:4])
    snapshot.sensor_data_valid, snapshot.sensor_invalid_reason = evaluate_sensor_validity(
        power_state,
        sensor_timestamp_us,
        snapshot.imu_status_regs[4],
        snapshot.joint_status_regs[4],
        snapshot.touch_status_regs[4],
    )
    snapshot.sensor_timestamp_us = sensor_timestamp_us
    return snapshot


def empty_snapshot() -> GloveSnapshot:
    return GloveSnapshot(
        timestamp=time.time(),
        basic=[0] * REG_BASIC_STATUS_COUNT,
        system=[0] * REG_SYSTEM_STATUS_COUNT,
        health_regs=[0] * REG_HEALTH_STATUS_COUNT,
        power_regs=[0] * REG_POWER_STATUS_COUNT,
        work_state=0,
        imu_status_regs=[0] * 5,
        calib_ctrl_regs=[0] * REG_IMU_CALIB_CTRL_COUNT,
        joint_status_regs=[0] * 7,
        touch_status_regs=[0] * 8,
        imu_regs=[0] * MODBUS_IMU_DATA_REG_COUNT,
        joint_regs=[0] * MODBUS_JOINT_DATA_REG_COUNT,
        touch_regs=[0] * MODBUS_TOUCH_DATA_REG_COUNT,
    )


def read_sensor_snapshot_120hz(
    client: ModbusRtuClient,
    slave: int,
    timeout_s: float,
    previous: GloveSnapshot | None,
    actual_hz: float,
) -> GloveSnapshot:
    """Read IMU, solved joint and touch data with one snapshot request."""
    base = previous if previous is not None else empty_snapshot()
    fast_timeout_s = min(timeout_s, SENSOR_120HZ_TIMEOUT_S)

    snapshot_regs = client.read_sensor_snapshot_registers(
        slave, fast_timeout_s, retries=SENSOR_120HZ_RETRIES
    )
    sensor_frame_id = snapshot_regs[0] | (snapshot_regs[1] << 16)
    sensor_time_regs = snapshot_regs[2:6]
    sensor_timestamp_us = ros_time_to_us(sensor_time_regs)
    power_state = snapshot_regs[SENSOR_SNAPSHOT_POWER_STATE_INDEX]
    imu_status = snapshot_regs[SENSOR_SNAPSHOT_IMU_STATUS_INDEX]
    joint_status = snapshot_regs[SENSOR_SNAPSHOT_JOINT_STATUS_INDEX]
    touch_status = snapshot_regs[SENSOR_SNAPSHOT_TOUCH_STATUS_INDEX]
    sensor_regs = snapshot_regs[SENSOR_SNAPSHOT_METADATA_REG_COUNT:]
    imu_end = MODBUS_IMU_DATA_REG_COUNT
    joint_end = imu_end + MODBUS_JOINT_DATA_REG_COUNT
    received_imu_regs = sensor_regs[:imu_end]
    received_joint_regs = sensor_regs[imu_end:joint_end]
    received_touch_regs = sensor_regs[joint_end:]

    sensor_data_valid, invalid_reason = evaluate_sensor_validity(
        power_state,
        sensor_timestamp_us,
        imu_status,
        joint_status,
        touch_status,
    )

    power_regs = list(base.power_regs)
    power_regs[6] = power_state

    joint_valid_bits = 0
    if sensor_data_valid:
        for index in range(MODBUS_JOINT_COUNT):
            pos = index * 2
            value = regs_to_f32_le_words(
                received_joint_regs[pos], received_joint_regs[pos + 1]
            )
            if math.isfinite(value) and -999999936.0 < value < 999999936.0:
                joint_valid_bits |= 1 << index

    if sensor_data_valid:
        imu_regs = received_imu_regs
        joint_regs = received_joint_regs
        touch_regs = received_touch_regs
        displayed_frame_id = sensor_frame_id
        displayed_timestamp_us = sensor_timestamp_us
    else:
        # 无效响应只更新状态，不让固件返回的占位0覆盖最后一帧有效数据。
        imu_regs = base.imu_regs
        joint_regs = base.joint_regs
        touch_regs = base.touch_regs
        displayed_frame_id = base.sensor_frame_id
        displayed_timestamp_us = base.sensor_timestamp_us

    comm_stats = client.get_low_latency_stats()
    return GloveSnapshot(
        timestamp=time.time(),
        basic=base.basic,
        system=base.system,
        health_regs=base.health_regs,
        power_regs=power_regs,
        work_state=base.work_state,
        imu_status_regs=[*sensor_time_regs, imu_status],
        calib_ctrl_regs=base.calib_ctrl_regs,
        joint_status_regs=[
            *sensor_time_regs,
            joint_status,
            joint_valid_bits & 0xFFFF,
            (joint_valid_bits >> 16) & 0xFFFF,
        ],
        touch_status_regs=[
            *sensor_time_regs,
            touch_status,
            MODBUS_TOUCH_COUNT if sensor_data_valid else 0,
            MODBUS_TOUCH_DATA_REG_COUNT,
            0,
        ],
        imu_regs=imu_regs,
        joint_regs=joint_regs,
        touch_regs=touch_regs,
        poll_mode="sensors120",
        actual_hz=actual_hz,
        sensor_frame_id=displayed_frame_id,
        sensor_timestamp_us=displayed_timestamp_us,
        comm_requests=comm_stats.requests,
        comm_timeouts=comm_stats.timeouts,
        comm_retries=comm_stats.retries,
        comm_max_request_ms=comm_stats.max_request_ms,
        comm_last_timeout_start=comm_stats.last_timeout_start,
        sensor_data_valid=sensor_data_valid,
        sensor_invalid_reason=invalid_reason,
    )


def wait_sensor_120hz_deadline(stop_event: threading.Event, deadline: float) -> None:
    """Wait to an absolute deadline without stretching an 8.333 ms period."""
    while not stop_event.is_set():
        remaining = deadline - time.perf_counter()
        if remaining <= 0.0:
            return
        if remaining > SENSOR_120HZ_SPIN_GUARD_S:
            # 先阻塞较长空闲时间，最后4ms忙等，避免Windows定时等待多睡1～2ms。
            if stop_event.wait(remaining - SENSOR_120HZ_SPIN_GUARD_S):
                return


def decode_imu(imu_regs: list[int]) -> list[list[float]]:
    imus: list[list[float]] = []
    for imu_index in range(MODBUS_IMU_COUNT):
        base = imu_index * MODBUS_IMU_REGS_PER_UNIT
        values = []
        for field_index in range(MODBUS_IMU_FLOATS_PER_UNIT):
            pos = base + field_index * 2
            values.append(regs_to_f32_le_words(imu_regs[pos], imu_regs[pos + 1]))
        imus.append(values)
    return imus


def decode_joint(joint_regs: list[int]) -> list[float]:
    return [
        regs_to_f32_le_words(joint_regs[index * 2], joint_regs[index * 2 + 1])
        for index in range(MODBUS_JOINT_COUNT)
    ]


def identity_calibration_table() -> list[list[float]]:
    return [[1.0, 0.0, 0.0, 0.0] for _ in range(MODBUS_IMU_COUNT)]


def decode_calibration_table(regs: list[int]) -> list[list[float]]:
    table: list[list[float]] = []
    for imu_index in range(MODBUS_IMU_COUNT):
        base = imu_index * MODBUS_IMU_CALIB_REGS_PER_UNIT
        quat = []
        for component in range(4):
            pos = base + component * 2
            quat.append(regs_to_f32_le_words(regs[pos], regs[pos + 1]))
        table.append(quat)
    return table


def encode_calibration_table(table: list[list[float]]) -> list[int]:
    regs: list[int] = []
    if len(table) != MODBUS_IMU_COUNT:
        raise ModbusError("calibration table must contain 16 IMU rows")
    for quat in table:
        if len(quat) != 4:
            raise ModbusError("each calibration row must contain w,x,y,z")
        for value in quat:
            regs.extend(f32_to_regs_le_words(value))
    return regs


def read_calibration(
    client: ModbusRtuClient, slave: int, timeout_s: float
) -> tuple[list[list[float]], list[list[float]], list[int]]:
    c_regs = client.read_block_split(
        slave, REG_IMU_CALIB_C_START, MODBUS_IMU_CALIB_TABLE_REG_COUNT, timeout_s
    )
    m_regs = client.read_block_split(
        slave, REG_IMU_CALIB_M_START, MODBUS_IMU_CALIB_TABLE_REG_COUNT, timeout_s
    )
    ctrl = client.read_holding_registers(slave, REG_IMU_CALIB_CTRL_START, 6, timeout_s)
    return decode_calibration_table(c_regs), decode_calibration_table(m_regs), ctrl


def parse_calibration_csv(path: str) -> list[tuple[str, int, list[float]]]:
    rows: list[tuple[str, int, list[float]]] = []
    with open(path, newline="", encoding="utf-8-sig") as handle:
        reader = csv.DictReader(handle)
        required = {"table", "imu", "w", "x", "y", "z"}
        if reader.fieldnames is None or not required.issubset(set(reader.fieldnames)):
            raise ModbusError("CSV header must include table,imu,w,x,y,z")
        for row in reader:
            table = (row["table"] or "").strip().upper()
            if table not in {"C", "M"}:
                raise ModbusError("table column must be C or M")
            imu = int((row["imu"] or "").strip(), 0)
            if not (0 <= imu < MODBUS_IMU_COUNT):
                raise ModbusError("imu column must be 0..15")
            quat = [
                float(row["w"]),
                float(row["x"]),
                float(row["y"]),
                float(row["z"]),
            ]
            rows.append((table, imu, quat))
    if not rows:
        raise ModbusError("CSV has no calibration rows")
    return rows


def set_text(widget: tk.Text, text: str) -> None:
    widget.configure(state="normal")
    widget.delete("1.0", tk.END)
    widget.insert(tk.END, text)
    widget.configure(state="disabled")


def append_text(widget: tk.Text, text: str) -> None:
    widget.configure(state="normal")
    widget.insert(tk.END, text)
    widget.see(tk.END)
    widget.configure(state="disabled")


class ModbusMonitorApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("Glove Modbus485 Monitor")
        self.geometry("1160x760")
        self.minsize(940, 620)

        self.client = ModbusRtuClient()
        self.worker: threading.Thread | None = None
        self.stop_event = threading.Event()
        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.last_snapshot: GloveSnapshot | None = None
        self.last_calibration: tuple[list[list[float]], list[list[float]], list[int]] | None = None
        self.read_count = 0
        self.error_count = 0

        self.port_var = tk.StringVar()
        self.baud_var = tk.StringVar(value=str(DEFAULT_BAUD))
        self.slave_var = tk.StringVar(value=str(DEFAULT_SLAVE))
        self.timeout_var = tk.StringVar(value=str(DEFAULT_TIMEOUT_S))
        self.poll_var = tk.StringVar(value=str(DEFAULT_POLL_MS))
        self.status_var = tk.StringVar(value="Disconnected")
        self.manual_start_var = tk.StringVar(value="0x0000")
        self.manual_count_var = tk.StringVar(value="1")
        self.manual_single_addr_var = tk.StringVar(value="0x1256")
        self.manual_single_value_var = tk.StringVar(value="0x0001")
        self.manual_multi_start_var = tk.StringVar(value="0x1254")
        self.manual_multi_values_var = tk.StringVar(value="0xCA1B 0x0001 0x0001")
        self.manual_raw_var = tk.StringVar(value="01 03 00 00 00 0E")
        self.manual_raw_crc_var = tk.BooleanVar(value=True)
        self.calib_seq_var = tk.StringVar(value="auto")
        self.use_fc06_calib_var = tk.BooleanVar(value=False)
        self.test_read_var = tk.BooleanVar(value=True)
        self.test_write_var = tk.BooleanVar(value=False)
        self.test_negative_var = tk.BooleanVar(value=True)

        self._build_ui()
        self.refresh_ports()
        self.after(80, self._process_events)
        self.protocol("WM_DELETE_WINDOW", self._on_close)

    def _build_ui(self) -> None:
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("TButton", padding=(8, 4))
        style.configure("TNotebook.Tab", padding=(12, 5))
        style.configure("Header.TLabel", font=("Segoe UI", 10, "bold"))

        top = ttk.Frame(self, padding=8)
        top.pack(side=tk.TOP, fill=tk.X)

        ttk.Label(top, text="Port").grid(row=0, column=0, sticky=tk.W)
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=16)
        self.port_combo.grid(row=0, column=1, padx=(4, 8), sticky=tk.W)
        ttk.Button(top, text="Refresh", command=self.refresh_ports).grid(
            row=0, column=2, padx=(0, 12)
        )

        ttk.Label(top, text="Baud").grid(row=0, column=3, sticky=tk.W)
        ttk.Entry(top, textvariable=self.baud_var, width=10).grid(
            row=0, column=4, padx=(4, 12)
        )
        ttk.Label(top, text="Slave").grid(row=0, column=5, sticky=tk.W)
        ttk.Entry(top, textvariable=self.slave_var, width=5).grid(
            row=0, column=6, padx=(4, 12)
        )
        ttk.Label(top, text="Timeout s").grid(row=0, column=7, sticky=tk.W)
        ttk.Entry(top, textvariable=self.timeout_var, width=6).grid(
            row=0, column=8, padx=(4, 12)
        )
        ttk.Label(top, text="Poll ms").grid(row=0, column=9, sticky=tk.W)
        ttk.Entry(top, textvariable=self.poll_var, width=7).grid(
            row=0, column=10, padx=(4, 12)
        )

        ttk.Button(top, text="Connect", command=self.connect).grid(row=0, column=11, padx=2)
        ttk.Button(top, text="Disconnect", command=self.disconnect).grid(
            row=0, column=12, padx=2
        )
        ttk.Button(top, text="Read Once", command=self.read_once).grid(
            row=0, column=13, padx=2
        )
        ttk.Button(top, text="Start Poll", command=self.start_poll).grid(
            row=0, column=14, padx=2
        )
        ttk.Button(top, text="120Hz Sensors", command=self.start_sensor_120hz).grid(
            row=0, column=15, padx=2
        )
        ttk.Button(top, text="Stop", command=self.stop_poll).grid(row=0, column=16, padx=2)
        ttk.Button(top, text="Save CSV", command=self.save_csv).grid(
            row=0, column=17, padx=(10, 0)
        )

        status = ttk.Label(self, textvariable=self.status_var, anchor=tk.W, padding=(8, 0))
        status.pack(side=tk.TOP, fill=tk.X)

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=8)

        self.summary_text = self._add_text_tab("Summary")
        self.health_text = self._add_health_tab()
        self.power_text = self._add_text_tab("Power")
        self.imu_text = self._add_text_tab("IMU")
        self.joint_text = self._add_text_tab("Joint")
        self.touch_text = self._add_text_tab("Touch")
        self.calib_text = self._add_calibration_tab()
        self.manual_text = self._add_manual_tab()
        self.time_text = self._add_time_sync_tab()
        self.test_text = self._add_test_tab()
        self.raw_text = self._add_text_tab("Raw Regs")
        self.log_text = self._add_text_tab("Log")

    def _add_text_tab(self, title: str) -> tk.Text:
        frame = ttk.Frame(self.notebook)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(0, weight=1)
        text = tk.Text(frame, wrap="none", font=("Consolas", 10), state="disabled")
        ybar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=text.yview)
        xbar = ttk.Scrollbar(frame, orient=tk.HORIZONTAL, command=text.xview)
        text.configure(yscrollcommand=ybar.set, xscrollcommand=xbar.set)
        text.grid(row=0, column=0, sticky="nsew")
        ybar.grid(row=0, column=1, sticky="ns")
        xbar.grid(row=1, column=0, sticky="ew")
        self.notebook.add(frame, text=title)
        return text

    def _add_health_tab(self) -> tk.Text:
        frame = ttk.Frame(self.notebook)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        controls = ttk.Frame(frame, padding=6)
        controls.grid(row=0, column=0, sticky="ew")
        ttk.Button(
            controls,
            text="Clear History",
            command=self.clear_health_history,
        ).pack(side=tk.LEFT, padx=(0, 8))
        ttk.Label(
            controls,
            text="Clears Last error and counters only; active faults remain.",
        ).pack(side=tk.LEFT)

        text = tk.Text(frame, wrap="none", font=("Consolas", 10), state="disabled")
        ybar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=text.yview)
        xbar = ttk.Scrollbar(frame, orient=tk.HORIZONTAL, command=text.xview)
        text.configure(yscrollcommand=ybar.set, xscrollcommand=xbar.set)
        text.grid(row=1, column=0, sticky="nsew")
        ybar.grid(row=1, column=1, sticky="ns")
        xbar.grid(row=2, column=0, sticky="ew")
        self.notebook.add(frame, text="Health")
        return text

    def _add_calibration_tab(self) -> tk.Text:
        frame = ttk.Frame(self.notebook)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        controls = ttk.Frame(frame, padding=6)
        controls.grid(row=0, column=0, sticky="ew")
        ttk.Button(controls, text="Read Calib", command=self.read_calibration_once).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(controls, text="Load CSV & Apply", command=self.load_apply_calibration).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(controls, text="Save CSV", command=self.save_calibration_csv).pack(
            side=tk.LEFT, padx=(0, 12)
        )
        ttk.Button(controls, text="Reset Identity", command=self.reset_calibration).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Checkbutton(controls, text="Use 0x06 for control", variable=self.use_fc06_calib_var).pack(
            side=tk.LEFT, padx=(12, 6)
        )
        ttk.Label(controls, text="Seq").pack(side=tk.LEFT)
        ttk.Entry(controls, textvariable=self.calib_seq_var, width=8).pack(
            side=tk.LEFT, padx=(4, 0)
        )

        text = tk.Text(frame, wrap="none", font=("Consolas", 10), state="disabled")
        ybar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=text.yview)
        xbar = ttk.Scrollbar(frame, orient=tk.HORIZONTAL, command=text.xview)
        text.configure(yscrollcommand=ybar.set, xscrollcommand=xbar.set)
        text.grid(row=1, column=0, sticky="nsew")
        ybar.grid(row=1, column=1, sticky="ns")
        xbar.grid(row=2, column=0, sticky="ew")
        self.notebook.add(frame, text="Calibration")
        return text

    def _add_manual_tab(self) -> tk.Text:
        frame = ttk.Frame(self.notebook)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(4, weight=1)

        read_box = ttk.LabelFrame(frame, text="Read Holding Registers", padding=8)
        read_box.grid(row=0, column=0, sticky="ew", padx=6, pady=(6, 3))
        ttk.Label(read_box, text="Start").pack(side=tk.LEFT)
        ttk.Entry(read_box, textvariable=self.manual_start_var, width=10).pack(
            side=tk.LEFT, padx=(4, 10)
        )
        ttk.Label(read_box, text="Count").pack(side=tk.LEFT)
        ttk.Entry(read_box, textvariable=self.manual_count_var, width=6).pack(
            side=tk.LEFT, padx=(4, 10)
        )
        ttk.Button(read_box, text="Read", command=self.manual_read).pack(side=tk.LEFT)

        single_box = ttk.LabelFrame(frame, text="Write Single Register 0x06", padding=8)
        single_box.grid(row=1, column=0, sticky="ew", padx=6, pady=3)
        ttk.Label(single_box, text="Address").pack(side=tk.LEFT)
        ttk.Entry(single_box, textvariable=self.manual_single_addr_var, width=10).pack(
            side=tk.LEFT, padx=(4, 10)
        )
        ttk.Label(single_box, text="Value").pack(side=tk.LEFT)
        ttk.Entry(single_box, textvariable=self.manual_single_value_var, width=10).pack(
            side=tk.LEFT, padx=(4, 10)
        )
        ttk.Button(single_box, text="Write 0x06", command=self.manual_write_single).pack(
            side=tk.LEFT
        )

        multi_box = ttk.LabelFrame(frame, text="Write Multiple Registers 0x10", padding=8)
        multi_box.grid(row=2, column=0, sticky="ew", padx=6, pady=3)
        ttk.Label(multi_box, text="Start").pack(side=tk.LEFT)
        ttk.Entry(multi_box, textvariable=self.manual_multi_start_var, width=10).pack(
            side=tk.LEFT, padx=(4, 10)
        )
        ttk.Label(multi_box, text="Values").pack(side=tk.LEFT)
        ttk.Entry(multi_box, textvariable=self.manual_multi_values_var, width=46).pack(
            side=tk.LEFT, padx=(4, 10), fill=tk.X, expand=True
        )
        ttk.Button(multi_box, text="Write 0x10", command=self.manual_write_multiple).pack(
            side=tk.LEFT
        )

        raw_box = ttk.LabelFrame(frame, text="Raw RTU", padding=8)
        raw_box.grid(row=3, column=0, sticky="ew", padx=6, pady=3)
        ttk.Label(raw_box, text="Bytes").pack(side=tk.LEFT)
        ttk.Entry(raw_box, textvariable=self.manual_raw_var, width=54).pack(
            side=tk.LEFT, padx=(4, 10), fill=tk.X, expand=True
        )
        ttk.Checkbutton(raw_box, text="Append CRC", variable=self.manual_raw_crc_var).pack(
            side=tk.LEFT, padx=(0, 10)
        )
        ttk.Button(raw_box, text="Send Raw", command=self.manual_send_raw).pack(
            side=tk.LEFT
        )
        ttk.Button(raw_box, text="Clear", command=lambda: set_text(self.manual_text, "")).pack(
            side=tk.LEFT, padx=(6, 0)
        )

        text = tk.Text(frame, wrap="none", font=("Consolas", 10), state="disabled")
        ybar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=text.yview)
        xbar = ttk.Scrollbar(frame, orient=tk.HORIZONTAL, command=text.xview)
        text.configure(yscrollcommand=ybar.set, xscrollcommand=xbar.set)
        text.grid(row=4, column=0, sticky="nsew", padx=(6, 0), pady=(3, 6))
        ybar.grid(row=4, column=1, sticky="ns", pady=(3, 6))
        xbar.grid(row=5, column=0, sticky="ew", padx=(6, 0))
        self.notebook.add(frame, text="Manual I/O")
        return text

    def _add_time_sync_tab(self) -> tk.Text:
        frame = ttk.Frame(self.notebook)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        controls = ttk.Frame(frame, padding=8)
        controls.grid(row=0, column=0, sticky="ew")
        ttk.Button(controls, text="Read Time", command=self.time_sync_read).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(controls, text="Sync Host UTC", command=self.time_sync_host).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(controls, text="Clear", command=lambda: set_text(self.time_text, "")).pack(
            side=tk.LEFT
        )

        text = tk.Text(frame, wrap="none", font=("Consolas", 10), state="disabled")
        ybar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=text.yview)
        xbar = ttk.Scrollbar(frame, orient=tk.HORIZONTAL, command=text.xview)
        text.configure(yscrollcommand=ybar.set, xscrollcommand=xbar.set)
        text.grid(row=1, column=0, sticky="nsew")
        ybar.grid(row=1, column=1, sticky="ns")
        xbar.grid(row=2, column=0, sticky="ew")
        self.notebook.add(frame, text="Time Sync")
        return text

    def _add_test_tab(self) -> tk.Text:
        frame = ttk.Frame(self.notebook)
        frame.columnconfigure(0, weight=1)
        frame.rowconfigure(1, weight=1)

        controls = ttk.Frame(frame, padding=8)
        controls.grid(row=0, column=0, sticky="ew")
        ttk.Checkbutton(controls, text="Read tests", variable=self.test_read_var).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Checkbutton(controls, text="Negative tests", variable=self.test_negative_var).pack(
            side=tk.LEFT, padx=(0, 8)
        )
        ttk.Checkbutton(
            controls, text="Write tests (safe)", variable=self.test_write_var
        ).pack(side=tk.LEFT, padx=(0, 16))
        ttk.Button(controls, text="Run Tests", command=self.run_tests).pack(
            side=tk.LEFT, padx=(0, 6)
        )
        ttk.Button(controls, text="Clear", command=lambda: set_text(self.test_text, "")).pack(
            side=tk.LEFT
        )

        text = tk.Text(frame, wrap="none", font=("Consolas", 10), state="disabled")
        ybar = ttk.Scrollbar(frame, orient=tk.VERTICAL, command=text.yview)
        xbar = ttk.Scrollbar(frame, orient=tk.HORIZONTAL, command=text.xview)
        text.configure(yscrollcommand=ybar.set, xscrollcommand=xbar.set)
        text.grid(row=1, column=0, sticky="nsew")
        ybar.grid(row=1, column=1, sticky="ns")
        xbar.grid(row=2, column=0, sticky="ew")
        self.notebook.add(frame, text="Test Suite")
        return text

    def refresh_ports(self) -> None:
        if list_ports is None:
            self.port_combo["values"] = ()
            return
        ports = [port.device for port in list_ports.comports()]
        self.port_combo["values"] = ports
        if ports and not self.port_var.get():
            self.port_var.set(ports[0])

    def _read_settings(self) -> tuple[str, int, int, float, int]:
        port_name = self.port_var.get().strip()
        if not port_name:
            raise ModbusError("select a serial port")
        baud = int(self.baud_var.get().strip())
        slave = int(self.slave_var.get().strip(), 0)
        timeout_s = float(self.timeout_var.get().strip())
        poll_ms = int(self.poll_var.get().strip())
        if timeout_s <= 0:
            raise ModbusError("timeout must be positive")
        if poll_ms < 1:
            raise ModbusError("poll interval must be >= 1 ms")
        return port_name, baud, slave, timeout_s, poll_ms

    def connect(self) -> None:
        try:
            port_name, baud, _slave, timeout_s, _poll_ms = self._read_settings()
            self.client.open(port_name, baud, timeout_s)
            self.status_var.set(f"Connected {port_name} @ {baud} 8N1")
        except Exception as exc:
            messagebox.showerror("Connect failed", str(exc))

    def disconnect(self) -> None:
        self.stop_poll()
        self.client.close()
        self.status_var.set("Disconnected")

    def read_once(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        self.stop_event.clear()
        self.worker = threading.Thread(target=self._read_once_worker, daemon=True)
        self.worker.start()

    def _ensure_connected_for_worker(self) -> tuple[int, float]:
        port_name, baud, slave, timeout_s, _poll_ms = self._read_settings()
        if not self.client.is_open:
            raise ModbusError("connect first")
        if (
            port_name != self.client.connected_port_name
            or baud != self.client.connected_baud
        ):
            raise ModbusError("port/baud changed after connect; disconnect and connect again")
        return slave, timeout_s

    def _start_worker(self, target, *args: object) -> bool:
        if self.worker is not None and self.worker.is_alive():
            self.status_var.set("Busy: stop polling or wait for the current operation")
            return False
        self.stop_event.clear()
        self.worker = threading.Thread(target=target, args=args, daemon=True)
        self.worker.start()
        return True

    def _read_calib_command_settings(self) -> tuple[int, bool]:
        seq_text = self.calib_seq_var.get().strip().lower()
        if seq_text in {"", "auto"}:
            seq = int(time.time()) & 0xFFFF
        else:
            seq = parse_u16(seq_text, "seq")
        return seq, self.use_fc06_calib_var.get()

    def _write_calibration_command(
        self, slave: int, timeout_s: float, command: int, seq: int, use_fc06: bool
    ) -> None:
        if use_fc06:
            self.client.write_single_register(
                slave, REG_IMU_CALIB_MAGIC, IMU_CALIB_MAGIC_VALUE, timeout_s
            )
            self.client.write_single_register(slave, REG_IMU_CALIB_SEQ, seq, timeout_s)
            self.client.write_single_register(slave, REG_IMU_CALIB_COMMAND, command, timeout_s)
        else:
            self.client.write_multiple_registers(
                slave,
                REG_IMU_CALIB_CTRL_START,
                [IMU_CALIB_MAGIC_VALUE, command, seq],
                timeout_s,
            )

    def _queue_exchange_log(self, title: str) -> None:
        self.events.put(
            (
                "log",
                f"[{time.strftime('%H:%M:%S')}] {title}\n"
                f"{self.client.last_exchange_hex()}\n\n",
            )
        )

    def _read_once_worker(self) -> None:
        try:
            slave, timeout_s = self._ensure_connected_for_worker()
            snapshot = read_snapshot(self.client, slave, timeout_s)
            self.events.put(("snapshot", snapshot))
        except Exception as exc:
            self.events.put(("error", exc))

    def clear_health_history(self) -> None:
        if not messagebox.askyesno(
            "Clear health history",
            "Clear Last error, error sequence/count, uptime and UART detail?\n"
            "Active faults will not be cleared.",
        ):
            return
        self._start_worker(self._clear_health_history_worker)

    def _clear_health_history_worker(self) -> None:
        try:
            slave, timeout_s = self._ensure_connected_for_worker()
            seq = int(time.time() * 1000.0) & 0xFFFF
            self.client.write_multiple_registers(
                slave,
                REG_CMD_START,
                [CMD_HEALTH_CLEAR_HISTORY, CMD_HEALTH_CLEAR_MAGIC, seq],
                timeout_s,
            )
            ack, ack_seq, error = self.client.read_holding_registers(
                slave, REG_CMD_ACK_START, REG_CMD_ACK_COUNT, timeout_s
            )
            if (ack != CMD_ACK_OK) or (ack_seq != seq) or (error != 0):
                raise ModbusError(
                    f"clear history rejected: ack=0x{ack:04X} "
                    f"ack_seq=0x{ack_seq:04X} expected=0x{seq:04X} "
                    f"error=0x{error:04X}"
                )
            snapshot = read_snapshot(self.client, slave, timeout_s)
            self.events.put(("snapshot", snapshot))
            self.events.put(
                (
                    "log",
                    f"[{time.strftime('%H:%M:%S')}] health history cleared "
                    f"seq=0x{seq:04X}\n\n",
                )
            )
        except Exception as exc:
            self.events.put(("error", exc))

    def read_calibration_once(self) -> None:
        self._start_worker(self._read_calibration_worker)

    def _read_calibration_worker(self) -> None:
        try:
            slave, timeout_s = self._ensure_connected_for_worker()
            c_table, m_table, ctrl = read_calibration(self.client, slave, timeout_s)
            self.events.put(("calibration", (c_table, m_table, ctrl)))
        except Exception as exc:
            self.events.put(("error", exc))

    def load_apply_calibration(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        path = filedialog.askopenfilename(
            title="Load calibration CSV",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if not path:
            return
        try:
            seq, use_fc06 = self._read_calib_command_settings()
        except Exception as exc:
            messagebox.showerror("Invalid calibration command", str(exc))
            return
        self._start_worker(self._load_apply_calibration_worker, path, seq, use_fc06)

    def _load_apply_calibration_worker(self, path: str, seq: int, use_fc06: bool) -> None:
        try:
            slave, timeout_s = self._ensure_connected_for_worker()
            c_table, m_table, _ctrl = read_calibration(self.client, slave, timeout_s)
            for table_name, imu, quat in parse_calibration_csv(path):
                if table_name == "C":
                    c_table[imu] = quat
                else:
                    m_table[imu] = quat
            validate_calibration_table(c_table, "C")
            validate_calibration_table(m_table, "M")
            self.client.write_block_split(
                slave, REG_IMU_CALIB_C_START, encode_calibration_table(c_table), timeout_s
            )
            self.client.write_block_split(
                slave, REG_IMU_CALIB_M_START, encode_calibration_table(m_table), timeout_s
            )
            self._write_calibration_command(slave, timeout_s, IMU_CALIB_CMD_APPLY, seq, use_fc06)
            self._queue_exchange_log(
                f"apply calibration seq={seq} via {'0x06' if use_fc06 else '0x10'}"
            )
            c_table, m_table, ctrl = read_calibration(self.client, slave, timeout_s)
            self.events.put(("calibration", (c_table, m_table, ctrl)))
        except Exception as exc:
            self.events.put(("error", exc))

    def reset_calibration(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        try:
            seq, use_fc06 = self._read_calib_command_settings()
        except Exception as exc:
            messagebox.showerror("Invalid calibration command", str(exc))
            return
        self._start_worker(self._reset_calibration_worker, seq, use_fc06)

    def _reset_calibration_worker(self, seq: int, use_fc06: bool) -> None:
        try:
            slave, timeout_s = self._ensure_connected_for_worker()
            self._write_calibration_command(
                slave, timeout_s, IMU_CALIB_CMD_RESET_IDENTITY, seq, use_fc06
            )
            self._queue_exchange_log(
                f"reset identity seq={seq} via {'0x06' if use_fc06 else '0x10'}"
            )
            c_table, m_table, ctrl = read_calibration(self.client, slave, timeout_s)
            self.events.put(("calibration", (c_table, m_table, ctrl)))
        except Exception as exc:
            self.events.put(("error", exc))

    def manual_read(self) -> None:
        try:
            start = parse_u16(self.manual_start_var.get(), "start")
            count = int(self.manual_count_var.get().strip(), 0)
            if not (1 <= count <= 125):
                raise ModbusError("count must be 1..125")
        except Exception as exc:
            messagebox.showerror("Invalid manual read", str(exc))
            return
        self._start_worker(self._manual_read_worker, start, count)

    def _manual_read_worker(self, start: int, count: int) -> None:
        try:
            slave, timeout_s = self._ensure_connected_for_worker()
            regs = self.client.read_holding_registers(slave, start, count, timeout_s)
            lines = [
                f"[{time.strftime('%H:%M:%S')}] FC03 read 0x{start:04X}+{count}",
                self._format_regs("registers", start, regs),
            ]
            floats = self._format_float_pairs(start, regs)
            if floats:
                lines.extend(("", floats))
            self.events.put(("manual", "\n".join(lines) + "\n\n"))
            self._queue_exchange_log(f"manual FC03 0x{start:04X}+{count}")
        except Exception as exc:
            self.events.put(("error", exc))

    def manual_write_single(self) -> None:
        try:
            addr = parse_u16(self.manual_single_addr_var.get(), "address")
            value = parse_u16(self.manual_single_value_var.get(), "value")
        except Exception as exc:
            messagebox.showerror("Invalid 0x06 write", str(exc))
            return
        self._start_worker(self._manual_write_single_worker, addr, value)

    def _manual_write_single_worker(self, addr: int, value: int) -> None:
        try:
            slave, timeout_s = self._ensure_connected_for_worker()
            self.client.write_single_register(slave, addr, value, timeout_s)
            self.events.put(
                (
                    "manual",
                    f"[{time.strftime('%H:%M:%S')}] FC06 write "
                    f"0x{addr:04X}=0x{value:04X} OK\n\n",
                )
            )
            self._queue_exchange_log(f"manual FC06 0x{addr:04X}=0x{value:04X}")
        except Exception as exc:
            self.events.put(("error", exc))

    def manual_write_multiple(self) -> None:
        try:
            start = parse_u16(self.manual_multi_start_var.get(), "start")
            regs = parse_u16_list(self.manual_multi_values_var.get())
            if len(regs) > 123:
                raise ModbusError("0x10 can write at most 123 registers")
        except Exception as exc:
            messagebox.showerror("Invalid 0x10 write", str(exc))
            return
        self._start_worker(self._manual_write_multiple_worker, start, regs)

    def _manual_write_multiple_worker(self, start: int, regs: list[int]) -> None:
        try:
            slave, timeout_s = self._ensure_connected_for_worker()
            self.client.write_multiple_registers(slave, start, regs, timeout_s)
            self.events.put(
                (
                    "manual",
                    f"[{time.strftime('%H:%M:%S')}] FC10 write "
                    f"0x{start:04X}+{len(regs)} OK\n"
                    f"{self._format_regs('written', start, regs)}\n\n",
                )
            )
            self._queue_exchange_log(f"manual FC10 0x{start:04X}+{len(regs)}")
        except Exception as exc:
            self.events.put(("error", exc))

    def manual_send_raw(self) -> None:
        try:
            frame = parse_hex_bytes(self.manual_raw_var.get())
            if self.manual_raw_crc_var.get():
                frame = append_crc(frame)
        except Exception as exc:
            messagebox.showerror("Invalid raw frame", str(exc))
            return
        self._start_worker(self._manual_send_raw_worker, frame)

    def _manual_send_raw_worker(self, frame: bytes) -> None:
        try:
            _slave, timeout_s = self._ensure_connected_for_worker()
            rx = self.client.transceive_raw(frame, timeout_s)
            rx_text = " ".join(f"{byte:02X}" for byte in rx) or "<empty>"
            self.events.put(
                (
                    "manual",
                    f"[{time.strftime('%H:%M:%S')}] Raw RTU sent {len(frame)} bytes, "
                    f"received {len(rx)} bytes\nRX {rx_text}\n\n",
                )
            )
            self._queue_exchange_log("manual raw RTU")
        except Exception as exc:
            self.events.put(("error", exc))

    def time_sync_read(self) -> None:
        self._start_worker(self._time_sync_read_worker)

    def _time_sync_read_worker(self) -> None:
        try:
            slave, timeout_s = self._ensure_connected_for_worker()
            regs = self.client.read_holding_registers(
                slave, REG_UTC_TIMESTAMP_US, MODBUS_ROS_TIME_REG_COUNT * 3, timeout_s
            )
            host_ns = time.time_ns()
            self.events.put(("time_log", self._format_time_sync_report(regs, host_ns)))
            self._queue_exchange_log("time sync read")
        except Exception as exc:
            self.events.put(("error", exc))

    def time_sync_host(self) -> None:
        self._start_worker(self._time_sync_host_worker)

    def _time_sync_host_worker(self) -> None:
        try:
            slave, timeout_s = self._ensure_connected_for_worker()
            host_ns_before = time.time_ns()
            sec = host_ns_before // 1_000_000_000
            nsec = host_ns_before % 1_000_000_000
            regs = ros_time_to_regs_le_words(sec, nsec)
            self.client.write_multiple_registers(
                slave, REG_TIME_SYNC_UTC_US, regs, timeout_s
            )
            host_ns_after_write = time.time_ns()
            self._queue_exchange_log("time sync write host UTC")

            read_regs = self.client.read_holding_registers(
                slave, REG_UTC_TIMESTAMP_US, MODBUS_ROS_TIME_REG_COUNT * 3, timeout_s
            )
            host_ns_after_read = time.time_ns()
            midpoint_ns = (host_ns_after_write + host_ns_after_read) // 2
            lines = [
                f"[{time.strftime('%H:%M:%S')}] Sync Host UTC",
                f"wrote host UTC  : {format_host_time_ns(host_ns_before)}",
                f"write latency   : {(host_ns_after_write - host_ns_before) / 1_000_000:.3f} ms",
                "",
                self._format_time_sync_report(read_regs, midpoint_ns).rstrip(),
            ]
            self.events.put(("time_log", "\n".join(lines) + "\n\n"))
            self._queue_exchange_log("time sync read after write")
        except Exception as exc:
            self.events.put(("error", exc))

    @staticmethod
    def _format_time_sync_report(regs: list[int], host_ns: int) -> str:
        if len(regs) < MODBUS_ROS_TIME_REG_COUNT * 3:
            raise ModbusError("time sync read needs 12 registers")

        utc_regs = regs[0:4]
        local_regs = regs[4:8]
        last_sync_regs = regs[8:12]
        utc_ns = ros_time_to_ns(utc_regs)
        delta = utc_ns - host_ns
        status = "not synced" if utc_ns == 0 else format_delta_ns(delta)

        return "\n".join(
            [
                f"[{time.strftime('%H:%M:%S')}] Read Time",
                f"device UTC      : {format_ros_time(utc_regs)}",
                f"device local    : {format_ros_time(local_regs)}",
                f"last sync UTC   : {format_ros_time(last_sync_regs)}",
                f"host UTC        : {format_host_time_ns(host_ns)}",
                f"device-host     : {status}",
                f"raw regs 0x0002 : {' '.join(f'{value:04X}' for value in regs)}",
                "",
            ]
        )

    def run_tests(self) -> None:
        do_read = self.test_read_var.get()
        do_negative = self.test_negative_var.get()
        do_write = self.test_write_var.get()
        if not (do_read or do_negative or do_write):
            messagebox.showinfo("Run Tests", "Select at least one test group")
            return
        set_text(self.test_text, "")
        self._start_worker(self._run_tests_worker, do_read, do_negative, do_write)

    def _run_tests_worker(
        self, do_read: bool, do_negative: bool, do_write: bool
    ) -> None:
        try:
            slave, timeout_s = self._ensure_connected_for_worker()
        except Exception as exc:
            self.events.put(("error", exc))
            return

        pass_count = 0
        fail_count = 0
        self.events.put(
            (
                "test_log",
                f"[{time.strftime('%H:%M:%S')}] Modbus test started "
                f"slave={slave} timeout={timeout_s:.3f}s\n",
            )
        )

        def step(label: str, func, expect_exception: bool = False) -> None:
            nonlocal pass_count, fail_count
            started = time.monotonic()
            try:
                detail = func()
                elapsed_ms = (time.monotonic() - started) * 1000.0
                if expect_exception:
                    fail_count += 1
                    self.events.put(
                        (
                            "test_log",
                            f"[FAIL] {label} ({elapsed_ms:.1f} ms) "
                            f"expected exception, got {detail}\n",
                        )
                    )
                    return
                pass_count += 1
                suffix = f" - {detail}" if detail else ""
                self.events.put(
                    ("test_log", f"[PASS] {label} ({elapsed_ms:.1f} ms){suffix}\n")
                )
            except Exception as exc:
                elapsed_ms = (time.monotonic() - started) * 1000.0
                if expect_exception and "modbus exception" in str(exc).lower():
                    pass_count += 1
                    self.events.put(
                        (
                            "test_log",
                            f"[PASS] {label} ({elapsed_ms:.1f} ms) rejected: {exc}\n",
                        )
                    )
                    return
                fail_count += 1
                self.events.put(
                    ("test_log", f"[FAIL] {label} ({elapsed_ms:.1f} ms): {exc}\n")
                )

        if do_read:
            step(
                "FC03 basic status 0x0000+14",
                lambda: f"{len(self.client.read_holding_registers(slave, REG_BASIC_STATUS_START, REG_BASIC_STATUS_COUNT, timeout_s))} regs",
            )
            step(
                "FC03 ROS time block 0x0002+12",
                lambda: self._test_read_time_block(slave, timeout_s),
            )
            step(
                "FC03 system status 0x0040+10",
                lambda: f"{len(self.client.read_holding_registers(slave, REG_SYSTEM_STATUS_START, REG_SYSTEM_STATUS_COUNT, timeout_s))} regs",
            )
            step(
                "FC03 health status 0x004A+22",
                lambda: self._test_read_health_status(slave, timeout_s),
            )
            step(
                "FC03 power status 0x0060+18",
                lambda: self._test_read_power_status(slave, timeout_s),
            )
            step(
                "FC03 work state 0x0500",
                lambda: f"0x{self.client.read_holding_registers(slave, REG_WORK_STATE, 1, timeout_s)[0]:04X}",
            )
            step(
                "FC03 IMU raw first unit 0x1000+20",
                lambda: self._test_read_first_imu(slave, timeout_s),
            )
            step(
                "FC03 calibration control 0x1254+6",
                lambda: self._test_read_calib_ctrl(slave, timeout_s),
            )
            step(
                "FC03 joint status 0x1340+7",
                lambda: self._test_read_joint_status(slave, timeout_s),
            )
            step(
                "FC03 touch status 0x2080+8",
                lambda: self._test_read_touch_status(slave, timeout_s),
            )

        if do_write:
            step(
                "FC06 write calibration seq back to same value",
                lambda: self._test_write_single_same(slave, timeout_s, REG_IMU_CALIB_SEQ),
            )
            step(
                "FC06 write C table first word back to same value",
                lambda: self._test_write_single_same(slave, timeout_s, REG_IMU_CALIB_C_START),
            )
            step(
                "FC10 write calibration seq back to same value",
                lambda: self._test_write_multiple_same(slave, timeout_s, REG_IMU_CALIB_SEQ, 1),
            )
            step(
                "FC10 write M table first quat back to same value",
                lambda: self._test_write_multiple_same(
                    slave, timeout_s, REG_IMU_CALIB_M_START, 8
                ),
            )

        if do_negative:
            step(
                "FC03 illegal address 0xFFFF",
                lambda: self.client.read_holding_registers(slave, 0xFFFF, 1, timeout_s),
                expect_exception=True,
            )
            if do_write:
                step(
                    "FC06 illegal address 0xFFFF",
                    lambda: self.client.write_single_register(slave, 0xFFFF, 0, timeout_s),
                    expect_exception=True,
                )
                step(
                    "FC10 illegal address 0xFFFF",
                    lambda: self.client.write_multiple_registers(slave, 0xFFFF, [0], timeout_s),
                    expect_exception=True,
                )

        total = pass_count + fail_count
        self.events.put(
            (
                "test_log",
                f"Done: {pass_count}/{total} passed, {fail_count} failed\n\n",
            )
        )

    def _test_read_first_imu(self, slave: int, timeout_s: float) -> str:
        regs = self.client.read_holding_registers(
            slave, REG_IMU_DATA_START, MODBUS_IMU_REGS_PER_UNIT, timeout_s
        )
        values = [
            regs_to_f32_le_words(regs[index], regs[index + 1])
            for index in range(0, len(regs), 2)
        ]
        require_finite = all(math.isfinite(value) for value in values)
        if not require_finite:
            raise ModbusError("IMU0 contains NaN or Inf")
        return f"quat=({values[6]:.4g},{values[7]:.4g},{values[8]:.4g},{values[9]:.4g})"

    def _test_read_time_block(self, slave: int, timeout_s: float) -> str:
        regs = self.client.read_holding_registers(
            slave, REG_UTC_TIMESTAMP_US, MODBUS_ROS_TIME_REG_COUNT * 3, timeout_s
        )
        return f"utc={format_ros_time(regs[0:4])} last_sync={format_ros_time(regs[8:12])}"

    def _test_read_calib_ctrl(self, slave: int, timeout_s: float) -> str:
        regs = self.client.read_holding_registers(
            slave, REG_IMU_CALIB_CTRL_START, REG_IMU_CALIB_CTRL_COUNT, timeout_s
        )
        return (
            f"cmd=0x{regs[1]:04X} status=0x{regs[3]:04X}"
            f"({calib_status_name(regs[3])}) last_seq={regs[5]}"
        )

    def _test_read_joint_status(self, slave: int, timeout_s: float) -> str:
        regs = self.client.read_holding_registers(slave, REG_JOINT_TIMESTAMP_US, 7, timeout_s)
        flags = regs[4]
        valid = regs[5] | (regs[6] << 16)
        return f"flags=0x{flags:04X}({format_flags(flags, JOINT_FLAG_NAMES)}) valid=0x{valid:08X}"

    def _test_read_touch_status(self, slave: int, timeout_s: float) -> str:
        regs = self.client.read_holding_registers(slave, REG_TOUCH_TIMESTAMP_US, 8, timeout_s)
        flags = regs[4]
        return (
            f"flags=0x{flags:04X}({format_flags(flags, TOUCH_FLAG_NAMES)}) "
            f"count/cap={regs[5]}/{regs[6]}"
        )

    def _test_read_power_status(self, slave: int, timeout_s: float) -> str:
        regs = self.client.read_holding_registers(
            slave, REG_POWER_STATUS_START, REG_POWER_STATUS_COUNT, timeout_s
        )
        power = decode_power(regs)
        return (
            f"VBAT={power.battery_voltage_v:.3f}V "
            f"IBAT={power.battery_current_a:+.3f}A "
            f"SOC={power.soc_percent:.2f}%(uncalibrated) flags=0x{power.flags:04X}"
        )

    def _test_read_health_status(self, slave: int, timeout_s: float) -> str:
        regs = self.client.read_holding_registers(
            slave, REG_HEALTH_STATUS_START, REG_HEALTH_STATUS_COUNT, timeout_s
        )
        health = decode_health(regs)
        error_name, _action = health_error_text(health.current_error)
        return (
            f"version=0x{health.version:04X} "
            f"state={HEALTH_STATE_NAMES.get(health.state, health.state)} "
            f"error=0x{health.current_error:04X}({error_name})"
        )

    def _test_write_single_same(self, slave: int, timeout_s: float, addr: int) -> str:
        original = self.client.read_holding_registers(slave, addr, 1, timeout_s)[0]
        self.client.write_single_register(slave, addr, original, timeout_s)
        read_back = self.client.read_holding_registers(slave, addr, 1, timeout_s)[0]
        if read_back != original:
            raise ModbusError(
                f"readback mismatch 0x{addr:04X}: 0x{read_back:04X} != 0x{original:04X}"
            )
        return f"0x{addr:04X}=0x{read_back:04X}"

    def _test_write_multiple_same(
        self, slave: int, timeout_s: float, start: int, count: int
    ) -> str:
        original = self.client.read_holding_registers(slave, start, count, timeout_s)
        self.client.write_multiple_registers(slave, start, original, timeout_s)
        read_back = self.client.read_holding_registers(slave, start, count, timeout_s)
        if read_back != original:
            raise ModbusError(f"readback mismatch at 0x{start:04X}+{count}")
        return f"0x{start:04X}+{count} restored"

    def start_poll(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        try:
            self._read_settings()
        except Exception as exc:
            messagebox.showerror("Invalid settings", str(exc))
            return
        if not self.client.is_open:
            self.connect()
            if not self.client.is_open:
                return
        self.stop_event.clear()
        self.worker = threading.Thread(target=self._poll_worker, daemon=True)
        self.worker.start()
        self.status_var.set(self.status_var.get() + " | polling")

    def start_sensor_120hz(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            return
        try:
            _port_name, _baud, slave, timeout_s, _poll_ms = self._read_settings()
        except Exception as exc:
            messagebox.showerror("Invalid settings", str(exc))
            return
        if not self.client.is_open:
            self.connect()
            if not self.client.is_open:
                return
        self.stop_event.clear()
        self.worker = threading.Thread(
            target=self._sensor_120hz_worker,
            args=(slave, timeout_s),
            daemon=True,
        )
        self.worker.start()
        self.status_var.set(self.status_var.get() + " | 120Hz IMU+Joint+Touch")

    def stop_poll(self) -> None:
        self.stop_event.set()

    def _poll_worker(self) -> None:
        while not self.stop_event.is_set():
            try:
                _port_name, _baud, slave, timeout_s, poll_ms = self._read_settings()
                snapshot = read_snapshot(self.client, slave, timeout_s)
                self.events.put(("snapshot", snapshot))
                self.stop_event.wait(poll_ms / 1000.0)
            except Exception as exc:
                self.events.put(("error", exc))
                self.stop_event.wait(0.5)

    def _sensor_120hz_worker(self, slave: int, timeout_s: float) -> None:
        previous = self.last_snapshot
        next_deadline = time.perf_counter()
        rate_started = next_deadline
        sensor_rate_start_frame_id: int | None = None
        sensor_rate_started = next_deadline
        last_sensor_frame_id: int | None = None
        last_new_sensor_frame_time = next_deadline
        rate_count = 0
        actual_hz = 0.0
        sensor_hz = 0.0
        duplicate_responses = 0
        next_health_poll = next_deadline

        try:
            self.client.prepare_low_latency_poll()
        except Exception as exc:
            self.events.put(("error", exc))
            return

        try:
            while not self.stop_event.is_set():
                try:
                    snapshot = read_sensor_snapshot_120hz(
                        self.client, slave, timeout_s, previous, actual_hz
                    )
                    rate_count += 1
                    now = time.perf_counter()

                    if now >= next_health_poll:
                        try:
                            # 健康块低频读取，避免改变FC41帧并尽量不扰动120Hz采集。
                            snapshot.health_regs = self.client.read_holding_registers(
                                slave,
                                REG_HEALTH_STATUS_START,
                                REG_HEALTH_STATUS_COUNT,
                                min(timeout_s, 0.02),
                                low_latency=True,
                            )
                        except Exception:
                            self.client.recover_low_latency_poll()
                            snapshot.health_regs = list(previous.health_regs) if previous else [
                                0
                            ] * REG_HEALTH_STATUS_COUNT
                        next_health_poll = now + 0.2
                    previous = snapshot

                    if snapshot.sensor_data_valid:
                        if last_sensor_frame_id == snapshot.sensor_frame_id:
                            duplicate_responses += 1
                        else:
                            last_new_sensor_frame_time = now
                        last_sensor_frame_id = snapshot.sensor_frame_id

                        if sensor_rate_start_frame_id is None:
                            sensor_rate_start_frame_id = snapshot.sensor_frame_id
                            sensor_rate_started = now

                        sensor_rate_elapsed = now - sensor_rate_started
                        if sensor_rate_elapsed >= 1.0:
                            frame_delta = (
                                snapshot.sensor_frame_id - sensor_rate_start_frame_id
                            ) & 0xFFFFFFFF
                            if frame_delta <= 0x7FFFFFFF:
                                # 用帧号增量统计完整传感器帧率，避免依赖设备时间戳精度。
                                sensor_hz = frame_delta / sensor_rate_elapsed
                            else:
                                # 从机复位导致帧号回退时重新建立统计窗口。
                                sensor_hz = 0.0
                            sensor_rate_start_frame_id = snapshot.sensor_frame_id
                            sensor_rate_started = now
                    else:
                        sensor_hz = 0.0
                        last_sensor_frame_id = None
                        sensor_rate_start_frame_id = None

                    if (now - last_new_sensor_frame_time) >= 1.0:
                        sensor_hz = 0.0

                    rate_elapsed = now - rate_started
                    if rate_elapsed >= 1.0:
                        actual_hz = rate_count / rate_elapsed
                        rate_count = 0
                        rate_started = now
                    snapshot.actual_hz = actual_hz
                    snapshot.sensor_hz = sensor_hz
                    snapshot.duplicate_responses = duplicate_responses
                    self.events.put(("snapshot", snapshot))

                    next_deadline += SENSOR_120HZ_PERIOD_S
                    now = time.perf_counter()
                    if next_deadline < now:
                        next_deadline = now
                    wait_sensor_120hz_deadline(self.stop_event, next_deadline)
                except Exception as exc:
                    self.events.put(("error", exc))
                    self.client.recover_low_latency_poll()
                    # 异常后只等待一个短分片，避免额外空转完整8.333ms周期。
                    next_deadline = time.perf_counter() + SENSOR_120HZ_PERIOD_S
                    self.stop_event.wait(SENSOR_120HZ_READ_SLICE_S)
        finally:
            self.client.finish_low_latency_poll()

    def _process_events(self) -> None:
        latest_snapshot: Optional[GloveSnapshot] = None
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "snapshot":
                    self.read_count += 1
                    self.last_snapshot = payload  # type: ignore[assignment]
                    # 高频采集时只绘制本轮消息中的最新帧，避免界面刷新堵塞通信线程。
                    latest_snapshot = payload  # type: ignore[assignment]
                elif kind == "calibration":
                    c_table, m_table, ctrl = payload  # type: ignore[misc]
                    self._render_calibration(c_table, m_table, ctrl)
                elif kind == "manual":
                    append_text(self.manual_text, str(payload))
                elif kind == "time_log":
                    append_text(self.time_text, str(payload))
                elif kind == "test_log":
                    append_text(self.test_text, str(payload))
                elif kind == "log":
                    append_text(self.log_text, str(payload))
                elif kind == "error":
                    self.error_count += 1
                    self.status_var.set(f"Error #{self.error_count}: {payload}")
                    append_text(
                        self.log_text,
                        f"[{time.strftime('%H:%M:%S')}] ERROR {payload}\n\n",
                    )
        except queue.Empty:
            pass
        if latest_snapshot is not None:
            self._render_snapshot(latest_snapshot)
        self.after(80, self._process_events)

    def _render_snapshot(self, snapshot: GloveSnapshot) -> None:
        power = decode_power(snapshot.power_regs)
        health = decode_health(snapshot.health_regs)
        health_state = HEALTH_STATE_NAMES.get(health.state, f"UNKNOWN({health.state})")
        current_error_name, current_action = health_error_text(health.current_error)
        last_error_name, _last_action = health_error_text(health.last_error)
        utc_time = format_ros_time(snapshot.basic[2:6])
        local_time = format_ros_time(snapshot.basic[6:10])
        last_sync_time = format_ros_time(snapshot.basic[10:14])
        imu_time = format_ros_time(snapshot.imu_status_regs[0:4])
        joint_time = format_ros_time(snapshot.joint_status_regs[0:4])
        touch_time = format_ros_time(snapshot.touch_status_regs[0:4])
        imu_status = snapshot.imu_status_regs[4]
        calib_magic = snapshot.calib_ctrl_regs[0] if len(snapshot.calib_ctrl_regs) > 0 else 0
        calib_command = snapshot.calib_ctrl_regs[1] if len(snapshot.calib_ctrl_regs) > 1 else 0
        calib_seq = snapshot.calib_ctrl_regs[2] if len(snapshot.calib_ctrl_regs) > 2 else 0
        calib_status = snapshot.calib_ctrl_regs[3] if len(snapshot.calib_ctrl_regs) > 3 else 0
        calib_error = snapshot.calib_ctrl_regs[4] if len(snapshot.calib_ctrl_regs) > 4 else 0
        calib_last_seq = snapshot.calib_ctrl_regs[5] if len(snapshot.calib_ctrl_regs) > 5 else 0
        joint_flags = snapshot.joint_status_regs[4]
        joint_valid = snapshot.joint_status_regs[5] | (snapshot.joint_status_regs[6] << 16)
        touch_flags = snapshot.touch_status_regs[4]
        touch_count = snapshot.touch_status_regs[5]
        touch_capacity = snapshot.touch_status_regs[6]

        if snapshot.poll_mode == "sensors120":
            last_timeout = (
                f" last=0x{snapshot.comm_last_timeout_start:04X}"
                if snapshot.comm_timeouts > 0
                else ""
            )
            poll_text = (
                f"120Hz Sensors comm={snapshot.actual_hz:.1f}Hz "
                f"sensor={snapshot.sensor_hz:.1f}Hz frame={snapshot.sensor_frame_id} "
                f"dup={snapshot.duplicate_responses} "
                f"timeout={snapshot.comm_timeouts} retry={snapshot.comm_retries} "
                f"max={snapshot.comm_max_request_ms:.2f}ms{last_timeout}"
            )
        else:
            poll_text = "Full poll"
        if snapshot.sensor_data_valid:
            data_status = "DATA READY"
        else:
            data_status = f"DATA PAUSED: {snapshot.sensor_invalid_reason}"
        if health.version == 0:
            health_banner = "HEALTH UNAVAILABLE"
        elif health.state == 4:
            health_banner = (
                f"RECOVERING: {RECOVERY_STAGE_NAMES.get(health.recovery_stage, health.recovery_stage)}"
            )
        elif health.current_error != 0:
            health_banner = f"{health_state}: {current_error_name}"
        else:
            health_banner = health_state
        self.status_var.set(
            f"{health_banner} | {data_status} | {poll_text} reads={self.read_count} "
            f"errors={self.error_count} "
            f"host={time.strftime('%H:%M:%S', time.localtime(snapshot.timestamp))}"
        )

        summary = [
            f"Read count        : {self.read_count}",
            f"Host time         : {time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(snapshot.timestamp))}",
            f"Communication Hz  : {snapshot.actual_hz:.3f}",
            f"Sensor frame Hz   : {snapshot.sensor_hz:.3f}",
            f"Sensor frame id   : {snapshot.sensor_frame_id}",
            f"Sensor timestamp  : {snapshot.sensor_timestamp_us} us",
            f"Sensor data       : {'VALID' if snapshot.sensor_data_valid else 'INVALID'}",
            f"Data reason       : {snapshot.sensor_invalid_reason}",
            f"Duplicate replies : {snapshot.duplicate_responses}",
            f"Slave addr reg    : {snapshot.basic[0]}",
            f"Baud code reg     : {snapshot.basic[1]}",
            f"UTC time          : {utc_time}",
            f"Local time        : {local_time}",
            f"Last sync UTC     : {last_sync_time}",
            f"Work state        : 0x{snapshot.work_state:04X}",
            "",
            f"Health state      : {health.state} ({health_state})",
            f"Current error     : 0x{health.current_error:04X} ({current_error_name})",
            f"Current source    : {HEALTH_SOURCE_NAMES.get(health.current_source, health.current_source)} "
            f"target={format_health_target(health.current_source, health.current_target)}",
            f"Recovery stage    : {health.recovery_stage} "
            f"({RECOVERY_STAGE_NAMES.get(health.recovery_stage, 'unknown')})",
            f"Recommended action: {current_action}",
            "",
            f"Legacy sys state  : 0x{snapshot.system[0]:04X}",
            f"Work mode         : 0x{snapshot.system[1]:04X}",
            f"Log state         : 0x{snapshot.system[2]:04X}",
            f"SD state          : 0x{snapshot.system[3]:04X}",
            f"Sensor state      : 0x{snapshot.system[4]:04X}",
            f"Comm state        : 0x{snapshot.system[5]:04X}",
            f"Reset cause       : 0x{snapshot.system[6]:04X} "
            f"({format_flags(snapshot.system[6], RESET_CAUSE_NAMES)})",
            f"Watchdog status   : 0x{snapshot.system[7]:04X} "
            f"({format_flags(snapshot.system[7], WATCHDOG_STATUS_NAMES)})",
            "",
            f"Battery voltage   : {power.battery_voltage_v:.3f} V",
            f"Battery current   : {power.battery_current_a:+.3f} A (+charge)",
            f"Battery SOC       : {power.soc_percent:.2f} % (uncalibrated estimate)",
            f"Power state       : {power.system_state} ({POWER_STATE_NAMES.get(power.system_state, 'UNKNOWN_VALUE')})",
            f"Charge state      : {power.charge_state} ({CHARGE_STATE_NAMES.get(power.charge_state, 'UNKNOWN_VALUE')})",
            f"Power flags       : 0x{power.flags:04X} ({format_flags(power.flags, POWER_FLAG_NAMES)})",
            f"Power fault       : 0x{power.fault_code:04X}",
            f"BQ diagnostic     : {power.bq_diag_stage} ({BQ_DIAG_STAGE_NAMES.get(power.bq_diag_stage, 'unknown')}) / "
            f"{power.bq_diag_status} ({GLOVE_STATUS_NAMES.get(power.bq_diag_status, 'unknown')})",
            f"BQ charger event  : 0x{power.bq_charger_events:04X} "
            f"({format_flags(power.bq_charger_events, BQ_CHARGER_EVENT_NAMES)})",
            f"BQ fault event    : 0x{power.bq_fault_events:02X} "
            f"({format_flags(power.bq_fault_events, BQ_FAULT_EVENT_NAMES)})",
            f"BQ INT count      : {power.bq_interrupt_count} (low 16 bits)",
            f"VBUS/input        : {power.vbus_voltage_v:.3f} V / {power.input_current_a:+.3f} A",
            "",
            f"IMU timestamp     : {imu_time}",
            f"IMU status bits   : 0x{imu_status:04X}",
            f"Calib magic/cmd   : 0x{calib_magic:04X}/0x{calib_command:04X}",
            f"Calib seq         : {calib_seq}",
            f"Calib status      : 0x{calib_status:04X} ({calib_status_name(calib_status)})",
            f"Calib error index : 0x{calib_error:04X}",
            f"Calib last seq    : {calib_last_seq}",
            f"Joint timestamp   : {joint_time}",
            f"Joint flags       : 0x{joint_flags:04X} ({format_flags(joint_flags, JOINT_FLAG_NAMES)})",
            f"Joint valid bits  : 0x{joint_valid:08X}",
            f"Touch timestamp   : {touch_time}",
            f"Touch flags       : 0x{touch_flags:04X} ({format_flags(touch_flags, TOUCH_FLAG_NAMES)})",
            f"Touch count/cap   : {touch_count}/{touch_capacity}",
        ]
        set_text(self.summary_text, "\n".join(summary))

        attempt_text = (
            f"{health.recovery_attempt}/{health.recovery_limit}"
            if health.recovery_limit
            else str(health.recovery_attempt)
        )
        age_text = "not available" if health.snapshot_age_ms == 0xFFFF else f"{health.snapshot_age_ms} ms"
        health_lines = [
            "Device health and recovery",
            "",
            f"Protocol version    : {health.version >> 8}.{health.version & 0xFF}",
            f"Overall state       : {health.state} ({health_state})",
            f"Active flags        : 0x{health.current_flags:08X}",
            f"                      {format_flags(health.current_flags, HEALTH_FLAG_NAMES)}",
            "",
            f"Current error       : 0x{health.current_error:04X} ({current_error_name})",
            f"Current source      : {health.current_source} "
            f"({HEALTH_SOURCE_NAMES.get(health.current_source, 'unknown')})",
            f"Current target      : {format_health_target(health.current_source, health.current_target)}",
            f"Recommended action  : {current_action}",
            "",
            f"Recovery stage      : {health.recovery_stage} "
            f"({RECOVERY_STAGE_NAMES.get(health.recovery_stage, 'unknown')})",
            f"Recovery attempt    : {attempt_text}",
            "",
            f"Last error          : 0x{health.last_error:04X} ({last_error_name})",
            f"Last source/target  : {HEALTH_SOURCE_NAMES.get(health.last_source, health.last_source)}"
            f"/{format_health_target(health.last_source, health.last_target)}",
            f"Last error uptime   : {health.last_error_uptime_ms} ms",
            f"Error sequence/count: {health.error_seq}/{health.error_count}",
            f"Last UART detail    : {format_uart_error_detail(health.rs485_uart_detail)}",
            "",
            f"Live IMU mask       : 0x{health.live_imu_mask:04X}",
            f"Ready flags         : 0x{health.ready_flags:04X}",
            f"                      {format_flags(health.ready_flags, READY_FLAG_NAMES)}",
            f"Full-frame age      : {age_text}",
            "",
            "Notes:",
            "- Current error clears after verified recovery; last error remains as history.",
            "- Target is an IMU logical node, CAN bus number, or subsystem-specific detail.",
        ]
        set_text(self.health_text, "\n".join(health_lines))

        power_lines = [
            "Battery and charger status",
            "",
            f"Battery voltage : {power.battery_voltage_v:.3f} V",
            f"Battery current : {power.battery_current_a:+.3f} A (positive=charging)",
            f"SOC             : {power.soc_percent:.2f} % (uncalibrated estimate)",
            f"VBUS voltage    : {power.vbus_voltage_v:.3f} V",
            f"Input current   : {power.input_current_a:+.3f} A",
            f"System state    : {power.system_state} ({POWER_STATE_NAMES.get(power.system_state, 'UNKNOWN_VALUE')})",
            f"Charge state    : {power.charge_state} ({CHARGE_STATE_NAMES.get(power.charge_state, 'UNKNOWN_VALUE')})",
            f"Flags           : 0x{power.flags:04X}",
            f"                  {format_flags(power.flags, POWER_FLAG_NAMES)}",
            f"Fault code      : 0x{power.fault_code:04X}",
            f"BQ diagnostic   : stage={power.bq_diag_stage} "
            f"({BQ_DIAG_STAGE_NAMES.get(power.bq_diag_stage, 'unknown')}), "
            f"status={power.bq_diag_status} "
            f"({GLOVE_STATUS_NAMES.get(power.bq_diag_status, 'unknown')})",
            f"BQ events       : charger=0x{power.bq_charger_events:04X} "
            f"({format_flags(power.bq_charger_events, BQ_CHARGER_EVENT_NAMES)})",
            f"                  fault=0x{power.bq_fault_events:02X} "
            f"({format_flags(power.bq_fault_events, BQ_FAULT_EVENT_NAMES)})",
            f"BQ INT count    : {power.bq_interrupt_count} (low 16 bits)",
            "",
            "Fault high bits: bit8=BQ comm, bit9=MAX17043 comm, bit10=voltage mismatch",
        ]
        set_text(self.power_text, "\n".join(power_lines))

        imus = decode_imu(snapshot.imu_regs)
        if snapshot.sensor_data_valid:
            imu_lines = ["idx " + " ".join(f"{name:>13}" for name in IMU_FIELDS)]
            for index, values in enumerate(imus):
                imu_lines.append(
                    f"{index:02d}  " + " ".join(f"{value:13.6f}" for value in values)
                )
        else:
            imu_lines = [
                "IMU data unavailable",
                f"reason: {snapshot.sensor_invalid_reason}",
                "The last valid frame is retained internally and is not displayed as current data.",
            ]
        set_text(self.imu_text, "\n".join(imu_lines))

        joints = decode_joint(snapshot.joint_regs)
        if snapshot.sensor_data_valid:
            joint_lines = [
                "Joint angles deg",
                f"flags=0x{joint_flags:04X} ({format_flags(joint_flags, JOINT_FLAG_NAMES)}) "
                f"valid_bits=0x{joint_valid:08X}",
                "",
            ]
            for index in range(0, MODBUS_JOINT_COUNT, 3):
                row = []
                for joint_index in range(index, min(index + 3, MODBUS_JOINT_COUNT)):
                    row.append(f"J{joint_index:02d}={joints[joint_index]:9.3f}")
                joint_lines.append("  ".join(row))
        else:
            joint_lines = [
                "Joint data unavailable",
                f"reason: {snapshot.sensor_invalid_reason}",
                "Joint processing is paused until a complete valid sensor frame arrives.",
            ]
        set_text(self.joint_text, "\n".join(joint_lines))

        if snapshot.sensor_data_valid:
            touch_lines = [
                "Touch raw values",
                f"flags=0x{touch_flags:04X} ({format_flags(touch_flags, TOUCH_FLAG_NAMES)}) "
                f"count={touch_count} capacity={touch_capacity}",
                "",
            ]
            for index in range(0, MODBUS_TOUCH_COUNT, 17):
                row = snapshot.touch_regs[index : index + 17]
                touch_lines.append(
                    f"{index:02d}: " + " ".join(f"{value:5d}" for value in row)
                )
        else:
            touch_lines = [
                "Touch data unavailable",
                f"reason: {snapshot.sensor_invalid_reason}",
                "The last valid touch frame is retained internally and is not displayed as current data.",
            ]
        set_text(self.touch_text, "\n".join(touch_lines))

        raw_lines = [
            self._format_regs("basic 0x0000", REG_BASIC_STATUS_START, snapshot.basic),
            self._format_regs("system 0x0040", REG_SYSTEM_STATUS_START, snapshot.system),
            self._format_regs("health 0x004A", REG_HEALTH_STATUS_START, snapshot.health_regs),
            self._format_regs("power 0x0060", REG_POWER_STATUS_START, snapshot.power_regs),
            self._format_regs("imu status 0x1140", REG_IMU_TIMESTAMP_US, snapshot.imu_status_regs),
            self._format_regs(
                "calib ctrl 0x1254", REG_IMU_CALIB_CTRL_START, snapshot.calib_ctrl_regs
            ),
            self._format_regs(
                "joint status 0x1340", REG_JOINT_TIMESTAMP_US, snapshot.joint_status_regs
            ),
            self._format_regs(
                "touch status 0x2080", REG_TOUCH_TIMESTAMP_US, snapshot.touch_status_regs
            ),
            self._format_regs("touch 0x2000", REG_TOUCH_DATA_START, snapshot.touch_regs),
        ]
        set_text(self.raw_text, "\n\n".join(raw_lines))

    def _render_calibration(
        self, c_table: list[list[float]], m_table: list[list[float]], ctrl: list[int]
    ) -> None:
        self.last_calibration = (c_table, m_table, ctrl)
        status = ctrl[3] if len(ctrl) > 3 else 0
        error_index = ctrl[4] if len(ctrl) > 4 else 0xFFFF
        last_seq = ctrl[5] if len(ctrl) > 5 else 0
        self.status_var.set(
            f"Calibration status=0x{status:04X} ({calib_status_name(status)}) "
            f"error_index=0x{error_index:04X} last_seq={last_seq}"
        )

        lines = [
            "Control",
            f"magic              : 0x{ctrl[0]:04X}" if len(ctrl) > 0 else "magic              : n/a",
            f"command            : 0x{ctrl[1]:04X}" if len(ctrl) > 1 else "command            : n/a",
            f"seq                : {ctrl[2]}" if len(ctrl) > 2 else "seq                : n/a",
            f"status             : 0x{status:04X} ({calib_status_name(status)})",
            f"error_index        : 0x{error_index:04X}",
            f"last_applied_seq   : {last_seq}",
            "",
            "CSV format: table,imu,w,x,y,z  (table is C or M, imu is 0..15)",
            "",
            "C table",
        ]
        for index, quat in enumerate(c_table):
            lines.append(
                f"C,{index},{quat[0]:.9g},{quat[1]:.9g},{quat[2]:.9g},{quat[3]:.9g}"
            )
        lines.append("")
        lines.append("M table")
        for index, quat in enumerate(m_table):
            lines.append(
                f"M,{index},{quat[0]:.9g},{quat[1]:.9g},{quat[2]:.9g},{quat[3]:.9g}"
            )
        set_text(self.calib_text, "\n".join(lines))

    @staticmethod
    def _format_regs(title: str, start: int, regs: list[int]) -> str:
        lines = [title]
        for offset in range(0, len(regs), 8):
            chunk = regs[offset : offset + 8]
            lines.append(
                f"0x{start + offset:04X}: "
                + " ".join(f"{value:04X}" for value in chunk)
            )
        return "\n".join(lines)

    @staticmethod
    def _format_float_pairs(start: int, regs: list[int]) -> str:
        if len(regs) < 2:
            return ""
        lines = ["float32 little-word pairs"]
        for offset in range(0, len(regs) - 1, 2):
            value = regs_to_f32_le_words(regs[offset], regs[offset + 1])
            lines.append(
                f"0x{start + offset:04X}/0x{start + offset + 1:04X}: {value:.9g}"
            )
        return "\n".join(lines)

    def save_calibration_csv(self) -> None:
        if self.last_calibration is None:
            messagebox.showinfo("Save Calibration", "Read calibration first")
            return
        path = filedialog.asksaveasfilename(
            title="Save calibration",
            defaultextension=".csv",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if not path:
            return

        c_table, m_table, _ctrl = self.last_calibration
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("table", "imu", "w", "x", "y", "z"))
            for table_name, table in (("C", c_table), ("M", m_table)):
                for index, quat in enumerate(table):
                    writer.writerow((table_name, index, quat[0], quat[1], quat[2], quat[3]))
        self.status_var.set(f"Saved calibration CSV: {path}")

    def save_csv(self) -> None:
        if self.last_snapshot is None:
            messagebox.showinfo("Save CSV", "No snapshot to save")
            return
        if not self.last_snapshot.sensor_data_valid:
            messagebox.showwarning(
                "Save CSV",
                f"Sensor data is invalid: {self.last_snapshot.sensor_invalid_reason}",
            )
            return
        path = filedialog.asksaveasfilename(
            title="Save snapshot",
            defaultextension=".csv",
            filetypes=(("CSV files", "*.csv"), ("All files", "*.*")),
        )
        if not path:
            return

        snapshot = self.last_snapshot
        power = decode_power(snapshot.power_regs)
        health = decode_health(snapshot.health_regs)
        imus = decode_imu(snapshot.imu_regs)
        joints = decode_joint(snapshot.joint_regs)
        imu_sec, imu_nsec = regs_to_ros_time_le_words(snapshot.imu_status_regs[0:4])
        joint_sec, joint_nsec = regs_to_ros_time_le_words(snapshot.joint_status_regs[0:4])
        touch_sec, touch_nsec = regs_to_ros_time_le_words(snapshot.touch_status_regs[0:4])
        imu_us = ros_time_to_us(snapshot.imu_status_regs[0:4])
        joint_us = ros_time_to_us(snapshot.joint_status_regs[0:4])
        touch_us = ros_time_to_us(snapshot.touch_status_regs[0:4])
        imu_time_text = format_ros_time(snapshot.imu_status_regs[0:4])
        joint_time_text = format_ros_time(snapshot.joint_status_regs[0:4])
        touch_time_text = format_ros_time(snapshot.touch_status_regs[0:4])
        joint_valid = snapshot.joint_status_regs[5] | (snapshot.joint_status_regs[6] << 16)
        with open(path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(("section", "index", "field", "value"))
            writer.writerow(("meta", "host_time", "unix", snapshot.timestamp))
            writer.writerow(("meta", "sensor", "communication_hz", snapshot.actual_hz))
            writer.writerow(("meta", "sensor", "frame_hz", snapshot.sensor_hz))
            writer.writerow(("meta", "sensor", "frame_id", snapshot.sensor_frame_id))
            writer.writerow(("meta", "sensor", "timestamp_us", snapshot.sensor_timestamp_us))
            writer.writerow(("meta", "sensor", "duplicate_responses", snapshot.duplicate_responses))
            writer.writerow(("status", "power", "battery_voltage_v", power.battery_voltage_v))
            writer.writerow(("status", "power", "battery_current_a", power.battery_current_a))
            writer.writerow(("status", "power", "soc_percent", power.soc_percent))
            writer.writerow(("status", "power", "system_state", power.system_state))
            writer.writerow(("status", "power", "charge_state", power.charge_state))
            writer.writerow(("status", "power", "flags", f"0x{power.flags:04X}"))
            writer.writerow(("status", "power", "fault_code", f"0x{power.fault_code:04X}"))
            writer.writerow(("status", "power", "vbus_voltage_v", power.vbus_voltage_v))
            writer.writerow(("status", "power", "input_current_a", power.input_current_a))
            writer.writerow(("status", "power", "bq_diag_stage", power.bq_diag_stage))
            writer.writerow(("status", "power", "bq_diag_status", power.bq_diag_status))
            writer.writerow(("status", "power", "bq_charger_events", f"0x{power.bq_charger_events:04X}"))
            writer.writerow(("status", "power", "bq_fault_events", f"0x{power.bq_fault_events:02X}"))
            writer.writerow(("status", "power", "bq_interrupt_count", power.bq_interrupt_count))
            writer.writerow(("status", "system", "reset_cause", f"0x{snapshot.system[6]:04X}"))
            writer.writerow(("status", "system", "watchdog_status", f"0x{snapshot.system[7]:04X}"))
            writer.writerow(("status", "health", "version", f"0x{health.version:04X}"))
            writer.writerow(("status", "health", "state", health.state))
            writer.writerow(("status", "health", "current_flags", f"0x{health.current_flags:08X}"))
            writer.writerow(("status", "health", "current_error", f"0x{health.current_error:04X}"))
            writer.writerow(("status", "health", "current_source", health.current_source))
            writer.writerow(("status", "health", "current_target", health.current_target))
            writer.writerow(("status", "health", "recovery_stage", health.recovery_stage))
            writer.writerow(("status", "health", "recovery_attempt", health.recovery_attempt))
            writer.writerow(("status", "health", "recovery_limit", health.recovery_limit))
            writer.writerow(("status", "health", "last_error", f"0x{health.last_error:04X}"))
            writer.writerow(("status", "health", "last_source", health.last_source))
            writer.writerow(("status", "health", "last_target", health.last_target))
            writer.writerow(("status", "health", "error_seq", health.error_seq))
            writer.writerow(("status", "health", "error_count", health.error_count))
            writer.writerow(("status", "health", "live_imu_mask", f"0x{health.live_imu_mask:04X}"))
            writer.writerow(("status", "health", "ready_flags", f"0x{health.ready_flags:04X}"))
            writer.writerow(("status", "health", "snapshot_age_ms", health.snapshot_age_ms))
            writer.writerow(("status", "health", "rs485_uart_detail", f"0x{health.rs485_uart_detail:04X}"))
            writer.writerow(("status", "imu", "timestamp_datetime_utc", imu_time_text))
            writer.writerow(("status", "imu", "timestamp_us", imu_us))
            writer.writerow(("status", "imu", "timestamp_sec", imu_sec))
            writer.writerow(("status", "imu", "timestamp_nsec", imu_nsec))
            writer.writerow(("status", "imu", "flags", snapshot.imu_status_regs[4]))
            writer.writerow(("status", "calib", "magic", snapshot.calib_ctrl_regs[0]))
            writer.writerow(("status", "calib", "command", snapshot.calib_ctrl_regs[1]))
            writer.writerow(("status", "calib", "seq", snapshot.calib_ctrl_regs[2]))
            writer.writerow(("status", "calib", "status", snapshot.calib_ctrl_regs[3]))
            writer.writerow(("status", "calib", "error_index", snapshot.calib_ctrl_regs[4]))
            writer.writerow(("status", "calib", "last_applied_seq", snapshot.calib_ctrl_regs[5]))
            writer.writerow(("status", "joint", "timestamp_datetime_utc", joint_time_text))
            writer.writerow(("status", "joint", "timestamp_us", joint_us))
            writer.writerow(("status", "joint", "timestamp_sec", joint_sec))
            writer.writerow(("status", "joint", "timestamp_nsec", joint_nsec))
            writer.writerow(("status", "joint", "flags", snapshot.joint_status_regs[4]))
            writer.writerow(("status", "joint", "valid_bits", joint_valid))
            writer.writerow(("status", "touch", "timestamp_datetime_utc", touch_time_text))
            writer.writerow(("status", "touch", "timestamp_us", touch_us))
            writer.writerow(("status", "touch", "timestamp_sec", touch_sec))
            writer.writerow(("status", "touch", "timestamp_nsec", touch_nsec))
            writer.writerow(("status", "touch", "flags", snapshot.touch_status_regs[4]))
            writer.writerow(("status", "touch", "count", snapshot.touch_status_regs[5]))
            writer.writerow(("status", "touch", "capacity", snapshot.touch_status_regs[6]))
            for index, values in enumerate(imus):
                for field, value in zip(IMU_FIELDS, values):
                    writer.writerow(("imu", index, field, value))
            for index, value in enumerate(joints):
                writer.writerow(("joint", index, "deg", value))
            for index, value in enumerate(snapshot.touch_regs):
                writer.writerow(("touch", index, "raw", value))

    def _on_close(self) -> None:
        self.stop_poll()
        self.client.close()
        self.destroy()


def main() -> None:
    app = ModbusMonitorApp()
    if serial is None:
        messagebox.showerror("Missing dependency", "Install pyserial first")
    app.mainloop()


if __name__ == "__main__":
    main()
