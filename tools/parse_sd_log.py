#!/usr/bin/env python3
"""Parse glove SD log V2 files into diagnostic and sensor CSV files."""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import struct
from contextlib import ExitStack
from pathlib import Path
from typing import Callable


MAGIC = b"GLV2"
FORMAT_VERSION = 2
HEADER_SIZE = 96
RECORD_SIZE = 1536
CONTENT_SIZE = RECORD_SIZE - 3
IMU_FIELDS = ("ax", "ay", "az", "gx", "gy", "gz", "qw", "qx", "qy", "qz")
ZERO_RECORD = bytes(RECORD_SIZE)
MAX_REPORTED_RECORD_ERRORS = 20


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc & 0xFFFF


def utc_text(timestamp_us: int) -> str:
    try:
        value = dt.datetime.fromtimestamp(timestamp_us / 1_000_000, tz=dt.timezone.utc)
        return value.isoformat(timespec="microseconds")
    except (OverflowError, OSError, ValueError):
        return ""


def parse_record(record: bytes, record_index: int) -> dict:
    if len(record) != RECORD_SIZE:
        raise ValueError(
            f"record {record_index}: size={len(record)}, expected={RECORD_SIZE}"
        )
    if record[:4] != MAGIC:
        raise ValueError(f"record {record_index}: invalid magic {record[:4]!r}")

    version, header_size, record_size, imu_count, joint_count, touch_count = (
        struct.unpack_from("<6H", record, 4)
    )
    if version != FORMAT_VERSION or header_size != HEADER_SIZE or record_size != RECORD_SIZE:
        raise ValueError(
            f"record {record_index}: unsupported layout "
            f"version={version}, header={header_size}, size={record_size}"
        )

    payload_end = header_size + imu_count * 10 * 4 + joint_count * 4 + touch_count * 4
    if payload_end > CONTENT_SIZE:
        raise ValueError(f"record {record_index}: payload exceeds record boundary")
    if record[-1] != 0:
        raise ValueError(f"record {record_index}: invalid separator 0x{record[-1]:02X}")

    stored_crc = struct.unpack_from("<H", record, CONTENT_SIZE)[0]
    calculated_crc = crc16_modbus(record[:CONTENT_SIZE])
    offset = header_size
    imu_values = struct.unpack_from(f"<{imu_count * 10}f", record, offset)
    offset += imu_count * 10 * 4
    joint_values = struct.unpack_from(f"<{joint_count}f", record, offset)
    offset += joint_count * 4
    touch_values = struct.unpack_from(f"<{touch_count}H", record, offset)
    offset += touch_count * 2
    touch_baselines = struct.unpack_from(f"<{touch_count}H", record, offset)

    return {
        "record": record_index,
        "version": version,
        "record_size": record_size,
        "imu_count": imu_count,
        "joint_count": joint_count,
        "touch_count": touch_count,
        "hand_side": record[16],
        "time_synced": record[17],
        "fw_major": struct.unpack_from("<H", record, 18)[0],
        "fw_minor": struct.unpack_from("<H", record, 20)[0],
        "fw_patch": struct.unpack_from("<H", record, 22)[0],
        "frame_id": struct.unpack_from("<I", record, 24)[0],
        "timestamp_us": struct.unpack_from("<Q", record, 28)[0],
        "full_valid_flags": struct.unpack_from("<I", record, 36)[0],
        "raw_valid_flags": struct.unpack_from("<I", record, 40)[0],
        "processed_valid_flags": struct.unpack_from("<I", record, 44)[0],
        "imu_valid_mask": struct.unpack_from("<H", record, 48)[0],
        "process_status": struct.unpack_from("<H", record, 50)[0],
        "calibration_seq": struct.unpack_from("<H", record, 52)[0],
        "calibration_applied": record[54],
        "imu_sensor_seq": struct.unpack_from("<I", record, 56)[0],
        "touch_sensor_seq": struct.unpack_from("<I", record, 60)[0],
        "imu_timestamp_us": struct.unpack_from("<Q", record, 64)[0],
        "touch_timestamp_us": struct.unpack_from("<Q", record, 72)[0],
        "health_flags": struct.unpack_from("<I", record, 80)[0],
        "health_state": struct.unpack_from("<H", record, 84)[0],
        "health_error": struct.unpack_from("<H", record, 86)[0],
        "health_source": struct.unpack_from("<H", record, 88)[0],
        "health_target": struct.unpack_from("<H", record, 90)[0],
        "health_live_imu_mask": struct.unpack_from("<H", record, 92)[0],
        "health_ready_flags": struct.unpack_from("<H", record, 94)[0],
        "crc_ok": stored_crc == calculated_crc,
        "stored_crc": stored_crc,
        "calculated_crc": calculated_crc,
        "imu_values": imu_values,
        "joint_values": joint_values,
        "touch_values": touch_values,
        "touch_baselines": touch_baselines,
    }


def metadata_values(row: dict) -> list:
    return [
        row["record"],
        row["frame_id"],
        row["timestamp_us"],
        utc_text(row["timestamp_us"]),
        int(row["crc_ok"]),
    ]


def parse_file(
    input_path: Path,
    output_dir: Path,
    strict: bool,
    progress_callback: Callable[[int, int], None] | None = None,
) -> dict[str, int | str]:
    size = input_path.stat().st_size
    total_records = size // RECORD_SIZE
    with input_path.open("rb") as probe:
        if probe.read(4) != MAGIC:
            raise ValueError(
                "input is not a GLV2 log; this parser only accepts firmware V2.2.0+ files"
            )

    output_dir.mkdir(parents=True, exist_ok=True)
    metadata_header = ["record", "frame_id", "timestamp_us", "utc", "crc_ok"]
    frame_header = [
        *metadata_header,
        "frame_delta", "timestamp_delta_us",
        "format_version", "record_size", "imu_count", "joint_count", "touch_count",
        "hand_side", "time_synced", "firmware", "full_valid_flags", "raw_valid_flags",
        "processed_valid_flags", "imu_valid_mask", "process_status", "calibration_seq",
        "calibration_applied", "imu_sensor_seq", "touch_sensor_seq", "imu_timestamp_us",
        "touch_timestamp_us", "health_flags", "health_state", "health_error",
        "health_source", "health_target", "health_live_imu_mask", "health_ready_flags",
        "stored_crc", "calculated_crc",
    ]

    records = bad_records = crc_bad_records = 0
    frame_gap_events = missing_frames = 0
    duplicate_frames = reordered_frames = timestamp_regressions = 0
    zero_padding_records = reported_record_errors = 0
    max_timestamp_gap_us = 0
    previous_frame_id: int | None = None
    previous_timestamp_us: int | None = None
    first_frame_id: int | None = None
    last_frame_id: int | None = None
    first_layout: tuple[int, int, int] | None = None
    with ExitStack() as stack:
        frames_file = stack.enter_context((output_dir / "frames.csv").open("w", newline="", encoding="utf-8"))
        imu_file = stack.enter_context((output_dir / "imu.csv").open("w", newline="", encoding="utf-8"))
        joint_file = stack.enter_context((output_dir / "joint.csv").open("w", newline="", encoding="utf-8"))
        tactile_file = stack.enter_context((output_dir / "tactile.csv").open("w", newline="", encoding="utf-8"))
        source = stack.enter_context(input_path.open("rb"))
        frame_writer = csv.writer(frames_file)
        imu_writer = csv.writer(imu_file)
        joint_writer = csv.writer(joint_file)
        tactile_writer = csv.writer(tactile_file)
        frame_writer.writerow(frame_header)

        record_index = 0
        trailing = 0
        progress_step = max(1, total_records // 500)
        while True:
            record = source.read(RECORD_SIZE)
            if not record:
                break
            if len(record) != RECORD_SIZE:
                trailing = len(record)
                message = f"trailing partial record: {len(record)} bytes"
                if strict:
                    raise ValueError(message)
                print(message)
                break
            if record == ZERO_RECORD:
                # 兼容旧固件异常断电后遗留的预分配空白区，不逐帧刷屏。
                if strict:
                    raise ValueError(f"record {record_index}: zero-filled padding record")
                zero_padding_records += 1
                record_index += 1
                if progress_callback is not None and (
                    record_index == total_records or record_index % progress_step == 0
                ):
                    progress_callback(record_index, total_records)
                continue
            try:
                row = parse_record(record, record_index)
                layout = (row["imu_count"], row["joint_count"], row["touch_count"])
                if first_layout is None:
                    first_layout = layout
                    imu_writer.writerow([
                        *metadata_header,
                        *(f"imu{index}_{field}" for index in range(layout[0]) for field in IMU_FIELDS),
                    ])
                    joint_writer.writerow([*metadata_header, *(f"joint_{index}" for index in range(layout[1]))])
                    tactile_writer.writerow([
                        *metadata_header,
                        *(f"touch_{index}" for index in range(layout[2])),
                        *(f"baseline_{index}" for index in range(layout[2])),
                    ])
                elif layout != first_layout:
                    raise ValueError(
                        f"record {record_index}: sensor counts changed {first_layout} -> {layout}"
                    )

                frame_id = int(row["frame_id"])
                timestamp_us = int(row["timestamp_us"])
                frame_delta: int | str = ""
                timestamp_delta_us: int | str = ""
                continuity_error = ""

                # CRC异常记录的帧号不可信，不参与连续性基准；下一帧会覆盖检测该缺口。
                if row["crc_ok"]:
                    if first_frame_id is None:
                        first_frame_id = frame_id
                    if previous_frame_id is not None:
                        frame_delta_value = (frame_id - previous_frame_id) & 0xFFFFFFFF
                        frame_delta = frame_delta_value
                        if frame_delta_value == 0:
                            duplicate_frames += 1
                            continuity_error = (
                                f"record {record_index}: duplicate frame_id {frame_id}"
                            )
                        elif frame_delta_value == 1:
                            pass
                        elif frame_delta_value < 0x80000000:
                            frame_gap_events += 1
                            missing_frames += frame_delta_value - 1
                            continuity_error = (
                                f"record {record_index}: frame_id gap "
                                f"{previous_frame_id} -> {frame_id}, "
                                f"missing={frame_delta_value - 1}"
                            )
                        else:
                            reordered_frames += 1
                            continuity_error = (
                                f"record {record_index}: frame_id moved backwards "
                                f"{previous_frame_id} -> {frame_id}"
                            )

                    if previous_timestamp_us is not None:
                        if timestamp_us < previous_timestamp_us:
                            timestamp_regressions += 1
                            timestamp_delta_us = timestamp_us - previous_timestamp_us
                            if not continuity_error:
                                continuity_error = (
                                    f"record {record_index}: timestamp moved backwards "
                                    f"{previous_timestamp_us} -> {timestamp_us}"
                                )
                        else:
                            timestamp_delta_us = timestamp_us - previous_timestamp_us
                            max_timestamp_gap_us = max(
                                max_timestamp_gap_us, int(timestamp_delta_us)
                            )

                    if continuity_error:
                        if strict:
                            raise ValueError(continuity_error)
                        if reported_record_errors < MAX_REPORTED_RECORD_ERRORS:
                            print(continuity_error)
                            reported_record_errors += 1

                    previous_frame_id = frame_id
                    previous_timestamp_us = timestamp_us
                    last_frame_id = frame_id
                meta = metadata_values(row)
                frame_writer.writerow([
                    *meta, frame_delta, timestamp_delta_us,
                    row["version"], row["record_size"], row["imu_count"],
                    row["joint_count"], row["touch_count"], row["hand_side"],
                    row["time_synced"],
                    f"V{row['fw_major']}.{row['fw_minor']}.{row['fw_patch']}",
                    f"0x{row['full_valid_flags']:08X}", f"0x{row['raw_valid_flags']:08X}",
                    f"0x{row['processed_valid_flags']:08X}", f"0x{row['imu_valid_mask']:04X}",
                    row["process_status"], row["calibration_seq"], row["calibration_applied"],
                    row["imu_sensor_seq"], row["touch_sensor_seq"], row["imu_timestamp_us"],
                    row["touch_timestamp_us"], f"0x{row['health_flags']:08X}",
                    row["health_state"], f"0x{row['health_error']:04X}",
                    row["health_source"], row["health_target"],
                    f"0x{row['health_live_imu_mask']:04X}",
                    f"0x{row['health_ready_flags']:04X}",
                    f"0x{row['stored_crc']:04X}", f"0x{row['calculated_crc']:04X}",
                ])
                imu_writer.writerow([*meta, *row["imu_values"]])
                joint_writer.writerow([*meta, *row["joint_values"]])
                tactile_writer.writerow([*meta, *row["touch_values"], *row["touch_baselines"]])
                records += 1
                if not row["crc_ok"]:
                    crc_bad_records += 1
                    if strict:
                        raise ValueError(f"record {record_index}: CRC mismatch")
            except ValueError as exc:
                bad_records += 1
                if strict:
                    raise
                if reported_record_errors < MAX_REPORTED_RECORD_ERRORS:
                    print(exc)
                    reported_record_errors += 1
            record_index += 1
            if progress_callback is not None and (
                record_index == total_records or record_index % progress_step == 0
            ):
                progress_callback(record_index, total_records)

    print(f"input: {input_path}")
    print(f"size: {size} bytes")
    print(
        f"records: {records}, bad_records: {bad_records}, "
        f"crc_bad_records: {crc_bad_records}, trailing_bytes: {trailing}, "
        f"frame_gap_events: {frame_gap_events}, missing_frames: {missing_frames}"
    )
    if zero_padding_records:
        print(f"recovered zero padding: {zero_padding_records} records")
    suppressed_errors = (
        bad_records + frame_gap_events + duplicate_frames + reordered_frames
        + timestamp_regressions - reported_record_errors
    )
    if suppressed_errors > 0:
        print(f"additional anomalies suppressed: {suppressed_errors}")
    print(f"output: {output_dir}")
    return {
        "input": str(input_path),
        "output": str(output_dir),
        "size": size,
        "total_records": total_records,
        "records": records,
        "bad_records": bad_records,
        "crc_bad_records": crc_bad_records,
        "trailing_bytes": trailing,
        "frame_gap_events": frame_gap_events,
        "missing_frames": missing_frames,
        "duplicate_frames": duplicate_frames,
        "reordered_frames": reordered_frames,
        "timestamp_regressions": timestamp_regressions,
        "max_timestamp_gap_us": max_timestamp_gap_us,
        "zero_padding_records": zero_padding_records,
        "first_frame_id": -1 if first_frame_id is None else first_frame_id,
        "last_frame_id": -1 if last_frame_id is None else last_frame_id,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description="Parse glove SD LOGxxxx.BIN V2 into CSV files.")
    parser.add_argument("input", type=Path, help="Path to LOGxxxx.BIN")
    parser.add_argument("-o", "--output", type=Path, default=Path("parsed_log"), help="Output directory")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Stop on the first malformed record, CRC error, or continuity anomaly",
    )
    args = parser.parse_args()
    parse_file(args.input, args.output, args.strict)


if __name__ == "__main__":
    main()
