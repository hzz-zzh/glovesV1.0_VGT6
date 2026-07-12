#!/usr/bin/env python3
import argparse
import csv
import struct
from pathlib import Path


BLOCK_SIZE = 1024
CONTENT_SIZE = 1021
FRAME_HEAD = 0xA5
FRAME_TAIL = 0x5A
SEPARATOR = 0x00

HAND_LEFT = 0x80
DATA_TYPE_MASK = 0x7F
TYPE_IMU = 0x01
TYPE_JOINT = 0x02
TYPE_TACTILE = 0x03

IMU_FLOAT_COUNT = 160
JOINT_FLOAT_COUNT = 21
TACTILE_U16_COUNT = 132


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
            crc &= 0xFFFF
    return crc


def parse_data_id(data_id: int) -> tuple[str, int]:
    hand = "left" if (data_id & HAND_LEFT) else "right"
    return hand, data_id & DATA_TYPE_MASK


def require_marker(block: bytes, offset: int, expected: int, name: str, block_index: int) -> int:
    actual = block[offset]
    if actual != expected:
        raise ValueError(
            f"block {block_index}: expected {name}=0x{expected:02X} at offset {offset}, got 0x{actual:02X}"
        )
    return offset + 1


def parse_payload(block: bytes, offset: int, data_type: int, block_index: int):
    if data_type == TYPE_IMU:
        size = IMU_FLOAT_COUNT * 4
        values = struct.unpack_from(f"<{IMU_FLOAT_COUNT}f", block, offset)
        return values, offset + size

    if data_type == TYPE_JOINT:
        size = JOINT_FLOAT_COUNT * 4
        values = struct.unpack_from(f"<{JOINT_FLOAT_COUNT}f", block, offset)
        return values, offset + size

    if data_type == TYPE_TACTILE:
        size = TACTILE_U16_COUNT * 2
        values = struct.unpack_from(f"<{TACTILE_U16_COUNT}H", block, offset)
        return values, offset + size

    raise ValueError(f"block {block_index}: unsupported data type 0x{data_type:02X}")


def parse_one_block(block: bytes, block_index: int) -> list[dict]:
    if len(block) != BLOCK_SIZE:
        raise ValueError(f"block {block_index}: size is {len(block)}, expected {BLOCK_SIZE}")

    stored_crc = block[CONTENT_SIZE] | (block[CONTENT_SIZE + 1] << 8)
    calc_crc = crc16_modbus(block[:CONTENT_SIZE])
    crc_ok = stored_crc == calc_crc

    if block[CONTENT_SIZE + 2] != SEPARATOR:
        raise ValueError(f"block {block_index}: invalid separator 0x{block[CONTENT_SIZE + 2]:02X}")

    offset = 0
    rows = []
    for frame_index in range(3):
        offset = require_marker(block, offset, FRAME_HEAD, "head", block_index)
        data_id = block[offset]
        offset += 1

        hand, data_type = parse_data_id(data_id)
        values, offset = parse_payload(block, offset, data_type, block_index)
        timestamp_us = struct.unpack_from("<Q", block, offset)[0]
        offset += 8
        offset = require_marker(block, offset, FRAME_TAIL, "tail", block_index)

        rows.append(
            {
                "block": block_index,
                "frame": frame_index,
                "hand": hand,
                "data_type": data_type,
                "data_id": data_id,
                "timestamp_us": timestamp_us,
                "crc_ok": crc_ok,
                "values": values,
            }
        )

    return rows


def value_headers(data_type: int) -> list[str]:
    if data_type == TYPE_IMU:
        names = []
        fields = ["ax", "ay", "az", "gx", "gy", "gz", "qw", "qx", "qy", "qz"]
        for imu_index in range(16):
            names.extend(f"imu{imu_index}_{field}" for field in fields)
        return names

    if data_type == TYPE_JOINT:
        return [f"joint_{index}" for index in range(JOINT_FLOAT_COUNT)]

    return [f"touch_{index}" for index in range(TACTILE_U16_COUNT)]


def write_csv(rows: list[dict], output_dir: Path) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    paths = {
        TYPE_IMU: output_dir / "imu.csv",
        TYPE_JOINT: output_dir / "joint.csv",
        TYPE_TACTILE: output_dir / "tactile.csv",
    }

    handles = {}
    writers = {}
    try:
        for data_type, path in paths.items():
            handle = path.open("w", newline="", encoding="utf-8")
            handles[data_type] = handle
            writer = csv.writer(handle)
            writer.writerow(["block", "hand", "data_id", "timestamp_us", "crc_ok", *value_headers(data_type)])
            writers[data_type] = writer

        for row in rows:
            writers[row["data_type"]].writerow(
                [
                    row["block"],
                    row["hand"],
                    f"0x{row['data_id']:02X}",
                    row["timestamp_us"],
                    int(row["crc_ok"]),
                    *row["values"],
                ]
            )
    finally:
        for handle in handles.values():
            handle.close()


def parse_file(input_path: Path, output_dir: Path, strict: bool) -> None:
    data = input_path.read_bytes()
    block_count = len(data) // BLOCK_SIZE
    trailing = len(data) % BLOCK_SIZE
    rows = []
    bad_blocks = 0
    crc_bad_blocks = 0

    for block_index in range(block_count):
        block = data[block_index * BLOCK_SIZE : (block_index + 1) * BLOCK_SIZE]
        try:
            parsed_rows = parse_one_block(block, block_index)
            rows.extend(parsed_rows)
            if not parsed_rows[0]["crc_ok"]:
                crc_bad_blocks += 1
        except ValueError as exc:
            bad_blocks += 1
            if strict:
                raise
            print(exc)

    write_csv(rows, output_dir)

    print(f"input: {input_path}")
    print(f"size: {len(data)} bytes")
    print(f"blocks: {block_count}, rows: {len(rows)}, bad_blocks: {bad_blocks}, crc_bad_blocks: {crc_bad_blocks}")
    print(f"trailing_bytes: {trailing}")
    print(f"output: {output_dir}")


def main() -> None:
    parser = argparse.ArgumentParser(description="Parse STM32 glove SD LOGxxxx.BIN files into CSV files.")
    parser.add_argument("input", type=Path, help="Path to LOGxxxx.BIN")
    parser.add_argument("-o", "--output", type=Path, default=Path("parsed_log"), help="Output directory")
    parser.add_argument("--strict", action="store_true", help="Stop on the first malformed block")
    args = parser.parse_args()

    parse_file(args.input, args.output, args.strict)


if __name__ == "__main__":
    main()
