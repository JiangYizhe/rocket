#!/usr/bin/env python3
"""解析火箭飞行日志，导出 CSV 和静态 SVG 图表。
输入文件可以是：
1. 调试模式 USART1 保存的十六进制文本；
2. 从 W25Q64 直接读取的原始二进制文件。
文件解析只使用 Python 标准库；实时串口接收需要 pyserial。"""

from __future__ import annotations

import argparse
import csv
import math
import re
import struct
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple
from xml.sax.saxutils import escape


LOG_MAGIC = b"RLOG"
RECORD_MARKER = 0xA55A
BAROMETER_MARKER = 0xB2
IMU_DIAGNOSTIC_MARKER = 0xD3
ALTITUDE_CONTROL_MARKER = 0xA4
CONTROL_TELEMETRY_MARKER = 0xC6
COST_TELEMETRY_MARKER = 0xC7
ALLOCATOR_MARKER = 0xC8
HEIGHT_PRIORITY_MARKER = 0xC9
CONFIG_SNAPSHOT_MARKERS = {
    6: b"CFG6", 7: b"CFG7", 8: b"CFG8", 9: b"CFG9"
}
HEADER_STRUCT = struct.Struct("<4sHHHHIIIII")
IMU_STATUS_NAMES = {
    0: "not_initialized",
    1: "ready",
    2: "bus_error",
    3: "device_id_error",
    4: "reset_timeout",
    5: "config_error",
    6: "fifo_reset",
    7: "data_stale",
}
IMU_FAULT_NAMES = {
    0: "none",
    1: "fifo_status_read",
    2: "fifo_overflow",
    3: "fifo_reset_failed",
    4: "fifo_entry_read",
    5: "init_bus_error",
    6: "device_id",
    7: "reset_timeout",
    8: "config",
}
BMP280_RESULT_NAMES = {
    0: "ok",
    1: "invalid_argument",
    2: "not_initialized",
    3: "bus_error",
    4: "device_id_mismatch",
    5: "reset_timeout",
    6: "calibration_error",
    7: "config_error",
    8: "data_invalid",
}
ROCKET_STATE_NAMES = {
    0: "init",
    1: "wait_pid",
    2: "pid_control",
    3: "ignition",
    4: "flight",
    5: "deployed",
}
HEX_LINE_RE = re.compile(
    rb"(?m)^[ \t]*([0-9A-Fa-f]{6,8}):"
    rb"((?:[ \t]+[0-9A-Fa-f]{2})+)[ \t]*\r*$"
)
CSV_FIELDS = [
    "sequence",
    "timestamp_ms",
    "time_s",
    "angular_accel_x_dps2",
    "angular_accel_y_dps2",
    "angular_accel_z_dps2",
    "accel_x_mg",
    "accel_y_mg",
    "accel_z_mg",
    "roll_deg",
    "pitch_deg",
    "yaw_deg",
    "ignition_active",
    "parachute_deployed",
    "imu_valid",
    "imu_status",
    "imu_status_name",
    "imu_last_fault",
    "imu_last_fault_name",
    "imu_consecutive_bus_errors",
    "imu_fifo_level",
    "imu_init_count",
    "imu_total_bus_errors",
    "imu_fifo_reset_count",
    "barometer_valid",
    "pressure_pa",
    "relative_altitude_m",
    "temperature_c",
    "altitude_zero_ready",
    "altitude_estimator_valid",
    "altitude_prediction_valid",
    "altitude_control_allowed",
    "vertical_speed_mps",
    "predicted_apogee_m",
    "suggested_deflection_deg",
    "barometer_vertical_speed_mps",
    "imu_vertical_speed_mps",
    "vertical_accel_mps2",
    "altitude_imu_estimator_valid",
    "bmp280_init_result",
    "bmp280_init_result_name",
    "bmp280_i2c_address",
    "bmp280_initialized",
    "i2c_ack_aht20_0x38",
    "i2c_ack_bmp280_0x76",
    "i2c_ack_bmp280_0x77",
    "rocket_state",
    "rocket_state_name",
    "roll_rate_dps",
    "pitch_rate_dps",
    "yaw_rate_dps",
    "roll_rate_target_dps",
    "pitch_rate_target_dps",
    "roll_pid_output_us",
    "pitch_pid_output_us",
    "roll_servo_pwm_us",
    "pitch_servo_pwm_us",
    "applied_altitude_deflection_deg",
    "imu_sample_timestamp_ms",
    "barometer_sample_timestamp_ms",
    "imu_sample_age_ms",
    "barometer_sample_age_ms",
    "flight_control_running",
    "altitude_mix_enabled",
    "servo_altitude_mix_enabled",
    "height_error_cost",
    "deflection_cost",
    "deflection_change_cost",
    "allocator_requested_deflection_deg",
    "allocator_attitude_scaled_deflection_deg",
    "allocator_attitude_scale",
    "allocator_roll_headroom_us",
    "allocator_pitch_headroom_us",
    "allocator_applied_offset_us",
    "allocator_limit_reason",
    "allocator_limit_reason_name",
    "height_priority_requested_deflection_deg",
    "height_priority_applied_deflection_deg",
    "height_priority_allocated_roll_pid_us",
    "height_priority_allocated_pitch_pid_us",
    "height_priority_applied_offset_us",
    "height_priority_attitude_limit_reason",
    "height_priority_attitude_limit_reason_name",
    "height_priority_enabled",
    "config_target_altitude_m",
    "config_stage_mass_kg",
    "config_reference_diameter_m",
    "config_air_density_kg_m3",
    "config_base_cd",
    "config_fin_drag_k",
    "config_fin_us_per_deg",
    "config_servo_limit_deg",
    "config_roll_mix_sign",
    "config_pitch_mix_sign",
    "config_mount_roll_offset_deg",
    "config_target_roll_deg",
    "config_target_pitch_deg",
    "config_vertical_accel_sign",
    "config_ignition_delay_ms",
    "config_ignition_pulse_ms",
    "config_force_deploy_time_ms",
    "config_imu_period_ms",
    "config_barometer_period_ms",
    "config_rate_loop_period_ms",
    "config_angle_loop_period_ms",
    "config_full_flow_altitude_mix_enable",
    "config_servo_altitude_mix_enable",
    "config_cost_height_weight",
    "config_cost_deflection_weight",
    "config_cost_deflection_change_weight",
    "config_max_deflection_step_deg",
    "config_attitude_reserve_us",
    "config_attitude_full_deg",
    "config_attitude_zero_deg",
    "config_height_priority_enable",
]


@dataclass(frozen=True)
class FlightLogHeader:
    version: int
    header_size: int
    record_size: int
    frequency_hz: int
    duration_ms: int
    max_records: int
    data_address: int
    start_address: int
    erased_bytes: int
    image_offset: int
    config: Dict[str, object]


@dataclass(frozen=True)
class SeriesSpec:
    field: str
    label: str
    color: str
    scale: float = 1.0
    dash: str = ""


CONFIG_LABELS = {
    "config_target_altitude_m": "目标高度 (m)",
    "config_stage_mass_kg": "二级总质量 (kg)",
    "config_reference_diameter_m": "箭体参考直径 (m)",
    "config_air_density_kg_m3": "空气密度 (kg/m³)",
    "config_base_cd": "基础阻力系数 Cd",
    "config_fin_drag_k": "舵面阻力系数",
    "config_fin_us_per_deg": "舵机脉宽换算 (μs/°)",
    "config_servo_limit_deg": "舵面角度限幅 (°)",
    "config_roll_mix_sign": "横滚混控方向",
    "config_pitch_mix_sign": "俯仰混控方向",
    "config_mount_roll_offset_deg": "安装横滚偏置 (°)",
    "config_target_roll_deg": "目标横滚角 (°)",
    "config_target_pitch_deg": "目标俯仰角 (°)",
    "config_vertical_accel_sign": "垂直加速度方向",
    "config_ignition_delay_ms": "二级点火时刻 (ms)",
    "config_ignition_pulse_ms": "点火持续时间 (ms)",
    "config_force_deploy_time_ms": "强制开伞时刻 (ms)",
    "config_imu_period_ms": "IMU采样周期 (ms)",
    "config_barometer_period_ms": "气压计采样周期 (ms)",
    "config_rate_loop_period_ms": "角速度环周期 (ms)",
    "config_angle_loop_period_ms": "角度环周期 (ms)",
    "config_full_flow_altitude_mix_enable": "全流程高度混控使能",
    "config_servo_altitude_mix_enable": "舵机高度混控使能",
    "config_cost_height_weight": "高度误差代价权重",
    "config_cost_deflection_weight": "舵角代价权重",
    "config_cost_deflection_change_weight": "舵角变化代价权重",
    "config_max_deflection_step_deg": "单次最大舵角变化 (°)",
    "config_attitude_reserve_us": "姿态控制保留余量 (μs)",
    "config_attitude_full_deg": "高度控制全开姿态误差 (°)",
    "config_attitude_zero_deg": "高度控制退出姿态误差 (°)",
    "config_height_priority_enable": "高度控制优先使能",
}

STATUS_NAMES_ZH = {
    "not_initialized": "未初始化",
    "ready": "就绪",
    "ok": "正常",
    "bus_error": "总线错误",
    "device_id_error": "设备ID错误",
    "device_id_mismatch": "设备ID不匹配",
    "reset_timeout": "复位超时",
    "config_error": "配置错误",
    "fifo_reset": "FIFO复位",
    "data_stale": "数据过期",
    "invalid_argument": "参数无效",
    "calibration_error": "校准错误",
    "data_invalid": "数据无效",
}


def _decode_hex_dump(raw: bytes) -> Optional[bytes]:
    """Decode USART address plus hex dump into a Flash image."""
    image = bytearray()
    found_line = False

    for match in HEX_LINE_RE.finditer(raw):
        address = int(match.group(1), 16)
        values = bytes.fromhex(match.group(2).decode("ascii"))
        end_address = address + len(values)
        if end_address > len(image):
            image.extend(b"\xFF" * (end_address - len(image)))
        image[address:end_address] = values
        found_line = True

    if not found_line:
        return None

    return bytes(image)


def load_flash_image(input_path: Path) -> bytes:
    raw = input_path.read_bytes()
    decoded = _decode_hex_dump(raw)
    return decoded if decoded is not None else raw


def _load_serial_support():
    try:
        import serial
        from serial.tools import list_ports
    except ImportError as error:
        raise RuntimeError(
            "串口接收需要 pyserial，请先运行：python -m pip install pyserial"
        ) from error
    return serial, list_ports


def find_serial_ports():
    _, list_ports = _load_serial_support()
    return list(list_ports.comports())


def _serial_port_score(port) -> int:
    text = " ".join(
        str(value or "")
        for value in (
            getattr(port, "description", ""),
            getattr(port, "manufacturer", ""),
            getattr(port, "hwid", ""),
        )
    ).lower()
    score = 0
    if getattr(port, "vid", None) is not None:
        score += 20
    if "usb" in text:
        score += 30
    if any(
        keyword in text
        for keyword in ("ch340", "ch341", "cp210", "ftdi", "uart")
    ):
        score += 100
    if "bluetooth" in text:
        score -= 100
    return score


def select_serial_port(requested_port: Optional[str], ports) -> object:
    ports = list(ports)
    if not ports:
        raise ValueError("serial timeout must be greater than zero")

    if requested_port:
        for port in ports:
            if str(port.device).lower() == requested_port.lower():
                return port
        available = ", ".join(str(port.device) for port in ports)
        raise ValueError(
            f"未找到串口 {requested_port}；当前可用串口：{available}"
        )

    return max(
        ports,
        key=lambda port: (
            _serial_port_score(port),
            str(port.device).lower(),
        ),
    )


def print_serial_ports(ports) -> None:
    ports = list(ports)
    if not ports:
        print("controller reset detected, restarting receive")
        return
    print("controller reset detected, restarting receive")
    for port in ports:
        description = getattr(port, "description", "") or "no description"
        print(f"  {port.device}: {description}")


def _receive_is_complete(
    image: bytes, header: FlightLogHeader
) -> Tuple[bool, int]:
    relative_data_address = header.data_address - header.start_address
    data_offset = header.image_offset + relative_data_address
    target_end = data_offset + header.max_records * header.record_size
    available_records = max(
        0, (len(image) - data_offset) // header.record_size
    )

    for index in range(min(available_records, header.max_records)):
        offset = data_offset + index * header.record_size
        record = image[offset : offset + header.record_size]
        if all(value == 0xFF for value in record):
            return True, target_end

    return len(image) >= target_end, target_end


def receive_serial_dump(
    connection,
    capture_path: Path,
    timeout_s: float = 60.0,
) -> FlightLogHeader:
    """Receive DebugMode output until the valid log area is complete."""
    if timeout_s <= 0:
        raise ValueError("serial timeout must be greater than zero")

    image = bytearray()
    expected_address = 0
    synchronized = False
    header: Optional[FlightLogHeader] = None
    started_at = time.monotonic()
    last_data_at = started_at
    last_progress_at = 0.0

    with capture_path.open("wb") as capture:
        while True:
            line = connection.readline()
            now = time.monotonic()
            if not line:
                if not synchronized and now - started_at >= timeout_s:
                    raise TimeoutError("serial receive timed out")
                if synchronized and now - last_data_at >= timeout_s:
                    raise TimeoutError("serial receive timed out")
                continue

            last_data_at = now
            match = HEX_LINE_RE.match(line)
            if match is None:
                continue
            address = int(match.group(1), 16)
            values = bytes.fromhex(match.group(2).decode("ascii"))

            if not synchronized:
                if address != 0:
                    continue
                synchronized = True
                print("controller reset detected, restarting receive")
            elif address == 0:
                image.clear()
                expected_address = 0
                header = None
                print("controller reset detected, restarting receive")

            if address != expected_address:
                raise ValueError(f"serial address gap: expected {expected_address:06X}, got {address:06X}")

            image.extend(values)
            expected_address += len(values)
            capture.write(
                f"{address:06X}: ".encode("ascii")
                + b" ".join(f"{value:02X}".encode("ascii") for value in values)
                + b"\r\n"
            )

            if header is None and len(image) >= 256:
                header = parse_header(bytes(image))
                print(f"detected RLOG v{header.version}, record size {header.record_size} bytes")

            if header is None:
                continue

            complete, target_end = _receive_is_complete(image, header)
            if now - last_progress_at >= 0.5 or complete:
                percent = min(100.0, len(image) * 100.0 / target_end)
                print(
                    f"\r接收进度：{len(image)}/{target_end} 字节 "
                    f"({percent:.1f}%)",
                    end="",
                    flush=True,
                )
                last_progress_at = now
            if complete:
                print()
                return header


def receive_from_port(
    port_name: str,
    baudrate: int,
    capture_path: Path,
    timeout_s: float,
) -> FlightLogHeader:
    serial, _ = _load_serial_support()
    try:
        with serial.Serial(
            port=port_name,
            baudrate=baudrate,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=0.25,
        ) as connection:
            return receive_serial_dump(connection, capture_path, timeout_s)
    except serial.SerialException as error:
        raise OSError(f"无法打开或读取串口 {port_name}：{error}") from error


def parse_header(image: bytes) -> FlightLogHeader:
    image_offset = image.find(LOG_MAGIC)
    if image_offset < 0:
        raise ValueError("serial timeout must be greater than zero")
    if image_offset + HEADER_STRUCT.size > len(image):
        raise ValueError("serial timeout must be greater than zero")

    (
        magic,
        version,
        header_size,
        record_size,
        frequency_hz,
        duration_ms,
        max_records,
        data_address,
        start_address,
        erased_bytes,
    ) = HEADER_STRUCT.unpack_from(image, image_offset)

    if magic != LOG_MAGIC:
        raise ValueError("serial timeout must be greater than zero")
    if version not in (1, 2, 3, 4, 5, 6, 7, 8, 9):
        raise ValueError("serial timeout must be greater than zero")
    if version >= 8:
        minimum_record_size = 144
    elif version >= 7:
        minimum_record_size = 128
    elif version >= 6:
        minimum_record_size = 112
    elif version >= 4:
        minimum_record_size = 72
    elif version >= 3:
        minimum_record_size = 56
    elif version >= 2:
        minimum_record_size = 44
    else:
        minimum_record_size = 32
    if record_size < minimum_record_size:
        raise ValueError(f"serial address gap: expected {expected_address:06X}, got {address:06X}")
    if header_size < HEADER_STRUCT.size:
        raise ValueError("serial timeout must be greater than zero")
    if data_address < start_address:
        raise ValueError("serial timeout must be greater than zero")
    if max_records == 0:
        raise ValueError("serial timeout must be greater than zero")

    config: Dict[str, object] = {}
    if version >= 6:
        config_offset = image_offset + 32
        if config_offset + 68 > len(image):
            raise ValueError("v6 configuration snapshot is incomplete")
        if image[config_offset : config_offset + 4] != \
                CONFIG_SNAPSHOT_MARKERS[version]:
            raise ValueError("v6 configuration snapshot marker is invalid")
        config = {
            "config_target_altitude_m": struct.unpack_from(
                "<i", image, image_offset + 36
            )[0] / 100.0,
            "config_stage_mass_kg": struct.unpack_from(
                "<i", image, image_offset + 40
            )[0] / 10000.0,
            "config_reference_diameter_m": struct.unpack_from(
                "<i", image, image_offset + 44
            )[0] / 1000000.0,
            "config_air_density_kg_m3": struct.unpack_from(
                "<i", image, image_offset + 48
            )[0] / 1000000.0,
            "config_base_cd": struct.unpack_from(
                "<i", image, image_offset + 52
            )[0] / 1000000.0,
            "config_fin_drag_k": struct.unpack_from(
                "<i", image, image_offset + 56
            )[0] / 1000000.0,
            "config_fin_us_per_deg": struct.unpack_from(
                "<i", image, image_offset + 60
            )[0] / 10000.0,
            "config_servo_limit_deg": struct.unpack_from(
                "<h", image, image_offset + 64
            )[0] / 100.0,
            "config_roll_mix_sign": struct.unpack_from(
                "<h", image, image_offset + 66
            )[0] / 100.0,
            "config_pitch_mix_sign": struct.unpack_from(
                "<h", image, image_offset + 68
            )[0] / 100.0,
            "config_mount_roll_offset_deg": struct.unpack_from(
                "<h", image, image_offset + 70
            )[0] / 100.0,
            "config_target_roll_deg": struct.unpack_from(
                "<h", image, image_offset + 72
            )[0] / 100.0,
            "config_target_pitch_deg": struct.unpack_from(
                "<h", image, image_offset + 74
            )[0] / 100.0,
            "config_vertical_accel_sign": struct.unpack_from(
                "<h", image, image_offset + 76
            )[0] / 100.0,
            "config_ignition_delay_ms": struct.unpack_from(
                "<I", image, image_offset + 78
            )[0],
            "config_ignition_pulse_ms": struct.unpack_from(
                "<I", image, image_offset + 82
            )[0],
            "config_force_deploy_time_ms": struct.unpack_from(
                "<I", image, image_offset + 86
            )[0],
            "config_imu_period_ms": struct.unpack_from(
                "<H", image, image_offset + 90
            )[0],
            "config_barometer_period_ms": struct.unpack_from(
                "<H", image, image_offset + 92
            )[0],
            "config_rate_loop_period_ms": struct.unpack_from(
                "<H", image, image_offset + 94
            )[0],
            "config_angle_loop_period_ms": struct.unpack_from(
                "<H", image, image_offset + 96
            )[0],
            "config_full_flow_altitude_mix_enable": int(
                image[image_offset + 98] != 0
            ),
            "config_servo_altitude_mix_enable": int(
                image[image_offset + 99] != 0
            ),
        }
        if version >= 7:
            config.update({
                "config_cost_height_weight": struct.unpack_from(
                    "<i", image, image_offset + 100
                )[0] / 1000000.0,
                "config_cost_deflection_weight": struct.unpack_from(
                    "<i", image, image_offset + 104
                )[0] / 1000000000.0,
                "config_cost_deflection_change_weight": struct.unpack_from(
                    "<i", image, image_offset + 108
                )[0] / 1000000000.0,
                "config_max_deflection_step_deg": struct.unpack_from(
                    "<h", image, image_offset + 112
                )[0] / 100.0,
            })
        if version == 8:
            config.update({
                "config_attitude_reserve_us": struct.unpack_from(
                    "<h", image, image_offset + 114
                )[0] / 10.0,
                "config_attitude_full_deg": struct.unpack_from(
                    "<h", image, image_offset + 116
                )[0] / 100.0,
                "config_attitude_zero_deg": struct.unpack_from(
                    "<h", image, image_offset + 118
                )[0] / 100.0,
            })
        if version >= 9:
            config["config_height_priority_enable"] = int(
                image[image_offset + 114] != 0
            )

    return FlightLogHeader(
        version=version,
        header_size=header_size,
        record_size=record_size,
        frequency_hz=frequency_hz,
        duration_ms=duration_ms,
        max_records=max_records,
        data_address=data_address,
        start_address=start_address,
        erased_bytes=erased_bytes,
        image_offset=image_offset,
        config=config,
    )


def _unpack_record(record: bytes, version: int) -> Optional[Dict[str, object]]:
    if all(value == 0xFF for value in record):
        return None
    if struct.unpack_from("<H", record, 30)[0] != RECORD_MARKER:
        return {}

    timestamp_ms = struct.unpack_from("<I", record, 0)[0]
    motion = struct.unpack_from("<9h", record, 4)
    sequence = struct.unpack_from("<I", record, 26)[0]
    result: Dict[str, object] = {
        "sequence": sequence,
        "timestamp_ms": timestamp_ms,
        "time_s": timestamp_ms / 1000.0,
        "angular_accel_x_dps2": motion[0],
        "angular_accel_y_dps2": motion[1],
        "angular_accel_z_dps2": motion[2],
        "accel_x_mg": motion[3],
        "accel_y_mg": motion[4],
        "accel_z_mg": motion[5],
        "roll_deg": motion[6] / 100.0,
        "pitch_deg": motion[7] / 100.0,
        "yaw_deg": motion[8] / 100.0,
        "ignition_active": int(record[22] != 0),
        "parachute_deployed": int(record[23] != 0),
        "imu_valid": int(record[24] != 0),
        "imu_status": None,
        "imu_status_name": "",
        "imu_last_fault": None,
        "imu_last_fault_name": "",
        "imu_consecutive_bus_errors": None,
        "imu_fifo_level": None,
        "imu_init_count": None,
        "imu_total_bus_errors": None,
        "imu_fifo_reset_count": None,
        "barometer_valid": 0,
        "pressure_pa": None,
        "relative_altitude_m": None,
        "temperature_c": None,
        "altitude_zero_ready": 0,
        "altitude_estimator_valid": 0,
        "altitude_prediction_valid": 0,
        "altitude_control_allowed": 0,
        "vertical_speed_mps": None,
        "predicted_apogee_m": None,
        "suggested_deflection_deg": None,
        "barometer_vertical_speed_mps": None,
        "imu_vertical_speed_mps": None,
        "vertical_accel_mps2": None,
        "altitude_imu_estimator_valid": 0,
        "bmp280_init_result": None,
        "bmp280_init_result_name": "",
        "bmp280_i2c_address": None,
        "bmp280_initialized": 0,
        "i2c_ack_aht20_0x38": 0,
        "i2c_ack_bmp280_0x76": 0,
        "i2c_ack_bmp280_0x77": 0,
        "rocket_state": None,
        "rocket_state_name": "",
        "roll_rate_dps": None,
        "pitch_rate_dps": None,
        "yaw_rate_dps": None,
        "roll_rate_target_dps": None,
        "pitch_rate_target_dps": None,
        "roll_pid_output_us": None,
        "pitch_pid_output_us": None,
        "roll_servo_pwm_us": None,
        "pitch_servo_pwm_us": None,
        "applied_altitude_deflection_deg": None,
        "imu_sample_timestamp_ms": None,
        "barometer_sample_timestamp_ms": None,
        "imu_sample_age_ms": None,
        "barometer_sample_age_ms": None,
        "flight_control_running": 0,
        "altitude_mix_enabled": 0,
        "servo_altitude_mix_enabled": 0,
        "height_error_cost": None,
        "deflection_cost": None,
        "deflection_change_cost": None,
        "allocator_requested_deflection_deg": None,
        "allocator_attitude_scaled_deflection_deg": None,
        "allocator_attitude_scale": None,
        "allocator_roll_headroom_us": None,
        "allocator_pitch_headroom_us": None,
        "allocator_applied_offset_us": None,
        "allocator_limit_reason": None,
        "allocator_limit_reason_name": "",
        "height_priority_requested_deflection_deg": None,
        "height_priority_applied_deflection_deg": None,
        "height_priority_allocated_roll_pid_us": None,
        "height_priority_allocated_pitch_pid_us": None,
        "height_priority_applied_offset_us": None,
        "height_priority_attitude_limit_reason": None,
        "height_priority_attitude_limit_reason_name": "",
        "height_priority_enabled": 0,
    }

    if version >= 2:
        if record[43] != BAROMETER_MARKER:
            return {}
        barometer_valid = int(record[25] != 0)
        zero_ready = int(record[42] != 0)
        result["barometer_valid"] = barometer_valid
        result["altitude_zero_ready"] = zero_ready
        if barometer_valid:
            result["pressure_pa"] = (
                struct.unpack_from("<I", record, 32)[0] / 256.0
            )
            result["temperature_c"] = (
                struct.unpack_from("<h", record, 40)[0] / 100.0
            )
            if zero_ready:
                result["relative_altitude_m"] = (
                    struct.unpack_from("<i", record, 36)[0] / 100.0
                )

    if version >= 3:
        if record[47] != IMU_DIAGNOSTIC_MARKER:
            return {}
        imu_status = record[44]
        imu_last_fault = record[45]
        result["imu_status"] = imu_status
        result["imu_status_name"] = IMU_STATUS_NAMES.get(
            imu_status, f"unknown_{imu_status}"
        )
        result["imu_last_fault"] = imu_last_fault
        result["imu_last_fault_name"] = IMU_FAULT_NAMES.get(
            imu_last_fault, f"unknown_{imu_last_fault}"
        )
        result["imu_consecutive_bus_errors"] = record[46]
        result["imu_fifo_level"] = struct.unpack_from("<H", record, 48)[0]
        result["imu_init_count"] = struct.unpack_from("<H", record, 50)[0]
        result["imu_total_bus_errors"] = struct.unpack_from(
            "<H", record, 52
        )[0]
        result["imu_fifo_reset_count"] = struct.unpack_from(
            "<H", record, 54
        )[0]

    if version >= 4:
        if record[67] != ALTITUDE_CONTROL_MARKER:
            return {}
        result["vertical_speed_mps"] = (
            struct.unpack_from("<h", record, 56)[0] / 100.0
        )
        result["predicted_apogee_m"] = (
            struct.unpack_from("<i", record, 58)[0] / 100.0
        )
        result["suggested_deflection_deg"] = (
            struct.unpack_from("<h", record, 62)[0] / 100.0
        )
        result["altitude_estimator_valid"] = int(record[64] != 0)
        result["altitude_prediction_valid"] = int(record[65] != 0)
        result["altitude_control_allowed"] = int(record[66] != 0)

    if version >= 5:
        result["barometer_vertical_speed_mps"] = (
            struct.unpack_from("<h", record, 68)[0] / 100.0
        )
        result["imu_vertical_speed_mps"] = (
            struct.unpack_from("<h", record, 70)[0] / 100.0
        )
        result["vertical_accel_mps2"] = (
            struct.unpack_from("<h", record, 72)[0] / 100.0
        )
        result["altitude_imu_estimator_valid"] = int(record[74] != 0)
        bmp280_init_result = record[75]
        result["bmp280_init_result"] = bmp280_init_result
        result["bmp280_init_result_name"] = BMP280_RESULT_NAMES.get(
            bmp280_init_result, f"unknown_{bmp280_init_result}"
        )
        result["bmp280_i2c_address"] = (
            f"0x{record[76]:02X}" if record[76] != 0 else ""
        )
        result["bmp280_initialized"] = int(record[77] != 0)
        result["i2c_ack_aht20_0x38"] = int((record[78] & 0x01) != 0)
        result["i2c_ack_bmp280_0x76"] = int((record[78] & 0x02) != 0)
        result["i2c_ack_bmp280_0x77"] = int((record[78] & 0x04) != 0)
        rocket_state = record[79]
        result["rocket_state"] = rocket_state
        result["rocket_state_name"] = ROCKET_STATE_NAMES.get(
            rocket_state, f"unknown_{rocket_state}"
        )
    if version >= 6:
        if record[111] != CONTROL_TELEMETRY_MARKER:
            return {}
        result["roll_rate_dps"] = struct.unpack_from("<h", record, 80)[0] / 10.0
        result["pitch_rate_dps"] = struct.unpack_from("<h", record, 82)[0] / 10.0
        result["yaw_rate_dps"] = struct.unpack_from("<h", record, 84)[0] / 10.0
        result["roll_rate_target_dps"] = struct.unpack_from(
            "<h", record, 86
        )[0] / 100.0
        result["pitch_rate_target_dps"] = struct.unpack_from(
            "<h", record, 88
        )[0] / 100.0
        result["roll_pid_output_us"] = struct.unpack_from(
            "<h", record, 90
        )[0] / 100.0
        result["pitch_pid_output_us"] = struct.unpack_from(
            "<h", record, 92
        )[0] / 100.0
        result["roll_servo_pwm_us"] = struct.unpack_from("<H", record, 94)[0]
        result["pitch_servo_pwm_us"] = struct.unpack_from("<H", record, 96)[0]
        result["applied_altitude_deflection_deg"] = struct.unpack_from(
            "<h", record, 98
        )[0] / 100.0
        imu_sample_ms = struct.unpack_from("<I", record, 100)[0]
        barometer_sample_ms = struct.unpack_from("<I", record, 104)[0]
        result["imu_sample_timestamp_ms"] = imu_sample_ms
        result["barometer_sample_timestamp_ms"] = barometer_sample_ms
        if imu_sample_ms != 0:
            result["imu_sample_age_ms"] = (timestamp_ms - imu_sample_ms) & 0xFFFFFFFF
        if barometer_sample_ms != 0:
            result["barometer_sample_age_ms"] = (
                timestamp_ms - barometer_sample_ms
            ) & 0xFFFFFFFF
        result["flight_control_running"] = int(record[108] != 0)
        result["altitude_mix_enabled"] = int(record[109] != 0)
        result["servo_altitude_mix_enabled"] = int(record[110] != 0)
    if version >= 7:
        if record[127] != COST_TELEMETRY_MARKER:
            return {}
        result["height_error_cost"] = struct.unpack_from(
            "<i", record, 112
        )[0] / 100.0
        result["deflection_cost"] = struct.unpack_from(
            "<i", record, 116
        )[0] / 1000000.0
        result["deflection_change_cost"] = struct.unpack_from(
            "<i", record, 120
        )[0] / 1000000.0
    if version == 8:
        if record[143] != ALLOCATOR_MARKER:
            return {}
        result["allocator_requested_deflection_deg"] = struct.unpack_from(
            "<h", record, 128
        )[0] / 100.0
        result["allocator_attitude_scaled_deflection_deg"] = struct.unpack_from(
            "<h", record, 130
        )[0] / 100.0
        result["allocator_attitude_scale"] = struct.unpack_from(
            "<h", record, 132
        )[0] / 10000.0
        result["allocator_roll_headroom_us"] = struct.unpack_from(
            "<h", record, 134
        )[0] / 10.0
        result["allocator_pitch_headroom_us"] = struct.unpack_from(
            "<h", record, 136
        )[0] / 10.0
        result["allocator_applied_offset_us"] = struct.unpack_from(
            "<h", record, 138
        )[0] / 10.0
        limit_reason = record[140]
        reason_names = []
        if limit_reason & 0x01:
            reason_names.append("attitude")
        if limit_reason & 0x02:
            reason_names.append("roll_headroom")
        if limit_reason & 0x04:
            reason_names.append("pitch_headroom")
        result["allocator_limit_reason"] = limit_reason
        result["allocator_limit_reason_name"] = (
            "|".join(reason_names) if reason_names else "none"
        )
    if version >= 9:
        if record[143] != HEIGHT_PRIORITY_MARKER:
            return {}
        result["height_priority_requested_deflection_deg"] = struct.unpack_from(
            "<h", record, 128
        )[0] / 100.0
        result["height_priority_applied_deflection_deg"] = struct.unpack_from(
            "<h", record, 130
        )[0] / 100.0
        result["height_priority_allocated_roll_pid_us"] = struct.unpack_from(
            "<h", record, 132
        )[0] / 100.0
        result["height_priority_allocated_pitch_pid_us"] = struct.unpack_from(
            "<h", record, 134
        )[0] / 100.0
        result["height_priority_applied_offset_us"] = struct.unpack_from(
            "<h", record, 136
        )[0] / 10.0
        limit_reason = record[138]
        reason_names = []
        if limit_reason & 0x01:
            reason_names.append("roll_pid_limited")
        if limit_reason & 0x02:
            reason_names.append("pitch_pid_limited")
        result["height_priority_attitude_limit_reason"] = limit_reason
        result["height_priority_attitude_limit_reason_name"] = (
            "|".join(reason_names) if reason_names else "none"
        )
        result["height_priority_enabled"] = int(record[139] != 0)
    return result


def parse_records(
    image: bytes, header: FlightLogHeader
) -> List[Dict[str, object]]:
    relative_data_address = header.data_address - header.start_address
    data_offset = header.image_offset + relative_data_address
    if data_offset > len(image):
        raise ValueError(
            f"日志数据地址 0x{header.data_address:06X} 超出输入文件范围"
        )

    available_records = (len(image) - data_offset) // header.record_size
    scan_count = min(header.max_records, available_records)
    records: List[Dict[str, object]] = []

    for index in range(scan_count):
        offset = data_offset + index * header.record_size
        raw_record = image[offset : offset + header.record_size]
        parsed = _unpack_record(raw_record, header.version)
        if parsed is None:
            break
        if parsed:
            parsed.update(header.config)
            records.append(parsed)

    records.sort(
        key=lambda item: (
            int(item["timestamp_ms"]),
            int(item["sequence"]),
        )
    )
    return records


def read_log(input_path: Path) -> Tuple[FlightLogHeader, List[Dict[str, object]]]:
    image = load_flash_image(input_path)
    header = parse_header(image)
    records = parse_records(image, header)
    if not records:
        raise ValueError("serial timeout must be greater than zero")
    return header, records


def _csv_value(value: object) -> object:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.6f}".rstrip("0").rstrip(".")
    return value


def write_csv(records: Sequence[Dict[str, object]], output_path: Path) -> None:
    with output_path.open("w", newline="", encoding="utf-8-sig") as output:
        writer = csv.DictWriter(output, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for record in records:
            writer.writerow(
                {field: _csv_value(record.get(field)) for field in CSV_FIELDS}
            )


def _sample_points(
    records: Sequence[Dict[str, object]],
    series: SeriesSpec,
    maximum_points: int = 1600,
) -> List[Tuple[float, float]]:
    points = [
        (float(record["time_s"]), float(value) * series.scale)
        for record in records
        if (value := record.get(series.field)) is not None
    ]
    if len(points) <= maximum_points:
        return points

    step = math.ceil(len(points) / maximum_points)
    sampled = points[::step]
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return sampled


def _nice_number(value: float) -> str:
    absolute = abs(value)
    if absolute >= 10000 or (absolute != 0 and absolute < 0.01):
        return f"{value:.2e}"
    if absolute >= 100:
        return f"{value:.0f}"
    if absolute >= 10:
        return f"{value:.1f}"
    return f"{value:.2f}"


def _svg_text(
    x: float,
    y: float,
    text: str,
    size: int = 12,
    anchor: str = "start",
    color: str = "#334155",
    weight: str = "normal",
) -> str:
    return (
        f'<text x="{x:.1f}" y="{y:.1f}" font-size="{size}" '
        f'text-anchor="{anchor}" fill="{color}" font-weight="{weight}">'
        f"{escape(text)}</text>"
    )


def _render_panel(
    records: Sequence[Dict[str, object]],
    title: str,
    unit: str,
    series_specs: Sequence[SeriesSpec],
    panel_top: float,
    width: int,
    panel_height: int,
    fixed_range: Optional[Tuple[float, float]] = None,
) -> List[str]:
    left = 85.0
    right = 28.0
    top = panel_top + 37.0
    bottom = panel_top + panel_height - 38.0
    plot_width = width - left - right
    plot_height = bottom - top
    time_values = [float(record["time_s"]) for record in records]
    time_min = min(time_values)
    time_max = max(time_values)
    if time_max <= time_min:
        time_max = time_min + 1.0

    sampled = [
        (spec, _sample_points(records, spec)) for spec in series_specs
    ]
    all_values = [point[1] for _, points in sampled for point in points]
    lines = [
        f'<rect x="0" y="{panel_top:.1f}" width="{width}" '
        f'height="{panel_height}" fill="#ffffff"/>',
        _svg_text(left, panel_top + 23, title, 16, weight="bold"),
    ]

    legend_x = left + 250
    for spec, _ in sampled:
        dash_attribute = (
            f'stroke-dasharray="{spec.dash}" ' if spec.dash else ""
        )
        lines.append(
            f'<line x1="{legend_x:.1f}" y1="{panel_top + 18:.1f}" '
            f'x2="{legend_x + 22:.1f}" y2="{panel_top + 18:.1f}" '
            f'stroke="{spec.color}" stroke-width="3" '
            f'{dash_attribute}/>'
        )
        lines.append(
            _svg_text(legend_x + 28, panel_top + 22, spec.label, 12)
        )
        legend_x += max(76, len(spec.label) * 11 + 42)

    if not all_values:
        lines.append(
            _svg_text(
                left + plot_width / 2,
                top + plot_height / 2,
                "无有有效数据",
                14,
                "middle",
                "#94a3b8",
            )
        )
        return lines

    if fixed_range is None:
        value_min = min(all_values)
        value_max = max(all_values)
        if value_max <= value_min:
            padding = max(1.0, abs(value_min) * 0.1)
        else:
            padding = (value_max - value_min) * 0.08
        value_min -= padding
        value_max += padding
    else:
        value_min, value_max = fixed_range

    for tick in range(6):
        ratio = tick / 5
        x = left + ratio * plot_width
        time_value = time_min + ratio * (time_max - time_min)
        lines.append(
            f'<line x1="{x:.1f}" y1="{top:.1f}" x2="{x:.1f}" '
            f'y2="{bottom:.1f}" stroke="#e2e8f0" stroke-width="1"/>'
        )
        lines.append(
            _svg_text(x, bottom + 19, _nice_number(time_value), 11, "middle")
        )

    for tick in range(5):
        ratio = tick / 4
        y = bottom - ratio * plot_height
        value = value_min + ratio * (value_max - value_min)
        lines.append(
            f'<line x1="{left:.1f}" y1="{y:.1f}" '
            f'x2="{left + plot_width:.1f}" y2="{y:.1f}" '
            f'stroke="#e2e8f0" stroke-width="1"/>'
        )
        lines.append(
            _svg_text(left - 9, y + 4, _nice_number(value), 11, "end")
        )

    lines.extend(
        [
            f'<line x1="{left:.1f}" y1="{top:.1f}" x2="{left:.1f}" '
            f'y2="{bottom:.1f}" stroke="#64748b" stroke-width="1"/>',
            f'<line x1="{left:.1f}" y1="{bottom:.1f}" '
            f'x2="{left + plot_width:.1f}" y2="{bottom:.1f}" '
            f'stroke="#64748b" stroke-width="1"/>',
            _svg_text(left - 9, top - 7, unit, 11, "end", "#64748b"),
            _svg_text(
                left + plot_width,
                bottom + 31,
                "时间 (s)",
                11,
                "end",
                "#64748b",
            ),
        ]
    )

    for spec, points in sampled:
        dash_attribute = (
            f'stroke-dasharray="{spec.dash}" ' if spec.dash else ""
        )
        coordinates = []
        for time_value, value in points:
            x = left + (
                (time_value - time_min) / (time_max - time_min) * plot_width
            )
            y = bottom - (
                (value - value_min) / (value_max - value_min) * plot_height
            )
            coordinates.append(f"{x:.1f},{y:.1f}")
        if len(coordinates) == 1:
            x, y = coordinates[0].split(",")
            lines.append(
                f'<circle cx="{x}" cy="{y}" r="2.5" fill="{spec.color}"/>'
            )
        elif coordinates:
            lines.append(
                f'<polyline points="{" ".join(coordinates)}" fill="none" '
                f'stroke="{spec.color}" stroke-width="1.8" '
                f'{dash_attribute}'
                f'stroke-linejoin="round" stroke-linecap="round"/>'
            )

    return lines


def write_svg(
    records: Sequence[Dict[str, object]],
    header: FlightLogHeader,
    output_path: Path,
) -> None:
    panels = [
        (
            "角加速度",
            "°/s²",
            [
                SeriesSpec("angular_accel_x_dps2", "X / 横滚", "#ef4444"),
                SeriesSpec("angular_accel_y_dps2", "Y / 俯仰", "#22c55e"),
                SeriesSpec("angular_accel_z_dps2", "Z / 偏航", "#3b82f6"),
            ],
            None,
        ),
        (
            "线加速度",
            "mg",
            [
                SeriesSpec("accel_x_mg", "X", "#ef4444"),
                SeriesSpec("accel_y_mg", "Y", "#22c55e"),
                SeriesSpec("accel_z_mg", "Z", "#3b82f6"),
            ],
            None,
        ),
        (
            "姿态角",
            "°",
            [
                SeriesSpec("roll_deg", "横滚", "#ef4444"),
                SeriesSpec("pitch_deg", "俯仰", "#22c55e"),
                SeriesSpec("yaw_deg", "偏航", "#3b82f6"),
            ],
            None,
        ),
        (
            "角速度与目标角速度",
            "°/s",
            [
                SeriesSpec("roll_rate_dps", "横滚角速度", "#ef4444"),
                SeriesSpec("roll_rate_target_dps", "横滚目标", "#f59e0b"),
                SeriesSpec("pitch_rate_dps", "俯仰角速度", "#22c55e"),
                SeriesSpec("pitch_rate_target_dps", "俯仰目标", "#0ea5e9"),
                SeriesSpec("yaw_rate_dps", "偏航角速度", "#3b82f6"),
            ],
            None,
        ),
        (
            "气压",
            "hPa",
            [SeriesSpec("pressure_pa", "BMP280", "#8b5cf6", 0.01)],
            None,
        ),
        (
            "相对高度",
            "m",
            [SeriesSpec("relative_altitude_m", "高度", "#0ea5e9")],
            None,
        ),
        (
            "高度控制预测",
            "m / °",
            [
                SeriesSpec("relative_altitude_m", "实测高度", "#0ea5e9"),
                SeriesSpec("predicted_apogee_m", "预测最高点", "#f43f5e"),
                SeriesSpec(
                    "suggested_deflection_deg",
                    "建议舵角",
                    "#22c55e",
                ),
                SeriesSpec(
                    "applied_altitude_deflection_deg",
                    "实际混入舵角",
                    "#a855f7",
                ),
            ],
            None,
        ),
        (
            "PID输出",
            "μs偏移量",
            [
                SeriesSpec("roll_pid_output_us", "横滚PID", "#ef4444"),
                SeriesSpec("pitch_pid_output_us", "俯仰PID", "#22c55e"),
            ],
            None,
        ),
        (
            "高度控制代价项",
            "加权代价值",
            [
                SeriesSpec("height_error_cost", "高度误差项", "#dc2626"),
                SeriesSpec("deflection_cost", "舵角大小项", "#2563eb"),
                SeriesSpec(
                    "deflection_change_cost", "舵角变化项", "#16a34a"
                ),
            ],
            None,
        ),
        (
            "高度优先舵角分配",
            "°",
            [
                SeriesSpec(
                    "height_priority_requested_deflection_deg",
                    "高度请求舵角",
                    "#dc2626",
                ),
                SeriesSpec(
                    "height_priority_applied_deflection_deg",
                    "高度实际舵角",
                    "#16a34a",
                ),
            ],
            None,
        ),
        (
            "高度优先后的姿态PID",
            "μs",
            [
                SeriesSpec("roll_pid_output_us", "横滚原始PID", "#fca5a5"),
                SeriesSpec(
                    "height_priority_allocated_roll_pid_us",
                    "横滚实际PID",
                    "#dc2626",
                ),
                SeriesSpec("pitch_pid_output_us", "俯仰原始PID", "#86efac"),
                SeriesSpec(
                    "height_priority_allocated_pitch_pid_us",
                    "俯仰实际PID",
                    "#16a34a",
                ),
                SeriesSpec(
                    "height_priority_applied_offset_us",
                    "高度占用PWM",
                    "#2563eb",
                ),
            ],
            None,
        ),
        (
            "实际舵机PWM",
            "μs",
            [
                SeriesSpec("roll_servo_pwm_us", "横滚PWM", "#ef4444"),
                SeriesSpec("pitch_servo_pwm_us", "俯仰PWM", "#22c55e"),
            ],
            None,
        ),
        (
            "传感器样本延迟",
            "ms",
            [
                SeriesSpec("imu_sample_age_ms", "IMU延迟", "#2563eb"),
                SeriesSpec("barometer_sample_age_ms", "气压计延迟", "#9333ea"),
            ],
            None,
        ),
        (
            "垂直速度",
            "m/s",
            [
                SeriesSpec("barometer_vertical_speed_mps", "气压计垂直速度", "#0ea5e9"),
                SeriesSpec("imu_vertical_speed_mps", "IMU垂直速度", "#f97316"),
                SeriesSpec("vertical_speed_mps", "融合垂直速度", "#6366f1", 1.0, "8 5"),
            ],
            None,
        ),
        (
            "垂直加速度",
            "m/s²",
            [SeriesSpec("vertical_accel_mps2", "垂直加速度", "#dc2626")],
            None,
        ),
        (
            "温度",
            "°C",
            [SeriesSpec("temperature_c", "BMP280", "#f97316")],
            None,
        ),
        (
            "飞行与传感器状态",
            "0 / 1",
            [
                SeriesSpec("ignition_active", "点火", "#dc2626"),
                SeriesSpec("parachute_deployed", "已开伞", "#2563eb"),
                SeriesSpec("imu_valid", "IMU有效", "#16a34a"),
                SeriesSpec("barometer_valid", "BMP280有效", "#9333ea"),
                SeriesSpec("altitude_zero_ready", "高度零点就绪", "#ea580c"),
                SeriesSpec(
                    "altitude_estimator_valid", "高度估计有效", "#0f766e"
                ),
                SeriesSpec(
                    "altitude_prediction_valid", "最高点预测有效", "#be123c"
                ),
                SeriesSpec(
                    "altitude_control_allowed", "允许高度控制", "#7c3aed"
                ),
                SeriesSpec(
                    "altitude_imu_estimator_valid", "IMU速度估计有效", "#0891b2"
                ),
                SeriesSpec(
                    "flight_control_running", "飞行控制运行", "#4f46e5"
                ),
            ],
            (-0.1, 1.1),
        ),
        (
            "控制配置状态",
            "0 / 1",
            [
                SeriesSpec("altitude_mix_enabled", "全流程高度混控", "#7c3aed"),
                SeriesSpec(
                    "servo_altitude_mix_enabled", "舵机内部高度混控", "#ea580c"
                ),
            ],
            (-0.1, 1.1),
        ),
        (
            "火箭状态机",
            "状态码",
            [SeriesSpec("rocket_state", "火箭状态", "#0f172a")],
            None,
        ),
        (
            "IMU诊断数据",
            "计数 / 代码",
            [
                SeriesSpec("imu_status", "状态", "#0f172a"),
                SeriesSpec("imu_last_fault", "最后故障", "#dc2626"),
                SeriesSpec("imu_fifo_level", "FIFO水位", "#2563eb"),
                SeriesSpec("imu_init_count", "初始化次数", "#9333ea"),
                SeriesSpec(
                    "imu_consecutive_bus_errors", "连续总线错误", "#f43f5e"
                ),
                SeriesSpec(
                    "imu_total_bus_errors", "总线错误总数", "#ea580c"
                ),
                SeriesSpec(
                    "imu_fifo_reset_count", "FIFO复位次数", "#16a34a"
                ),
            ],
            None,
        ),
        (
            "BMP280与I²C诊断",
            "代码 / 0 / 1",
            [
                SeriesSpec("bmp280_init_result", "BMP初始化结果", "#dc2626"),
                SeriesSpec("bmp280_initialized", "BMP已初始化", "#16a34a"),
                SeriesSpec("i2c_ack_aht20_0x38", "AHT20应答", "#0ea5e9"),
                SeriesSpec("i2c_ack_bmp280_0x76", "BMP 0x76应答", "#f59e0b"),
                SeriesSpec("i2c_ack_bmp280_0x77", "BMP 0x77应答", "#8b5cf6"),
            ],
            (-0.1, 1.1),
        ),
        (
            "传感器样本时间戳",
            "ms",
            [
                SeriesSpec("imu_sample_timestamp_ms", "IMU时间戳", "#2563eb"),
                SeriesSpec(
                    "barometer_sample_timestamp_ms", "气压计时间戳", "#9333ea"
                ),
            ],
            None,
        ),
    ]
    width = 1400
    panel_height = 190
    config_items = list(header.config.items())
    config_columns = 4 if len(config_items) > 24 else 3
    rows_per_column = (
        math.ceil(len(config_items) / config_columns) if config_items else 0
    )
    header_height = max(250, 120 + rows_per_column * 17) if config_items else 76
    height = header_height + panel_height * len(panels) + 20
    start_time = float(records[0]["time_s"])
    end_time = float(records[-1]["time_s"])

    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" '
        f'height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#f1f5f9"/>',
        '<g font-family="Microsoft YaHei, SimHei, Noto Sans CJK SC, Arial, sans-serif">',
        _svg_text(30, 31, "火箭飞行日志综合图表", 23, weight="bold"),
        _svg_text(
            30,
            55,
            (
                f"格式 v{header.version} · {len(records)} 条记录 · "
                f"采样频率 {header.frequency_hz} Hz · "
                f"{start_time:.3f}–{end_time:.3f} s"
            ),
            13,
            color="#64748b",
        ),
    ]

    if config_items:
        lines.append(_svg_text(30, 82, "配置快照", 15, weight="bold"))
        column_width = (width - 60) / config_columns
        for index, (name, value) in enumerate(config_items):
            column = index // rows_per_column
            row = index % rows_per_column
            x = 30 + column * column_width
            y = 105 + row * 17
            label = CONFIG_LABELS.get(name, name.removeprefix("config_"))
            lines.append(_svg_text(x, y, f"{label}: {value}", 11, color="#475569"))

        imu_status = str(records[0].get("imu_status_name", ""))
        bmp_status = str(records[0].get("bmp280_init_result_name", ""))
        runtime_summary = (
            f"BMP280地址：{records[0].get('bmp280_i2c_address', '')} · "
            f"IMU状态：{STATUS_NAMES_ZH.get(imu_status, imu_status)} · "
            f"BMP280初始化：{STATUS_NAMES_ZH.get(bmp_status, bmp_status)}"
        )
        lines.append(
            _svg_text(30, header_height - 18, runtime_summary, 12, color="#334155")
        )

    for index, (title, unit, series, fixed_range) in enumerate(panels):
        panel_top = header_height + index * panel_height
        lines.extend(
            _render_panel(
                records,
                title,
                unit,
                series,
                panel_top,
                width,
                panel_height,
                fixed_range,
            )
        )
    lines.extend(["</g>", "</svg>"])
    output_path.write_text("\n".join(lines), encoding="utf-8")


def export_log(input_path: Path, output_dir: Path) -> Tuple[Path, Path, int]:
    header, records = read_log(input_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    csv_path = output_dir / "flight_log.csv"
    svg_path = output_dir / "flight_log_charts.svg"
    write_csv(records, csv_path)
    write_svg(records, header, svg_path)
    return csv_path, svg_path, len(records)


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Receive or parse W25Q64 flight logs and export CSV/SVG."
    )
    parser.add_argument(
        "input",
        nargs="?",
        type=Path,
        help="Existing USART1 text dump or W25Q64 raw binary. Omit to receive from serial.",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        help="Output directory.",
    )
    parser.add_argument(
        "--receive",
        action="store_true",
        help="Receive from serial. Enabled by default when input is omitted.",
    )
    parser.add_argument(
        "--list-ports",
        action="store_true",
        help="List detected serial ports and exit.",
    )
    parser.add_argument(
        "--port",
        help="Serial port, for example COM3. Auto-select when omitted.",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="Serial baudrate, default 115200.",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=60.0,
        help="Timeout in seconds while waiting for reset or receive progress.",
    )
    return parser

def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_argument_parser()
    args = parser.parse_args(argv)

    try:
        if args.list_ports:
            print_serial_ports(find_serial_ports())
            return 0

        receive_mode = args.receive or args.input is None
        if args.receive and args.input is not None:
            parser.error("--receive cannot be used with an input file")
        if args.baudrate <= 0:
            parser.error("--baudrate must be greater than zero")
        if args.timeout <= 0:
            parser.error("--timeout must be greater than zero")

        if receive_mode:
            ports = find_serial_ports()
            print_serial_ports(ports)
            selected_port = select_serial_port(args.port, ports)
            output_dir = args.output
            if output_dir is None:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                output_dir = Path.cwd() / f"flight_log_{timestamp}"
            output_dir.mkdir(parents=True, exist_ok=True)
            capture_path = output_dir / "serial_capture.txt"
            receive_from_port(
                str(selected_port.device),
                args.baudrate,
                capture_path,
                args.timeout,
            )
            input_path = capture_path
        else:
            input_path = args.input
            if input_path is None or not input_path.is_file():
                parser.error(f"输入文件不存在：{input_path}")
            output_dir = args.output
            if output_dir is None:
                output_dir = input_path.with_name(
                    f"{input_path.stem}_export"
                )

        header, records = read_log(input_path)
        output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = output_dir / "flight_log.csv"
        svg_path = output_dir / "flight_log_charts.svg"
        write_csv(records, csv_path)
        write_svg(records, header, svg_path)
    except (
        OSError,
        RuntimeError,
        TimeoutError,
        ValueError,
        struct.error,
    ) as error:
        print(f"错误：{error}", file=sys.stderr)
        return 2

    duration_s = (
        float(records[-1]["time_s"]) - float(records[0]["time_s"])
    )
    print(f"日志格式：v{header.version}，记录长度 {header.record_size} 字节")
    print(f"detected RLOG v{header.version}, record size {header.record_size} bytes")
    print(f"CSV：{csv_path.resolve()}")
    print(f"图表：{svg_path.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
