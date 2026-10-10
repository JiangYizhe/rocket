#!/usr/bin/env python3
"""flight_log_plot.py 的主机侧解析回归测试。"""

import csv
import struct
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "Tools"))

import flight_log_plot  # noqa: E402


def build_v2_image() -> bytes:
    image = bytearray(b"\xFF" * (256 + 3 * 44))
    image[:32] = struct.pack(
        "<4sHHHHIIIII",
        b"RLOG",
        2,
        256,
        44,
        50,
        20000,
        1000,
        256,
        0,
        45056,
    )
    records = [
        {
            "timestamp": 1000,
            "motion": (10, -20, 30, 100, -200, 1000, 1250, -250, 9000),
            "ignition": 0,
            "parachute": 0,
            "sequence": 0,
            "pressure_q24_8": int(101325.0 * 256),
            "altitude_cm": 0,
            "temperature_centi_c": 2512,
            "zero_ready": 1,
        },
        {
            "timestamp": 1020,
            "motion": (11, -18, 35, 110, -190, 990, 1300, -225, 9010),
            "ignition": 1,
            "parachute": 0,
            "sequence": 1,
            "pressure_q24_8": int(101300.5 * 256),
            "altitude_cm": 208,
            "temperature_centi_c": 2515,
            "zero_ready": 1,
        },
    ]

    for index, values in enumerate(records):
        offset = 256 + index * 44
        record = bytearray(44)
        struct.pack_into("<I", record, 0, values["timestamp"])
        struct.pack_into("<9h", record, 4, *values["motion"])
        record[22] = values["ignition"]
        record[23] = values["parachute"]
        record[24] = 1
        record[25] = 1
        struct.pack_into("<I", record, 26, values["sequence"])
        struct.pack_into("<H", record, 30, 0xA55A)
        struct.pack_into("<I", record, 32, values["pressure_q24_8"])
        struct.pack_into("<i", record, 36, values["altitude_cm"])
        struct.pack_into("<h", record, 40, values["temperature_centi_c"])
        record[42] = values["zero_ready"]
        record[43] = 0xB2
        image[offset : offset + 44] = record
    return bytes(image)


def build_v3_image() -> bytes:
    image = bytearray(b"\xFF" * (256 + 3 * 56))
    image[:32] = struct.pack(
        "<4sHHHHIIIII",
        b"RLOG",
        3,
        256,
        56,
        50,
        20000,
        1000,
        256,
        0,
        57344,
    )
    diagnostics = [
        {
            "status": 1,
            "fault": 0,
            "consecutive_errors": 0,
            "fifo_level": 4,
            "init_count": 1,
            "total_errors": 0,
            "fifo_resets": 0,
        },
        {
            "status": 2,
            "fault": 3,
            "consecutive_errors": 1,
            "fifo_level": 12,
            "init_count": 2,
            "total_errors": 4,
            "fifo_resets": 1,
        },
    ]

    for index, values in enumerate(diagnostics):
        offset = 256 + index * 56
        record = bytearray(56)
        struct.pack_into("<I", record, 0, 1000 + index * 20)
        struct.pack_into(
            "<9h",
            record,
            4,
            10 + index,
            -20,
            30,
            100,
            -200,
            1000,
            1250,
            -250,
            9000,
        )
        record[22] = index
        record[24] = 1
        record[25] = 1
        struct.pack_into("<I", record, 26, index)
        struct.pack_into("<H", record, 30, 0xA55A)
        struct.pack_into("<I", record, 32, int(101325.0 * 256))
        struct.pack_into("<i", record, 36, 208 if index else 0)
        struct.pack_into("<h", record, 40, 2512)
        record[42] = 1
        record[43] = 0xB2
        record[44] = values["status"]
        record[45] = values["fault"]
        record[46] = values["consecutive_errors"]
        record[47] = 0xD3
        struct.pack_into("<H", record, 48, values["fifo_level"])
        struct.pack_into("<H", record, 50, values["init_count"])
        struct.pack_into("<H", record, 52, values["total_errors"])
        struct.pack_into("<H", record, 54, values["fifo_resets"])
        image[offset : offset + 56] = record
    return bytes(image)


def build_v6_image() -> bytes:
    image = bytearray(b"\xFF" * (256 + 3 * 112))
    image[:32] = struct.pack(
        "<4sHHHHIIIII",
        b"RLOG",
        6,
        256,
        112,
        50,
        20000,
        1000,
        256,
        0,
        114688,
    )
    image[32:36] = b"CFG6"
    struct.pack_into("<i", image, 36, 8000)
    struct.pack_into("<i", image, 40, 10000)
    struct.pack_into("<i", image, 44, 75000)
    struct.pack_into("<i", image, 48, 1200000)
    struct.pack_into("<i", image, 52, 500000)
    struct.pack_into("<i", image, 56, 20000)
    struct.pack_into("<i", image, 60, 100000)
    struct.pack_into("<hhhhhhh", image, 64, 1000, 100, -100, 18000, 0, 0, 100)
    struct.pack_into("<III", image, 78, 3000, 2000, 15000)
    struct.pack_into("<HHHH", image, 90, 10, 40, 10, 20)
    image[98] = 1
    image[99] = 0

    for index in range(2):
        offset = 256 + index * 112
        record = bytearray(112)
        timestamp = 5100 + index * 20
        struct.pack_into("<I", record, 0, timestamp)
        struct.pack_into(
            "<9h", record, 4, 10, -20, 30, 100, -200, -1000, 100, -50, 9000
        )
        record[24] = 1
        record[25] = 1
        struct.pack_into("<I", record, 26, index)
        struct.pack_into("<H", record, 30, 0xA55A)
        struct.pack_into("<I", record, 32, int(101325.0 * 256))
        struct.pack_into("<i", record, 36, index * 25)
        struct.pack_into("<h", record, 40, 2512)
        record[42] = 1
        record[43] = 0xB2
        record[44] = 1
        record[47] = 0xD3
        struct.pack_into("<hih", record, 56, 1234, 8450, 300)
        record[64] = 1
        record[65] = 1
        record[66] = 1
        record[67] = 0xA4
        struct.pack_into("<hhh", record, 68, 1100, 1300, 25)
        record[74] = 1
        record[76] = 0x77
        record[77] = 1
        record[78] = 0x05
        record[79] = 4
        struct.pack_into("<hhhhhhh", record, 80, 15, -25, 3, 200, -300, 450, -550)
        struct.pack_into("<HHhII", record, 94, 1545, 1445, 300, timestamp - 2, timestamp - 18)
        record[108] = 1
        record[109] = 1
        record[110] = 0
        record[111] = 0xC6
        image[offset : offset + 112] = record
    return bytes(image)


def build_v7_image() -> bytes:
    v6_image = build_v6_image()
    image = bytearray(b"\xFF" * (256 + 3 * 128))
    image[:256] = v6_image[:256]
    struct.pack_into("<H", image, 4, 7)
    struct.pack_into("<H", image, 8, 128)
    image[32:36] = b"CFG7"
    struct.pack_into("<i", image, 100, 1000000)
    struct.pack_into("<i", image, 104, 1000)
    struct.pack_into("<i", image, 108, 3000)
    struct.pack_into("<h", image, 112, 300)

    for index in range(2):
        old_offset = 256 + index * 112
        new_offset = 256 + index * 128
        record = bytearray(128)
        record[:112] = v6_image[old_offset : old_offset + 112]
        struct.pack_into("<iii", record, 112, 80560, 2025, 27)
        record[127] = 0xC7
        image[new_offset : new_offset + 128] = record
    return bytes(image)


def build_v8_image() -> bytes:
    v7_image = build_v7_image()
    image = bytearray(b"\xFF" * (256 + 3 * 144))
    image[:256] = v7_image[:256]
    struct.pack_into("<H", image, 4, 8)
    struct.pack_into("<H", image, 8, 144)
    image[32:36] = b"CFG8"
    struct.pack_into("<hhh", image, 114, 500, 1000, 3000)

    for index in range(2):
        old_offset = 256 + index * 128
        new_offset = 256 + index * 144
        record = bytearray(144)
        record[:128] = v7_image[old_offset : old_offset + 128]
        struct.pack_into(
            "<hhhhhh", record, 128, 4500, 2250, 5000, 1584, 2500, 1250
        )
        record[140] = 0x03
        record[143] = 0xC8
        image[new_offset : new_offset + 144] = record
    return bytes(image)


def build_v9_image() -> bytes:
    v8_image = build_v8_image()
    image = bytearray(v8_image)
    struct.pack_into("<H", image, 4, 9)
    image[32:36] = b"CFG9"
    image[114] = 1

    for index in range(2):
        offset = 256 + index * 144
        struct.pack_into("<hhhhh", image, offset + 128,
                         4500, 4500, 5000, -6000, 2500)
        image[offset + 138] = 0x03
        image[offset + 139] = 1
        image[offset + 143] = 0xC9
    return bytes(image)


def to_usart_dump(image: bytes) -> bytes:
    padded = image + b"\xFF" * ((16 - len(image) % 16) % 16)
    lines = [b"=== PHYSICAL DEBUG MODE ==="]
    for address in range(0, len(padded), 16):
        values = padded[address : address + 16]
        data = b" ".join(f"{value:02X}".encode() for value in values)
        lines.append(f"{address:06X}:".encode() + b" " + data)
    lines.append(b"--- W25Q64 DUMP RESTART ---")
    return b"\r\n".join(lines)


class FakePort:
    def __init__(
        self,
        device: str,
        description: str,
        hwid: str = "",
        vid=None,
    ) -> None:
        self.device = device
        self.description = description
        self.manufacturer = ""
        self.hwid = hwid
        self.vid = vid


class FakeSerialConnection:
    def __init__(self, data: bytes) -> None:
        self.lines = iter(data.splitlines(keepends=True))

    def readline(self) -> bytes:
        return next(self.lines, b"")


class FlightLogPlotTests(unittest.TestCase):
    def test_parse_v9_height_priority_allocator(self) -> None:
        image = build_v9_image()
        header = flight_log_plot.parse_header(image)
        records = flight_log_plot.parse_records(image, header)

        self.assertEqual(header.version, 9)
        self.assertEqual(header.record_size, 144)
        self.assertEqual(len(records), 2)
        self.assertAlmostEqual(
            records[0]["height_priority_requested_deflection_deg"], 45.0
        )
        self.assertAlmostEqual(
            records[0]["height_priority_applied_deflection_deg"], 45.0
        )
        self.assertAlmostEqual(
            records[0]["height_priority_allocated_roll_pid_us"], 50.0
        )
        self.assertAlmostEqual(
            records[0]["height_priority_allocated_pitch_pid_us"], -60.0
        )
        self.assertAlmostEqual(
            records[0]["height_priority_applied_offset_us"], 250.0
        )
        self.assertEqual(
            records[0]["height_priority_attitude_limit_reason"], 3
        )
        self.assertEqual(
            records[0]["height_priority_attitude_limit_reason_name"],
            "roll_pid_limited|pitch_pid_limited",
        )
        self.assertEqual(records[0]["height_priority_enabled"], 1)
        self.assertEqual(records[0]["config_height_priority_enable"], 1)

    def test_parse_v8_attitude_priority_allocator(self) -> None:
        image = build_v8_image()
        header = flight_log_plot.parse_header(image)
        records = flight_log_plot.parse_records(image, header)

        self.assertEqual(header.version, 8)
        self.assertEqual(header.record_size, 144)
        self.assertEqual(len(records), 2)
        self.assertAlmostEqual(
            records[0]["allocator_requested_deflection_deg"], 45.0
        )
        self.assertAlmostEqual(
            records[0]["allocator_attitude_scaled_deflection_deg"], 22.5
        )
        self.assertAlmostEqual(records[0]["allocator_attitude_scale"], 0.5)
        self.assertAlmostEqual(records[0]["allocator_roll_headroom_us"], 158.4)
        self.assertAlmostEqual(records[0]["allocator_pitch_headroom_us"], 250.0)
        self.assertAlmostEqual(records[0]["allocator_applied_offset_us"], 125.0)
        self.assertEqual(records[0]["allocator_limit_reason"], 3)
        self.assertEqual(
            records[0]["allocator_limit_reason_name"],
            "attitude|roll_headroom",
        )
        self.assertAlmostEqual(records[0]["config_attitude_reserve_us"], 50.0)
        self.assertAlmostEqual(records[0]["config_attitude_full_deg"], 10.0)
        self.assertAlmostEqual(records[0]["config_attitude_zero_deg"], 30.0)

    def test_parse_v7_altitude_cost_telemetry(self) -> None:
        image = build_v7_image()
        header = flight_log_plot.parse_header(image)
        records = flight_log_plot.parse_records(image, header)

        self.assertEqual(header.version, 7)
        self.assertEqual(header.record_size, 128)
        self.assertEqual(len(records), 2)
        self.assertAlmostEqual(records[0]["height_error_cost"], 805.6)
        self.assertAlmostEqual(records[0]["deflection_cost"], 0.002025)
        self.assertAlmostEqual(records[0]["deflection_change_cost"], 0.000027)
        self.assertAlmostEqual(records[0]["config_cost_height_weight"], 1.0)
        self.assertAlmostEqual(
            records[0]["config_cost_deflection_weight"], 0.000001
        )
        self.assertAlmostEqual(
            records[0]["config_cost_deflection_change_weight"], 0.000003
        )
        self.assertAlmostEqual(
            records[0]["config_max_deflection_step_deg"], 3.0
        )

    def test_parse_v6_control_telemetry_and_config(self) -> None:
        image = build_v6_image()
        header = flight_log_plot.parse_header(image)
        records = flight_log_plot.parse_records(image, header)

        self.assertEqual(header.version, 6)
        self.assertEqual(header.record_size, 112)
        self.assertEqual(len(records), 2)
        self.assertAlmostEqual(records[0]["roll_rate_dps"], 1.5)
        self.assertAlmostEqual(records[0]["pitch_rate_target_dps"], -3.0)
        self.assertAlmostEqual(records[0]["roll_pid_output_us"], 4.5)
        self.assertEqual(records[0]["roll_servo_pwm_us"], 1545)
        self.assertAlmostEqual(
            records[0]["applied_altitude_deflection_deg"], 3.0
        )
        self.assertEqual(records[0]["imu_sample_age_ms"], 2)
        self.assertEqual(records[0]["barometer_sample_age_ms"], 18)
        self.assertAlmostEqual(records[0]["config_target_altitude_m"], 80.0)
        self.assertAlmostEqual(records[0]["config_base_cd"], 0.5)
        self.assertEqual(
            records[0]["config_full_flow_altitude_mix_enable"], 1
        )

    def test_parse_binary_v2(self) -> None:
        image = build_v2_image()
        header = flight_log_plot.parse_header(image)
        records = flight_log_plot.parse_records(image, header)

        self.assertEqual(header.version, 2)
        self.assertEqual(header.record_size, 44)
        self.assertEqual(len(records), 2)
        self.assertEqual(records[1]["sequence"], 1)
        self.assertEqual(records[1]["ignition_active"], 1)
        self.assertAlmostEqual(records[1]["pressure_pa"], 101300.5)
        self.assertAlmostEqual(records[1]["relative_altitude_m"], 2.08)
        self.assertAlmostEqual(records[0]["temperature_c"], 25.12)
        self.assertAlmostEqual(records[0]["roll_deg"], 12.5)

    def test_parse_usart_dump_and_export_v3(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            input_path = root / "serial_capture.txt"
            output_dir = root / "export"
            input_path.write_bytes(to_usart_dump(build_v3_image()))

            csv_path, svg_path, count = flight_log_plot.export_log(
                input_path, output_dir
            )

            self.assertEqual(count, 2)
            self.assertTrue(csv_path.is_file())
            self.assertTrue(svg_path.is_file())
            with csv_path.open(encoding="utf-8-sig", newline="") as csv_file:
                rows = list(csv.DictReader(csv_file))
            self.assertEqual(rows[1]["relative_altitude_m"], "2.08")
            self.assertEqual(rows[1]["ignition_active"], "1")
            self.assertEqual(rows[1]["imu_status_name"], "bus_error")
            self.assertEqual(
                rows[1]["imu_last_fault_name"], "fifo_reset_failed"
            )
            self.assertEqual(rows[1]["imu_fifo_level"], "12")
            self.assertEqual(rows[1]["imu_init_count"], "2")
            self.assertEqual(rows[1]["imu_total_bus_errors"], "4")
            self.assertEqual(rows[1]["imu_fifo_reset_count"], "1")
            ET.parse(svg_path)
            svg = svg_path.read_text(encoding="utf-8")
            self.assertIn("火箭飞行日志综合图表", svg)
            self.assertIn("相对高度", svg)
            self.assertIn("飞行与传感器状态", svg)
            self.assertIn("IMU诊断数据", svg)

    def test_auto_selects_usb_serial_adapter(self) -> None:
        ports = [
            FakePort("COM1", "Communications Port"),
            FakePort(
                "COM7",
                "USB-SERIAL CH340",
                "USB VID:PID=1A86:7523",
                0x1A86,
            ),
        ]

        selected = flight_log_plot.select_serial_port(None, ports)

        self.assertEqual(selected.device, "COM7")
        self.assertEqual(
            flight_log_plot.select_serial_port("com1", ports).device,
            "COM1",
        )

    def test_receive_serial_dump_until_first_erased_record(self) -> None:
        with tempfile.TemporaryDirectory() as temporary_directory:
            capture_path = Path(temporary_directory) / "serial_capture.txt"
            connection = FakeSerialConnection(
                to_usart_dump(build_v3_image())
            )

            header = flight_log_plot.receive_serial_dump(
                connection, capture_path, timeout_s=1.0
            )
            parsed_header, records = flight_log_plot.read_log(capture_path)

            self.assertEqual(header.version, 3)
            self.assertEqual(parsed_header.record_size, 56)
            self.assertEqual(len(records), 2)
            self.assertTrue(capture_path.read_bytes().startswith(b"000000:"))


if __name__ == "__main__":
    unittest.main()
