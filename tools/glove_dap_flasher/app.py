from __future__ import annotations

import hashlib
import json
import os
import sys
import traceback
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable


APP_DIR = Path(__file__).resolve().parent


def _bundle_root() -> Path:
    """返回源码目录或 PyInstaller 运行时资源目录。"""
    return Path(getattr(sys, "_MEIPASS", APP_DIR))


# 源码调试时从项目内加载依赖，打包后依赖已经进入可执行文件。
VENDOR_DIR = APP_DIR / ".vendor"
if VENDOR_DIR.is_dir():
    sys.path.insert(0, str(VENDOR_DIR))

from intelhex import IntelHex
from pyocd.core.session import Session
from pyocd.core.target import Target
from pyocd.flash.file_programmer import FileProgrammer
from pyocd.probe.cmsis_dap_probe import CMSISDAPProbe
from pyocd.target.pack.cmsis_pack import CmsisPack
from PySide6.QtCore import QEasingCurve, QObject, QPoint, QPropertyAnimation, QRectF, Qt, QThread, QTimer, Signal
from PySide6.QtGui import QColor, QFont, QIcon, QPainter, QPen
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QGraphicsDropShadowEffect,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QSizePolicy,
    QSpacerItem,
    QVBoxLayout,
    QWidget,
)


APP_NAME = "Glove DAP Flasher"
APP_VERSION = "1.0.2"
TARGET_NAME = "STM32H563VGTx"
FLASH_START = 0x08000000
FLASH_END = 0x08100000
FLASH_SIZE_REGISTER = 0x08FFF80C
UID_REGISTER = 0x08FFF800
DBGMCU_IDCODE_REGISTER = 0x44024000
SWD_FREQUENCY_HZ = 1_000_000


class UserFacingError(RuntimeError):
    pass


@dataclass(frozen=True)
class FirmwareInfo:
    product: str
    version: str
    build_time: str
    filename: str
    sha256: str
    byte_count: int


@dataclass(frozen=True)
class ProbeInfo:
    unique_id: str
    description: str


@dataclass(frozen=True)
class DeviceInfo:
    uid: str
    flash_kib: int
    idcode: int


def resource_path(relative_path: str) -> Path:
    return _bundle_root() / relative_path


def load_firmware_info() -> FirmwareInfo:
    metadata_path = resource_path("resources/firmware.json")
    firmware_dir = resource_path("resources/firmware")
    # Windows PowerShell 5.1 写入 UTF-8 JSON 时会带 BOM，读取时一并兼容。
    metadata = json.loads(metadata_path.read_text(encoding="utf-8-sig"))
    firmware_path = firmware_dir / metadata["filename"]
    digest = hashlib.sha256(firmware_path.read_bytes()).hexdigest()
    if digest.lower() != metadata["sha256"].lower():
        raise UserFacingError("内置固件校验失败，软件文件可能已经损坏。")

    image = IntelHex(str(firmware_path))
    segments = image.segments()
    if not segments:
        raise UserFacingError("内置固件没有有效数据。")
    if any(start < FLASH_START or end > FLASH_END for start, end in segments):
        raise UserFacingError("内置固件地址超出 STM32H563VGT6 的 Flash 范围。")

    return FirmwareInfo(
        product=metadata["product"],
        version=metadata["version"],
        build_time=metadata["build_time"],
        filename=metadata["filename"],
        sha256=digest,
        byte_count=sum(end - start for start, end in segments),
    )


def enumerate_dap_probes() -> list[ProbeInfo]:
    probes = CMSISDAPProbe.get_all_connected_probes()
    return [
        ProbeInfo(unique_id=probe.unique_id, description=probe.description or "CMSIS-DAP")
        for probe in probes
    ]


def run_self_test() -> int:
    """检查打包资源和 USB 后端，不连接也不改写目标芯片。"""
    firmware = load_firmware_info()
    pack = CmsisPack(resource_path("resources/pack"))
    targets = {device.part_number for device in pack.devices}
    if TARGET_NAME not in targets:
        raise UserFacingError(f"芯片支持包中缺少 {TARGET_NAME}。")

    probes = enumerate_dap_probes()
    print(f"firmware={firmware.version}")
    print(f"sha256={firmware.sha256}")
    print(f"target={TARGET_NAME}")
    print(f"dap_count={len(probes)}")
    for probe in probes:
        print(f"dap={probe.description}|{probe.unique_id}")
    print("self_test=ok")
    return 0


class DapProgrammer:
    def __init__(
        self,
        probe_uid: str,
        progress: Callable[[int, str], None],
        log: Callable[[str], None],
    ) -> None:
        self._probe_uid = probe_uid
        self._progress = progress
        self._log = log

    def program(self) -> DeviceInfo:
        firmware_info = load_firmware_info()
        firmware_path = resource_path(f"resources/firmware/{firmware_info.filename}")
        pack_path = resource_path("resources/pack")
        image = IntelHex(str(firmware_path))

        self._progress(4, "正在查找 CMSIS-DAP")
        probe = self._select_probe()
        self._log(f"DAP：{probe.description}")
        self._log(f"序列号：{probe.unique_id}")

        options = {
            "target_override": TARGET_NAME,
            "pack": str(pack_path),
            "dap_protocol": "swd",
            "frequency": SWD_FREQUENCY_HZ,
            # 下载器没有连接 NRST，使用 halt 模式和软件复位。
            "connect_mode": "halt",
            "auto_unlock": False,
            "resume_on_disconnect": True,
            "chip_erase": "sector",
            "keep_unwritten": False,
            "fast_program": False,
            "hide_programming_progress": True,
            "load.pre_reset": "off",
            "load.post_reset": "off",
            "flash.timeout.init": 15.0,
            "flash.timeout.erase_sector": 30.0,
            "flash.timeout.program": 30.0,
        }

        session = Session(probe, auto_open=False, options=options, command="load")
        try:
            self._progress(10, "正在连接目标芯片")
            session.open()
            target = session.target

            # 普通 halt 可能停在中断上下文中，先用 SYSRESETREQ 回到干净的线程上下文。
            # 这是 Cortex-M 软件复位，不需要 CMSIS-DAP 连接 NRST。
            target.reset_and_halt(Target.ResetType.SYSTEM)
            self._log("已执行软件系统复位并接管 CPU")

            device = self._read_device_info(target)
            self._log(f"芯片 UID：{device.uid}")
            self._log(f"Flash：{device.flash_kib} KiB")
            self._log(f"DBGMCU IDCODE：0x{device.idcode:08X}")

            if device.flash_kib != 1024:
                raise UserFacingError(
                    f"目标 Flash 容量为 {device.flash_kib} KiB，不是 STM32H563VGT6 的 1024 KiB。"
                )

            self._progress(18, "芯片检查通过")
            self._log(f"固件：{firmware_info.version}")
            self._log(f"有效数据：{firmware_info.byte_count / 1024:.1f} KiB")
            self._log(f"SHA-256：{firmware_info.sha256}")

            progress_phase = {"index": 0, "previous": 0.0}

            def on_program_progress(value: float) -> None:
                fraction = max(0.0, min(1.0, value))
                # pyOCD会分别汇报擦除和写入进度，第二阶段从0重新开始。
                if fraction + 0.05 < progress_phase["previous"]:
                    progress_phase["index"] = 1
                progress_phase["previous"] = fraction
                if progress_phase["index"] == 0:
                    percent = 20 + int(fraction * 25)
                else:
                    percent = 45 + int(fraction * 30)
                self._progress(percent, "正在擦除并写入固件")

            programmer = FileProgrammer(
                session,
                progress=on_program_progress,
                chip_erase="sector",
                smart_flash=False,
                trust_crc=False,
                keep_unwritten=False,
            )
            programmer.program(str(firmware_path), file_format="hex")

            self._progress(76, "正在读回校验")
            self._verify_image(target, image)

            self._progress(97, "正在启动设备")
            # 无 NRST 时使用 SYSRESETREQ，让程序从复位向量重新运行。
            target.reset()
            self._progress(100, "烧录成功")
            return device
        except UserFacingError:
            raise
        except Exception as exc:
            raise UserFacingError(self._translate_error(exc)) from exc
        finally:
            try:
                session.close()
            except Exception:
                pass

    def _select_probe(self):
        probes = CMSISDAPProbe.get_all_connected_probes(unique_id=self._probe_uid, is_explicit=True)
        if not probes:
            raise UserFacingError("未找到已选择的 CMSIS-DAP，请重新插拔下载器。")
        return probes[0]

    def _read_device_info(self, target) -> DeviceInfo:
        flash_kib = int(target.read16(FLASH_SIZE_REGISTER))
        uid_words = target.read_memory_block32(UID_REGISTER, 3)
        uid = "".join(f"{word:08X}" for word in reversed(uid_words))
        idcode = int(target.read32(DBGMCU_IDCODE_REGISTER))
        return DeviceInfo(uid=uid, flash_kib=flash_kib, idcode=idcode)

    def _verify_image(self, target, image: IntelHex) -> None:
        segments = image.segments()
        total = sum(end - start for start, end in segments)
        verified = 0
        chunk_size = 1024

        for start, end in segments:
            address = start
            while address < end:
                size = min(chunk_size, end - address)
                expected = bytes(image.tobinarray(start=address, size=size))
                actual = bytes(target.read_memory_block8(address, size))
                if actual != expected:
                    mismatch = next(
                        index for index, (left, right) in enumerate(zip(actual, expected)) if left != right
                    )
                    failed_address = address + mismatch
                    raise UserFacingError(f"烧录校验失败，地址 0x{failed_address:08X} 的数据不一致。")
                address += size
                verified += size
                percent = 76 + int((verified / total) * 20)
                self._progress(percent, "正在读回校验")

    @staticmethod
    def _translate_error(exc: Exception) -> str:
        message = str(exc)
        lowered = message.lower()
        if "no ack" in lowered or "transfer fault" in lowered or "transfer error" in lowered:
            return "目标芯片没有响应。请检查供电、SWDIO、SWCLK 和 GND，然后重新上电再试。"
        if "probe" in lowered and ("open" in lowered or "busy" in lowered):
            return "CMSIS-DAP 正被其他软件占用，请关闭 Keil、调试器或其他烧录软件。"
        if "timeout" in lowered:
            return "连接或烧录超时。请重新上电，并检查 SWD 线缆是否过长或接触不良。"
        if "permission" in lowered or "access denied" in lowered:
            return "Windows 无法访问 CMSIS-DAP，请检查下载器驱动或 USB 权限。"
        return f"烧录失败：{message or exc.__class__.__name__}"


class WorkerSignals(QObject):
    progress = Signal(int, str)
    log = Signal(str)
    completed = Signal(object)
    failed = Signal(str, str)


class FlashWorker(QThread):
    def __init__(self, probe_uid: str) -> None:
        super().__init__()
        self.signals = WorkerSignals()
        self._probe_uid = probe_uid

    def run(self) -> None:
        try:
            programmer = DapProgrammer(
                probe_uid=self._probe_uid,
                progress=self.signals.progress.emit,
                log=self.signals.log.emit,
            )
            device = programmer.program()
            self.signals.completed.emit(device)
        except Exception as exc:
            details = traceback.format_exc()
            self.signals.failed.emit(str(exc), details)


class ScanWorker(QThread):
    completed = Signal(object)
    failed = Signal(str)

    def run(self) -> None:
        try:
            self.completed.emit(enumerate_dap_probes())
        except Exception as exc:
            self.failed.emit(str(exc))


class ProgressRing(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self._value = 0
        self._caption = "等待连接"
        self.setFixedSize(190, 190)

    def set_progress(self, value: int, caption: str) -> None:
        self._value = max(0, min(100, value))
        self._caption = caption
        self.update()

    def paintEvent(self, event) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        rect = QRectF(14, 14, self.width() - 28, self.height() - 28)

        painter.setPen(QPen(QColor("#E8EDF4"), 12, Qt.SolidLine, Qt.RoundCap))
        painter.drawArc(rect, 0, 360 * 16)

        color = QColor("#2D7FF9") if self._value < 100 else QColor("#16A36A")
        painter.setPen(QPen(color, 12, Qt.SolidLine, Qt.RoundCap))
        painter.drawArc(rect, 90 * 16, -int(360 * 16 * self._value / 100))

        painter.setPen(QColor("#132238"))
        painter.setFont(QFont("Segoe UI", 30, QFont.Bold))
        painter.drawText(QRectF(0, 54, self.width(), 54), Qt.AlignCenter, f"{self._value}%")
        painter.setPen(QColor("#6C7C91"))
        painter.setFont(QFont("Microsoft YaHei UI", 10))
        painter.drawText(QRectF(20, 112, self.width() - 40, 44), Qt.AlignCenter | Qt.TextWordWrap, self._caption)


class StatusDot(QWidget):
    def __init__(self) -> None:
        super().__init__()
        self._color = QColor("#AAB5C3")
        self.setFixedSize(12, 12)

    def set_color(self, color: str) -> None:
        self._color = QColor(color)
        self.update()

    def paintEvent(self, event) -> None:
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setBrush(self._color)
        painter.setPen(Qt.NoPen)
        painter.drawEllipse(1, 1, 10, 10)


def add_shadow(widget: QWidget) -> None:
    effect = QGraphicsDropShadowEffect(widget)
    effect.setBlurRadius(28)
    effect.setOffset(QPoint(0, 8))
    effect.setColor(QColor(17, 32, 52, 28))
    widget.setGraphicsEffect(effect)


class MainWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self._probe: ProbeInfo | None = None
        self._scan_worker: ScanWorker | None = None
        self._flash_worker: FlashWorker | None = None
        self._busy = False
        self._last_probe_signature: tuple[tuple[str, str], ...] | None = None
        self._firmware = load_firmware_info()

        self.setWindowTitle(APP_NAME)
        self.setMinimumSize(980, 680)
        self.resize(1040, 720)
        icon_path = resource_path("resources/app_icon.svg")
        if icon_path.exists():
            self.setWindowIcon(QIcon(str(icon_path)))

        self._build_ui()
        self._apply_style()
        QTimer.singleShot(250, self.scan_probes)

        # 定时检测下载器的 USB 插拔状态；烧录期间 scan_probes 会自动跳过。
        self._probe_monitor = QTimer(self)
        self._probe_monitor.setInterval(1200)
        self._probe_monitor.timeout.connect(lambda: self.scan_probes(silent=True))
        self._probe_monitor.start()

    def _build_ui(self) -> None:
        root = QWidget()
        root.setObjectName("root")
        self.setCentralWidget(root)
        layout = QHBoxLayout(root)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        sidebar = QFrame()
        sidebar.setObjectName("sidebar")
        sidebar.setFixedWidth(286)
        side_layout = QVBoxLayout(sidebar)
        side_layout.setContentsMargins(30, 32, 30, 30)
        side_layout.setSpacing(0)

        logo = QLabel("G")
        logo.setObjectName("logo")
        logo.setAlignment(Qt.AlignCenter)
        logo.setFixedSize(52, 52)
        side_layout.addWidget(logo, 0, Qt.AlignLeft)

        brand = QLabel("Glove\nDAP Flasher")
        brand.setObjectName("brand")
        side_layout.addSpacing(18)
        side_layout.addWidget(brand)

        subtitle = QLabel("STM32H563 专用固件烧录工具")
        subtitle.setObjectName("sideSubtitle")
        subtitle.setWordWrap(True)
        side_layout.addSpacing(8)
        side_layout.addWidget(subtitle)
        side_layout.addSpacing(44)

        steps = [
            ("01", "连接下载器", "插入 CMSIS-DAP 并连接目标板"),
            ("02", "确认供电", "连接 SWDIO、SWCLK 与 GND"),
            ("03", "一键烧录", "自动写入、校验并启动设备"),
        ]
        for number, title, detail in steps:
            row = QHBoxLayout()
            number_label = QLabel(number)
            number_label.setObjectName("stepNumber")
            number_label.setFixedSize(38, 38)
            number_label.setAlignment(Qt.AlignCenter)
            text_layout = QVBoxLayout()
            text_layout.setSpacing(2)
            title_label = QLabel(title)
            title_label.setObjectName("stepTitle")
            detail_label = QLabel(detail)
            detail_label.setObjectName("stepDetail")
            detail_label.setWordWrap(True)
            text_layout.addWidget(title_label)
            text_layout.addWidget(detail_label)
            row.addWidget(number_label)
            row.addSpacing(12)
            row.addLayout(text_layout, 1)
            side_layout.addLayout(row)
            side_layout.addSpacing(24)

        side_layout.addStretch(1)
        version_label = QLabel(f"Application {APP_VERSION}")
        version_label.setObjectName("sideVersion")
        side_layout.addWidget(version_label)
        layout.addWidget(sidebar)

        content = QWidget()
        content.setObjectName("content")
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(38, 30, 38, 30)
        content_layout.setSpacing(20)

        header = QHBoxLayout()
        title_column = QVBoxLayout()
        title_column.setSpacing(4)
        title = QLabel("固件烧录")
        title.setObjectName("pageTitle")
        greeting = QLabel("连接设备后，点击按钮即可完成全部操作")
        greeting.setObjectName("pageSubtitle")
        title_column.addWidget(title)
        title_column.addWidget(greeting)
        header.addLayout(title_column)
        header.addStretch(1)
        self.refresh_button = QPushButton("重新检测")
        self.refresh_button.setObjectName("secondaryButton")
        self.refresh_button.clicked.connect(self.scan_probes)
        header.addWidget(self.refresh_button)
        content_layout.addLayout(header)

        status_row = QHBoxLayout()
        status_row.setSpacing(16)
        dap_card, dap_layout = self._make_info_card("CMSIS-DAP")
        dap_status = QHBoxLayout()
        self.status_dot = StatusDot()
        self.dap_value = QLabel("正在检测…")
        self.dap_value.setObjectName("cardValue")
        dap_status.addWidget(self.status_dot)
        dap_status.addSpacing(8)
        dap_status.addWidget(self.dap_value, 1)
        dap_layout.addLayout(dap_status)
        self.dap_detail = QLabel("请插入下载器")
        self.dap_detail.setObjectName("cardDetail")
        dap_layout.addWidget(self.dap_detail)

        fw_card, fw_layout = self._make_info_card("内置固件")
        fw_value = QLabel(self._firmware.version)
        fw_value.setObjectName("cardValue")
        fw_layout.addWidget(fw_value)
        fw_detail = QLabel(f"{self._firmware.byte_count / 1024:.1f} KiB  ·  {self._firmware.build_time}")
        fw_detail.setObjectName("cardDetail")
        fw_layout.addWidget(fw_detail)

        status_row.addWidget(dap_card, 1)
        status_row.addWidget(fw_card, 1)
        content_layout.addLayout(status_row)

        action_card = QFrame()
        action_card.setObjectName("card")
        add_shadow(action_card)
        action_layout = QHBoxLayout(action_card)
        action_layout.setContentsMargins(30, 26, 30, 26)
        action_layout.setSpacing(28)
        self.progress_ring = ProgressRing()
        action_layout.addWidget(self.progress_ring)

        action_column = QVBoxLayout()
        action_column.setSpacing(10)
        self.action_title = QLabel("准备烧录")
        self.action_title.setObjectName("actionTitle")
        self.action_description = QLabel("检测到 CMSIS-DAP 后即可开始。烧录期间请勿断开电源或 USB。")
        self.action_description.setObjectName("actionDescription")
        self.action_description.setWordWrap(True)
        self.flash_button = QPushButton("一键烧录")
        self.flash_button.setObjectName("primaryButton")
        self.flash_button.setMinimumHeight(52)
        self.flash_button.setEnabled(False)
        self.flash_button.clicked.connect(self.start_flash)
        action_column.addWidget(self.action_title)
        action_column.addWidget(self.action_description)
        action_column.addStretch(1)
        action_column.addWidget(self.flash_button)
        action_layout.addLayout(action_column, 1)
        content_layout.addWidget(action_card)

        log_card = QFrame()
        log_card.setObjectName("logCard")
        log_layout = QVBoxLayout(log_card)
        log_layout.setContentsMargins(20, 16, 20, 16)
        log_title = QLabel("运行记录")
        log_title.setObjectName("logTitle")
        self.log_view = QPlainTextEdit()
        self.log_view.setObjectName("logView")
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(500)
        self.log_view.setMinimumHeight(105)
        log_layout.addWidget(log_title)
        log_layout.addWidget(self.log_view)
        content_layout.addWidget(log_card, 1)

        layout.addWidget(content, 1)

    @staticmethod
    def _make_info_card(title: str) -> tuple[QFrame, QVBoxLayout]:
        card = QFrame()
        card.setObjectName("card")
        add_shadow(card)
        card_layout = QVBoxLayout(card)
        card_layout.setContentsMargins(22, 18, 22, 18)
        card_layout.setSpacing(7)
        title_label = QLabel(title)
        title_label.setObjectName("cardTitle")
        card_layout.addWidget(title_label)
        return card, card_layout

    def _apply_style(self) -> None:
        self.setStyleSheet(
            """
            QWidget { font-family: "Microsoft YaHei UI", "Segoe UI"; }
            #root, #content { background: #F4F7FB; }
            #sidebar { background: #0D1B2E; }
            #logo {
                color: white; background: #2D7FF9; border-radius: 15px;
                font-family: "Segoe UI"; font-size: 28px; font-weight: 800;
            }
            #brand { color: #FFFFFF; font-size: 27px; font-weight: 700; line-height: 1.15; }
            #sideSubtitle { color: #91A2BA; font-size: 12px; }
            #stepNumber {
                color: #78AEFF; background: #172A44; border: 1px solid #244467;
                border-radius: 12px; font-family: "Segoe UI"; font-weight: 700;
            }
            #stepTitle { color: #EAF1FA; font-size: 13px; font-weight: 600; }
            #stepDetail { color: #7589A4; font-size: 10px; }
            #sideVersion { color: #536A87; font-family: "Segoe UI"; font-size: 10px; }
            #pageTitle { color: #132238; font-size: 28px; font-weight: 700; }
            #pageSubtitle { color: #748398; font-size: 12px; }
            #card {
                background: #FFFFFF; border: 1px solid #E9EEF5; border-radius: 16px;
            }
            #cardTitle { color: #8491A4; font-size: 11px; font-weight: 600; }
            #cardValue { color: #17263B; font-size: 16px; font-weight: 650; }
            #cardDetail { color: #7C8A9D; font-size: 10px; }
            #actionTitle { color: #15243A; font-size: 21px; font-weight: 700; }
            #actionDescription { color: #738196; font-size: 11px; line-height: 1.5; }
            QPushButton { border: none; font-weight: 600; }
            #primaryButton {
                color: white; background: #2D7FF9; border-radius: 11px;
                padding: 0 28px; font-size: 14px;
            }
            #primaryButton:hover { background: #1F6FE8; }
            #primaryButton:pressed { background: #185FCA; }
            #primaryButton:disabled { background: #B9C6D7; color: #EEF2F7; }
            #secondaryButton {
                color: #2D6FCC; background: #E8F1FF; border-radius: 9px;
                padding: 10px 18px; font-size: 11px;
            }
            #secondaryButton:hover { background: #DCEAFF; }
            #secondaryButton:disabled { color: #96A4B6; background: #E9EDF3; }
            #logCard { background: #101D2F; border-radius: 14px; }
            #logTitle { color: #AFC0D6; font-size: 11px; font-weight: 600; }
            #logView {
                color: #AFC0D6; background: transparent; border: none;
                font-family: "Cascadia Mono", "Consolas"; font-size: 10px;
                selection-background-color: #2D7FF9;
            }
            QScrollBar:vertical { background: transparent; width: 8px; margin: 0; }
            QScrollBar::handle:vertical { background: #31445E; border-radius: 4px; min-height: 20px; }
            QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
            """
        )

    def append_log(self, message: str) -> None:
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.log_view.appendPlainText(f"[{timestamp}] {message}")

    def scan_probes(self, silent: bool = False) -> None:
        if self._busy or (self._scan_worker and self._scan_worker.isRunning()):
            return
        if not silent:
            self.refresh_button.setEnabled(False)
            self.dap_value.setText("正在检测…")
            self.dap_detail.setText("正在扫描 USB 设备")
            self.status_dot.set_color("#F3A530")
        self._scan_worker = ScanWorker()
        self._scan_worker.completed.connect(
            lambda probes, is_silent=silent: self._on_scan_completed(probes, is_silent)
        )
        self._scan_worker.failed.connect(
            lambda message, is_silent=silent: self._on_scan_failed(message, is_silent)
        )
        self._scan_worker.finished.connect(lambda: self.refresh_button.setEnabled(not self._busy))
        self._scan_worker.start()

    def _on_scan_completed(self, probes: list[ProbeInfo], silent: bool = False) -> None:
        signature = tuple(sorted((probe.unique_id, probe.description) for probe in probes))
        state_changed = signature != self._last_probe_signature
        self._last_probe_signature = signature

        if not probes:
            self._probe = None
            self.status_dot.set_color("#AAB5C3")
            self.dap_value.setText("未连接")
            self.dap_detail.setText("请插入 CMSIS-DAP 下载器")
            self.flash_button.setEnabled(False)
            self.progress_ring.set_progress(0, "等待连接")
            if state_changed:
                self.append_log("CMSIS-DAP 已断开" if silent else "未检测到 CMSIS-DAP")
                self.action_title.setText("等待设备")
                self.action_description.setText("请插入 CMSIS-DAP，并连接目标板的 SWDIO、SWCLK 与 GND。")
            return

        self._probe = probes[0]
        self.status_dot.set_color("#16A36A")
        self.dap_value.setText(self._probe.description)
        suffix = self._probe.unique_id[-12:] if len(self._probe.unique_id) > 12 else self._probe.unique_id
        detail = f"序列号 …{suffix}"
        if len(probes) > 1:
            detail += f"  ·  已连接 {len(probes)} 个，使用第一个"
        self.dap_detail.setText(detail)
        self.flash_button.setEnabled(True)
        self.progress_ring.set_progress(0, "设备已就绪")
        if state_changed:
            self.append_log(f"检测到 {self._probe.description}，可以开始烧录")
            self.action_title.setText("准备烧录")
            self.action_description.setText("设备连接正常，点击一键烧录即可写入并校验内置固件。")

    def _on_scan_failed(self, message: str, silent: bool = False) -> None:
        # 后台轮询偶发失败时保留上一次状态，手动检测才显示具体错误。
        if silent:
            return
        self._probe = None
        self._last_probe_signature = None
        self.status_dot.set_color("#E0515B")
        self.dap_value.setText("检测失败")
        self.dap_detail.setText("请检查 USB 连接或驱动")
        self.flash_button.setEnabled(False)
        if not silent:
            self.append_log(f"DAP 检测失败：{message}")

    def start_flash(self) -> None:
        if self._busy or self._probe is None:
            return
        self._busy = True
        self.flash_button.setEnabled(False)
        self.refresh_button.setEnabled(False)
        self.action_title.setText("正在烧录")
        self.action_description.setText("请保持目标板和 CMSIS-DAP 连接，烧录完成前不要断开电源。")
        self.log_view.clear()
        self.append_log("开始一键烧录")

        self._flash_worker = FlashWorker(self._probe.unique_id)
        self._flash_worker.signals.progress.connect(self._on_progress)
        self._flash_worker.signals.log.connect(self.append_log)
        self._flash_worker.signals.completed.connect(self._on_flash_completed)
        self._flash_worker.signals.failed.connect(self._on_flash_failed)
        self._flash_worker.start()

    def _on_progress(self, value: int, caption: str) -> None:
        self.progress_ring.set_progress(value, caption)
        if value in (4, 10, 18, 76, 97, 100):
            self.append_log(caption)

    def _on_flash_completed(self, device: DeviceInfo) -> None:
        self._busy = False
        self.status_dot.set_color("#16A36A")
        self.action_title.setText("烧录成功")
        self.action_description.setText(f"设备已通过读回校验并启动。芯片 UID：{device.uid}")
        self.flash_button.setText("再次烧录")
        self.flash_button.setEnabled(True)
        self.refresh_button.setEnabled(True)
        self.append_log("全部数据校验通过，设备已执行软件复位")

    def _on_flash_failed(self, message: str, details: str) -> None:
        self._busy = False
        self.status_dot.set_color("#E0515B")
        self.progress_ring.set_progress(0, "烧录失败")
        self.action_title.setText("烧录未完成")
        self.action_description.setText(message)
        self.flash_button.setText("重新烧录")
        self.flash_button.setEnabled(self._probe is not None)
        self.refresh_button.setEnabled(True)
        self.append_log(message)

        log_dir = Path(os.getenv("LOCALAPPDATA", str(Path.home()))) / "GloveDAPFlasher" / "logs"
        try:
            log_dir.mkdir(parents=True, exist_ok=True)
            log_path = log_dir / f"error_{datetime.now():%Y%m%d_%H%M%S}.log"
            log_path.write_text(details, encoding="utf-8")
            self.append_log(f"详细错误已保存：{log_path}")
        except OSError:
            pass

        QMessageBox.warning(self, "烧录失败", message)

    def closeEvent(self, event) -> None:
        if self._busy:
            QMessageBox.information(self, "正在烧录", "烧录完成前不能关闭软件。")
            event.ignore()
            return
        self._probe_monitor.stop()
        event.accept()


def main() -> int:
    if "--self-test" in sys.argv:
        return run_self_test()

    if sys.platform == "win32":
        try:
            import ctypes

            ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("Glove.DAP.Flasher.1")
        except Exception:
            pass

    app = QApplication(sys.argv)
    app.setApplicationName(APP_NAME)
    app.setApplicationVersion(APP_VERSION)
    app.setStyle("Fusion")
    window = MainWindow()
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
