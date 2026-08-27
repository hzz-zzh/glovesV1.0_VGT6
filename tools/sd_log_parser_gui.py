#!/usr/bin/env python3
"""Glove SD 日志图形化解析工具。"""

from __future__ import annotations

import datetime as dt
import os
import queue
import threading
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, messagebox, ttk

from parse_sd_log import RECORD_SIZE, parse_file


class SdLogParserApp(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title("手套 SD 日志解析器")
        self.geometry("780x540")
        self.minsize(680, 460)

        self.events: queue.Queue[tuple[str, object]] = queue.Queue()
        self.worker: threading.Thread | None = None
        self.output_dir: Path | None = None

        self.input_var = tk.StringVar(value="尚未选择日志文件")
        self.output_var = tk.StringVar(value="解析结果将自动保存到 Documents\\GloveSdLogParsed")
        self.status_var = tk.StringVar(value="请选择 LOGxxxx.BIN 文件")
        self.progress_var = tk.DoubleVar(value=0.0)

        self._build_ui()
        self.after(100, self._process_events)

    def _build_ui(self) -> None:
        style = ttk.Style(self)
        try:
            style.theme_use("clam")
        except tk.TclError:
            pass
        style.configure("Title.TLabel", font=("Microsoft YaHei UI", 18, "bold"))
        style.configure("Status.TLabel", font=("Microsoft YaHei UI", 10))
        style.configure("Primary.TButton", font=("Microsoft YaHei UI", 12), padding=(16, 10))

        outer = ttk.Frame(self, padding=20)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=1)
        outer.rowconfigure(5, weight=1)

        ttk.Label(outer, text="手套 SD 日志解析器", style="Title.TLabel").grid(
            row=0, column=0, sticky="w"
        )
        ttk.Label(
            outer,
            text="选择固件 V2.2.0+ 生成的 LOGxxxx.BIN，程序会自动校验并导出 CSV。",
            style="Status.TLabel",
        ).grid(row=1, column=0, sticky="w", pady=(4, 18))

        button_bar = ttk.Frame(outer)
        button_bar.grid(row=2, column=0, sticky="ew")
        self.select_button = ttk.Button(
            button_bar,
            text="选择 SD 日志并解析",
            command=self.select_and_parse,
            style="Primary.TButton",
        )
        self.select_button.pack(side=tk.LEFT)
        self.open_button = ttk.Button(
            button_bar,
            text="打开结果目录",
            command=self.open_output_directory,
            state=tk.DISABLED,
        )
        self.open_button.pack(side=tk.LEFT, padx=(10, 0))

        info = ttk.LabelFrame(outer, text="任务信息", padding=12)
        info.grid(row=3, column=0, sticky="ew", pady=(18, 12))
        info.columnconfigure(1, weight=1)
        ttk.Label(info, text="输入文件：").grid(row=0, column=0, sticky="nw")
        ttk.Label(info, textvariable=self.input_var, wraplength=620).grid(
            row=0, column=1, sticky="w"
        )
        ttk.Label(info, text="输出目录：").grid(row=1, column=0, sticky="nw", pady=(8, 0))
        ttk.Label(info, textvariable=self.output_var, wraplength=620).grid(
            row=1, column=1, sticky="w", pady=(8, 0)
        )

        progress_frame = ttk.Frame(outer)
        progress_frame.grid(row=4, column=0, sticky="ew", pady=(0, 12))
        progress_frame.columnconfigure(0, weight=1)
        self.progress = ttk.Progressbar(
            progress_frame,
            variable=self.progress_var,
            maximum=100.0,
            mode="determinate",
        )
        self.progress.grid(row=0, column=0, sticky="ew")
        ttk.Label(progress_frame, textvariable=self.status_var).grid(
            row=1, column=0, sticky="w", pady=(6, 0)
        )

        result_frame = ttk.LabelFrame(outer, text="解析结果", padding=8)
        result_frame.grid(row=5, column=0, sticky="nsew")
        result_frame.columnconfigure(0, weight=1)
        result_frame.rowconfigure(0, weight=1)
        self.result_text = tk.Text(
            result_frame,
            wrap="word",
            font=("Consolas", 10),
            state=tk.DISABLED,
        )
        scrollbar = ttk.Scrollbar(result_frame, command=self.result_text.yview)
        self.result_text.configure(yscrollcommand=scrollbar.set)
        self.result_text.grid(row=0, column=0, sticky="nsew")
        scrollbar.grid(row=0, column=1, sticky="ns")

    def select_and_parse(self) -> None:
        if self.worker is not None and self.worker.is_alive():
            return

        filename = filedialog.askopenfilename(
            title="选择手套 SD 日志",
            filetypes=(("手套日志", "LOG*.BIN"), ("BIN 文件", "*.BIN"), ("所有文件", "*.*")),
        )
        if not filename:
            return

        input_path = Path(filename)
        timestamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        # CSV通常比BIN更大，默认输出到电脑文档目录，避免继续占用SD卡空间。
        self.output_dir = (
            Path.home()
            / "Documents"
            / "GloveSdLogParsed"
            / f"{input_path.stem}_{timestamp}"
        )
        self.input_var.set(str(input_path))
        self.output_var.set(str(self.output_dir))
        self.status_var.set("正在检查日志文件……")
        self.progress_var.set(0.0)
        self.open_button.configure(state=tk.DISABLED)
        self.select_button.configure(state=tk.DISABLED)
        self._set_result("正在解析，请勿拔出 SD 卡或关闭程序。\n")

        self.worker = threading.Thread(
            target=self._parse_worker,
            args=(input_path, self.output_dir),
            daemon=True,
        )
        self.worker.start()

    def _parse_worker(self, input_path: Path, output_dir: Path) -> None:
        def report_progress(done: int, total: int) -> None:
            self.events.put(("progress", (done, total)))

        try:
            summary = parse_file(
                input_path,
                output_dir,
                strict=False,
                progress_callback=report_progress,
            )
            self.events.put(("done", summary))
        except Exception as exc:
            self.events.put(("error", exc))

    def _process_events(self) -> None:
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "progress":
                    done, total = payload  # type: ignore[misc]
                    percent = 100.0 if total == 0 else done * 100.0 / total
                    self.progress_var.set(percent)
                    self.status_var.set(
                        f"正在解析：{done:,}/{total:,} 帧（{percent:.1f}%）"
                    )
                elif kind == "done":
                    self._show_summary(payload)  # type: ignore[arg-type]
                elif kind == "error":
                    self.status_var.set("解析失败")
                    self.select_button.configure(state=tk.NORMAL)
                    self._set_result(f"解析失败：\n\n{payload}\n")
                    messagebox.showerror("解析失败", str(payload))
        except queue.Empty:
            pass
        self.after(100, self._process_events)

    def _show_summary(self, summary: dict[str, int | str]) -> None:
        self.progress_var.set(100.0)
        self.select_button.configure(state=tk.NORMAL)
        self.open_button.configure(state=tk.NORMAL)

        bad_records = int(summary["bad_records"])
        crc_bad_records = int(summary["crc_bad_records"])
        trailing_bytes = int(summary["trailing_bytes"])
        frame_gap_events = int(summary["frame_gap_events"])
        missing_frames = int(summary["missing_frames"])
        duplicate_frames = int(summary["duplicate_frames"])
        reordered_frames = int(summary["reordered_frames"])
        timestamp_regressions = int(summary["timestamp_regressions"])
        zero_padding_records = int(summary["zero_padding_records"])
        clean = (
            bad_records == 0
            and crc_bad_records == 0
            and trailing_bytes == 0
            and frame_gap_events == 0
            and duplicate_frames == 0
            and reordered_frames == 0
            and timestamp_regressions == 0
            and zero_padding_records == 0
        )
        result_name = "解析完成，日志完整" if clean else "解析完成，但检测到异常"
        self.status_var.set(result_name)

        size_mib = int(summary["size"]) / (1024 * 1024)
        text = "\n".join(
            [
                result_name,
                "",
                f"输入大小       : {size_mib:.2f} MiB",
                f"完整记录数     : {int(summary['records']):,}",
                f"首帧/末帧ID    : {int(summary['first_frame_id'])} / {int(summary['last_frame_id'])}",
                f"结构异常记录   : {bad_records:,}",
                f"CRC异常记录    : {crc_bad_records:,}",
                f"帧号跳变次数   : {frame_gap_events:,}",
                f"估算缺失帧数   : {missing_frames:,}",
                f"重复帧数       : {duplicate_frames:,}",
                f"倒序帧数       : {reordered_frames:,}",
                f"时间戳倒退次数 : {timestamp_regressions:,}",
                f"最大时间间隔   : {int(summary['max_timestamp_gap_us']):,} us",
                f"文件尾残留字节 : {trailing_bytes:,}",
                f"恢复空白记录   : {zero_padding_records:,}",
                "",
                "已生成文件：",
                "- frames.csv   帧号、时间、有效标志及健康诊断",
                "- imu.csv      16路IMU数据",
                "- joint.csv    27个关节角",
                "- tactile.csv  68点触觉值及baseline",
                "",
                f"输出目录：\n{summary['output']}",
            ]
        )
        self._set_result(text)
        if clean:
            messagebox.showinfo("解析完成", "日志解析完成，帧号连续且未发现CRC或结构异常。")
        else:
            messagebox.showwarning(
                "解析完成",
                "日志已解析，但检测到丢帧、重复帧、损坏记录或不完整文件尾。",
            )

    def open_output_directory(self) -> None:
        if self.output_dir is None or not self.output_dir.exists():
            messagebox.showerror("无法打开", "解析结果目录不存在。")
            return
        try:
            os.startfile(self.output_dir)  # type: ignore[attr-defined]
        except OSError as exc:
            messagebox.showerror("无法打开", str(exc))

    def _set_result(self, text: str) -> None:
        self.result_text.configure(state=tk.NORMAL)
        self.result_text.delete("1.0", tk.END)
        self.result_text.insert(tk.END, text)
        self.result_text.configure(state=tk.DISABLED)


def main() -> None:
    app = SdLogParserApp()
    app.mainloop()


if __name__ == "__main__":
    main()
