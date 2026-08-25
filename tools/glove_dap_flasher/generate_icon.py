from pathlib import Path

from PySide6.QtCore import QRectF, Qt
from PySide6.QtGui import QGuiApplication, QImage, QPainter
from PySide6.QtSvg import QSvgRenderer


def main() -> int:
    root = Path(__file__).resolve().parent
    source = root / "resources" / "app_icon.svg"
    target = root / "resources" / "app_icon.ico"

    app = QGuiApplication.instance() or QGuiApplication([])
    del app
    renderer = QSvgRenderer(str(source))
    if not renderer.isValid():
        raise RuntimeError(f"无法读取图标：{source}")

    image = QImage(256, 256, QImage.Format_ARGB32)
    image.fill(Qt.transparent)
    painter = QPainter(image)
    painter.setRenderHint(QPainter.Antialiasing)
    renderer.render(painter, QRectF(0, 0, 256, 256))
    painter.end()

    if not image.save(str(target), "ICO"):
        raise RuntimeError(f"无法生成 Windows 图标：{target}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
