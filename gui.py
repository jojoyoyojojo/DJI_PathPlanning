#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PySide6 GUI for AeroFacade Studio

Provides a desktop interface for the drone mission planning tool with:
- Drag & drop image/folder input
- Parameter configuration for all algorithm settings
- Real-time GPS metadata display
- Mission generation and KMZ export
"""

import sys
import os
import struct
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QLineEdit, QDoubleSpinBox, QSpinBox,
    QCheckBox, QComboBox, QTextEdit, QFileDialog, QMessageBox,
    QGridLayout, QFrame, QSizePolicy, QScrollArea
)
from PySide6.QtCore import Qt, Signal, QMimeData
from PySide6.QtGui import QPixmap, QDragEnterEvent, QDropEvent, QImage
from loguru import logger

# Import core algorithm functions
import mavic3T_pp_kmz as core


# =============================================================================
# DNG preview extraction (minimal TIFF parser, pure-Python)
# =============================================================================
# Qt's TIFF plugin on many builds cannot decode JPEG-compressed strips inside
# DNG (errors like "JPEG compression support is not configured"), so we parse
# the TIFF structure ourselves, walk IFD0 + all SubIFDs, and pull out the
# largest embedded JPEG preview. The raw JPEG bytes are then decoded via
# QImage.fromData, which works everywhere Qt's JPEG plugin does.

_TIFF_TAG_IMAGE_WIDTH = 256
_TIFF_TAG_IMAGE_LENGTH = 257
_TIFF_TAG_COMPRESSION = 259
_TIFF_TAG_STRIP_OFFSETS = 273
_TIFF_TAG_STRIP_BYTE_COUNTS = 279
_TIFF_TAG_SUB_IFDS = 330
_TIFF_TAG_JPEG_IF_OFFSET = 513
_TIFF_TAG_JPEG_IF_LENGTH = 514

_TIFF_TYPE_SIZES = {
    1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 6: 1, 7: 1, 8: 2, 9: 4, 10: 8, 11: 4, 12: 8,
}


def _extract_dng_jpeg_preview(path: str) -> Optional[bytes]:
    """Return the largest embedded JPEG preview inside a DNG/TIFF file.

    Returns raw JPEG bytes (starting with ``\\xff\\xd8``) on success, or None
    if the file is not a TIFF-based container or contains no JPEG preview.
    """
    try:
        with open(path, "rb") as f:
            data = f.read()
    except Exception as e:
        logger.debug(f"DNG read failed ({path}): {e}")
        return None

    if len(data) < 8 or data[:2] not in (b"II", b"MM"):
        return None
    endian = "<" if data[:2] == b"II" else ">"
    if struct.unpack(endian + "H", data[2:4])[0] != 42:
        return None
    first_ifd = struct.unpack(endian + "I", data[4:8])[0]

    best: Optional[Tuple[int, bytes]] = None
    visited: set = set()

    def ifd_scalars(entries: Dict[int, Tuple[int, int, bytes]], tag: int) -> List[int]:
        if tag not in entries:
            return []
        ttype, count, value_field = entries[tag]
        size = _TIFF_TYPE_SIZES.get(ttype, 0) * count
        if size == 0:
            return []
        if size <= 4:
            buf = value_field[:size]
        else:
            off = struct.unpack(endian + "I", value_field)[0]
            if off + size > len(data):
                return []
            buf = data[off:off + size]
        if ttype == 3:
            return list(struct.unpack(endian + f"{count}H", buf))
        if ttype == 4:
            return list(struct.unpack(endian + f"{count}I", buf))
        if ttype == 1:
            return list(buf)
        return []

    def walk(offset: int) -> None:
        nonlocal best
        if offset in visited or offset <= 0 or offset + 2 > len(data):
            return
        visited.add(offset)
        try:
            num_entries = struct.unpack(endian + "H", data[offset:offset + 2])[0]
        except struct.error:
            return
        base = offset + 2
        if base + num_entries * 12 > len(data):
            return

        entries: Dict[int, Tuple[int, int, bytes]] = {}
        for i in range(num_entries):
            eo = base + i * 12
            tag, ttype, count = struct.unpack(endian + "HHI", data[eo:eo + 8])
            entries[tag] = (ttype, count, data[eo + 8:eo + 12])

        width = ifd_scalars(entries, _TIFF_TAG_IMAGE_WIDTH)
        height = ifd_scalars(entries, _TIFF_TAG_IMAGE_LENGTH)
        compression = ifd_scalars(entries, _TIFF_TAG_COMPRESSION)
        strip_offsets = ifd_scalars(entries, _TIFF_TAG_STRIP_OFFSETS)
        strip_counts = ifd_scalars(entries, _TIFF_TAG_STRIP_BYTE_COUNTS)
        jpeg_if_off = ifd_scalars(entries, _TIFF_TAG_JPEG_IF_OFFSET)
        jpeg_if_len = ifd_scalars(entries, _TIFF_TAG_JPEG_IF_LENGTH)

        comp = compression[0] if compression else 0
        area = (width[0] if width else 0) * (height[0] if height else 0)

        payload: Optional[bytes] = None
        if jpeg_if_off and jpeg_if_len:
            off, ln = jpeg_if_off[0], jpeg_if_len[0]
            if ln > 0 and off + ln <= len(data):
                payload = data[off:off + ln]
        elif comp == 7 and strip_offsets and strip_counts:
            if len(strip_offsets) == 1:
                off, ln = strip_offsets[0], strip_counts[0]
                if ln > 0 and off + ln <= len(data):
                    payload = data[off:off + ln]
            else:
                chunks = []
                for off, ln in zip(strip_offsets, strip_counts):
                    if ln > 0 and off + ln <= len(data):
                        chunks.append(data[off:off + ln])
                if chunks:
                    payload = b"".join(chunks)

        if payload and payload[:2] == b"\xff\xd8":
            if best is None or area > best[0]:
                best = (area, payload)

        for sub_off in ifd_scalars(entries, _TIFF_TAG_SUB_IFDS):
            walk(sub_off)

        next_pos = base + num_entries * 12
        if next_pos + 4 <= len(data):
            next_off = struct.unpack(endian + "I", data[next_pos:next_pos + 4])[0]
            if next_off:
                walk(next_off)

    walk(first_ifd)
    return best[1] if best else None


def _load_image_pixmap(path: str) -> QPixmap:
    """Load a QPixmap for JPG/JPEG directly, or DNG via embedded JPEG preview."""
    pix = QPixmap(path)
    if not pix.isNull():
        return pix
    if path.lower().endswith(".dng"):
        jpeg = _extract_dng_jpeg_preview(path)
        if jpeg:
            img = QImage.fromData(jpeg, "JPEG")
            if not img.isNull():
                return QPixmap.fromImage(img)
    return QPixmap()


# =============================================================================
# Facade corner slots (2×2, facing the wall) → core order [BL, TL, TR, BR]
# =============================================================================

# Slot indices match mavic3T_pp_kmz.FacadeTransformer: 0=BL, 1=TL, 2=TR, 3=BR


class FacadeCornerSlot(QFrame):
    """Single corner cell: drop one JPG/JPEG here."""

    image_dropped = Signal(int, str)  # slot_index, path

    def __init__(self, slot_index: int, title: str, parent=None):
        super().__init__(parent)
        self.slot_index = slot_index
        self.setAcceptDrops(True)
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Raised)
        self.setMinimumSize(108, 118)
        self.setStyleSheet("""
            FacadeCornerSlot {
                background-color: #fafafa;
                border: 2px dashed #bbb;
                border-radius: 6px;
            }
            FacadeCornerSlot[drag_over="true"] {
                background-color: #e3f2fd;
                border-color: #2196f3;
            }
        """)

        lay = QVBoxLayout(self)
        lay.setSpacing(4)
        self._title = QLabel(title)
        self._title.setAlignment(Qt.AlignCenter)
        self._title.setStyleSheet("font-size: 11px; font-weight: bold; color: #333;")
        lay.addWidget(self._title)

        self.thumb = QLabel()
        self.thumb.setFixedSize(72, 72)
        self.thumb.setFrameStyle(QFrame.Box | QFrame.Plain)
        self.thumb.setAlignment(Qt.AlignCenter)
        self.thumb.setStyleSheet("background-color: #eee; border: 1px solid #ccc; color: #999;")
        self.thumb.setText("—")
        lay.addWidget(self.thumb, alignment=Qt.AlignCenter)

        self.name_lbl = QLabel("Empty")
        self.name_lbl.setAlignment(Qt.AlignCenter)
        self.name_lbl.setWordWrap(True)
        self.name_lbl.setStyleSheet("font-size: 9px; color: #666; max-width: 96px;")
        lay.addWidget(self.name_lbl)

        self._drag_over = False

    def _set_drag_over(self, on: bool):
        self._drag_over = on
        self.setProperty("drag_over", on)
        self.style().unpolish(self)
        self.style().polish(self)

    def set_path(self, path: Optional[str]) -> None:
        if path and os.path.isfile(path):
            pix = _load_image_pixmap(path)
            if not pix.isNull():
                self.thumb.setPixmap(
                    pix.scaled(72, 72, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                )
            else:
                self.thumb.clear()
                self.thumb.setText("DNG" if path.lower().endswith(".dng") else "ERR")
            self.name_lbl.setText(Path(path).name[:16])
        else:
            self.thumb.clear()
            self.thumb.setText("—")
            self.name_lbl.setText("Empty")

    def dragEnterEvent(self, event: QDragEnterEvent):
        if event.mimeData().hasUrls():
            event.acceptProposedAction()
            self._set_drag_over(True)

    def dragLeaveEvent(self, event):
        self._set_drag_over(False)

    def dropEvent(self, event: QDropEvent):
        self._set_drag_over(False)
        for url in event.mimeData().urls():
            path = url.toLocalFile()
            if path.lower().endswith((".jpg", ".jpeg", ".dng")) and os.path.isfile(path):
                self.image_dropped.emit(self.slot_index, path)
                break


class FacadeGridDropZone(QFrame):
    """
    2×2 grid as seen when facing the wall (top = higher on wall).
    Maps to algorithm indices: BL=0, TL=1, TR=2, BR=3.
    """

    grid_changed = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)
        self.slot_paths: List[Optional[str]] = [None, None, None, None]

        outer = QVBoxLayout(self)
        outer.setSpacing(8)

        hint = QLabel("Facing the wall — top row is upper on the facade ↑")
        hint.setAlignment(Qt.AlignCenter)
        hint.setStyleSheet("color: #555; font-size: 11px;")
        outer.addWidget(hint)

        grid = QGridLayout()
        grid.setSpacing(10)
        # (slot_index, grid_row, grid_col, title)
        layout_spec = [
            (1, 0, 0, "Top-Left TL\n#2"),
            (2, 0, 1, "Top-Right TR\n#3"),
            (0, 1, 0, "Bottom-Left BL\n#1"),
            (3, 1, 1, "Bottom-Right BR\n#4"),
        ]
        by_slot: Dict[int, FacadeCornerSlot] = {}
        for slot_idx, row, col, title in layout_spec:
            cell = FacadeCornerSlot(slot_idx, title, self)
            cell.image_dropped.connect(self._on_slot_image)
            by_slot[slot_idx] = cell
            grid.addWidget(cell, row, col)
        self._cells = [by_slot[i] for i in range(4)]

        outer.addLayout(grid)

        self.instruction = QLabel(
            "Drop on each cell, or use Select / Folder (fills BL→TL→TR→BR in order)."
        )
        self.instruction.setAlignment(Qt.AlignCenter)
        self.instruction.setWordWrap(True)
        self.instruction.setStyleSheet("color: #888; font-size: 11px;")
        outer.addWidget(self.instruction)

    def _on_slot_image(self, slot: int, path: str) -> None:
        self.slot_paths[slot] = path
        self._cells[slot].set_path(path)
        self._update_instruction()
        self.grid_changed.emit()

    def _update_instruction(self) -> None:
        n = sum(1 for p in self.slot_paths if p)
        if n == 4:
            self.instruction.setText("All 4 corners set — order matches FacadeTransformer (BL, TL, TR, BR).")
            self.instruction.setStyleSheet("color: #4caf50; font-size: 11px;")
        elif n > 0:
            self.instruction.setText(f"{n}/4 corners set — fill BL, TL, TR, BR as when facing the wall.")
            self.instruction.setStyleSheet("color: #ff9800; font-size: 11px;")
        else:
            self.instruction.setText(
                "Drop on each cell, or use Select / Folder (fills BL→TL→TR→BR in order)."
            )
            self.instruction.setStyleSheet("color: #888; font-size: 11px;")

    def apply_sequential_paths(self, paths: List[str]) -> None:
        """Assign paths to slots 0..3 in order BL, TL, TR, BR."""
        for i in range(4):
            self.slot_paths[i] = paths[i] if i < len(paths) else None
            self._cells[i].set_path(self.slot_paths[i])
        self._update_instruction()
        self.grid_changed.emit()

    def clear(self) -> None:
        for i in range(4):
            self.slot_paths[i] = None
            self._cells[i].set_path(None)
        self._update_instruction()
        self.grid_changed.emit()

    @staticmethod
    def scan_folder_images(folder: str) -> List[str]:
        images = []
        for f in sorted(os.listdir(folder)):
            if f.lower().endswith((".jpg", ".jpeg", ".dng")):
                images.append(os.path.join(folder, f))
        return images


# =============================================================================
# Main Window
# =============================================================================

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AeroFacade Studio")
        self.setMinimumSize(600, 800)

        # State
        self.image_paths: List[str] = []
        self.gps_data: List[Optional[Tuple[float, float, float]]] = []
        self.photo_metadata: List[Optional[Dict[str, Any]]] = []
        self.all_photos_rtk_fix: bool = False
        self.generated_waypoints: List[Tuple[float, float, float]] = []
        self.transformer: Optional[core.FacadeTransformer] = None
        self.flight_direction: str = ""
        self.corner_geometry_ok: bool = False
        self.facade_width: Optional[float] = None
        self.facade_height: Optional[float] = None

        # Central widget with scroll area
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        central = QWidget()
        scroll.setWidget(central)
        self.setCentralWidget(scroll)

        layout = QVBoxLayout(central)
        layout.setSpacing(10)

        # Create zones
        layout.addWidget(self._create_zone1_input())
        layout.addWidget(self._create_zone2_camera())
        layout.addWidget(self._create_zone3_flight())
        layout.addWidget(self._create_zone4_info())
        layout.addWidget(self._create_zone5_generate())
        layout.addWidget(self._create_zone6_output())

        layout.addStretch()

        # Initial state (default M3T locks FOV from profile)
        self._on_drone_type_changed()
        self._update_capture_controls()
        self._update_ui_state()

    # -------------------------------------------------------------------------
    # Zone 1: Input Settings
    # -------------------------------------------------------------------------
    def _create_zone1_input(self) -> QGroupBox:
        group = QGroupBox("1. Input Settings")
        layout = QVBoxLayout(group)

        # 2×2 corner grid (facing wall → BL, TL, TR, BR)
        self.drop_zone = FacadeGridDropZone()
        self.drop_zone.grid_changed.connect(self._on_facade_grid_changed)
        layout.addWidget(self.drop_zone)

        # Buttons row
        btn_layout = QHBoxLayout()

        self.btn_select_images = QPushButton("Select Images...")
        self.btn_select_images.clicked.connect(self._select_images)
        btn_layout.addWidget(self.btn_select_images)

        self.btn_select_folder = QPushButton("Select Folder...")
        self.btn_select_folder.clicked.connect(self._select_folder)
        btn_layout.addWidget(self.btn_select_folder)

        self.btn_clear = QPushButton("Clear All")
        self.btn_clear.clicked.connect(self._clear_images)
        btn_layout.addWidget(self.btn_clear)

        layout.addLayout(btn_layout)

        # Parameters grid
        params = QGridLayout()

        params.addWidget(QLabel("Mission Name:"), 0, 0)
        self.edit_mission_name = QLineEdit("Facade Mission")
        params.addWidget(self.edit_mission_name, 0, 1, 1, 3)

        params.addWidget(QLabel("Distance (m):"), 1, 0)
        self.spin_photo_dist = QDoubleSpinBox()
        self.spin_photo_dist.setRange(core.PHOTO_DISTANCE_MIN, core.PHOTO_DISTANCE_MAX)
        self.spin_photo_dist.setValue(5.0)
        self.spin_photo_dist.setDecimals(1)
        self.spin_photo_dist.setToolTip(f"{core.PHOTO_DISTANCE_MIN:.0f}–{core.PHOTO_DISTANCE_MAX:.0f} m")
        self.spin_photo_dist.valueChanged.connect(self._on_param_changed)
        params.addWidget(self.spin_photo_dist, 1, 1, 1, 3)

        layout.addLayout(params)

        return group

    # -------------------------------------------------------------------------
    # Zone 2: Camera & Planning Settings
    # -------------------------------------------------------------------------
    def _create_zone2_camera(self) -> QGroupBox:
        group = QGroupBox("2. Camera & Planning Settings")
        layout = QGridLayout(group)

        layout.addWidget(QLabel("Drone Type:"), 0, 0)
        self.combo_drone = QComboBox()
        self.combo_drone.addItems(core.DRONE_WPML_PROFILES.keys())
        self.combo_drone.setCurrentText(core.DRONE_TYPE)
        self.combo_drone.currentTextChanged.connect(self._on_drone_type_changed)
        self.combo_drone.currentTextChanged.connect(self._on_param_changed)
        layout.addWidget(self.combo_drone, 0, 1, 1, 3)

        layout.addWidget(QLabel("HFOV (°):"), 1, 0)
        self.spin_hfov = QDoubleSpinBox()
        self.spin_hfov.setRange(1.0, 180.0)
        self.spin_hfov.setValue(core.CAMERA_HFOV)
        self.spin_hfov.setDecimals(1)
        self.spin_hfov.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_hfov, 1, 1)

        layout.addWidget(QLabel("VFOV (°):"), 1, 2)
        self.spin_vfov = QDoubleSpinBox()
        self.spin_vfov.setRange(1.0, 180.0)
        self.spin_vfov.setValue(core.CAMERA_VFOV)
        self.spin_vfov.setDecimals(1)
        self.spin_vfov.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_vfov, 1, 3)

        layout.addWidget(QLabel("Overlap (%):"), 2, 0)
        self.spin_overlap = QSpinBox()
        self.spin_overlap.setRange(0, 100)
        self.spin_overlap.setValue(65)
        self.spin_overlap.setToolTip("Along-track photo overlap as percentage (0–100). Recommend ≥ 65.")
        self.spin_overlap.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_overlap, 2, 1)

        lbl_overlap_hint = QLabel("Recommend ≥ 65%")
        lbl_overlap_hint.setStyleSheet("color: #666; font-size: 11px;")
        layout.addWidget(lbl_overlap_hint, 2, 2, 1, 2)

        self.chk_smart_planning = QCheckBox("Enable Smart Planning")
        self.chk_smart_planning.setChecked(True)
        self.chk_smart_planning.stateChanged.connect(self._on_param_changed)
        layout.addWidget(self.chk_smart_planning, 3, 0, 1, 4)

        self.chk_force_vertical = QCheckBox("Force Vertical Plane")
        self.chk_force_vertical.setChecked(True)
        self.chk_force_vertical.setToolTip(
            "Ensure flight path is on a true vertical plane regardless of camera tilt"
        )
        self.chk_force_vertical.stateChanged.connect(self._on_param_changed)
        layout.addWidget(self.chk_force_vertical, 4, 0, 1, 4)

        return group

    # -------------------------------------------------------------------------
    # Zone 3: Flight Settings
    # -------------------------------------------------------------------------
    def _create_zone3_flight(self) -> QGroupBox:
        group = QGroupBox("3. Flight Settings")
        layout = QGridLayout(group)

        layout.addWidget(QLabel("Speed (m/s):"), 0, 0)
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(core.AUTO_FLIGHT_SPEED_MIN, core.AUTO_FLIGHT_SPEED_MAX)
        self.spin_speed.setValue(4.0)
        self.spin_speed.setDecimals(1)
        self.spin_speed.setToolTip(f"{core.AUTO_FLIGHT_SPEED_MIN}–{core.AUTO_FLIGHT_SPEED_MAX} m/s")
        self.spin_speed.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_speed, 0, 1)

        layout.addWidget(QLabel("Gimbal Pitch (°):"), 0, 2)
        self.spin_gimbal = QDoubleSpinBox()
        self.spin_gimbal.setRange(-90.0, 30.0)
        self.spin_gimbal.setValue(0.0)
        self.spin_gimbal.setDecimals(1)
        self.spin_gimbal.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_gimbal, 0, 3)

        layout.addWidget(QLabel("Capture Mode:"), 1, 0)
        self.combo_capture_mode = QComboBox()
        self.combo_capture_mode.addItem("By time", "time")
        self.combo_capture_mode.addItem("Fixed-point photos", "waypoint")
        self.combo_capture_mode.addItem("Video", "video")
        self.combo_capture_mode.addItem("No capture", "none")
        self.combo_capture_mode.setToolTip(
            "Time mode takes photos by interval; waypoint mode stops for photos; "
            "video mode records from first waypoint to last."
        )
        self.combo_capture_mode.currentIndexChanged.connect(self._on_capture_mode_changed)
        layout.addWidget(self.combo_capture_mode, 1, 1, 1, 3)

        layout.addWidget(QLabel("Time Interval (s):"), 2, 0)
        self.spin_capture_interval = QDoubleSpinBox()
        self.spin_capture_interval.setRange(core.CAPTURE_TIME_INTERVAL_MIN, 60.0)
        self.spin_capture_interval.setValue(core.CAPTURE_TIME_INTERVAL)
        self.spin_capture_interval.setDecimals(1)
        self.spin_capture_interval.setToolTip("M3T interval capture is usually stable at 2s or slower.")
        self.spin_capture_interval.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_capture_interval, 2, 1)

        self.btn_use_recommended_speed = QPushButton("Use Recommended Speed")
        self.btn_use_recommended_speed.clicked.connect(self._apply_recommended_speed)
        layout.addWidget(self.btn_use_recommended_speed, 2, 2, 1, 2)

        self.lbl_capture_hint = QLabel("")
        self.lbl_capture_hint.setWordWrap(True)
        self.lbl_capture_hint.setStyleSheet("color: #666; font-size: 11px;")
        layout.addWidget(self.lbl_capture_hint, 3, 0, 1, 4)

        self.lbl_time_speed_warning = QLabel("")
        self.lbl_time_speed_warning.setWordWrap(True)
        self.lbl_time_speed_warning.setStyleSheet("color: #f57c00; font-size: 12px; font-weight: bold;")
        self.lbl_time_speed_warning.setVisible(False)
        layout.addWidget(self.lbl_time_speed_warning, 4, 0, 1, 4)

        layout.addWidget(QLabel("Image Format:"), 5, 0)
        self.combo_image_format = QComboBox()
        self.combo_image_format.currentIndexChanged.connect(self._on_param_changed)
        layout.addWidget(self.combo_image_format, 5, 1, 1, 3)

        advanced = QGroupBox("Advanced Safety Settings")
        adv = QGridLayout(advanced)

        adv.addWidget(QLabel("Finish Action:"), 0, 0)
        self.combo_finish_action = QComboBox()
        self.combo_finish_action.addItem("No action", "noAction")
        self.combo_finish_action.addItem("Return to home", "goHome")
        self.combo_finish_action.addItem("Auto land", "autoLand")
        self.combo_finish_action.addItem("Go to first waypoint", "gotoFirstWaypoint")
        self.combo_finish_action.currentIndexChanged.connect(self._on_param_changed)
        adv.addWidget(self.combo_finish_action, 0, 1)

        adv.addWidget(QLabel("RC Lost:"), 0, 2)
        self.combo_exit_on_rc_lost = QComboBox()
        self.combo_exit_on_rc_lost.addItem("Execute lost action", "executeLostAction")
        self.combo_exit_on_rc_lost.addItem("Continue route", "goContinue")
        self.combo_exit_on_rc_lost.currentIndexChanged.connect(self._on_param_changed)
        adv.addWidget(self.combo_exit_on_rc_lost, 0, 3)

        adv.addWidget(QLabel("Lost Action:"), 1, 0)
        self.combo_execute_rc_lost = QComboBox()
        self.combo_execute_rc_lost.addItem("Hover", "hover")
        self.combo_execute_rc_lost.addItem("Go back", "goBack")
        self.combo_execute_rc_lost.addItem("Land", "landing")
        self.combo_execute_rc_lost.currentIndexChanged.connect(self._on_param_changed)
        adv.addWidget(self.combo_execute_rc_lost, 1, 1)

        adv.addWidget(QLabel("Takeoff Sec. Height (m):"), 1, 2)
        self.spin_takeoff_security_height = QDoubleSpinBox()
        self.spin_takeoff_security_height.setRange(1.2, 1500.0)
        self.spin_takeoff_security_height.setValue(core.TAKE_OFF_SECURITY_HEIGHT)
        self.spin_takeoff_security_height.setDecimals(1)
        self.spin_takeoff_security_height.valueChanged.connect(self._on_param_changed)
        adv.addWidget(self.spin_takeoff_security_height, 1, 3)

        adv.addWidget(QLabel("Transition Speed (m/s):"), 2, 0)
        self.spin_global_transition_speed = QDoubleSpinBox()
        self.spin_global_transition_speed.setRange(1.0, 15.0)
        self.spin_global_transition_speed.setValue(core.GLOBAL_TRANSITIONAL_SPEED)
        self.spin_global_transition_speed.setDecimals(1)
        self.spin_global_transition_speed.valueChanged.connect(self._on_param_changed)
        adv.addWidget(self.spin_global_transition_speed, 2, 1)

        self.lbl_rc_lost_hint = QLabel(
            "If RC Lost is 'Continue route', DJI may continue the mission instead of using the lost action."
        )
        self.lbl_rc_lost_hint.setWordWrap(True)
        self.lbl_rc_lost_hint.setStyleSheet("color: #666; font-size: 11px;")
        adv.addWidget(self.lbl_rc_lost_hint, 2, 2, 1, 2)

        layout.addWidget(advanced, 6, 0, 1, 4)

        return group

    # -------------------------------------------------------------------------
    # Zone 4: Image Info Display
    # -------------------------------------------------------------------------
    def _create_zone4_info(self) -> QGroupBox:
        group = QGroupBox("4. Image Info")
        layout = QVBoxLayout(group)

        self.txt_info = QTextEdit()
        self.txt_info.setReadOnly(True)
        self.txt_info.setMaximumHeight(120)
        self.txt_info.setPlaceholderText(
            "Drop 4 corner photos (BL → TL → TR → BR, facing the wall) to display GPS…"
        )
        layout.addWidget(self.txt_info)

        self.lbl_rtk_warning = QLabel("")
        self.lbl_rtk_warning.setWordWrap(True)
        self.lbl_rtk_warning.setVisible(False)
        layout.addWidget(self.lbl_rtk_warning)

        return group

    # -------------------------------------------------------------------------
    # Zone 5: Path Generation
    # -------------------------------------------------------------------------
    def _create_zone5_generate(self) -> QGroupBox:
        group = QGroupBox("5. Path Generation")
        layout = QVBoxLayout(group)

        self.btn_generate = QPushButton("Generate Mission")
        self.btn_generate.setMinimumHeight(40)
        self.btn_generate.setStyleSheet("font-weight: bold;")
        self.btn_generate.clicked.connect(self._generate_mission)
        layout.addWidget(self.btn_generate)

        self.lbl_raw_order_warning = QLabel("")
        self.lbl_raw_order_warning.setWordWrap(True)
        self.lbl_raw_order_warning.setStyleSheet("color: #f57c00; font-weight: bold;")
        self.lbl_raw_order_warning.setVisible(False)
        layout.addWidget(self.lbl_raw_order_warning)

        self.lbl_gen_status = QLabel("Status: Waiting for images...")
        self.lbl_gen_status.setWordWrap(True)
        layout.addWidget(self.lbl_gen_status)

        return group

    # -------------------------------------------------------------------------
    # Zone 6: Output
    # -------------------------------------------------------------------------
    def _create_zone6_output(self) -> QGroupBox:
        group = QGroupBox("6. Output")
        layout = QVBoxLayout(group)

        # Output directory
        dir_layout = QHBoxLayout()
        dir_layout.addWidget(QLabel("Output Directory:"))
        self.edit_output_dir = QLineEdit(str(Path.home() / "Desktop"))
        dir_layout.addWidget(self.edit_output_dir)
        self.btn_browse_output = QPushButton("Browse...")
        self.btn_browse_output.clicked.connect(self._browse_output_dir)
        dir_layout.addWidget(self.btn_browse_output)
        layout.addLayout(dir_layout)

        # Save buttons
        btn_layout = QHBoxLayout()

        self.btn_save_kmz = QPushButton("Save KMZ")
        self.btn_save_kmz.clicked.connect(lambda: self._save_output("kmz"))
        btn_layout.addWidget(self.btn_save_kmz)

        self.btn_save_kml = QPushButton("Save Preview KML")
        self.btn_save_kml.clicked.connect(lambda: self._save_output("kml"))
        btn_layout.addWidget(self.btn_save_kml)

        self.btn_save_both = QPushButton("Save Both")
        self.btn_save_both.clicked.connect(lambda: self._save_output("both"))
        btn_layout.addWidget(self.btn_save_both)

        layout.addLayout(btn_layout)

        self.lbl_save_status = QLabel("")
        self.lbl_save_status.setWordWrap(True)
        layout.addWidget(self.lbl_save_status)

        return group

    # -------------------------------------------------------------------------
    # Event Handlers
    # -------------------------------------------------------------------------
    def _on_facade_grid_changed(self):
        """Sync image_paths from 2×2 corner slots (BL, TL, TR, BR)."""
        if all(self.drop_zone.slot_paths):
            self.image_paths = [p for p in self.drop_zone.slot_paths if p]
            logger.info("All 4 facade corners assigned (BL→TL→TR→BR order)")
        else:
            self.image_paths = []
        self._extract_gps()
        self._invalidate_generation()
        self._update_ui_state()

    def _select_images(self):
        """Open file dialog to select images."""
        files, _ = QFileDialog.getOpenFileNames(
            self,
            "Select 4 images in order: BL → TL → TR → BR (facing wall)",
            "",
            "Images (*.jpg *.jpeg *.dng *.JPG *.JPEG *.DNG)",
        )
        if files:
            if len(files) != 4:
                QMessageBox.warning(
                    self,
                    "Need 4 images",
                    "Select exactly 4 images in corner order: "
                    "Bottom-Left (BL), Top-Left (TL), Top-Right (TR), Bottom-Right (BR).",
                )
                return
            self.drop_zone.apply_sequential_paths(files)

    def _select_folder(self):
        """Open folder dialog to select image folder."""
        folder = QFileDialog.getExistingDirectory(self, "Select Folder")
        if folder:
            images = []
            for f in sorted(os.listdir(folder)):
                if f.lower().endswith(('.jpg', '.jpeg')):
                    images.append(os.path.join(folder, f))

            if not images:
                QMessageBox.warning(self, "No Images", "No JPG/JPEG images found in folder.")
                return

            if len(images) < 4:
                QMessageBox.warning(
                    self, "Not Enough Images",
                    f"Found only {len(images)} images. Need 4 for facade detection.",
                )
                return
            if len(images) > 4:
                QMessageBox.information(
                    self, "Multiple Images",
                    f"Found {len(images)} images. Using first 4 alphabetically — "
                    "verify they match BL, TL, TR, BR or assign per cell.",
                )
                images = images[:4]

            self.drop_zone.apply_sequential_paths(images)

    def _clear_images(self):
        """Clear all loaded images."""
        logger.info("Clearing all loaded images")
        self.image_paths = []
        self.gps_data = []
        self.photo_metadata = []
        self.all_photos_rtk_fix = False
        self.corner_geometry_ok = False
        self.facade_width = None
        self.facade_height = None
        self.drop_zone.clear()
        self.txt_info.clear()
        if hasattr(self, "lbl_rtk_warning"):
            self.lbl_rtk_warning.clear()
            self.lbl_rtk_warning.setVisible(False)
        if hasattr(self, "lbl_raw_order_warning"):
            self.lbl_raw_order_warning.clear()
            self.lbl_raw_order_warning.setVisible(False)
        self._invalidate_generation()
        self._update_ui_state()

    def _on_param_changed(self):
        """Handle parameter change - invalidate generation."""
        if hasattr(self, "lbl_capture_hint"):
            self._update_capture_controls()
        self._invalidate_generation()

    def _on_capture_mode_changed(self):
        """Update capture-mode dependent controls."""
        self._update_capture_controls()
        self._invalidate_generation()

    def _estimated_direction(self) -> str:
        if self.flight_direction:
            return self.flight_direction
        if (
            self.chk_smart_planning.isChecked()
            and self.facade_width is not None
            and self.facade_height is not None
        ):
            return "vertical" if self.facade_height > self.facade_width else "horizontal"
        return "horizontal"

    def _capture_hint_values(self) -> Tuple[str, float, float, float]:
        direction = self._estimated_direction()
        distance = self.spin_photo_dist.value()
        overlap = self.spin_overlap.value() / 100.0
        hfov = core.radians(self.spin_hfov.value())
        vfov = core.radians(self.spin_vfov.value())
        fov = vfov if direction == "vertical" else hfov
        coverage = max(2.0 * distance * core.tan(fov / 2.0), 0.01)
        target_spacing = max(coverage * (1.0 - overlap), 0.01)
        interval = self.spin_capture_interval.value()
        recommended_speed = target_spacing / max(interval, core.CAPTURE_TIME_INTERVAL_MIN)
        actual_spacing = self.spin_speed.value() * interval
        actual_overlap = max(0.0, min(1.0, 1.0 - actual_spacing / coverage))
        return direction, target_spacing, recommended_speed, actual_overlap

    def _time_speed_warning_text(self) -> Optional[str]:
        if self.combo_capture_mode.currentData() != "time":
            return None
        _, _, recommended_speed, actual_overlap = self._capture_hint_values()
        target_speed = max(
            core.AUTO_FLIGHT_SPEED_MIN,
            min(core.AUTO_FLIGHT_SPEED_MAX, recommended_speed),
        )
        target_speed = round(target_speed, self.spin_speed.decimals())
        current_speed = self.spin_speed.value()
        if abs(current_speed - target_speed) <= 0.05:
            return None
        return (
            "Warning: By time mode needs the recommended speed to maintain the target overlap. "
            f"Recommended {target_speed:.1f} m/s, current {current_speed:.1f} m/s "
            f"(estimated overlap {actual_overlap * 100:.0f}%). Click 'Use Recommended Speed' "
            "or manually match the speed before generating."
        )

    def _update_capture_controls(self):
        """Refresh controls and estimate text for continuous capture."""
        mode = self.combo_capture_mode.currentData()
        is_time = mode == "time"
        self.spin_capture_interval.setEnabled(is_time)
        self.btn_use_recommended_speed.setEnabled(is_time)

        direction, target_spacing, recommended_speed, actual_overlap = self._capture_hint_values()
        if mode == "waypoint":
            text = "Current mode: drone stops at each waypoint, then shoots. Highest positional accuracy."
        elif mode == "time":
            interval = self.spin_capture_interval.value()
            actual_spacing = self.spin_speed.value() * interval
            text = (
                f"Continuous time mode: {interval:.1f}s interval. Recommended speed "
                f"{recommended_speed:.2f} m/s for {self.spin_overlap.value()}% overlap; "
                f"current speed gives {actual_spacing:.2f} m/photo and about "
                f"{actual_overlap * 100:.0f}% overlap ({direction} route). "
                "Route corners still stop to preserve facade coverage."
            )
        elif mode == "video":
            text = (
                "Video mode: recording starts at the first waypoint and stops at the last waypoint. "
                "The route uses continuous turns for smoother constant-speed footage."
            )
        else:
            text = "Current mode: no photo or video actions will be written to the KMZ."
        self.lbl_capture_hint.setText(text)
        if hasattr(self, "lbl_time_speed_warning"):
            warning = self._time_speed_warning_text()
            self.lbl_time_speed_warning.setText(warning or "")
            self.lbl_time_speed_warning.setVisible(bool(warning))

    def _apply_recommended_speed(self):
        """Set speed from the current time interval and target overlap."""
        _, _, recommended_speed, _ = self._capture_hint_values()
        recommended_speed = max(
            core.AUTO_FLIGHT_SPEED_MIN,
            min(core.AUTO_FLIGHT_SPEED_MAX, recommended_speed),
        )
        self.spin_speed.setValue(recommended_speed)

    def _on_drone_type_changed(self, _text: Optional[str] = None):
        """Apply selected aircraft/payload profile and lock FOV to the preset."""
        dt = self.combo_drone.currentText()
        prof = core.DRONE_WPML_PROFILES[dt]
        core.DRONE_TYPE = dt
        core.apply_drone_wpml_profile(dt)
        self.spin_hfov.blockSignals(True)
        self.spin_vfov.blockSignals(True)
        self.spin_hfov.setValue(float(prof["hfov"]))
        self.spin_vfov.setValue(float(prof["vfov"]))
        self.spin_hfov.blockSignals(False)
        self.spin_vfov.blockSignals(False)
        self.spin_hfov.setEnabled(False)
        self.spin_vfov.setEnabled(False)
        self._refresh_image_format_options()

    def _refresh_image_format_options(self):
        if not hasattr(self, "combo_image_format"):
            return
        current = self.combo_image_format.currentData() or core.IMAGE_FORMAT
        self.combo_image_format.blockSignals(True)
        self.combo_image_format.clear()
        if self.combo_drone.currentText() == "M3T":
            options = [
                ("Visible only (wide)", "wide"),
                ("Infrared only", "ir"),
                ("Visible + infrared", "wide,ir"),
            ]
        else:
            options = [("Visible only (wide)", "wide")]
        for label, value in options:
            self.combo_image_format.addItem(label, value)
        idx = self.combo_image_format.findData(current)
        self.combo_image_format.setCurrentIndex(idx if idx >= 0 else 0)
        self.combo_image_format.blockSignals(False)

    def _extract_gps(self):
        """Extract GPS per facade corner slot (BL, TL, TR, BR — facing wall)."""
        logger.debug("Extracting GPS data from images")
        self.gps_data = []
        self.photo_metadata = []
        self.all_photos_rtk_fix = False
        self.corner_geometry_ok = False
        self.facade_width = None
        self.facade_height = None
        info_lines = []

        corner_labels = ("Bottom-Left BL", "Top-Left TL", "Top-Right TR", "Bottom-Right BR")
        slot_paths = self.drop_zone.slot_paths

        # Reset raw-order warning (will be set again if geometry check passes)
        if hasattr(self, "lbl_raw_order_warning"):
            self.lbl_raw_order_warning.clear()
            self.lbl_raw_order_warning.setVisible(False)

        for i, path in enumerate(slot_paths):
            label = corner_labels[i]
            if not path:
                info_lines.append(f"{label}: (no photo assigned)")
                self.gps_data.append(None)
                self.photo_metadata.append(None)
                continue
            try:
                meta = core.read_photo_metadata(path)
                lat, lon, alt = meta["lat"], meta["lon"], meta["alt"]
                self.gps_data.append((lat, lon, alt))
                self.photo_metadata.append(meta)
                lat_dir = "N" if lat >= 0 else "S"
                lon_dir = "E" if lon >= 0 else "W"
                rtk_text = (
                    "RTK FIX"
                    if meta["is_rtk_fix"]
                    else f"RTK not confirmed (GPSStatus={meta['gps_status'] or 'unknown'}, "
                         f"RtkFlag={meta['rtk_flag'] or 'unknown'})"
                )
                info_lines.append(
                    f"{label}: {abs(lat):.6f}°{lat_dir}, {abs(lon):.6f}°{lon_dir}, "
                    f"{alt:.1f}m (WGS84), {rtk_text}"
                )
            except Exception as e:
                logger.error(f"Failed to extract GPS for {label}: {e}")
                info_lines.append(f"{label}: ERROR — {e}")
                self.gps_data.append(None)
                self.photo_metadata.append(None)

        all_slots_have_meta = len(self.photo_metadata) == 4 and all(m is not None for m in self.photo_metadata)
        self.all_photos_rtk_fix = (
            all_slots_have_meta and all(bool(m["is_rtk_fix"]) for m in self.photo_metadata if m)
        )
        if hasattr(self, "lbl_rtk_warning"):
            if all_slots_have_meta and self.all_photos_rtk_fix:
                self.lbl_rtk_warning.setText("RTK status: all 4 photos are confirmed RTK FIX.")
                self.lbl_rtk_warning.setStyleSheet("color: #2e7d32; font-size: 11px;")
                self.lbl_rtk_warning.setVisible(True)
            elif all_slots_have_meta:
                self.lbl_rtk_warning.setText(
                    "Warning: Not all photos are confirmed RTK FIX. Facade geometry may be inaccurate; "
                    "close facade flight is not recommended."
                )
                self.lbl_rtk_warning.setStyleSheet("color: #f57c00; font-size: 11px; font-weight: bold;")
                self.lbl_rtk_warning.setVisible(True)
            else:
                self.lbl_rtk_warning.clear()
                self.lbl_rtk_warning.setVisible(False)

        # Facade geometry + near-rectangle corners (blocks Generate if invalid)
        if len(self.gps_data) == 4 and all(g is not None for g in self.gps_data):
            try:
                tf = core.FacadeTransformer(self.gps_data)
                core.validate_facade_corner_geometry(tf)
                self.corner_geometry_ok = True

                raw_warn = getattr(tf, "raw_order_warning", None)
                if raw_warn and hasattr(self, "lbl_raw_order_warning"):
                    self.lbl_raw_order_warning.setText(
                        "Warning: The uploaded corner order may not match the recommended order, "
                        "which could affect facade interpretation and planning accuracy. "
                        "The system has continued using automatic geometric corner identification. "
                        "Please review the selected corners."
                    )
                    self.lbl_raw_order_warning.setVisible(True)
                xs = [p[0] for p in tf.facade_pts]
                zs = [p[2] for p in tf.facade_pts]
                width = max(xs) - min(xs)
                height = max(zs) - min(zs)
                self.facade_width = width
                self.facade_height = height
                info_lines.append(f"\nFacade: {width:.1f}m × {height:.1f}m")
            except Exception as e:
                logger.error(f"Facade analysis error: {e}")
                info_lines.append(f"\nFacade analysis error: {str(e)}")

        self.txt_info.setPlainText("\n".join(info_lines))

    def _invalidate_generation(self):
        """Mark generation as invalid (needs regeneration)."""
        self.generated_waypoints = []
        self.transformer = None
        self.flight_direction = ""
        self.lbl_gen_status.setText("Status: Parameters changed, regenerate mission")
        self.lbl_gen_status.setStyleSheet("color: #ff9800;")
        self._update_ui_state()

    def _generate_mission(self):
        """Generate mission waypoints."""
        logger.info("Starting mission generation")
        if len(self.image_paths) != 4:
            logger.warning("Cannot generate: need exactly 4 images")
            QMessageBox.warning(self, "Error", "Please load exactly 4 images.")
            return

        if not all(g is not None for g in self.gps_data):
            logger.warning("Cannot generate: some images have invalid GPS data")
            QMessageBox.warning(self, "Error", "Some images have invalid GPS data.")
            return

        time_speed_warning = self._time_speed_warning_text()
        if time_speed_warning:
            reply = QMessageBox.warning(
                self,
                "By Time Speed Warning",
                time_speed_warning + "\n\nGenerate anyway?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply != QMessageBox.Yes:
                return

        try:
            # Update core module parameters
            logger.debug(f"Parameters: distance={self.spin_photo_dist.value()}")
            logger.debug(f"Parameters: HFOV={self.spin_hfov.value()}, VFOV={self.spin_vfov.value()}, overlap={self.spin_overlap.value()}")
            logger.debug(f"Parameters: force_vertical={self.chk_force_vertical.isChecked()}, smart_planning={self.chk_smart_planning.isChecked()}")
            drone_type = self.combo_drone.currentText()
            core.DRONE_TYPE = drone_type
            core.apply_drone_wpml_profile(drone_type)
            core.PHOTO_DISTANCE = self.spin_photo_dist.value()
            core.CAMERA_HFOV = self.spin_hfov.value()
            core.CAMERA_VFOV = self.spin_vfov.value()
            core.HFOV_RAD = core.radians(core.CAMERA_HFOV)
            core.VFOV_RAD = core.radians(core.CAMERA_VFOV)
            core.OVERLAP_RATE = self.spin_overlap.value() / 100.0
            core.ENABLE_SMART_PLANNING = self.chk_smart_planning.isChecked()
            core.FORCE_VERTICAL_PLANE = self.chk_force_vertical.isChecked()
            core.AUTO_FLIGHT_SPEED = self.spin_speed.value()
            core.GIMBAL_PITCH_DEG = self.spin_gimbal.value()
            core.MISSION_FINISH_ACTION = self.combo_finish_action.currentData()
            core.MISSION_EXIT_ON_RC_LOST = self.combo_exit_on_rc_lost.currentData()
            core.MISSION_EXECUTE_RC_LOST_ACTION = self.combo_execute_rc_lost.currentData()
            core.TAKE_OFF_SECURITY_HEIGHT = self.spin_takeoff_security_height.value()
            core.GLOBAL_TRANSITIONAL_SPEED = self.spin_global_transition_speed.value()
            core.IMAGE_FORMAT = self.combo_image_format.currentData()
            core.POSITIONING_TYPE = "RTKBaseStation" if self.all_photos_rtk_fix else "GPS"
            capture_mode = self.combo_capture_mode.currentData()
            core.ENABLE_PHOTO_CAPTURE = capture_mode != "none"
            core.CAPTURE_MODE = capture_mode
            core.CAPTURE_TIME_INTERVAL = self.spin_capture_interval.value()

            # Generate
            self.transformer, self.flight_direction, self.generated_waypoints = \
                core.build_waypoints_from_images(self.image_paths)

            logger.success(f"Generated {len(self.generated_waypoints)} waypoints ({self.flight_direction} pattern)")
            status = (
                f"Status: {len(self.generated_waypoints)} waypoints generated\n"
                f"Drone: {drone_type}\n"
                f"Positioning: {core.POSITIONING_TYPE}\n"
                f"Direction: {self.flight_direction} snake pattern\n"
                f"Capture: {self.combo_capture_mode.currentText()}\n"
                f"Image format: {core.IMAGE_FORMAT}"
            )
            self.lbl_gen_status.setText(status)
            self.lbl_gen_status.setStyleSheet("color: #4caf50;")
            self._update_capture_controls()

        except Exception as e:
            logger.error(f"Generation failed: {e}")
            QMessageBox.critical(self, "Generation Error", str(e))
            self.lbl_gen_status.setText(f"Status: Error - {str(e)}")
            self.lbl_gen_status.setStyleSheet("color: #f44336;")

        self._update_ui_state()

    def _browse_output_dir(self):
        """Browse for output directory."""
        folder = QFileDialog.getExistingDirectory(self, "Select Output Directory")
        if folder:
            self.edit_output_dir.setText(folder)

    def _save_output(self, mode: str):
        """Save output files."""
        logger.info(f"Saving output files (mode: {mode})")
        if not self.generated_waypoints or not self.transformer:
            logger.warning("Cannot save: no mission generated")
            QMessageBox.warning(self, "Error", "Please generate mission first.")
            return

        output_dir = self.edit_output_dir.text()
        if not os.path.isdir(output_dir):
            logger.warning(f"Invalid output directory: {output_dir}")
            QMessageBox.warning(self, "Error", "Invalid output directory.")
            return

        mission_name = core.sanitize_name(self.edit_mission_name.text()) or "Mission"
        saved_files = []

        try:
            if mode in ("kml", "both"):
                kml = core.build_generic_kml(self.generated_waypoints)
                kml_path = os.path.join(output_dir, f"{mission_name}_preview.kml")
                core.ElementTree(kml).write(kml_path, encoding="utf-8", xml_declaration=True)
                saved_files.append(kml_path)
                logger.success(f"Saved preview KML: {kml_path}")

            if mode in ("kmz", "both"):
                waylines = core.build_waylines_wpml(
                    self.transformer, self.generated_waypoints, self.flight_direction
                )
                template = core.build_template_kml(
                    self.transformer, self.generated_waypoints, self.flight_direction
                )
                kmz_path = os.path.join(output_dir, f"{mission_name}.kmz")
                core.write_kmz(template, waylines, kmz_path)
                saved_files.append(kmz_path)

            self.lbl_save_status.setText(f"Saved: {', '.join(Path(f).name for f in saved_files)}")
            self.lbl_save_status.setStyleSheet("color: #4caf50;")

        except Exception as e:
            logger.error(f"Save failed: {e}")
            QMessageBox.critical(self, "Save Error", str(e))
            self.lbl_save_status.setText(f"Error: {str(e)}")
            self.lbl_save_status.setStyleSheet("color: #f44336;")

    def _update_ui_state(self):
        """Update UI enabled/disabled states."""
        slots_ok = all(self.drop_zone.slot_paths)
        has_4_images = (
            slots_ok
            and len(self.image_paths) == 4
            and len(self.gps_data) == 4
            and all(g is not None for g in self.gps_data)
            and self.corner_geometry_ok
        )
        has_generation = len(self.generated_waypoints) > 0

        self.btn_generate.setEnabled(has_4_images)
        self.btn_save_kmz.setEnabled(has_generation)
        self.btn_save_kml.setEnabled(has_generation)
        self.btn_save_both.setEnabled(has_generation)

        if not has_4_images:
            if (
                slots_ok
                and len(self.gps_data) == 4
                and all(g is not None for g in self.gps_data)
                and not self.corner_geometry_ok
            ):
                self.lbl_gen_status.setText("Status: Corner geometry check failed — see panel below.")
            else:
                self.lbl_gen_status.setText("Status: Load 4 valid images to enable generation")
            self.lbl_gen_status.setStyleSheet("color: #888;")


# =============================================================================
# Entry Point
# =============================================================================

def main():
    logger.info("Starting AeroFacade Studio GUI")
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = MainWindow()
    window.show()
    logger.info("GUI window displayed")

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
