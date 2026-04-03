#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PySide6 GUI for Mavic 3T Facade Mission Planner

Provides a desktop interface for the drone mission planning tool with:
- Drag & drop image/folder input
- Parameter configuration for all algorithm settings
- Real-time GPS metadata display
- Mission generation and KMZ export
"""

import sys
import os
from pathlib import Path
from typing import Dict, List, Optional, Tuple

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
            pix = QPixmap(path)
            if not pix.isNull():
                self.thumb.setPixmap(
                    pix.scaled(72, 72, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                )
            else:
                self.thumb.clear()
                self.thumb.setText("ERR")
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
            if path.lower().endswith((".jpg", ".jpeg")) and os.path.isfile(path):
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
            (1, 0, 0, "左上 TL\n#2"),
            (2, 0, 1, "右上 TR\n#3"),
            (0, 1, 0, "左下 BL\n#1"),
            (3, 1, 1, "右下 BR\n#4"),
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
            if f.lower().endswith((".jpg", ".jpeg")):
                images.append(os.path.join(folder, f))
        return images


# =============================================================================
# Main Window
# =============================================================================

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mavic 3T Facade Mission Planner")
        self.setMinimumSize(600, 800)

        # State
        self.image_paths: List[str] = []
        self.gps_data: List[Optional[Tuple[float, float, float]]] = []
        self.generated_waypoints: List[Tuple[float, float, float]] = []
        self.transformer: Optional[core.FacadeTransformer] = None
        self.flight_direction: str = ""
        self.corner_geometry_ok: bool = False

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

        params.addWidget(QLabel("Photo Distance (m):"), 1, 0)
        self.spin_photo_dist = QDoubleSpinBox()
        self.spin_photo_dist.setRange(core.PHOTO_DISTANCE_MIN, core.PHOTO_DISTANCE_MAX)
        self.spin_photo_dist.setValue(5.0)
        self.spin_photo_dist.setDecimals(1)
        self.spin_photo_dist.setToolTip(f"{core.PHOTO_DISTANCE_MIN:.0f}–{core.PHOTO_DISTANCE_MAX:.0f} m")
        self.spin_photo_dist.valueChanged.connect(self._on_param_changed)
        params.addWidget(self.spin_photo_dist, 1, 1)

        params.addWidget(QLabel("Flight Distance (m):"), 1, 2)
        self.spin_flight_dist = QDoubleSpinBox()
        self.spin_flight_dist.setRange(core.FLIGHT_DISTANCE_MIN, core.FLIGHT_DISTANCE_MAX)
        self.spin_flight_dist.setValue(5.0)
        self.spin_flight_dist.setDecimals(1)
        self.spin_flight_dist.setToolTip(f"{core.FLIGHT_DISTANCE_MIN:.0f}–{core.FLIGHT_DISTANCE_MAX:.0f} m")
        self.spin_flight_dist.valueChanged.connect(self._on_param_changed)
        params.addWidget(self.spin_flight_dist, 1, 3)

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
        self.combo_drone.addItems(["M3E", "M3T"])
        self.combo_drone.setCurrentText("M3T")
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
        layout.addWidget(self.spin_speed, 0, 1)

        layout.addWidget(QLabel("Gimbal Pitch (°):"), 0, 2)
        self.spin_gimbal = QDoubleSpinBox()
        self.spin_gimbal.setRange(-90.0, 30.0)
        self.spin_gimbal.setValue(0.0)
        self.spin_gimbal.setDecimals(1)
        layout.addWidget(self.spin_gimbal, 0, 3)

        layout.addWidget(QLabel("Execute Height:"), 1, 0)
        self.combo_exec_height = QComboBox()
        self.combo_exec_height.addItems(["WGS84", "relativeToStartPoint"])
        layout.addWidget(self.combo_exec_height, 1, 1)

        layout.addWidget(QLabel("Template Height:"), 1, 2)
        self.combo_template_height = QComboBox()
        self.combo_template_height.addItems(["EGM96", "relativeToStartPoint"])
        layout.addWidget(self.combo_template_height, 1, 3)

        layout.addWidget(QLabel("Height Offset (m):"), 2, 0)
        self.spin_height_offset = QDoubleSpinBox()
        self.spin_height_offset.setRange(-100.0, 100.0)
        self.spin_height_offset.setValue(0.0)
        self.spin_height_offset.setDecimals(1)
        self.spin_height_offset.setToolTip("Constant offset added to all waypoint altitudes (+ = higher)")
        self.spin_height_offset.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_height_offset, 2, 1)

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
            "在田字格中放满四张角点照片（左下→左上→右上→右下，面对墙）后显示 GPS…"
        )
        layout.addWidget(self.txt_info)

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
        self.lbl_raw_order_warning.setStyleSheet("color: #f44336; font-weight: bold;")
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
            "Images (*.jpg *.jpeg *.JPG *.JPEG)",
        )
        if files:
            if len(files) != 4:
                QMessageBox.warning(
                    self,
                    "Need 4 images",
                    "Select exactly 4 images in corner order: "
                    "左下(BL), 左上(TL), 右上(TR), 右下(BR).",
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
        self.corner_geometry_ok = False
        self.drop_zone.clear()
        self.txt_info.clear()
        if hasattr(self, "lbl_raw_order_warning"):
            self.lbl_raw_order_warning.clear()
            self.lbl_raw_order_warning.setVisible(False)
        self._invalidate_generation()
        self._update_ui_state()

    def _on_param_changed(self):
        """Handle parameter change - invalidate generation."""
        self._invalidate_generation()

    def _on_drone_type_changed(self, _text: Optional[str] = None):
        """M3 line presets: FOV fixed from profile, editors disabled."""
        dt = self.combo_drone.currentText()
        prof = core.DRONE_WPML_PROFILES[dt]
        self.spin_hfov.blockSignals(True)
        self.spin_vfov.blockSignals(True)
        self.spin_hfov.setValue(float(prof["hfov"]))
        self.spin_vfov.setValue(float(prof["vfov"]))
        self.spin_hfov.blockSignals(False)
        self.spin_vfov.blockSignals(False)
        self.spin_hfov.setEnabled(False)
        self.spin_vfov.setEnabled(False)

    def _extract_gps(self):
        """Extract GPS per facade corner slot (BL, TL, TR, BR — facing wall)."""
        logger.debug("Extracting GPS data from images")
        self.gps_data = []
        self.corner_geometry_ok = False
        info_lines = []

        corner_labels = ("左下 BL", "左上 TL", "右上 TR", "右下 BR")
        slot_paths = self.drop_zone.slot_paths

        # Reset raw-order warning (will be set again if geometry check passes)
        if hasattr(self, "lbl_raw_order_warning"):
            self.lbl_raw_order_warning.clear()
            self.lbl_raw_order_warning.setVisible(False)

        for i, path in enumerate(slot_paths):
            label = corner_labels[i]
            if not path:
                info_lines.append(f"{label}: (未放置照片)")
                self.gps_data.append(None)
                continue
            try:
                lat, lon, alt = core.read_gps(path)
                self.gps_data.append((lat, lon, alt))
                lat_dir = "N" if lat >= 0 else "S"
                lon_dir = "E" if lon >= 0 else "W"
                info_lines.append(
                    f"{label}: {abs(lat):.6f}°{lat_dir}, {abs(lon):.6f}°{lon_dir}, {alt:.1f}m (WGS84)"
                )
            except Exception as e:
                logger.error(f"Failed to extract GPS for {label}: {e}")
                info_lines.append(f"{label}: ERROR — {e}")
                self.gps_data.append(None)

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

        try:
            # Update core module parameters
            logger.debug(f"Parameters: photo_dist={self.spin_photo_dist.value()}, flight_dist={self.spin_flight_dist.value()}")
            logger.debug(f"Parameters: HFOV={self.spin_hfov.value()}, VFOV={self.spin_vfov.value()}, overlap={self.spin_overlap.value()}")
            logger.debug(f"Parameters: force_vertical={self.chk_force_vertical.isChecked()}, smart_planning={self.chk_smart_planning.isChecked()}")
            logger.debug(f"Parameters: height_offset={self.spin_height_offset.value()}")
            core.PHOTO_DISTANCE = self.spin_photo_dist.value()
            core.FLIGHT_DISTANCE = self.spin_flight_dist.value()
            core.CAMERA_HFOV = self.spin_hfov.value()
            core.CAMERA_VFOV = self.spin_vfov.value()
            core.HFOV_RAD = core.radians(core.CAMERA_HFOV)
            core.VFOV_RAD = core.radians(core.CAMERA_VFOV)
            core.OVERLAP_RATE = self.spin_overlap.value() / 100.0
            core.ENABLE_SMART_PLANNING = self.chk_smart_planning.isChecked()
            core.FORCE_VERTICAL_PLANE = self.chk_force_vertical.isChecked()
            core.AUTO_FLIGHT_SPEED = self.spin_speed.value()
            core.GIMBAL_PITCH_DEG = self.spin_gimbal.value()
            core.EXECUTE_HEIGHT_MODE = self.combo_exec_height.currentText()
            core.TEMPLATE_HEIGHT_MODE = self.combo_template_height.currentText()
            core.HEIGHT_OFFSET = self.spin_height_offset.value()

            # Generate
            self.transformer, self.flight_direction, self.generated_waypoints = \
                core.build_waypoints_from_images(self.image_paths)

            logger.success(f"Generated {len(self.generated_waypoints)} waypoints ({self.flight_direction} pattern)")
            status = (
                f"Status: {len(self.generated_waypoints)} waypoints generated\n"
                f"Direction: {self.flight_direction} snake pattern"
            )
            self.lbl_gen_status.setText(status)
            self.lbl_gen_status.setStyleSheet("color: #4caf50;")

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
                waylines = core.build_waylines_wpml(self.transformer, self.generated_waypoints)
                template = core.build_template_kml(self.transformer, self.generated_waypoints)
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
    logger.info("Starting Mavic 3T Facade Mission Planner GUI")
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    window = MainWindow()
    window.show()
    logger.info("GUI window displayed")

    sys.exit(app.exec())


if __name__ == "__main__":
    main()
