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
from typing import List, Optional, Tuple

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QLineEdit, QDoubleSpinBox,
    QCheckBox, QComboBox, QTextEdit, QFileDialog, QMessageBox,
    QGridLayout, QFrame, QSizePolicy, QScrollArea
)
from PySide6.QtCore import Qt, Signal, QMimeData
from PySide6.QtGui import QPixmap, QDragEnterEvent, QDropEvent, QImage
from loguru import logger

# Import core algorithm functions
import mavic3T_pp_kmz as core


# =============================================================================
# Drop Zone Widget
# =============================================================================

class DropZone(QFrame):
    """Widget that accepts drag-and-drop of images or folders."""

    images_dropped = Signal(list)  # Emits list of file paths

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setAcceptDrops(True)
        self.setMinimumHeight(150)
        self.setFrameStyle(QFrame.StyledPanel | QFrame.Sunken)
        self.setStyleSheet("""
            DropZone {
                background-color: #f5f5f5;
                border: 2px dashed #aaa;
                border-radius: 8px;
            }
            DropZone[drag_over="true"] {
                background-color: #e3f2fd;
                border-color: #2196f3;
            }
        """)

        # Layout
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignCenter)

        # Thumbnail grid
        self.thumb_layout = QHBoxLayout()
        self.thumb_layout.setSpacing(10)
        self.thumb_layout.setAlignment(Qt.AlignCenter)

        self.thumb_labels: List[QLabel] = []
        self.name_labels: List[QLabel] = []

        for i in range(4):
            container = QVBoxLayout()

            thumb = QLabel()
            thumb.setFixedSize(64, 64)
            thumb.setFrameStyle(QFrame.Box)
            thumb.setAlignment(Qt.AlignCenter)
            thumb.setStyleSheet("background-color: #ddd; border: 1px solid #bbb;")
            thumb.setText(f"{i+1}")
            self.thumb_labels.append(thumb)

            name = QLabel("Empty")
            name.setFixedWidth(80)
            name.setAlignment(Qt.AlignCenter)
            name.setStyleSheet("font-size: 10px; color: #666;")
            self.name_labels.append(name)

            container.addWidget(thumb, alignment=Qt.AlignCenter)
            container.addWidget(name, alignment=Qt.AlignCenter)
            self.thumb_layout.addLayout(container)

        layout.addLayout(self.thumb_layout)

        # Instruction label
        self.instruction = QLabel("Drop images or folder here")
        self.instruction.setStyleSheet("color: #888; font-size: 12px;")
        self.instruction.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.instruction)

        self._drag_over = False

    def set_drag_over(self, value: bool):
        self._drag_over = value
        self.setProperty("drag_over", value)
        self.style().unpolish(self)
        self.style().polish(self)

    def dragEnterEvent(self, event: QDragEnterEvent):
        if event.mimeData().hasUrls():
            event.acceptProposedAction()
            self.set_drag_over(True)

    def dragLeaveEvent(self, event):
        self.set_drag_over(False)

    def dropEvent(self, event: QDropEvent):
        self.set_drag_over(False)
        urls = event.mimeData().urls()
        paths = []

        for url in urls:
            path = url.toLocalFile()
            if os.path.isdir(path):
                # Scan folder for images
                logger.debug(f"Scanning folder: {path}")
                paths.extend(self._scan_folder(path))
            elif self._is_image(path):
                paths.append(path)

        if paths:
            logger.info(f"Dropped {len(paths)} images")
            self.images_dropped.emit(paths[:4])  # Limit to 4
            if len(paths) > 4:
                logger.warning(f"Too many images ({len(paths)}), using first 4")
                QMessageBox.warning(
                    self, "Too Many Images",
                    f"Found {len(paths)} images. Only the first 4 will be used."
                )

    def _scan_folder(self, folder: str) -> List[str]:
        """Scan folder for JPG/JPEG images, sorted alphabetically."""
        images = []
        for f in sorted(os.listdir(folder)):
            if self._is_image(f):
                images.append(os.path.join(folder, f))
        return images

    def _is_image(self, path: str) -> bool:
        return path.lower().endswith(('.jpg', '.jpeg'))

    def update_thumbnails(self, paths: List[str]):
        """Update thumbnail display with loaded images."""
        for i, thumb in enumerate(self.thumb_labels):
            if i < len(paths):
                pixmap = QPixmap(paths[i])
                if not pixmap.isNull():
                    scaled = pixmap.scaled(64, 64, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                    thumb.setPixmap(scaled)
                else:
                    thumb.setText("ERR")
                self.name_labels[i].setText(Path(paths[i]).name[:12])
            else:
                thumb.clear()
                thumb.setText(f"{i+1}")
                self.name_labels[i].setText("Empty")

        if len(paths) == 4:
            self.instruction.setText("4 images loaded")
            self.instruction.setStyleSheet("color: #4caf50; font-size: 12px;")
        elif len(paths) > 0:
            self.instruction.setText(f"{len(paths)}/4 images loaded")
            self.instruction.setStyleSheet("color: #ff9800; font-size: 12px;")
        else:
            self.instruction.setText("Drop images or folder here")
            self.instruction.setStyleSheet("color: #888; font-size: 12px;")

    def clear(self):
        """Clear all thumbnails."""
        self.update_thumbnails([])


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
        self.gps_data: List[Tuple[float, float, float]] = []
        self.generated_waypoints: List[Tuple[float, float, float]] = []
        self.transformer: Optional[core.FacadeTransformer] = None
        self.flight_direction: str = ""

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

        # Initial state
        self._update_ui_state()

    # -------------------------------------------------------------------------
    # Zone 1: Input Settings
    # -------------------------------------------------------------------------
    def _create_zone1_input(self) -> QGroupBox:
        group = QGroupBox("1. Input Settings")
        layout = QVBoxLayout(group)

        # Drop zone
        self.drop_zone = DropZone()
        self.drop_zone.images_dropped.connect(self._on_images_dropped)
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
        self.spin_photo_dist.setRange(0.1, 100.0)
        self.spin_photo_dist.setValue(5.0)
        self.spin_photo_dist.setDecimals(1)
        self.spin_photo_dist.valueChanged.connect(self._on_param_changed)
        params.addWidget(self.spin_photo_dist, 1, 1)

        params.addWidget(QLabel("Flight Distance (m):"), 1, 2)
        self.spin_flight_dist = QDoubleSpinBox()
        self.spin_flight_dist.setRange(0.1, 100.0)
        self.spin_flight_dist.setValue(5.0)
        self.spin_flight_dist.setDecimals(1)
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

        layout.addWidget(QLabel("HFOV (°):"), 0, 0)
        self.spin_hfov = QDoubleSpinBox()
        self.spin_hfov.setRange(1.0, 180.0)
        self.spin_hfov.setValue(84.0)
        self.spin_hfov.setDecimals(1)
        self.spin_hfov.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_hfov, 0, 1)

        layout.addWidget(QLabel("VFOV (°):"), 0, 2)
        self.spin_vfov = QDoubleSpinBox()
        self.spin_vfov.setRange(1.0, 180.0)
        self.spin_vfov.setValue(62.0)
        self.spin_vfov.setDecimals(1)
        self.spin_vfov.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_vfov, 0, 3)

        layout.addWidget(QLabel("Overlap:"), 1, 0)
        self.spin_overlap = QDoubleSpinBox()
        self.spin_overlap.setRange(0.0, 0.99)
        self.spin_overlap.setValue(0.65)
        self.spin_overlap.setDecimals(2)
        self.spin_overlap.setSingleStep(0.05)
        self.spin_overlap.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_overlap, 1, 1)

        self.chk_smart_planning = QCheckBox("Enable Smart Planning")
        self.chk_smart_planning.setChecked(True)
        self.chk_smart_planning.stateChanged.connect(self._on_param_changed)
        layout.addWidget(self.chk_smart_planning, 1, 2, 1, 2)

        self.chk_force_vertical = QCheckBox("Force Vertical Plane")
        self.chk_force_vertical.setChecked(True)
        self.chk_force_vertical.setToolTip("Ensure flight path is on a true vertical plane regardless of camera position tilt")
        self.chk_force_vertical.stateChanged.connect(self._on_param_changed)
        layout.addWidget(self.chk_force_vertical, 2, 0, 1, 4)

        return group

    # -------------------------------------------------------------------------
    # Zone 3: Flight Settings
    # -------------------------------------------------------------------------
    def _create_zone3_flight(self) -> QGroupBox:
        group = QGroupBox("3. Flight Settings")
        layout = QGridLayout(group)

        layout.addWidget(QLabel("Speed (m/s):"), 0, 0)
        self.spin_speed = QDoubleSpinBox()
        self.spin_speed.setRange(0.1, 15.0)
        self.spin_speed.setValue(4.0)
        self.spin_speed.setDecimals(1)
        layout.addWidget(self.spin_speed, 0, 1)

        layout.addWidget(QLabel("Gimbal Pitch (°):"), 0, 2)
        self.spin_gimbal = QDoubleSpinBox()
        self.spin_gimbal.setRange(-90.0, 30.0)
        self.spin_gimbal.setValue(0.0)
        self.spin_gimbal.setDecimals(1)
        layout.addWidget(self.spin_gimbal, 0, 3)

        layout.addWidget(QLabel("Drone Type:"), 1, 0)
        self.combo_drone = QComboBox()
        self.combo_drone.addItems(["M3T"])
        layout.addWidget(self.combo_drone, 1, 1)

        layout.addWidget(QLabel("Height Offset (m):"), 1, 2)
        self.spin_height_offset = QDoubleSpinBox()
        self.spin_height_offset.setRange(-100.0, 100.0)
        self.spin_height_offset.setValue(0.0)
        self.spin_height_offset.setDecimals(1)
        self.spin_height_offset.setToolTip("Add constant offset to all waypoint altitudes (positive = higher)")
        self.spin_height_offset.valueChanged.connect(self._on_param_changed)
        layout.addWidget(self.spin_height_offset, 1, 3)

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
        self.txt_info.setPlaceholderText("Load 4 images to see GPS metadata...")
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
    def _on_images_dropped(self, paths: List[str]):
        """Handle images dropped or selected."""
        logger.info(f"Loading {len(paths[:4])} images")
        self.image_paths = paths[:4]
        self.drop_zone.update_thumbnails(self.image_paths)
        self._extract_gps()
        self._invalidate_generation()
        self._update_ui_state()

    def _select_images(self):
        """Open file dialog to select images."""
        files, _ = QFileDialog.getOpenFileNames(
            self, "Select Images", "",
            "Images (*.jpg *.jpeg *.JPG *.JPEG)"
        )
        if files:
            self._on_images_dropped(files)

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
                    f"Found only {len(images)} images. Need 4 for facade detection."
                )
            elif len(images) > 4:
                QMessageBox.information(
                    self, "Multiple Images",
                    f"Found {len(images)} images. Using first 4 (alphabetically sorted)."
                )

            self._on_images_dropped(images)

    def _clear_images(self):
        """Clear all loaded images."""
        logger.info("Clearing all loaded images")
        self.image_paths = []
        self.gps_data = []
        self.drop_zone.clear()
        self.txt_info.clear()
        self._invalidate_generation()
        self._update_ui_state()

    def _on_param_changed(self):
        """Handle parameter change - invalidate generation."""
        self._invalidate_generation()

    def _extract_gps(self):
        """Extract GPS from loaded images."""
        logger.debug("Extracting GPS data from images")
        self.gps_data = []
        info_lines = []

        for i, path in enumerate(self.image_paths):
            try:
                lat, lon, alt = core.read_gps(path)
                self.gps_data.append((lat, lon, alt))

                lat_dir = "N" if lat >= 0 else "S"
                lon_dir = "E" if lon >= 0 else "W"
                info_lines.append(
                    f"Photo {i+1}: {abs(lat):.6f}°{lat_dir}, {abs(lon):.6f}°{lon_dir}, {alt:.1f}m (WGS84)"
                )
            except Exception as e:
                logger.error(f"Failed to extract GPS from photo {i+1}: {e}")
                info_lines.append(f"Photo {i+1}: ERROR - {str(e)}")
                self.gps_data.append(None)

        # Calculate facade info if all 4 valid
        if len(self.gps_data) == 4 and all(g is not None for g in self.gps_data):
            try:
                tf = core.FacadeTransformer(self.gps_data)
                xs = [p[0] for p in tf.facade_pts]
                zs = [p[2] for p in tf.facade_pts]
                width = max(xs) - min(xs)
                height = max(zs) - min(zs)
                info_lines.append(f"\nFacade: {width:.1f}m × {height:.1f}m")
                logger.info(f"Facade detected: {width:.1f}m × {height:.1f}m")
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
            core.OVERLAP_RATE = self.spin_overlap.value()
            core.ENABLE_SMART_PLANNING = self.chk_smart_planning.isChecked()
            core.FORCE_VERTICAL_PLANE = self.chk_force_vertical.isChecked()
            core.AUTO_FLIGHT_SPEED = self.spin_speed.value()
            core.GIMBAL_PITCH_DEG = self.spin_gimbal.value()
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
        has_4_images = len(self.image_paths) == 4 and all(g is not None for g in self.gps_data)
        has_generation = len(self.generated_waypoints) > 0

        self.btn_generate.setEnabled(has_4_images)
        self.btn_save_kmz.setEnabled(has_generation)
        self.btn_save_kml.setEnabled(has_generation)
        self.btn_save_both.setEnabled(has_generation)

        if not has_4_images:
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
