#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mavic 3T Facade Waypoint Planner → KMZ (DJI WPML standard package)

What this script does
- Reads 4 facade-corner photos' EXIF GPS (Lat/Lon/Alt) taken ~5 m from wall
- Fits facade plane, builds local X'(width)/Y'(normal-out)/Z'(up) frame
- Plans snake path using HFOV/VFOV + overlap on the same aircraft sampling plane (no extra Y' offset)
- Exports BOTH:
  (A) A generic KML for Google Earth quick visual check
  (B) A **KMZ** archive that contains:
      - template.kml  (editor-facing; supports EGM96/relative height mode)
      - waylines.wpml (execution; supports WGS84/relative height mode)

Height reference modes
- EXECUTION (waylines.wpml): WGS84 ellipsoidal height OR relativeToStartPoint
- EDITOR (template.kml): EGM96 orthometric (MSL) OR relativeToStartPoint

Note: If your input altitudes are not in the target reference, you must convert.
For EGM96, plug your geoid model in `egm96_geoid_offset(lat, lon)`.
"""

import sys
import exifread
import os
from math import radians, cos, tan, sqrt, degrees, atan2, hypot
from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, ElementTree, tostring
from typing import Any, Dict, List, Optional, Tuple
import zipfile
import io
import numpy as np
from loguru import logger

# ========================= User Config =========================
# Photos (replace or pass via CLI)
PHOTO_PATHS = [
    '/Users/andyliu/Downloads/hkstp_test/11/DJI_0109.JPG',
    '/Users/andyliu/Downloads/hkstp_test/11/DJI_0110.JPG',
    '/Users/andyliu/Downloads/hkstp_test/11/DJI_0111.JPG',
    '/Users/andyliu/Downloads/hkstp_test/11/DJI_0112.JPG',
]

# Planning（与 GUI / 校验一致：统一距离 3–20 m）
PHOTO_DISTANCE_MIN = 3.0
PHOTO_DISTANCE_MAX = 20.0
AUTO_FLIGHT_SPEED_MIN = 0.1
AUTO_FLIGHT_SPEED_MAX = 10.0

PHOTO_DISTANCE = 5.0      # Distance from camera to facade when corner photos were taken (meters) - RTK prior knowledge
OVERLAP_RATE = 0.65       # 0–1，对应 0%–100% 重叠
ENABLE_SMART_PLANNING = True
FORCE_VERTICAL_PLANE = True  # Force flight plane to be vertical regardless of camera position tilt

# Camera FOV (deg)；M3E/M3T 广角典型 71.5°×56.8°（与 GUI 机型预设一致）
CAMERA_HFOV = 71.5
CAMERA_VFOV = 56.8

# DJI / WPML params
DRONE_TYPE = "M3T"
AUTO_FLIGHT_SPEED = 4.0

# Mavic 3 Enterprise series → FOV + WPML ids (verified for M3 line)
DRONE_WPML_PROFILES: Dict[str, Dict[str, Any]] = {
    "M3E": {
        "hfov": 71.5, "vfov": 56.8,
        "drone": "67", "drone_sub": "0", "payload": "52", "payload_sub": "0",
    },
    "M3T": {
        "hfov": 71.5, "vfov": 56.8,
        "drone": "67", "drone_sub": "1", "payload": "52", "payload_sub": "0",
    },
}

WPML_DRONE_ENUM = "67"
WPML_DRONE_SUB_ENUM = "1"
WPML_PAYLOAD_ENUM = "52"
WPML_PAYLOAD_SUB_ENUM = "0"


def apply_drone_wpml_profile(drone_key: str) -> None:
    """Set WPML enums and FOV/radians from presets (CLI / DRONE_TYPE)."""
    global WPML_DRONE_ENUM, WPML_DRONE_SUB_ENUM, WPML_PAYLOAD_ENUM, WPML_PAYLOAD_SUB_ENUM
    global CAMERA_HFOV, CAMERA_VFOV, HFOV_RAD, VFOV_RAD
    p = DRONE_WPML_PROFILES.get(drone_key, DRONE_WPML_PROFILES["M3T"])
    WPML_DRONE_ENUM = str(p["drone"])
    WPML_DRONE_SUB_ENUM = str(p["drone_sub"])
    WPML_PAYLOAD_ENUM = str(p["payload"])
    WPML_PAYLOAD_SUB_ENUM = str(p["payload_sub"])
    CAMERA_HFOV = float(p["hfov"])
    CAMERA_VFOV = float(p["vfov"])
    HFOV_RAD = radians(CAMERA_HFOV)
    VFOV_RAD = radians(CAMERA_VFOV)


GIMBAL_PITCH_DEG = 0.0     # level shot at each point
ENABLE_PHOTO_CAPTURE = True  # Take a photo at each waypoint pause
# Waypoint heading: fixed = keep yaw between legs (lateral snake without nose-along-path yaw)
WAYPOINT_HEADING_MODE = "fixed"
HEIGHT_OFFSET = 0.0        # Constant offset added to all waypoint altitudes (meters)

# Height reference modes
# waylines.wpml (execution): "WGS84" or "relativeToStartPoint"
EXECUTE_HEIGHT_MODE = "WGS84"
# template.kml (editor): "EGM96" or "relativeToStartPoint"
TEMPLATE_HEIGHT_MODE = "EGM96"

# Mission naming (avoid illegal chars: < > : " / \ | ? * . _)
MISSION_NAME = "Facade Mission"

# Geoid separation for height conversion
# Geoid separation (N) = WGS84 ellipsoidal height - EGM96 orthometric height
# 
# 香港地区参数：
# - HKPD (Hong Kong Principal Datum) 比全球 geoid 低约 72.8 cm
# - 香港地区的 WGS84/EGM96 geoid separation 约为 6.0-6.5 米
# - 转换公式: EGM96高度 = WGS84椭球高度 - geoid_offset

def egm96_geoid_offset(lat: float, lon: float) -> float:
    """
    返回指定位置的 geoid separation (N值)
    
    香港地区使用固定值 6.3 米
    
    参数:
        lat: 纬度（度）
        lon: 经度（度）
    
    返回:
        geoid separation（米），正值表示椭球面在大地水准面之上
    """
    # 香港地区的 geoid separation 固定值
    return 6.3

# ========================= Derived =============================
HFOV_RAD = radians(CAMERA_HFOV)
VFOV_RAD = radians(CAMERA_VFOV)
EARTH_R = 6378137.0

# 四角共面 + 立面四边形（允许轻微直角梯形，不再要求四角近 90°）
FACADE_PLANE_MAX_RESIDUAL_M = 2.5
FACADE_MIN_WIDTH_M = 2.0
FACADE_MIN_HEIGHT_M = 2.0

# X′–Z′ 投影上：左右边近竖直、顶边近水平、底边可斜但不可近竖直或退化
FACADE_LR_VERTICAL_MIN_DZ_OVER_DX = 2.0  # 左/右边：|Δz| ≥ 此值 × |Δx|（水平分量极小时另见 EPS）
FACADE_TOP_HORIZONTAL_MIN_DX_OVER_DZ = 2.0  # 顶边：|Δx| ≥ 此值 × |Δz|
FACADE_BOTTOM_MAX_DZ_OVER_DX = 5.0  # 底边：若 |Δz| > 此值 × |Δx| 视为近竖直，拒绝
FACADE_DIR_RATIO_EPS_M = 0.08  # 比值判定时对 |Δx|/|Δz| 的下限钳制，避免除零

# 顶整体高于底、面积与边长
FACADE_TOP_BOTTOM_MIN_Z_SEP_M = 0.35  # mean(z_top) − mean(z_bot) 下限（m）
FACADE_MIN_QUAD_AREA_M2 = 3.0  # 四边形有向面积绝对值下限（m²）
FACADE_MIN_EDGE_LEN_M = 0.35  # 任一边投影长度下限（m）

# 对边是否真正相交（蝴蝶结）：叉积相对尺度 + 参数 t,u 开区间容差
FACADE_SEG_CROSS_REL_EPS = 1e-10
FACADE_SEG_OPEN_TOL = 1e-9

# Raw input order diagnostic (soft warning only; semantic reassignment is still the source of truth)
FACADE_RAW_ORDER_SIDE_MARGIN_FRAC = 0.03       # x-side separation tolerance as fraction of facade width in X′
FACADE_RAW_ORDER_VERTICAL_TOL_FRAC = 0.02     # z-top/bottom tolerance as fraction of facade height in Z′

# Temporary debug: set to 1 (or export env var) to print raw-order diagnostics.
FACADE_RAW_ORDER_DIAG_DEBUG = os.environ.get("FACADE_RAW_ORDER_DIAG_DEBUG", "0") == "1"

# ===================== EXIF GPS helpers ========================

def _dms_to_deg(dms, ref):
    d = dms[0].num / dms[0].den
    m = dms[1].num / dms[1].den
    s = dms[2].num / dms[2].den
    sign = -1 if ref in ("S", "W") else 1
    return sign * (d + m/60 + s/3600)


def read_gps(path: str) -> Tuple[float, float, float]:
    logger.debug(f"Reading GPS from: {path}")
    with open(path, "rb") as f:
        tags = exifread.process_file(f, details=False)
    lat = _dms_to_deg(tags["GPS GPSLatitude"].values, tags["GPS GPSLatitudeRef"].printable)
    lon = _dms_to_deg(tags["GPS GPSLongitude"].values, tags["GPS GPSLongitudeRef"].printable)
    alt_tag = tags.get("GPS GPSAltitude")
    if alt_tag:
        try:
            alt = float(alt_tag.values[0].num) / float(alt_tag.values[0].den)
        except Exception:
            alt = float(str(alt_tag).split("/")[0])
        alt_ref = tags.get("GPS GPSAltitudeRef")
        try:
            ref_val = int(alt_ref.values[0]) if alt_ref is not None else 0
        except Exception:
            ref_val = 0
        if ref_val == 1:
            alt = -alt
    else:
        alt = 0.0
    logger.info(f"GPS extracted: lat={lat:.6f}, lon={lon:.6f}, alt={alt:.2f}m")
    return lat, lon, alt

# ================= ENU & Facade Frame (approx) =================

def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0):
    d_lat = radians(lat - lat0)
    d_lon = radians(lon - lon0)
    x = d_lon * cos(radians(lat0)) * EARTH_R
    y = d_lat * EARTH_R
    z = alt - alt0
    return x, y, z


def enu_to_geodetic(x, y, z, lat0, lon0, alt0):
    lat = lat0 + degrees(y / EARTH_R)
    lon = lon0 + degrees(x / (EARTH_R * cos(radians(lat0))))
    alt = alt0 + z
    return lat, lon, alt

class FacadeTransformer:
    """
    Expects four facade-corner camera positions in ENU order:
      index 0 = bottom-left, 1 = top-left, 2 = top-right, 3 = bottom-right
    as seen when **facing the wall** (camera outside, CCW on the wall).
    That order makes (p1-p0)×(p2-p1) point from the wall toward the camera;
    +Y' is aligned to **toward the building** (opposite), for RTK offset math.
    """

    def __init__(self, gps4: List[Tuple[float, float, float]]):
        logger.info("Initializing FacadeTransformer with 4 GPS points")
        if len(gps4) != 4:
            raise ValueError("Exactly 4 GPS points are required.")
        self.gps = gps4
        self.ref = gps4[0]
        logger.debug(f"Reference point: lat={self.ref[0]:.6f}, lon={self.ref[1]:.6f}, alt={self.ref[2]:.2f}m")
        self.enu = [geodetic_to_enu(lat, lon, alt, *self.ref) for (lat,lon,alt) in gps4]
        logger.debug(f"ENU coordinates: {[(f'{e[0]:.2f}', f'{e[1]:.2f}', f'{e[2]:.2f}') for e in self.enu]}")
        self.R = None  # rows: x', y', z' in ENU
        self.origin = None
        self.plane = None
        self._build()
        self.facade_pts = [self.enu_to_facade(p) for p in self.enu]
        logger.info("FacadeTransformer initialized successfully")

    @staticmethod
    def _cross(a,b):
        return [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]]

    def _fit_plane(self, P):
        cx = sum(p[0] for p in P)/4; cy = sum(p[1] for p in P)/4; cz = sum(p[2] for p in P)/4
        C = [[p[0]-cx, p[1]-cy, p[2]-cz] for p in P]
        (p1,p2,p3,p4) = C
        best_n, best_err = None, 1e100
        for a,b,c,_ in ((p1,p2,p3,p4),(p1,p2,p4,p3),(p1,p3,p4,p2),(p2,p3,p4,p1)):
            v1=[b[i]-a[i] for i in range(3)]; v2=[c[i]-a[i] for i in range(3)]
            n = self._cross(v1,v2)
            ln = sqrt(sum(x*x for x in n))
            if ln<1e-9: continue
            n=[x/ln for x in n]
            err = sum(abs(sum(pt[i]*n[i] for i in range(3)))**2 for pt in C)
            if err<best_err: best_err, best_n = err, n
        d = -(best_n[0]*cx + best_n[1]*cy + best_n[2]*cz)
        return [best_n[0], best_n[1], best_n[2], d]

    def _ensure_yprime_toward_building(
        self, xprime: List[float], yprime: List[float], zprime: List[float]
    ) -> Tuple[List[float], List[float], List[float]]:
        """
        Camera positions BL(0)→TL(1)→TR(2) viewed from outside the wall form a
        CCW loop, but the right-hand cross product (BL→TL)×(TL→TR) points
        *through* the camera plane away from the observer — i.e. toward the
        building.  We use this directly as the target direction for +Y'.
        """
        u = [self.enu[1][i] - self.enu[0][i] for i in range(3)]
        v = [self.enu[2][i] - self.enu[1][i] for i in range(3)]
        n_toward = self._cross(u, v)
        ln = sqrt(sum(x * x for x in n_toward))
        if ln < 1e-9:
            logger.warning(
                "Degenerate edge cross (BL→TL)×(TL→TR); cannot verify Y' sign from corner order"
            )
            return xprime, yprime, zprime
        n_toward = [x / ln for x in n_toward]

        if FORCE_VERTICAL_PLANE:
            y_toward_b = [n_toward[0], n_toward[1], 0.0]
            lh = sqrt(y_toward_b[0] ** 2 + y_toward_b[1] ** 2)
            if lh < 1e-9:
                logger.warning("Cross-product normal has no horizontal part; skip Y' alignment")
                return xprime, yprime, zprime
            y_toward_b = [y_toward_b[0] / lh, y_toward_b[1] / lh, 0.0]
        else:
            y_toward_b = [n_toward[0], n_toward[1], n_toward[2]]
            lb = sqrt(sum(x * x for x in y_toward_b))
            if lb < 1e-9:
                return xprime, yprime, zprime
            y_toward_b = [x / lb for x in y_toward_b]

        dotp = sum(yprime[i] * y_toward_b[i] for i in range(3))
        if dotp < 0:
            logger.info(
                "Adjusted X'/Y' so +Y' points toward building (CCW corner order from camera)"
            )
            xprime = [-x for x in xprime]
            yprime = [-y for y in yprime]
        return xprime, yprime, zprime

    def _build(self):
        logger.debug("Fitting plane to camera positions")
        a,b,c,d = self._fit_plane(self.enu)
        nlen = sqrt(a*a+b*b+c*c); yprime_raw=[a/nlen,b/nlen,c/nlen]
        logger.debug(f"Raw plane normal: [{yprime_raw[0]:.4f}, {yprime_raw[1]:.4f}, {yprime_raw[2]:.4f}]")
        avg = sum(a*p[0]+b*p[1]+c*p[2]+d for p in self.enu)/4.0
        if avg<0: yprime_raw=[-yprime_raw[0],-yprime_raw[1],-yprime_raw[2]]; a,b,c,d=-a,-b,-c,-d

        if FORCE_VERTICAL_PLANE:
            logger.info("Force vertical plane enabled - projecting normal to horizontal")
            horiz_len = sqrt(yprime_raw[0]**2 + yprime_raw[1]**2)
            if horiz_len < 1e-9:
                logger.error("Facade normal is nearly vertical, cannot determine horizontal direction")
                raise ValueError(
                    "Facade normal is nearly vertical in ENU; cannot determine horizontal approach direction."
                )
            yprime = [yprime_raw[0]/horiz_len, yprime_raw[1]/horiz_len, 0.0]
            zprime = [0.0, 0.0, 1.0]
            xprime = self._cross(yprime, zprime)
            logger.debug(f"Vertical plane - Y': [{yprime[0]:.4f}, {yprime[1]:.4f}, {yprime[2]:.4f}], Z': [0, 0, 1]")
        else:
            logger.info("Using original tilted plane fitting")
            yprime = yprime_raw
            sz=sorted(self.enu,key=lambda p:p[2]); low=sz[:2]; high=sz[2:]
            vlow=[low[1][i]-low[0][i] for i in range(3)]
            vhigh=[high[1][i]-high[0][i] for i in range(3)]
            wvec = vlow if sqrt(sum(x*x for x in vlow))>sqrt(sum(x*x for x in vhigh)) else vhigh
            wproj = sum(wvec[i]*yprime[i] for i in range(3))
            wplane = [wvec[i]-wproj*yprime[i] for i in range(3)]
            lwp = sqrt(sum(x*x for x in wplane))
            if lwp<1e-9:
                raise ValueError("Width vector is degenerate relative to facade normal; cannot define X' axis.")
            xprime=[x/lwp for x in wplane]
            zprime=self._cross(xprime,yprime)
            if zprime[2]<0: zprime=[-z for z in zprime]; xprime=[-x for x in xprime]
            logger.debug(f"Tilted plane - Z': [{zprime[0]:.4f}, {zprime[1]:.4f}, {zprime[2]:.4f}]")

        xprime, yprime, zprime = self._ensure_yprime_toward_building(xprime, yprime, zprime)
        self.R=[xprime,yprime,zprime]
        self.origin=[sum(p[i] for p in self.enu)/4.0 for i in range(3)]
        self.plane=[a,b,c,d]
        logger.debug(f"Facade origin: [{self.origin[0]:.2f}, {self.origin[1]:.2f}, {self.origin[2]:.2f}]")

    def enu_to_facade(self, enu):
        t=[enu[i]-self.origin[i] for i in range(3)]
        return tuple(sum(t[j]*self.R[i][j] for j in range(3)) for i in range(3))

    def facade_to_enu(self, fp):
        enu_t=[sum(fp[j]*self.R[j][i] for j in range(3)) for i in range(3)]
        return tuple(enu_t[i]+self.origin[i] for i in range(3))

    def facade_to_gps(self, fp):
        x,y,z=self.facade_to_enu(fp)
        return enu_to_geodetic(x,y,z,*self.ref)


def validate_planning_params() -> None:
    """Enforce the same physical bounds as the GUI; raises ValueError with English messages."""
    if not (PHOTO_DISTANCE_MIN <= PHOTO_DISTANCE <= PHOTO_DISTANCE_MAX):
        raise ValueError(
            f"Photo distance must be between {PHOTO_DISTANCE_MIN:.0f} and {PHOTO_DISTANCE_MAX:.0f} m (got {PHOTO_DISTANCE} m)."
        )
    if not (AUTO_FLIGHT_SPEED_MIN <= AUTO_FLIGHT_SPEED <= AUTO_FLIGHT_SPEED_MAX):
        raise ValueError(
            f"Cruise speed must be between {AUTO_FLIGHT_SPEED_MIN} and {AUTO_FLIGHT_SPEED_MAX} m/s (got {AUTO_FLIGHT_SPEED} m/s)."
        )
    if not (0.0 <= OVERLAP_RATE <= 1.0):
        raise ValueError("Overlap rate must be between 0% and 100% (inclusive).")


def _cross2(ax: float, az: float, bx: float, bz: float) -> float:
    return ax * bz - az * bx


def _seg_proper_intersect_2d(
    ax: float,
    az: float,
    bx: float,
    bz: float,
    cx: float,
    cz: float,
    dx: float,
    dz: float,
) -> bool:
    """True if open segments (a→b) and (c→d) intersect in the interior (not only at endpoints)."""
    rx, rz = bx - ax, bz - az
    sx, sz = dx - cx, dz - cz
    qx, qz = cx - ax, cz - az
    rxs = _cross2(rx, rz, sx, sz)
    scale = hypot(rx, rz) * hypot(sx, sz) + 1e-12
    if abs(rxs) <= FACADE_SEG_CROSS_REL_EPS * scale:
        return False
    t = _cross2(qx, qz, sx, sz) / rxs
    u = _cross2(qx, qz, rx, rz) / rxs
    tol = FACADE_SEG_OPEN_TOL
    return tol < t < 1.0 - tol and tol < u < 1.0 - tol


def _assign_facade_semantic_corners_xz(
    pts_xz: List[Tuple[float, float]],
) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
    """
    Map four (x,z) samples to semantic corners without polar sorting:
    split left/right by X′ (two smaller X → left wall edge), then top/bottom by Z′ on each side.
    Returns (left_bottom, left_top, right_top, right_bottom).
    """
    if len(pts_xz) != 4:
        raise ValueError("Exactly 4 facade (x,z) points are required.")
    order_by_x = sorted(range(4), key=lambda i: (pts_xz[i][0], pts_xz[i][1], i))
    left_i = order_by_x[:2]
    right_i = order_by_x[2:]
    left_i = sorted(left_i, key=lambda i: pts_xz[i][1])
    right_i = sorted(right_i, key=lambda i: pts_xz[i][1])
    lb = pts_xz[left_i[0]]
    lt = pts_xz[left_i[1]]
    rb = pts_xz[right_i[0]]
    rt = pts_xz[right_i[1]]
    return lb, lt, rt, rb


def _quad_polygon_area_xz(corners: List[Tuple[float, float]]) -> float:
    """Shoelace area for polygon given in order (CCW or CW); returns absolute area."""
    n = len(corners)
    s = 0.0
    for i in range(n):
        x0, z0 = corners[i]
        x1, z1 = corners[(i + 1) % n]
        s += x0 * z1 - x1 * z0
    return abs(s) * 0.5


def _diagnose_raw_corner_order_consistency_xz(
    pts_xz: List[Tuple[float, float]],
) -> Tuple[bool, Optional[str]]:
    """
    Soft diagnostic only (no hard failure).

    Raw input is expected as: BL -> TL -> TR -> BR
    in tf.facade_pts order (indices 0..3). We check whether the raw order
    is consistent in X′–Z′ space; otherwise return a warning message.
    """
    if len(pts_xz) != 4:
        return True, None

    # Raw mapping (by tf.facade_pts indices)
    lb = pts_xz[0]
    lt = pts_xz[1]
    rt = pts_xz[2]
    rb = pts_xz[3]

    if FACADE_RAW_ORDER_DIAG_DEBUG:
        print("\n[RAW_ORDER_DIAG] incoming raw pts_xz by index:")
        for idx, (xv, zv) in enumerate(pts_xz):
            print(f"  idx {idx}: X'={xv:.4f}, Z'={zv:.4f}")
        print("[RAW_ORDER_DIAG] interpreted as:")
        print(f"  BL (idx0): X'={lb[0]:.4f}, Z'={lb[1]:.4f}")
        print(f"  TL (idx1): X'={lt[0]:.4f}, Z'={lt[1]:.4f}")
        print(f"  TR (idx2): X'={rt[0]:.4f}, Z'={rt[1]:.4f}")
        print(f"  BR (idx3): X'={rb[0]:.4f}, Z'={rb[1]:.4f}")

    xs = [p[0] for p in pts_xz]
    zs = [p[1] for p in pts_xz]
    w = max(xs) - min(xs)
    h = max(zs) - min(zs)
    w = w if w > 1e-12 else 1.0
    h = h if h > 1e-12 else 1.0

    side_margin = FACADE_RAW_ORDER_SIDE_MARGIN_FRAC * w
    vert_tol = FACADE_RAW_ORDER_VERTICAL_TOL_FRAC * h

    avg_x_left = 0.5 * (lb[0] + lt[0])
    avg_x_right = 0.5 * (rt[0] + rb[0])
    avg_z_top = 0.5 * (lt[1] + rt[1])
    avg_z_bot = 0.5 * (lb[1] + rb[1])

    # 1. BL & TL should be on the left side overall
    cond1 = max(lb[0], lt[0]) <= min(rt[0], rb[0]) + side_margin
    # 2. TR & BR should be on the right side overall
    cond2 = min(rt[0], rb[0]) >= max(lb[0], lt[0]) - side_margin
    # 3. TL should be above BL in Z′
    cond3 = lt[1] >= lb[1] - vert_tol
    # 4. TR should be above BR in Z′
    cond4 = rt[1] >= rb[1] - vert_tol
    # 5. average(top corners) above average(bottom corners)
    cond5 = avg_z_top >= avg_z_bot - vert_tol
    # 6. average(left corners) should have smaller X′ than average(right corners)
    cond6 = avg_x_left < avg_x_right + side_margin

    conds = [cond1, cond2, cond3, cond4, cond5, cond6]
    if all(conds):
        return True, None

    if FACADE_RAW_ORDER_DIAG_DEBUG:
        print("[RAW_ORDER_DIAG] computed scale/thresholds:")
        print(f"  w(X')={w:.4f}, h(Z')={h:.4f}")
        print(f"  side_margin={side_margin:.4f}, vert_tol={vert_tol:.4f}")
        print("[RAW_ORDER_DIAG] condition results:")
        print(f"  cond1 (left side): {cond1} ; max(leftX)={max(lb[0], lt[0]):.4f}, min(rightX)={min(rt[0], rb[0]):.4f}")
        print(f"  cond2 (right side): {cond2} ; min(rightX)={min(rt[0], rb[0]):.4f}, max(leftX)={max(lb[0], lt[0]):.4f}")
        print(f"  cond3 (TL above BL): {cond3} ; TL_Z'={lt[1]:.4f}, BL_Z'={lb[1]:.4f}")
        print(f"  cond4 (TR above BR): {cond4} ; TR_Z'={rt[1]:.4f}, BR_Z'={rb[1]:.4f}")
        print(f"  cond5 (avg top above avg bottom): {cond5} ; avg_top_Z'={avg_z_top:.4f}, avg_bot_Z'={avg_z_bot:.4f}")
        print(f"  cond6 (avg left X' < avg right X'): {cond6} ; avg_left_X'={avg_x_left:.4f}, avg_right_X'={avg_x_right:.4f}")

    violated = [str(i + 1) for i, ok in enumerate(conds) if not ok]

    likely_causes: List[str] = []
    if not cond6 and avg_x_left > avg_x_right + side_margin * 0.5:
        likely_causes.append("possible left-right swap")
    if not cond3:
        likely_causes.append("possible BL/TL swap")
    if not cond4:
        likely_causes.append("possible TR/BR swap")

    if not likely_causes:
        likely_causes.append("raw order inconsistent with expected BL->TL->TR->BR")

    msg = (
        "Raw corner order BL->TL->TR->BR looks inconsistent in X′–Z′ projection "
        f"(violated: {', '.join(violated)}). Likely: {', '.join(likely_causes)}. "
        "Planner will continue using geometry-based semantic reassignment (no hard fail)."
    )
    return False, msg


def validate_facade_corner_geometry(tf: FacadeTransformer) -> None:
    """
    Require four corners to be nearly coplanar and form a simple vertical-facade quadrilateral
    in the X′–Z′ plane: left/right edges ~vertical, top ~horizontal, bottom may be slanted.
    Raises ValueError if they are unlikely to be valid facade corners.
    """
    a, b, c, d = tf.plane
    denom = sqrt(a * a + b * b + c * c)
    if denom < 1e-12:
        raise ValueError("Could not fit a facade plane from GPS; check inputs.")
    for i, enu in enumerate(tf.enu):
        dist = abs(a * enu[0] + b * enu[1] + c * enu[2] + d) / denom
        if dist > FACADE_PLANE_MAX_RESIDUAL_M:
            raise ValueError(
                f"A corner lies about {dist:.2f} m off the fitted facade plane "
                f"(limit {FACADE_PLANE_MAX_RESIDUAL_M} m). Points may not lie on one facade or order is wrong."
            )

    pts = [(p[0], p[2]) for p in tf.facade_pts]
    xs = [x for x, _ in pts]
    zs = [z for _, z in pts]
    w = max(xs) - min(xs)
    h = max(zs) - min(zs)
    if w < FACADE_MIN_WIDTH_M or h < FACADE_MIN_HEIGHT_M:
        raise ValueError(
            f"Facade extent is only about {w:.2f} m × {h:.2f} m "
            f"(minimum {FACADE_MIN_WIDTH_M} m × {FACADE_MIN_HEIGHT_M} m). Check corner positions or units."
        )

    raw_ok, raw_warn = _diagnose_raw_corner_order_consistency_xz(pts)
    # Expose for potential debugging; does not change validation behavior.
    tf.raw_order_consistent = raw_ok  # type: ignore[attr-defined]
    tf.raw_order_warning = raw_warn  # type: ignore[attr-defined]
    if raw_warn:
        logger.warning(raw_warn)
        # Also print so users can observe soft warnings even if log level/filter hides them.
        print(f"[RAW_ORDER_WARNING] {raw_warn}")

    lb, lt, rt, rb = _assign_facade_semantic_corners_xz(pts)

    # Non-self-intersecting: opposite edges of (lb → lt → rt → rb) must not cross in the interior
    if _seg_proper_intersect_2d(lb[0], lb[1], lt[0], lt[1], rt[0], rt[1], rb[0], rb[1]):
        raise ValueError(
            "Facade corners form a self-intersecting quadrilateral in the X′–Z′ plane (bow-tie); "
            "check GPS or corner assignment."
        )
    if _seg_proper_intersect_2d(lt[0], lt[1], rt[0], rt[1], rb[0], rb[1], lb[0], lb[1]):
        raise ValueError(
            "Facade corners form a self-intersecting quadrilateral in the X′–Z′ plane (bow-tie); "
            "check GPS or corner assignment."
        )

    # Edge vectors (semantic naming)
    v_left = (lt[0] - lb[0], lt[1] - lb[1])
    v_right = (rt[0] - rb[0], rt[1] - rb[1])
    v_top = (rt[0] - lt[0], rt[1] - lt[1])
    v_bot = (rb[0] - lb[0], rb[1] - lb[1])

    def _len_xz(v: Tuple[float, float]) -> float:
        return hypot(v[0], v[1])

    lens = [_len_xz(v_left), _len_xz(v_right), _len_xz(v_top), _len_xz(v_bot)]
    min_len = min(lens)
    if min_len < FACADE_MIN_EDGE_LEN_M:
        raise ValueError(
            f"Shortest facade edge in X′–Z′ projection is about {min_len:.2f} m "
            f"(minimum {FACADE_MIN_EDGE_LEN_M} m); corners may be degenerate."
        )

    def _ratio_vertical_ok(dx: float, dz: float) -> bool:
        """Left/right: |dz| should dominate |dx|."""
        adx, adz = abs(dx), abs(dz)
        return adz >= FACADE_LR_VERTICAL_MIN_DZ_OVER_DX * max(adx, FACADE_DIR_RATIO_EPS_M)

    def _ratio_horizontal_ok(dx: float, dz: float) -> bool:
        """Top: |dx| should dominate |dz|."""
        adx, adz = abs(dx), abs(dz)
        return adx >= FACADE_TOP_HORIZONTAL_MIN_DX_OVER_DZ * max(adz, FACADE_DIR_RATIO_EPS_M)

    def _bottom_not_near_vertical(dx: float, dz: float) -> bool:
        """Bottom may slope; reject if nearly vertical (|dz| >> |dx|) or near-point."""
        adx, adz = abs(dx), abs(dz)
        if adx < FACADE_DIR_RATIO_EPS_M and adz < FACADE_DIR_RATIO_EPS_M:
            return False
        if adx < FACADE_DIR_RATIO_EPS_M:
            return False
        return adz <= FACADE_BOTTOM_MAX_DZ_OVER_DX * adx

    if not _ratio_vertical_ok(*v_left):
        raise ValueError(
            "Left facade edge is not sufficiently vertical in the X′–Z′ plane "
            f"(|Δz| should be ≥ {FACADE_LR_VERTICAL_MIN_DZ_OVER_DX:g}× effective |Δx|). "
            "Check corner positions or GPS noise."
        )
    if not _ratio_vertical_ok(*v_right):
        raise ValueError(
            "Right facade edge is not sufficiently vertical in the X′–Z′ plane "
            f"(|Δz| should be ≥ {FACADE_LR_VERTICAL_MIN_DZ_OVER_DX:g}× effective |Δx|). "
            "Check corner positions or GPS noise."
        )
    if not _ratio_horizontal_ok(*v_top):
        raise ValueError(
            "Top facade edge is not sufficiently horizontal in the X′–Z′ plane "
            f"(|Δx| should be ≥ {FACADE_TOP_HORIZONTAL_MIN_DX_OVER_DZ:g}× effective |Δz|). "
            "Check corner positions or GPS noise."
        )
    if not _bottom_not_near_vertical(*v_bot):
        raise ValueError(
            "Bottom facade edge is too steep (nearly vertical) or degenerate in the X′–Z′ plane; "
            "a slanted bottom is allowed but it must retain a clear horizontal run."
        )

    z_top_mean = (lt[1] + rt[1]) * 0.5
    z_bot_mean = (lb[1] + rb[1]) * 0.5
    if z_top_mean < z_bot_mean + FACADE_TOP_BOTTOM_MIN_Z_SEP_M:
        raise ValueError(
            f"Top of facade is not clearly above bottom in Z′ "
            f"(mean top − mean bottom = {z_top_mean - z_bot_mean:.2f} m, "
            f"need ≥ {FACADE_TOP_BOTTOM_MIN_Z_SEP_M} m)."
        )

    area = _quad_polygon_area_xz([lb, lt, rt, rb])
    if area < FACADE_MIN_QUAD_AREA_M2:
        raise ValueError(
            f"Facade quadrilateral area in X′–Z′ is only about {area:.2f} m² "
            f"(minimum {FACADE_MIN_QUAD_AREA_M2} m²)."
        )

# ===================== Planning helpers ========================

def fov_step(distance_m: float, fov_rad: float, overlap: float) -> float:
    cov = 2*distance_m*tan(fov_rad/2.0)
    return max(cov*(1.0-overlap), 0.01)


def plan_steps(direction: str, distance: float, overlap: float):
    if direction=="vertical":
        return fov_step(distance, HFOV_RAD, overlap), fov_step(distance, VFOV_RAD, overlap)
    else:
        return fov_step(distance, VFOV_RAD, overlap), fov_step(distance, HFOV_RAD, overlap)

# =================== Build mission & outputs ===================

def build_waypoints_from_images(images: List[str]):
    validate_planning_params()
    logger.info(f"Building waypoints from {len(images)} images")
    gps4=[read_gps(p) for p in images]
    tf=FacadeTransformer(gps4)
    validate_facade_corner_geometry(tf)

    xs=[p[0] for p in tf.facade_pts]; zs=[p[2] for p in tf.facade_pts]
    w=max(xs)-min(xs); h=max(zs)-min(zs)
    logger.info(f"Facade dimensions: width={w:.2f}m, height={h:.2f}m")

    if ENABLE_SMART_PLANNING:
        direction = "vertical" if h>w else "horizontal"
        logger.debug(f"Smart planning enabled, selected direction: {direction}")
    else:
        direction = "horizontal"
        logger.debug("Smart planning disabled, using horizontal direction")

    step_cross, step_along = plan_steps(direction, PHOTO_DISTANCE, OVERLAP_RATE)
    cross_span = w if direction=="vertical" else h
    num_lines = int(max(1, round(cross_span/step_cross))) + 1
    logger.debug(f"Step cross={step_cross:.2f}m, step along={step_along:.2f}m, num_lines={num_lines}")

    minx,maxx=min(xs),max(xs); minz,maxz=min(zs),max(zs)
    yps=[p[1] for p in tf.facade_pts]
    avg_y = sum(yps)/len(yps)
    # Origin = centroid of the four cameras → sum(facade_pts)==0 ⇒ avg_y ≈ 0.
    # +Y' = toward building (enforced from BL→TL→TR CCW order in FacadeTransformer).
    # Flight reuses the same stand-off as corner photos, so mission flies on camera sampling plane.
    safe_y = avg_y
    logger.info(f"RTK offset: camera Y'={avg_y:.2f}m, photo_dist={PHOTO_DISTANCE}m (shared for planning/flight) → flight Y'={safe_y:.2f}m")

    wps=[]
    if direction=="vertical":
        num_along = int(max(1, round((maxz-minz)/step_along))) + 1
        for i in range(num_lines):
            xr = i/(num_lines-1) if num_lines>1 else 0.0
            X = minx + xr*(maxx-minx)
            for j in range(num_along):
                zr = (j if i%2==0 else (num_along-1-j))/(num_along-1) if num_along>1 else 0.0
                Z = minz + zr*(maxz-minz)
                wps.append(tf.facade_to_gps((X, safe_y, Z)))
    else:
        num_along = int(max(1, round((maxx-minx)/step_along))) + 1
        for i in range(num_lines):
            zr = i/(num_lines-1) if num_lines>1 else 0.0
            Z = minz + zr*(maxz-minz)
            for j in range(num_along):
                xr = (j if i%2==0 else (num_along-1-j))/(num_along-1) if num_along>1 else 0.0
                X = minx + xr*(maxx-minx)
                wps.append(tf.facade_to_gps((X, safe_y, Z)))

    # Apply height offset if specified
    if HEIGHT_OFFSET != 0.0:
        logger.info(f"Applying height offset: {HEIGHT_OFFSET:+.1f}m to all waypoints")
        wps = [(lat, lon, alt + HEIGHT_OFFSET) for lat, lon, alt in wps]

    logger.info(f"Generated {len(wps)} waypoints with {direction} snake pattern")
    return tf, direction, wps

# ---------- Generic KML (for Google Earth) ----------

def build_generic_kml(wps: List[Tuple[float,float,float]]):
    root = Element("kml"); root.set("xmlns","http://www.opengis.net/kml/2.2")
    doc = SubElement(root, "Document")
    SubElement(doc, "name").text = "Facade Mission (Preview)"
    SubElement(doc, "open").text = "1"
    folder = SubElement(doc, "Folder")
    SubElement(folder, "name").text = "waypoints"
    coords_for_line=[]
    for i,(lat,lon,alt) in enumerate(wps,1):
        pm = SubElement(folder, "Placemark")
        SubElement(pm, "name").text = f"WP{i:03d}"
        pt = SubElement(pm, "Point")
        SubElement(pt, "altitudeMode").text = "absolute"
        SubElement(pt, "coordinates").text = f"{lon},{lat},{alt}"
        coords_for_line.append((lon,lat,alt))
    wayline = SubElement(doc, "Placemark")
    SubElement(wayline, "name").text = "Snake"
    ls = SubElement(wayline, "LineString")
    SubElement(ls, "tessellate").text = "1"
    SubElement(ls, "altitudeMode").text = "absolute"
    SubElement(ls, "coordinates").text = " ".join(f"{lo},{la},{al}" for lo,la,al in coords_for_line)
    return root

# ---------- Build waylines.wpml (execution) ----------

def build_waylines_wpml(tf: FacadeTransformer, wps: List[Tuple[float,float,float]]):
    import time
    root = Element("kml")
    root.set("xmlns","http://www.opengis.net/kml/2.2")
    root.set("xmlns:wpml","http://www.dji.com/wpmz/1.0.6")
    doc = SubElement(root, "Document")

    # Mission Config
    mcfg = SubElement(doc, "wpml:missionConfig")
    SubElement(mcfg, "wpml:flyToWaylineMode").text = "safely"
    SubElement(mcfg, "wpml:finishAction").text = "goHome"
    SubElement(mcfg, "wpml:exitOnRCLost").text = "goContinue"
    SubElement(mcfg, "wpml:executeRCLostAction").text = "goBack"
    SubElement(mcfg, "wpml:takeOffSecurityHeight").text = "100"
    SubElement(mcfg, "wpml:globalTransitionalSpeed").text = "15"
    SubElement(mcfg, "wpml:globalRTHHeight").text = "100"
    
    # Drone Info (use enum values)
    dinfo = SubElement(mcfg, "wpml:droneInfo")
    SubElement(dinfo, "wpml:droneEnumValue").text = WPML_DRONE_ENUM
    SubElement(dinfo, "wpml:droneSubEnumValue").text = WPML_DRONE_SUB_ENUM
    
    SubElement(mcfg, "wpml:waylineAvoidLimitAreaMode").text = "0"
    
    # Payload Info
    pinfo = SubElement(mcfg, "wpml:payloadInfo")
    SubElement(pinfo, "wpml:payloadEnumValue").text = WPML_PAYLOAD_ENUM
    SubElement(pinfo, "wpml:payloadSubEnumValue").text = WPML_PAYLOAD_SUB_ENUM
    SubElement(pinfo, "wpml:payloadPositionIndex").text = "0"

    # Folder with waypoints
    folder = SubElement(doc, "Folder")
    SubElement(folder, "wpml:templateId").text = "0"
    SubElement(folder, "wpml:executeHeightMode").text = EXECUTE_HEIGHT_MODE
    SubElement(folder, "wpml:waylineId").text = "0"
    SubElement(folder, "wpml:distance").text = "0"
    SubElement(folder, "wpml:duration").text = "0"
    SubElement(folder, "wpml:autoFlightSpeed").text = str(AUTO_FLIGHT_SPEED)

    # Heading facing wall: Y' points toward facade, so drone faces in Y' direction
    R = tf.R; yprime_enu = np.array(R[1], dtype=float)
    to_wall = yprime_enu
    nrm = np.linalg.norm(to_wall[:2]) + 1e-12
    to_wall[:2] /= nrm
    def yaw_from_xy(vxy):
        east, north = float(vxy[0]), float(vxy[1])
        bearing = degrees(atan2(east, north))
        return (bearing + 360.0) % 360.0
    head_deg = yaw_from_xy(to_wall)

    lat0,lon0,alt0 = tf.ref

    for i,(lat,lon,alt_wgs84) in enumerate(wps):
        pm = SubElement(folder, "Placemark")
        
        # Point coordinates
        pt = SubElement(pm, "Point")
        coords = SubElement(pt, "coordinates")
        coords.text = f"\n            {lon},{lat}\n          "
        
        # Index
        SubElement(pm, "wpml:index").text = str(i)
        
        # Execute height
        if EXECUTE_HEIGHT_MODE == "WGS84":
            exec_h = alt_wgs84
        elif EXECUTE_HEIGHT_MODE == "relativeToStartPoint":
            exec_h = alt_wgs84 - alt0
        else:
            exec_h = alt_wgs84
        SubElement(pm, "wpml:executeHeight").text = f"{exec_h:.12f}"
        
        # Waypoint speed
        SubElement(pm, "wpml:waypointSpeed").text = str(AUTO_FLIGHT_SPEED)
        
        # Waypoint heading param — fixed at facade-facing yaw for sideways flight
        hparam = SubElement(pm, "wpml:waypointHeadingParam")
        SubElement(hparam, "wpml:waypointHeadingMode").text = WAYPOINT_HEADING_MODE
        SubElement(hparam, "wpml:waypointHeadingAngle").text = f"{head_deg:.1f}"
        SubElement(hparam, "wpml:waypointPoiPoint").text = "0.000000,0.000000,0.000000"
        SubElement(hparam, "wpml:waypointHeadingAngleEnable").text = "1"
        SubElement(hparam, "wpml:waypointHeadingPathMode").text = "followBadArc"
        SubElement(hparam, "wpml:waypointHeadingPoiIndex").text = "0"
        
        # Waypoint turn param
        tparam = SubElement(pm, "wpml:waypointTurnParam")
        SubElement(tparam, "wpml:waypointTurnMode").text = "toPointAndStopWithDiscontinuityCurvature"
        SubElement(tparam, "wpml:waypointTurnDampingDist").text = "0"
        
        SubElement(pm, "wpml:useStraightLine").text = "1"
        
        # Action group
        agroup = SubElement(pm, "wpml:actionGroup")
        SubElement(agroup, "wpml:actionGroupId").text = str(i)
        SubElement(agroup, "wpml:actionGroupStartIndex").text = str(i)
        SubElement(agroup, "wpml:actionGroupEndIndex").text = str(i)
        SubElement(agroup, "wpml:actionGroupMode").text = "sequence"
        
        atrigger = SubElement(agroup, "wpml:actionTrigger")
        SubElement(atrigger, "wpml:actionTriggerType").text = "reachPoint"
        
        # Rotate Yaw action
        act1 = SubElement(agroup, "wpml:action")
        SubElement(act1, "wpml:actionId").text = "0"
        SubElement(act1, "wpml:actionActuatorFunc").text = "rotateYaw"
        aparam1 = SubElement(act1, "wpml:actionActuatorFuncParam")
        SubElement(aparam1, "wpml:aircraftHeading").text = f"{head_deg:.1f}"
        SubElement(aparam1, "wpml:aircraftPathMode").text = "counterClockwise"
        
        # Gimbal Rotate action
        act2 = SubElement(agroup, "wpml:action")
        SubElement(act2, "wpml:actionId").text = "1"
        SubElement(act2, "wpml:actionActuatorFunc").text = "gimbalRotate"
        aparam2 = SubElement(act2, "wpml:actionActuatorFuncParam")
        SubElement(aparam2, "wpml:gimbalHeadingYawBase").text = "north"
        SubElement(aparam2, "wpml:gimbalRotateMode").text = "absoluteAngle"
        SubElement(aparam2, "wpml:gimbalPitchRotateEnable").text = "1"
        SubElement(aparam2, "wpml:gimbalPitchRotateAngle").text = f"{GIMBAL_PITCH_DEG:.0f}"
        SubElement(aparam2, "wpml:gimbalRollRotateEnable").text = "0"
        SubElement(aparam2, "wpml:gimbalRollRotateAngle").text = "0"
        SubElement(aparam2, "wpml:gimbalYawRotateEnable").text = "0"
        SubElement(aparam2, "wpml:gimbalYawRotateAngle").text = "0"
        SubElement(aparam2, "wpml:gimbalRotateTimeEnable").text = "0"
        SubElement(aparam2, "wpml:gimbalRotateTime").text = "0"
        SubElement(aparam2, "wpml:payloadPositionIndex").text = "0"

        if ENABLE_PHOTO_CAPTURE:
            act3 = SubElement(agroup, "wpml:action")
            SubElement(act3, "wpml:actionId").text = "2"
            SubElement(act3, "wpml:actionActuatorFunc").text = "takePhoto"
            aparam3 = SubElement(act3, "wpml:actionActuatorFuncParam")
            SubElement(aparam3, "wpml:payloadPositionIndex").text = "0"
            SubElement(aparam3, "wpml:fileSuffix").text = "point"
            SubElement(aparam3, "wpml:payloadLensIndex").text = "wide"
            SubElement(aparam3, "wpml:useGlobalPayloadLensIndex").text = "0"

        # Gimbal heading param
        ghparam = SubElement(pm, "wpml:waypointGimbalHeadingParam")
        SubElement(ghparam, "wpml:waypointGimbalPitchAngle").text = "0"
        SubElement(ghparam, "wpml:waypointGimbalYawAngle").text = "0"
        
        SubElement(pm, "wpml:isRisky").text = "0"
        SubElement(pm, "wpml:waypointWorkType").text = "0"

    return root

# ---------- Build template.kml (editor) ----------

def build_template_kml(tf: FacadeTransformer, wps: List[Tuple[float,float,float]]):
    import time
    root = Element("kml")
    root.set("xmlns","http://www.opengis.net/kml/2.2")
    root.set("xmlns:wpml","http://www.dji.com/wpmz/1.0.6")
    doc = SubElement(root, "Document")

    # Author and timestamps
    SubElement(doc, "wpml:author").text = ""
    SubElement(doc, "wpml:createTime").text = str(int(time.time() * 1000))
    SubElement(doc, "wpml:updateTime").text = str(int(time.time() * 1000))

    # Mission Config
    mcfg = SubElement(doc, "wpml:missionConfig")
    SubElement(mcfg, "wpml:flyToWaylineMode").text = "safely"
    SubElement(mcfg, "wpml:finishAction").text = "goHome"
    SubElement(mcfg, "wpml:exitOnRCLost").text = "goContinue"
    SubElement(mcfg, "wpml:executeRCLostAction").text = "goBack"
    SubElement(mcfg, "wpml:takeOffSecurityHeight").text = "100"
    
    # Add takeOffRefPoint (first waypoint)
    lat0, lon0, alt0 = wps[0]
    SubElement(mcfg, "wpml:takeOffRefPoint").text = f"{lat0},{lon0},{alt0}"
    SubElement(mcfg, "wpml:takeOffRefPointAGLHeight").text = "5.0"
    
    SubElement(mcfg, "wpml:globalTransitionalSpeed").text = "15"
    SubElement(mcfg, "wpml:globalRTHHeight").text = "100"
    
    # Drone Info (use enum values)
    dinfo = SubElement(mcfg, "wpml:droneInfo")
    SubElement(dinfo, "wpml:droneEnumValue").text = WPML_DRONE_ENUM
    SubElement(dinfo, "wpml:droneSubEnumValue").text = WPML_DRONE_SUB_ENUM
    
    SubElement(mcfg, "wpml:waylineAvoidLimitAreaMode").text = "0"
    
    # Payload Info
    pinfo = SubElement(mcfg, "wpml:payloadInfo")
    SubElement(pinfo, "wpml:payloadEnumValue").text = WPML_PAYLOAD_ENUM
    SubElement(pinfo, "wpml:payloadSubEnumValue").text = WPML_PAYLOAD_SUB_ENUM
    SubElement(pinfo, "wpml:payloadPositionIndex").text = "0"

    # Folder
    folder = SubElement(doc, "Folder")
    SubElement(folder, "wpml:templateType").text = "waypoint"
    SubElement(folder, "wpml:templateId").text = "0"
    
    # Coordinate system params
    coordsys = SubElement(folder, "wpml:waylineCoordinateSysParam")
    SubElement(coordsys, "wpml:coordinateMode").text = "WGS84"
    SubElement(coordsys, "wpml:heightMode").text = TEMPLATE_HEIGHT_MODE
    
    SubElement(folder, "wpml:autoFlightSpeed").text = str(AUTO_FLIGHT_SPEED)
    
    # Calculate global height (average)
    avg_height = sum(alt for _, _, alt in wps) / len(wps)
    SubElement(folder, "wpml:globalHeight").text = f"{avg_height:.9f}"
    
    SubElement(folder, "wpml:caliFlightEnable").text = "0"
    SubElement(folder, "wpml:gimbalPitchMode").text = "manual"

    # Heading facing wall: Y' points toward facade, so drone faces in Y' direction
    R = tf.R; yprime_enu = np.array(R[1], dtype=float)
    to_wall = yprime_enu
    nrm = np.linalg.norm(to_wall[:2]) + 1e-12
    to_wall[:2] /= nrm
    def yaw_from_xy(vxy):
        east, north = float(vxy[0]), float(vxy[1])
        bearing = degrees(atan2(east, north))
        return (bearing + 360.0) % 360.0
    head_deg = yaw_from_xy(to_wall)
    
    # Global waypoint heading param — fixed at facade-facing yaw for sideways flight
    ghparam = SubElement(folder, "wpml:globalWaypointHeadingParam")
    SubElement(ghparam, "wpml:waypointHeadingMode").text = WAYPOINT_HEADING_MODE
    SubElement(ghparam, "wpml:waypointHeadingAngle").text = f"{head_deg:.1f}"
    SubElement(ghparam, "wpml:waypointPoiPoint").text = "0.000000,0.000000,0.000000"
    SubElement(ghparam, "wpml:waypointHeadingPathMode").text = "followBadArc"
    SubElement(ghparam, "wpml:waypointHeadingPoiIndex").text = "0"
    
    SubElement(folder, "wpml:globalWaypointTurnMode").text = "toPointAndStopWithDiscontinuityCurvature"
    SubElement(folder, "wpml:globalUseStraightLine").text = "1"

    # Waypoints
    for i,(lat,lon,alt_wgs84) in enumerate(wps):
        pm = SubElement(folder, "Placemark")
        
        # Point coordinates
        pt = SubElement(pm, "Point")
        coords = SubElement(pt, "coordinates")
        coords.text = f"\n            {lon},{lat}\n          "
        
        # Index
        SubElement(pm, "wpml:index").text = str(i)
        
        # Heights
        if TEMPLATE_HEIGHT_MODE == "EGM96":
            ellipsoid_h = alt_wgs84
            h_field = alt_wgs84 - egm96_geoid_offset(lat, lon)
        else:
            ellipsoid_h = alt_wgs84
            h_field = alt_wgs84
        
        SubElement(pm, "wpml:ellipsoidHeight").text = f"{ellipsoid_h:.12f}"
        SubElement(pm, "wpml:height").text = f"{h_field:.9f}"
        
        # Speed
        SubElement(pm, "wpml:waypointSpeed").text = str(AUTO_FLIGHT_SPEED)
        
        # Waypoint heading param — fixed at facade-facing yaw for sideways flight
        hparam = SubElement(pm, "wpml:waypointHeadingParam")
        SubElement(hparam, "wpml:waypointHeadingMode").text = WAYPOINT_HEADING_MODE
        SubElement(hparam, "wpml:waypointHeadingAngle").text = f"{head_deg:.1f}"
        SubElement(hparam, "wpml:waypointPoiPoint").text = "0.000000,0.000000,0.000000"
        SubElement(hparam, "wpml:waypointHeadingPathMode").text = "followBadArc"
        SubElement(hparam, "wpml:waypointHeadingPoiIndex").text = "0"
        
        # Waypoint turn param
        tparam = SubElement(pm, "wpml:waypointTurnParam")
        SubElement(tparam, "wpml:waypointTurnMode").text = "toPointAndStopWithDiscontinuityCurvature"
        SubElement(tparam, "wpml:waypointTurnDampingDist").text = "0.2"
        
        # Use global settings (but NOT for height - each waypoint has different height)
        SubElement(pm, "wpml:useGlobalHeight").text = "0"  # ✓ 修复：使用各自高度
        SubElement(pm, "wpml:useGlobalSpeed").text = "1"
        SubElement(pm, "wpml:useGlobalHeadingParam").text = "1"
        SubElement(pm, "wpml:useGlobalTurnParam").text = "1"
        SubElement(pm, "wpml:useStraightLine").text = "1"
        
        # Action group
        agroup = SubElement(pm, "wpml:actionGroup")
        SubElement(agroup, "wpml:actionGroupId").text = str(i)
        SubElement(agroup, "wpml:actionGroupStartIndex").text = str(i)
        SubElement(agroup, "wpml:actionGroupEndIndex").text = str(i)
        SubElement(agroup, "wpml:actionGroupMode").text = "sequence"
        
        atrigger = SubElement(agroup, "wpml:actionTrigger")
        SubElement(atrigger, "wpml:actionTriggerType").text = "reachPoint"
        
        # Rotate Yaw action
        act1 = SubElement(agroup, "wpml:action")
        SubElement(act1, "wpml:actionId").text = "0"
        SubElement(act1, "wpml:actionActuatorFunc").text = "rotateYaw"
        aparam1 = SubElement(act1, "wpml:actionActuatorFuncParam")
        SubElement(aparam1, "wpml:aircraftHeading").text = f"{head_deg:.1f}"
        SubElement(aparam1, "wpml:aircraftPathMode").text = "counterClockwise"
        
        # Gimbal Rotate action
        act2 = SubElement(agroup, "wpml:action")
        SubElement(act2, "wpml:actionId").text = "1"
        SubElement(act2, "wpml:actionActuatorFunc").text = "gimbalRotate"
        aparam2 = SubElement(act2, "wpml:actionActuatorFuncParam")
        SubElement(aparam2, "wpml:gimbalHeadingYawBase").text = "north"
        SubElement(aparam2, "wpml:gimbalRotateMode").text = "absoluteAngle"
        SubElement(aparam2, "wpml:gimbalPitchRotateEnable").text = "1"
        SubElement(aparam2, "wpml:gimbalPitchRotateAngle").text = f"{GIMBAL_PITCH_DEG:.0f}"
        SubElement(aparam2, "wpml:gimbalRollRotateEnable").text = "0"
        SubElement(aparam2, "wpml:gimbalRollRotateAngle").text = "0"
        SubElement(aparam2, "wpml:gimbalYawRotateEnable").text = "0"
        SubElement(aparam2, "wpml:gimbalYawRotateAngle").text = "0"
        SubElement(aparam2, "wpml:gimbalRotateTimeEnable").text = "0"
        SubElement(aparam2, "wpml:gimbalRotateTime").text = "0"
        SubElement(aparam2, "wpml:payloadPositionIndex").text = "0"

        if ENABLE_PHOTO_CAPTURE:
            act3 = SubElement(agroup, "wpml:action")
            SubElement(act3, "wpml:actionId").text = "2"
            SubElement(act3, "wpml:actionActuatorFunc").text = "takePhoto"
            aparam3 = SubElement(act3, "wpml:actionActuatorFuncParam")
            SubElement(aparam3, "wpml:payloadPositionIndex").text = "0"
            SubElement(aparam3, "wpml:fileSuffix").text = "point"
            SubElement(aparam3, "wpml:payloadLensIndex").text = "wide"
            SubElement(aparam3, "wpml:useGlobalPayloadLensIndex").text = "0"

        SubElement(pm, "wpml:isRisky").text = "0"
    
    # Payload param at folder level
    pparam = SubElement(folder, "wpml:payloadParam")
    SubElement(pparam, "wpml:payloadPositionIndex").text = "0"
    SubElement(pparam, "wpml:imageFormat").text = "wide"

    return root

# ====================== KMZ Packaging ==========================
import re

def sanitize_name(name: str) -> str:
    # allow letters, numbers, spaces, and hyphens only
    safe = re.sub(r"[^A-Za-z0-9\- ]", "", name)
    return safe.strip()

def save_xml(elem: Element) -> bytes:
    from xml.dom import minidom
    rough = tostring(elem, encoding="utf-8")
    try:
        parsed = minidom.parseString(rough)
        pretty = parsed.toprettyxml(indent="  ", encoding="utf-8")
        return pretty
    except Exception:
        return rough


def write_kmz(template_xml: Element, waylines_xml: Element, kmz_path: str):
    # Ensure .kmz extension
    if not kmz_path.lower().endswith('.kmz'):
        kmz_path = kmz_path + '.kmz'

    logger.info(f"Writing KMZ file: {kmz_path}")
    # Create KMZ with correct DJI structure: wpmz/ subdirectory
    with zipfile.ZipFile(kmz_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        # DJI requires files in wpmz/ subdirectory
        zf.writestr("wpmz/template.kml", save_xml(template_xml))
        zf.writestr("wpmz/waylines.wpml", save_xml(waylines_xml))
        # Optional: add res/ folder for resources if needed
        # zf.writestr("wpmz/res/.keep", b"")
    logger.success(f"KMZ saved: {kmz_path}")

# ============================ Main =============================

def main(argv: List[str]):
    logger.info("Starting Mavic 3T Facade Mission Planner")
    if len(argv) >= 5:
        images = argv[1:]
        logger.debug(f"Using {len(images)} images from command line arguments")
    else:
        images = PHOTO_PATHS
        logger.debug(f"Using {len(images)} images from PHOTO_PATHS config")
        for p in images:
            if not Path(p).exists():
                logger.error(f"Missing photo: {p}")
                sys.exit(1)

    mission_name = sanitize_name(MISSION_NAME) or "Mission"
    logger.info(f"Mission name: {mission_name}")

    apply_drone_wpml_profile(DRONE_TYPE)
    logger.info("Building facade mission...")
    tf, direction, wps = build_waypoints_from_images(images)
    logger.info(f"Direction: {direction}, waypoints: {len(wps)}")

    # Preview KML
    kml = build_generic_kml(wps)
    preview_path = f"{mission_name}_preview.kml"
    ElementTree(kml).write(preview_path, encoding="utf-8", xml_declaration=True)
    logger.success(f"Saved preview: {preview_path}")

    # KMZ (wpmz/template.kml + wpmz/waylines.wpml)
    waylines = build_waylines_wpml(tf, wps)
    template = build_template_kml(tf, wps)
    kmz_file = f"{mission_name}.kmz"
    write_kmz(template, waylines, kmz_file)

    logger.info("Mission generation complete")


if __name__ == "__main__":
    main(sys.argv)
