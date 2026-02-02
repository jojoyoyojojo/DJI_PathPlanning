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
from math import radians, cos, tan, sqrt, degrees, atan2
from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, ElementTree, tostring
from typing import List, Tuple
import zipfile
import io
import numpy as np

# ========================= User Config =========================
# Photos (replace or pass via CLI)
PHOTO_PATHS = [
    '/Users/andyliu/Downloads/hkstp_test/11/DJI_0109.JPG',
    '/Users/andyliu/Downloads/hkstp_test/11/DJI_0110.JPG',
    '/Users/andyliu/Downloads/hkstp_test/11/DJI_0111.JPG',
    '/Users/andyliu/Downloads/hkstp_test/11/DJI_0112.JPG',
]

# Planning
PHOTO_DISTANCE = 5.0      # Distance from camera to facade when corner photos were taken (meters) - RTK prior knowledge
FLIGHT_DISTANCE = 5.0     # Desired distance from facade for mission flight (meters)
OVERLAP_RATE = 0.65
ENABLE_SMART_PLANNING = True

# Camera FOV (deg)
CAMERA_HFOV = 84.0
CAMERA_VFOV = 62.0

# DJI / WPML params
DRONE_TYPE = "M3T"
AUTO_FLIGHT_SPEED = 4.0
GIMBAL_PITCH_DEG = 0.0     # level shot at each point
HEADING_MODE = "UsePointSetting"

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

# ===================== EXIF GPS helpers ========================

def _dms_to_deg(dms, ref):
    d = dms[0].num / dms[0].den
    m = dms[1].num / dms[1].den
    s = dms[2].num / dms[2].den
    sign = -1 if ref in ("S", "W") else 1
    return sign * (d + m/60 + s/3600)


def read_gps(path: str) -> Tuple[float, float, float]:
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
    def __init__(self, gps4: List[Tuple[float, float, float]]):
        if len(gps4) != 4:
            raise ValueError("需要 4 个点")
        self.gps = gps4
        self.ref = gps4[0]
        self.enu = [geodetic_to_enu(lat, lon, alt, *self.ref) for (lat,lon,alt) in gps4]
        self.R = None  # rows: x', y', z' in ENU
        self.origin = None
        self.plane = None
        self._build()
        self.facade_pts = [self.enu_to_facade(p) for p in self.enu]

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

    def _build(self):
        a,b,c,d = self._fit_plane(self.enu)
        nlen = sqrt(a*a+b*b+c*c); yprime=[a/nlen,b/nlen,c/nlen]
        avg = sum(a*p[0]+b*p[1]+c*p[2]+d for p in self.enu)/4.0
        if avg<0: yprime=[-yprime[0],-yprime[1],-yprime[2]]; a,b,c,d=-a,-b,-c,-d
        # width by low/high edges
        sz=sorted(self.enu,key=lambda p:p[2]); low=sz[:2]; high=sz[2:]
        vlow=[low[1][i]-low[0][i] for i in range(3)]
        vhigh=[high[1][i]-high[0][i] for i in range(3)]
        wvec = vlow if sqrt(sum(x*x for x in vlow))>sqrt(sum(x*x for x in vhigh)) else vhigh
        wproj = sum(wvec[i]*yprime[i] for i in range(3))
        wplane = [wvec[i]-wproj*yprime[i] for i in range(3)]
        lwp = sqrt(sum(x*x for x in wplane))
        if lwp<1e-9: raise ValueError("宽度向量与外立面垂直，无法确定X'")
        xprime=[x/lwp for x in wplane]
        zprime=self._cross(xprime,yprime)
        if zprime[2]<0: zprime=[-z for z in zprime]; xprime=[-x for x in xprime]
        self.R=[xprime,yprime,zprime]
        self.origin=[sum(p[i] for p in self.enu)/4.0 for i in range(3)]
        self.plane=[a,b,c,d]

    def enu_to_facade(self, enu):
        t=[enu[i]-self.origin[i] for i in range(3)]
        return tuple(sum(t[j]*self.R[i][j] for j in range(3)) for i in range(3))

    def facade_to_enu(self, fp):
        enu_t=[sum(fp[j]*self.R[j][i] for j in range(3)) for i in range(3)]
        return tuple(enu_t[i]+self.origin[i] for i in range(3))

    def facade_to_gps(self, fp):
        x,y,z=self.facade_to_enu(fp)
        return enu_to_geodetic(x,y,z,*self.ref)

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
    gps4=[read_gps(p) for p in images]
    tf=FacadeTransformer(gps4)

    xs=[p[0] for p in tf.facade_pts]; zs=[p[2] for p in tf.facade_pts]
    w=max(xs)-min(xs); h=max(zs)-min(zs)

    if ENABLE_SMART_PLANNING:
        direction = "vertical" if h>w else "horizontal"
    else:
        direction = "horizontal"

    step_cross, step_along = plan_steps(direction, FLIGHT_DISTANCE, OVERLAP_RATE)
    cross_span = w if direction=="vertical" else h
    num_lines = int(max(1, round(cross_span/step_cross))) + 1

    minx,maxx=min(xs),max(xs); minz,maxz=min(zs),max(zs)
    yps=[p[1] for p in tf.facade_pts]
    avg_y = sum(yps)/len(yps)
    # RTK workflow: camera plane is parallel to facade, offset by PHOTO_DISTANCE
    # Flight plane = camera plane - PHOTO_DISTANCE (to facade) + FLIGHT_DISTANCE (from facade)
    safe_y = avg_y - PHOTO_DISTANCE + FLIGHT_DISTANCE
    print(f"RTK offset: camera Y'={avg_y:.2f}m, photo_dist={PHOTO_DISTANCE}m, flight_dist={FLIGHT_DISTANCE}m → flight Y'={safe_y:.2f}m")

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
    SubElement(dinfo, "wpml:droneEnumValue").text = "67"  # M3T
    SubElement(dinfo, "wpml:droneSubEnumValue").text = "1"
    
    SubElement(mcfg, "wpml:waylineAvoidLimitAreaMode").text = "0"
    
    # Payload Info
    pinfo = SubElement(mcfg, "wpml:payloadInfo")
    SubElement(pinfo, "wpml:payloadEnumValue").text = "52"  # M3T camera
    SubElement(pinfo, "wpml:payloadSubEnumValue").text = "0"
    SubElement(pinfo, "wpml:payloadPositionIndex").text = "0"

    # Folder with waypoints
    folder = SubElement(doc, "Folder")
    SubElement(folder, "wpml:templateId").text = "0"
    SubElement(folder, "wpml:executeHeightMode").text = EXECUTE_HEIGHT_MODE
    SubElement(folder, "wpml:waylineId").text = "0"
    SubElement(folder, "wpml:distance").text = "0"
    SubElement(folder, "wpml:duration").text = "0"
    SubElement(folder, "wpml:autoFlightSpeed").text = str(AUTO_FLIGHT_SPEED)

    # Heading facing wall: −Y' in ENU
    R = tf.R; yprime_enu = np.array(R[1], dtype=float)
    to_wall = -yprime_enu
    nrm = np.linalg.norm(to_wall[:2]) + 1e-12
    to_wall[:2] /= nrm
    def yaw_from_xy(vxy):
        dx,dy=float(vxy[0]),float(vxy[1])
        ang = degrees(atan2(dy,dx))
        return (ang+360.0)%360.0
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
        
        # Waypoint heading param
        hparam = SubElement(pm, "wpml:waypointHeadingParam")
        SubElement(hparam, "wpml:waypointHeadingMode").text = "followWayline"
        SubElement(hparam, "wpml:waypointHeadingAngle").text = "0"
        SubElement(hparam, "wpml:waypointPoiPoint").text = "0.000000,0.000000,0.000000"
        SubElement(hparam, "wpml:waypointHeadingAngleEnable").text = "0"
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
    SubElement(dinfo, "wpml:droneEnumValue").text = "67"  # M3T
    SubElement(dinfo, "wpml:droneSubEnumValue").text = "1"
    
    SubElement(mcfg, "wpml:waylineAvoidLimitAreaMode").text = "0"
    
    # Payload Info
    pinfo = SubElement(mcfg, "wpml:payloadInfo")
    SubElement(pinfo, "wpml:payloadEnumValue").text = "52"  # M3T camera
    SubElement(pinfo, "wpml:payloadSubEnumValue").text = "0"
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
    
    # Global waypoint heading param
    ghparam = SubElement(folder, "wpml:globalWaypointHeadingParam")
    SubElement(ghparam, "wpml:waypointHeadingMode").text = "followWayline"
    SubElement(ghparam, "wpml:waypointHeadingAngle").text = "0"
    SubElement(ghparam, "wpml:waypointPoiPoint").text = "0.000000,0.000000,0.000000"
    SubElement(ghparam, "wpml:waypointHeadingPathMode").text = "followBadArc"
    SubElement(ghparam, "wpml:waypointHeadingPoiIndex").text = "0"
    
    SubElement(folder, "wpml:globalWaypointTurnMode").text = "toPointAndStopWithDiscontinuityCurvature"
    SubElement(folder, "wpml:globalUseStraightLine").text = "1"

    # Heading facing wall: −Y' in ENU
    R = tf.R; yprime_enu = np.array(R[1], dtype=float)
    to_wall = -yprime_enu
    nrm = np.linalg.norm(to_wall[:2]) + 1e-12
    to_wall[:2] /= nrm
    def yaw_from_xy(vxy):
        dx,dy=float(vxy[0]),float(vxy[1])
        ang = degrees(atan2(dy,dx))
        return (ang+360.0)%360.0
    head_deg = yaw_from_xy(to_wall)

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
        
        # Waypoint heading param
        hparam = SubElement(pm, "wpml:waypointHeadingParam")
        SubElement(hparam, "wpml:waypointHeadingMode").text = "followWayline"
        SubElement(hparam, "wpml:waypointHeadingAngle").text = "0"
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
    
    # Create KMZ with correct DJI structure: wpmz/ subdirectory
    with zipfile.ZipFile(kmz_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        # DJI requires files in wpmz/ subdirectory
        zf.writestr("wpmz/template.kml", save_xml(template_xml))
        zf.writestr("wpmz/waylines.wpml", save_xml(waylines_xml))
        # Optional: add res/ folder for resources if needed
        # zf.writestr("wpmz/res/.keep", b"")

# ============================ Main =============================

def main(argv: List[str]):
    if len(argv) >= 5:
        images = argv[1:]
    else:
        images = PHOTO_PATHS
        for p in images:
            if not Path(p).exists():
                print(f"Missing photo: {p}")
                sys.exit(1)

    mission_name = sanitize_name(MISSION_NAME) or "Mission"

    print("Building facade mission...")
    tf, direction, wps = build_waypoints_from_images(images)
    print(f"Direction: {direction}, waypoints: {len(wps)}")

    # Preview KML
    kml = build_generic_kml(wps)
    ElementTree(kml).write(f"{mission_name}_preview.kml", encoding="utf-8", xml_declaration=True)
    print(f"✓ Saved preview: {mission_name}_preview.kml")

    # KMZ (wpmz/template.kml + wpmz/waylines.wpml)
    waylines = build_waylines_wpml(tf, wps)
    template = build_template_kml(tf, wps)
    kmz_file = f"{mission_name}.kmz"
    write_kmz(template, waylines, kmz_file)
    print(f"✓ Saved KMZ (WPML package): {kmz_file}")


if __name__ == "__main__":
    main(sys.argv)
