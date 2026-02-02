#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import exifread
from math import radians, cos, sin, tan, sqrt, degrees, acos, atan2
from pathlib import Path
from xml.etree.ElementTree import Element, SubElement, ElementTree
from datetime import datetime
import numpy as np

# ============================================================================
# 可调整的参数配置
# ============================================================================

# image path
PHOTO_PATH_1 = "/Users/houyujie/Downloads/hkstp_test/1/DJI_0066.JPG"
PHOTO_PATH_2 = "/Users/houyujie/Downloads/hkstp_test/1/DJI_0067.JPG"
PHOTO_PATH_3 = "/Users/houyujie/Downloads/hkstp_test/1/DJI_0068.JPG"
PHOTO_PATH_4 = "/Users/houyujie/Downloads/hkstp_test/1/DJI_0069.JPG"
DEFAULT_PHOTOS = [PHOTO_PATH_1, PHOTO_PATH_2, PHOTO_PATH_3, PHOTO_PATH_4]

# flight parameters
PHOTO_DISTANCE = 5.0       # Distance from camera to facade when corner photos were taken (meters) - RTK prior knowledge
FLIGHT_DISTANCE = 5.0      # Desired distance from facade for mission flight (meters)
OVERLAP_RATE = 0.65        # overlap rate

# intelligent flight path planning
ENABLE_SMART_PLANNING = True  # True: auto-select vertical/horizontal based on facade size
                              # False: always use horizontal flight mode

# camera parameters
CAMERA_HFOV = 84.0         # horizontal field of view
CAMERA_VFOV = 62.0         # vertical field of view

# earth radius
EARTH_R = 6378137.0

# ============================================================================
# Camera parameter conversion
# ============================================================================
HFOV_RGB = radians(CAMERA_HFOV)
VFOV_RGB = radians(CAMERA_VFOV)

# ============================================================================
# Function definitions
# ============================================================================

class FacadeCoordinateTransformer:
    """外立面坐标转换器 - 真正使用四个点拟合平面"""
    
    def __init__(self, four_gps_points):
        self.gps_points = four_gps_points
        self.reference_point = four_gps_points[0]
        
        # 转换为ENU坐标
        self.enu_points = self._gps_to_enu_batch(four_gps_points)
        
        # 建立外立面局部坐标系
        self.facade_transform_matrix = None
        self.facade_origin = None
        self.fitted_plane_params = None
        self._establish_facade_coordinate_system()
        
        # 转换为外立面坐标
        self.facade_points = self._enu_to_facade_batch(self.enu_points)
    
    def _gps_to_enu_batch(self, gps_points):
        """批量转换GPS到ENU坐标"""
        lat0, lon0, alt0 = self.reference_point
        enu_points = []
        
        for lat, lon, alt in gps_points:
            x, y, z = geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0)
            enu_points.append((x, y, z))
        
        return enu_points
    
    def _establish_facade_coordinate_system(self):
        """建立外立面局部坐标系 - 真正使用四个点"""
        if len(self.enu_points) != 4:
            raise ValueError("需要4个点来建立外立面坐标系")
        
        # 用四个点拟合最佳平面
        plane_params = self._fit_plane_with_four_points(self.enu_points)
        a, b, c, d = plane_params
        
        # 归一化法向量得到Y'轴（垂直于外立面方向）
        normal_length = sqrt(a**2 + b**2 + c**2)
        if normal_length < 0.001:
            raise ValueError("四个点无法确定有效平面")
        
        y_prime_axis = [a/normal_length, b/normal_length, c/normal_length]
        
        # 确保Y'轴指向"远离外立面"的方向
        avg_distance = sum(a*p[0] + b*p[1] + c*p[2] + d for p in self.enu_points) / 4
        if avg_distance < 0:
            y_prime_axis = [-x for x in y_prime_axis]
            plane_params = [-a, -b, -c, -d]
        
        # 计算X'轴（外立面宽度方向）
        sorted_by_z = sorted(self.enu_points, key=lambda p: p[2])
        low_points = sorted_by_z[:2]
        high_points = sorted_by_z[2:]
        
        low_vector = [low_points[1][i] - low_points[0][i] for i in range(3)]
        high_vector = [high_points[1][i] - high_points[0][i] for i in range(3)]
        
        low_length = sqrt(sum(x**2 for x in low_vector))
        high_length = sqrt(sum(x**2 for x in high_vector))
        
        width_vector = low_vector if low_length > high_length else high_vector
        
        if sqrt(sum(x**2 for x in width_vector)) < 0.001:
            raise ValueError("无法确定外立面宽度方向")
        
        # 将宽度向量投影到外立面平面上
        width_proj_y = sum(width_vector[i] * y_prime_axis[i] for i in range(3))
        width_in_plane = [width_vector[i] - width_proj_y * y_prime_axis[i] for i in range(3)]
        
        width_plane_length = sqrt(sum(x**2 for x in width_in_plane))
        if width_plane_length < 0.001:
            raise ValueError("宽度向量与外立面垂直，无法确定X'轴")
        
        x_prime_axis = [x / width_plane_length for x in width_in_plane]
        
        # 计算Z'轴，确保右手坐标系
        z_prime_axis = self._cross_product(x_prime_axis, y_prime_axis)
        
        if z_prime_axis[2] < 0:
            z_prime_axis = [-x for x in z_prime_axis]
            x_prime_axis = [-x for x in x_prime_axis]
        
        self.facade_transform_matrix = [x_prime_axis, y_prime_axis, z_prime_axis]
        
        # 使用四个点的重心作为原点
        centroid = [sum(p[i] for p in self.enu_points) / 4 for i in range(3)]
        self.facade_origin = centroid
        self.fitted_plane_params = plane_params
    
    def _fit_plane_with_four_points(self, points):
        """用四个点拟合最佳平面（最小二乘法）"""
        # 计算重心
        cx = sum(p[0] for p in points) / len(points)
        cy = sum(p[1] for p in points) / len(points)
        cz = sum(p[2] for p in points) / len(points)
        
        # 中心化点
        centered_points = [[p[0]-cx, p[1]-cy, p[2]-cz] for p in points]
        
        p1, p2, p3, p4 = centered_points
        
        # 尝试不同的三点组合，选择最好的
        combinations = [
            (p1, p2, p3, p4), (p1, p2, p4, p3),
            (p1, p3, p4, p2), (p2, p3, p4, p1)
        ]
        
        best_normal = None
        min_total_error = float('inf')
        
        for p_i, p_j, p_k, p_test in combinations:
            v1 = [p_j[i] - p_i[i] for i in range(3)]
            v2 = [p_k[i] - p_i[i] for i in range(3)]
            normal = self._cross_product(v1, v2)
            normal_len = sqrt(sum(x**2 for x in normal))
            
            if normal_len < 0.001:
                continue
            
            normal = [x/normal_len for x in normal]
            
            total_error = sum(abs(sum(point[i] * normal[i] for i in range(3)))**2 
                            for point in centered_points)
            
            if total_error < min_total_error:
                min_total_error = total_error
                best_normal = normal
        
        if best_normal is None:
            raise ValueError("无法从四个点拟合平面")
        
        d = -(best_normal[0] * cx + best_normal[1] * cy + best_normal[2] * cz)
        return [best_normal[0], best_normal[1], best_normal[2], d]
    
    def _cross_product(self, v1, v2):
        """叉积计算"""
        return [
            v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0]
        ]
    
    def _enu_to_facade_batch(self, enu_points):
        """批量转换ENU到外立面坐标"""
        facade_points = []
        for enu_point in enu_points:
            facade_point = self.enu_to_facade(enu_point)
            facade_points.append(facade_point)
        return facade_points
    
    def enu_to_facade(self, enu_point):
        """ENU坐标转外立面坐标"""
        translated = [enu_point[i] - self.facade_origin[i] for i in range(3)]
        
        facade_point = []
        for i in range(3):
            coord = sum(translated[j] * self.facade_transform_matrix[i][j] for j in range(3))
            facade_point.append(coord)
        
        return tuple(facade_point)
    
    def facade_to_enu(self, facade_point):
        """外立面坐标转ENU坐标"""
        enu_translated = []
        for i in range(3):
            coord = sum(facade_point[j] * self.facade_transform_matrix[j][i] for j in range(3))
            enu_translated.append(coord)
        
        enu_point = [enu_translated[i] + self.facade_origin[i] for i in range(3)]
        return tuple(enu_point)
    
    def facade_to_gps(self, facade_point):
        """外立面坐标转GPS坐标"""
        enu_point = self.facade_to_enu(facade_point)
        return enu_to_geodetic(enu_point[0], enu_point[1], enu_point[2], 
                              self.reference_point[0], self.reference_point[1], self.reference_point[2])
    
    def point_to_plane_distance(self, point):
        """计算点到拟合平面的距离"""
        a, b, c, d = self.fitted_plane_params
        x, y, z = point
        
        numerator = abs(a*x + b*y + c*z + d)
        denominator = sqrt(a**2 + b**2 + c**2)
        
        return numerator / denominator if denominator > 0 else 0


def geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0):
    """GPS坐标转ENU坐标"""
    d_lat = radians(lat - lat0)
    d_lon = radians(lon - lon0)
    x = d_lon * cos(radians(lat0)) * EARTH_R
    y = d_lat * EARTH_R
    z = alt - alt0
    return x, y, z


def enu_to_geodetic(x, y, z, lat0, lon0, alt0):
    """ENU坐标转GPS坐标"""
    lat = lat0 + degrees(y / EARTH_R)
    lon = lon0 + degrees(x / (EARTH_R * cos(radians(lat0))))
    alt = alt0 + z
    return lat, lon, alt


def validate_coplanar_in_facade_system(facade_points, tolerance=10.0):
    """在外立面坐标系中验证共面性"""
    y_primes = [p[1] for p in facade_points]
    
    y_min, y_max = min(y_primes), max(y_primes)
    y_prime_span = y_max - y_min
    y_prime_mean = sum(y_primes) / len(y_primes)
    y_prime_std = sqrt(sum((y - y_prime_mean)**2 for y in y_primes) / len(y_primes))
    
    # 质量评级
    if y_prime_span < 0.1:
        quality = "优秀"
        quality_score = 1.0
    elif y_prime_span < 0.5:
        quality = "良好"
        quality_score = 0.8
    elif y_prime_span < 1.0:
        quality = "一般"
        quality_score = 0.6
    elif y_prime_span < tolerance:
        quality = "勉强"
        quality_score = 0.4
    else:
        quality = "很差"
        quality_score = 0.2
    
    return {
        'is_coplanar': y_prime_span <= tolerance,
        'y_prime_span': y_prime_span,
        'y_prime_mean': y_prime_mean,
        'y_prime_std': y_prime_std,
        'quality_score': quality_score,
        'quality_level': quality,
        'individual_y_primes': y_primes,
        'tolerance': tolerance
    }


def find_close_pairs(coordinates, tolerance):
    """寻找接近的坐标对"""
    pairs = []
    n = len(coordinates)
    
    for i in range(n):
        for j in range(i+1, n):
            diff = abs(coordinates[i] - coordinates[j])
            if diff <= tolerance:
                pairs.append((i, j, diff))
    
    return pairs


def validate_rectangular_distribution(facade_points, tolerance_ratio=0.15):
    """验证矩形分布"""
    x_primes = [p[0] for p in facade_points]
    z_primes = [p[2] for p in facade_points]
    
    x_total_span = max(x_primes) - min(x_primes)
    z_total_span = max(z_primes) - min(z_primes)
    
    x_tolerance = x_total_span * tolerance_ratio
    z_tolerance = z_total_span * tolerance_ratio
    
    x_pairs = find_close_pairs(x_primes, x_tolerance)
    z_pairs = find_close_pairs(z_primes, z_tolerance)
    
    # 验证矩形条件
    errors = []
    
    if len(x_pairs) != 2:
        errors.append(f"X'方向应该有2对接近点，实际有{len(x_pairs)}对")
    
    if len(z_pairs) != 2:
        errors.append(f"Z'方向应该有2对接近点，实际有{len(z_pairs)}对")
    
    # 检查配对覆盖
    if len(x_pairs) == 2 and len(z_pairs) == 2:
        x_points = set()
        z_points = set()
        
        for i, j, _ in x_pairs:
            x_points.update([i, j])
        for i, j, _ in z_pairs:
            z_points.update([i, j])
        
        if x_points != {0, 1, 2, 3}:
            errors.append("X'配对未覆盖所有点")
        if z_points != {0, 1, 2, 3}:
            errors.append("Z'配对未覆盖所有点")
        
        # 检查配对重叠
        x_pair_sets = [set([i, j]) for i, j, _ in x_pairs]
        z_pair_sets = [set([i, j]) for i, j, _ in z_pairs]
        
        for x_set in x_pair_sets:
            for z_set in z_pair_sets:
                if x_set == z_set:
                    errors.append(f"X'和Z'配对重叠")
    
    return {
        'is_rectangular': len(errors) == 0,
        'errors': errors,
        'x_pairs': x_pairs,
        'z_pairs': z_pairs,
        'facade_width': x_total_span,
        'facade_height': z_total_span
    }


def comprehensive_point_validation(gps_points):
    """全面的四点验证 - 四点拟合版本"""
    print(f"\n🔧 开始四点验证（深度方向共面标准: 10.0m）...")
    
    try:
        # 建立坐标转换器
        transformer = FacadeCoordinateTransformer(gps_points)
        facade_points = transformer.facade_points
        
        print("✅ 外立面坐标系建立成功")
        print(f"  拟合平面质量:")
        for i, enu_point in enumerate(transformer.enu_points):
            distance = transformer.point_to_plane_distance(enu_point)
            print(f"    点{i+1}到拟合平面距离: {distance:.3f}m")
        
        # 共面性验证
        coplanar_result = validate_coplanar_in_facade_system(facade_points)
        
        # 矩形分布验证
        rect_result = validate_rectangular_distribution(facade_points)
        
        # 输出详细分析
        print_validation_report(gps_points, transformer.enu_points, facade_points, 
                              coplanar_result, rect_result)
        
        # 综合判断
        is_valid = coplanar_result['is_coplanar'] and rect_result['is_rectangular']
        
        if not is_valid:
            print("\n❌ 验证失败：输入点不符合外立面要求")
            print("建议：请重新选择四个外立面角点照片")
            sys.exit(1)
        
        print("\n✅ 验证通过：可以进行外立面航点生成")
        return transformer
        
    except Exception as e:
        print(f"\n❌ 验证失败：{str(e)}")
        print("建议：请检查输入的四张照片是否正确")
        sys.exit(1)


def print_validation_report(gps_points, enu_points, facade_points, coplanar_result, rect_result):
    """打印详细的验证报告"""
    print(f"\n📍 GPS坐标输入:")
    for i, (lat, lon, alt) in enumerate(gps_points):
        print(f"  点{i+1}: {lat:.6f}°, {lon:.6f}°, {alt:.1f}m")
    
    print(f"\n🔄 ENU坐标系:")
    for i, (x, y, z) in enumerate(enu_points):
        print(f"  点{i+1}: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m")
    
    print(f"\n🏢 外立面坐标系:")
    for i, (x_prime, y_prime, z_prime) in enumerate(facade_points):
        print(f"  点{i+1}: X'={x_prime:.2f}m, Y'={y_prime:.2f}m, Z'={z_prime:.2f}m")
    
    print(f"\n🔍 共面性分析（深度方向）:")
    y_primes = coplanar_result['individual_y_primes']
    for i, y_prime in enumerate(y_primes):
        print(f"  点{i+1}: Y' = {y_prime:.3f}m")
    
    print(f"  Y'坐标统计:")
    print(f"    跨度: {coplanar_result['y_prime_span']:.3f}m")
    print(f"    标准差: {coplanar_result['y_prime_std']:.3f}m")
    print(f"  共面性判断:")
    print(f"    允许偏差: ≤ {coplanar_result['tolerance']:.1f}m (深度方向共面标准)")
    print(f"    实际偏差: {coplanar_result['y_prime_span']:.3f}m")
    
    if coplanar_result['is_coplanar']:
        print(f"    判断结果: ✅ 深度方向共面")
        print(f"    质量等级: {coplanar_result['quality_level']} (评分: {coplanar_result['quality_score']})")
    else:
        print(f"    判断结果: ❌ 深度方向不共面")
        print(f"    质量等级: {coplanar_result['quality_level']} (评分: {coplanar_result['quality_score']})")
    
    print(f"\n📐 矩形分布验证:")
    print(f"  外立面尺寸: {rect_result['facade_width']:.1f}m × {rect_result['facade_height']:.1f}m")
    print(f"  X'配对: {len(rect_result['x_pairs'])}对")
    print(f"  Z'配对: {len(rect_result['z_pairs'])}对")
    
    if rect_result['is_rectangular']:
        print(f"  矩形判断: ✅ 矩形分布")
        print(f"  X'接近配对:")
        for i, j, diff in rect_result['x_pairs']:
            print(f"    点{i+1}-点{j+1}: 差值={diff:.3f}m")
        print(f"  Z'接近配对:")
        for i, j, diff in rect_result['z_pairs']:
            print(f"    点{i+1}-点{j+1}: 差值={diff:.3f}m")
    else:
        print(f"  矩形判断: ❌ 非矩形分布")
        print(f"  错误信息:")
        for error in rect_result['errors']:
            print(f"    • {error}")
        if rect_result['x_pairs']:
            print(f"  X'接近配对:")
            for i, j, diff in rect_result['x_pairs']:
                print(f"    点{i+1}-点{j+1}: 差值={diff:.3f}m")
        if rect_result['z_pairs']:
            print(f"  Z'接近配对:")
            for i, j, diff in rect_result['z_pairs']:
                print(f"    点{i+1}-点{j+1}: 差值={diff:.3f}m")


def _dms_to_deg(dms, ref):
    d = dms[0].num / dms[0].den
    m = dms[1].num / dms[1].den
    s = dms[2].num / dms[2].den
    sign = -1 if ref in ("S", "W") else 1
    return sign * (d + m/60 + s/3600)


def read_gps(path):
    with open(path, "rb") as f:
        tags = exifread.process_file(f, details=False)
    lat = _dms_to_deg(tags["GPS GPSLatitude"].values, tags["GPS GPSLatitudeRef"].printable)
    lon = _dms_to_deg(tags["GPS GPSLongitude"].values, tags["GPS GPSLongitudeRef"].printable)
    
    # Fix altitude data extraction
    alt_tag = tags.get("GPS GPSAltitude")
    if alt_tag:
        # Correctly handle exifread's IfdTag object
        alt = float(alt_tag.values[0].num) / float(alt_tag.values[0].den)
        # Check altitude reference (above/below sea level)
        alt_ref = tags.get("GPS GPSAltitudeRef")
        if alt_ref and alt_ref.values[0] == 1:
            alt = -alt  # Below sea level
    else:
        alt = 0.0
    
    return lat, lon, alt


class CoordinateTransformer:
    """三种坐标系的转换器"""
    
    def __init__(self, four_gps_points):
        """
        初始化坐标转换器
        four_gps_points: 四个GPS坐标点 [(lat1, lon1, alt1), ...]
        """
        self.gps_points = four_gps_points
        self.reference_point = four_gps_points[0]  # 第一个点作为参考点
        
        # 转换为ENU坐标
        self.enu_points = self._gps_to_enu_batch(four_gps_points)
        
        # 建立外立面局部坐标系
        self.facade_transform_matrix = None
        self.facade_origin = None
        self._establish_facade_coordinate_system()
        
        # 转换为外立面坐标
        self.facade_points = self._enu_to_facade_batch(self.enu_points)
    
    def _gps_to_enu_batch(self, gps_points):
        """批量转换GPS到ENU坐标"""
        lat0, lon0, alt0 = self.reference_point
        enu_points = []
        
        for lat, lon, alt in gps_points:
            x, y, z = self.geodetic_to_enu(lat, lon, alt, lat0, lon0, alt0)
            enu_points.append((x, y, z))
        
        return enu_points
    
    def geodetic_to_enu(self, lat, lon, alt, lat0, lon0, alt0):
        """GPS坐标转ENU坐标"""
        d_lat = radians(lat - lat0)
        d_lon = radians(lon - lon0)
        x = d_lon * cos(radians(lat0)) * 6378137.0
        y = d_lat * 6378137.0
        z = alt - alt0
        return x, y, z
    
    def _establish_facade_coordinate_system(self):
        """建立外立面局部坐标系"""
        if len(self.enu_points) != 4:
            raise ValueError("需要4个点来建立外立面坐标系")
        
        # 按高度排序，分为低点和高点
        sorted_points = sorted(self.enu_points, key=lambda p: p[2])
        low_points = sorted_points[:2]   # 两个低点
        high_points = sorted_points[2:]  # 两个高点
        
        # 计算外立面的宽度方向 (X'轴)
        # 使用低点的连线作为宽度方向的参考
        low_p1, low_p2 = low_points
        width_vector = [low_p2[i] - low_p1[i] for i in range(3)]
        
        # 如果低点连线太短，使用高点连线
        width_length = sqrt(sum(x**2 for x in width_vector))
        if width_length < 1.0:
            high_p1, high_p2 = high_points
            width_vector = [high_p2[i] - high_p1[i] for i in range(3)]
            width_length = sqrt(sum(x**2 for x in width_vector))
        
        if width_length < 0.1:
            raise ValueError("无法确定外立面宽度方向")
        
        # 归一化宽度向量，得到X'轴单位向量
        x_prime_axis = [x / width_length for x in width_vector]
        
        # 计算外立面的高度方向 (Z'轴)
        # 使用所有点的高度范围
        all_z = [p[2] for p in self.enu_points]
        z_prime_axis = [0, 0, 1]  # 高度方向就是ENU的Z轴
        
        # 计算垂直于外立面的方向 (Y'轴)
        # Y' = Z' × X' (右手定则)
        y_prime_axis = self._cross_product(z_prime_axis, x_prime_axis)
        y_prime_length = sqrt(sum(x**2 for x in y_prime_axis))
        
        if y_prime_length < 0.1:
            raise ValueError("X'轴和Z'轴过于平行，无法确定Y'轴")
        
        y_prime_axis = [x / y_prime_length for x in y_prime_axis]
        
        # 重新计算Z'轴确保正交 (Z' = X' × Y')
        z_prime_axis = self._cross_product(x_prime_axis, y_prime_axis)
        
        # 构建转换矩阵 [X', Y', Z'] = R * [X, Y, Z]
        # 这里R是从ENU到外立面坐标系的旋转矩阵
        self.facade_transform_matrix = [
            x_prime_axis,  # X'轴在ENU中的表示
            y_prime_axis,  # Y'轴在ENU中的表示  
            z_prime_axis   # Z'轴在ENU中的表示
        ]
        
        # 选择外立面坐标系的原点（四个点的重心）
        centroid = [
            sum(p[i] for p in self.enu_points) / 4 for i in range(3)
        ]
        self.facade_origin = centroid
        
        print(f"\n🔧 外立面坐标系建立成功:")
        print(f"原点 (ENU): ({centroid[0]:.2f}, {centroid[1]:.2f}, {centroid[2]:.2f})")
        print(f"X'轴 (宽度): ({x_prime_axis[0]:.3f}, {x_prime_axis[1]:.3f}, {x_prime_axis[2]:.3f})")
        print(f"Y'轴 (深度): ({y_prime_axis[0]:.3f}, {y_prime_axis[1]:.3f}, {y_prime_axis[2]:.3f})")
        print(f"Z'轴 (高度): ({z_prime_axis[0]:.3f}, {z_prime_axis[1]:.3f}, {z_prime_axis[2]:.3f})")
    
    def _cross_product(self, v1, v2):
        """计算叉积"""
        return [
            v1[1] * v2[2] - v1[2] * v2[1],
            v1[2] * v2[0] - v1[0] * v2[2],
            v1[0] * v2[1] - v1[1] * v2[0]
        ]
    
    def _enu_to_facade_batch(self, enu_points):
        """批量转换ENU到外立面坐标"""
        facade_points = []
        for enu_point in enu_points:
            facade_point = self.enu_to_facade(enu_point)
            facade_points.append(facade_point)
        return facade_points
    
    def enu_to_facade(self, enu_point):
        """ENU坐标转外立面坐标"""
        if self.facade_transform_matrix is None:
            raise ValueError("外立面坐标系未建立")
        
        # 平移到外立面原点
        translated = [enu_point[i] - self.facade_origin[i] for i in range(3)]
        
        # 旋转到外立面坐标系
        facade_point = []
        for i in range(3):
            # 点积运算: 新坐标 = 旧坐标 · 新轴向量
            coord = sum(translated[j] * self.facade_transform_matrix[i][j] for j in range(3))
            facade_point.append(coord)
        
        return tuple(facade_point)
    
    def facade_to_enu(self, facade_point):
        """外立面坐标转ENU坐标"""
        if self.facade_transform_matrix is None:
            raise ValueError("外立面坐标系未建立")
        
        # 逆旋转 (转置矩阵)
        enu_translated = []
        for i in range(3):
            coord = sum(facade_point[j] * self.facade_transform_matrix[j][i] for j in range(3))
            enu_translated.append(coord)
        
        # 逆平移
        enu_point = [enu_translated[i] + self.facade_origin[i] for i in range(3)]
        
        return tuple(enu_point)
    
    def facade_to_gps(self, facade_point):
        """外立面坐标转GPS坐标"""
        enu_point = self.facade_to_enu(facade_point)
        return self.enu_to_geodetic(enu_point)
    
    def enu_to_geodetic(self, enu_point):
        """ENU坐标转GPS坐标"""
        x, y, z = enu_point
        lat0, lon0, alt0 = self.reference_point
        
        lat = lat0 + degrees(y / 6378137.0)
        lon = lon0 + degrees(x / (6378137.0 * cos(radians(lat0))))
        alt = alt0 + z
        
        return lat, lon, alt


def build_wpml_template(images, distance=FLIGHT_DISTANCE, overlap=OVERLAP_RATE):
    """Generate KML file according to official format"""
    coords = [read_gps(p) for p in images]
    
    # 进行四点验证并建立坐标转换器
    transformer = comprehensive_point_validation(coords)
    
    # 获取外立面尺寸用于智能规划
    facade_points = transformer.facade_points
    x_primes = [p[0] for p in facade_points]
    z_primes = [p[2] for p in facade_points]
    facade_width = max(x_primes) - min(x_primes)
    facade_height = max(z_primes) - min(z_primes)
    
    if ENABLE_SMART_PLANNING:
        if facade_height > facade_width:
            flight_direction = "vertical"
            print(f"\n🚁 智能规划: 选择垂直航线 (高度{facade_height:.1f}m > 宽度{facade_width:.1f}m)")
        else:
            flight_direction = "horizontal"
            print(f"\n🚁 智能规划: 选择水平航线 (宽度{facade_width:.1f}m >= 高度{facade_height:.1f}m)")
    else:
        flight_direction = "horizontal"
        print(f"\n🚁 固定模式: 使用水平航线")
    
    # 计算航线数量
    # Calculate shooting distance
    shooting_distance = distance
    
    # Calculate coverage per shot
    if flight_direction == "vertical":
        # Vertical flight, calculate horizontal coverage
        coverage_per_shot = 2 * shooting_distance * tan(HFOV_RGB / 2)
    else:
        # Horizontal flight, calculate vertical coverage
        coverage_per_shot = 2 * shooting_distance * tan(VFOV_RGB / 2)
    
    # Calculate effective coverage (considering overlap)
    effective_coverage = coverage_per_shot * (1 - overlap)
    
    # Calculate number of main flight lines
    num_main_lines = int(facade_width / effective_coverage) + 1
    
    print(f"  Shooting distance: {shooting_distance:.2f}m")
    print(f"  Overlap rate: {overlap*100:.0f}%")
    print(f"  Coverage per shot: {coverage_per_shot:.2f}m")
    print(f"  Effective coverage: {effective_coverage:.2f}m")
    print(f"  Number of main flight lines: {num_main_lines}")
    
    # 生成航点
    waypoints = generate_facade_waypoints(transformer, flight_direction, num_main_lines, distance, overlap)
    
    # 生成KML文件
    root = Element("kml")
    root.set("xmlns", "http://www.opengis.net/kml/2.2")
    
    doc = SubElement(root, "Document")
    name = SubElement(doc, "name")
    name.text = "无人机巡检航线"
    
    open_tag = SubElement(doc, "open")
    open_tag.text = "1"
    
    # 添加ExtendedData
    ext_data = SubElement(doc, "ExtendedData")
    ext_data.set("xmlns:mis", "www.dji.com")
    mis_type = SubElement(ext_data, "mis:type")
    mis_type.text = "Waypoint"
    
    # 创建航点文件夹
    folder = SubElement(doc, "Folder")
    folder_name = SubElement(folder, "name")
    folder_name.text = "waypoints"
    folder_desc = SubElement(folder, "description")
    folder_desc.text = "Waypoints in the Mission"
    
    # 生成航点
    waypoint_coords = []
    for i, (lat, lon, alt) in enumerate(waypoints):
        # 创建航点Placemark
        pm = SubElement(folder, "Placemark")
        pm_name = SubElement(pm, "name")
        pm_name.text = f"Waypoint{str(i+1).zfill(3)}"
        pm_desc = SubElement(pm, "description")
        pm_desc.text = "Waypoint"
        pm_vis = SubElement(pm, "visibility")
        pm_vis.text = "1"
        
        # 添加ExtendedData
        pm_ext = SubElement(pm, "ExtendedData")
        pm_ext.set("xmlns:mis", "www.dji.com")
        
        # 添加航点参数
        heading = SubElement(pm_ext, "mis:heading")
        heading.text = "151"
        speed = SubElement(pm_ext, "mis:speed")
        speed.text = "4"
        shooting_dist = SubElement(pm_ext, "mis:shootingDistance")
        shooting_dist.text = "5"
        shooting_coords = SubElement(pm_ext, "mis:shootingPointCoordinates")
        shooting_coords.text = f"{lon},{lat},{alt}"
        use_alt = SubElement(pm_ext, "mis:useWaylineAltitude")
        use_alt.text = "false"
        gimbal = SubElement(pm_ext, "mis:gimbalPitch")
        gimbal.text = "0"
        shooting_name = SubElement(pm_ext, "mis:shootingPointName")
        shooting_name.text = str(i+1)
        turn_mode = SubElement(pm_ext, "mis:turnMode")
        turn_mode.text = "Auto"
        
        # 添加动作
        action1 = SubElement(pm_ext, "mis:actions")
        action1.set("param", "151")
        action1.set("label", "飞行器偏航角")
        action1.text = "AircraftYaw"
        
        action2 = SubElement(pm_ext, "mis:actions")
        action2.set("param", "0")
        action2.set("label", "云台俯仰角度")
        action2.text = "GimbalPitch"
        
        action3 = SubElement(pm_ext, "mis:actions")
        action3.set("label", "拍照")
        action3.set("targetMode", "1")
        action3.text = "ShootPhoto"
        
        # 创建Point
        pt = SubElement(pm, "Point")
        alt_mode = SubElement(pt, "altitudeMode")
        alt_mode.text = "absolute"
        coords_elem = SubElement(pt, "coordinates")
        coords_elem.text = f"{lon},{lat},{alt}"
        
        waypoint_coords.append((lon, lat, alt))
    
    # 创建航线
    wayline = SubElement(doc, "Placemark")
    wayline_name = SubElement(wayline, "name")
    wayline_name.text = "Wayline"
    wayline_desc = SubElement(wayline, "description")
    wayline_desc.text = "Wayline"
    wayline_vis = SubElement(wayline, "visibility")
    wayline_vis.text = "1"
    
    # 添加航线ExtendedData
    wayline_ext = SubElement(wayline, "ExtendedData")
    wayline_ext.set("xmlns:mis", "www.dji.com")
    
    auto_speed = SubElement(wayline_ext, "mis:autoFlightSpeed")
    auto_speed.text = "4"
    action_finish = SubElement(wayline_ext, "mis:actionOnFinish")
    action_finish.text = "GoHome"
    heading_mode = SubElement(wayline_ext, "mis:headingMode")
    heading_mode.text = "UsePointSetting"
    idle_vel = SubElement(wayline_ext, "mis:idleVel")
    idle_vel.text = "13"
    shooting_dist_wayline = SubElement(wayline_ext, "mis:shootingDistance")
    shooting_dist_wayline.text = "5"
    ratio = SubElement(wayline_ext, "mis:ratio")
    ratio.text = "2"
    safe_dist = SubElement(wayline_ext, "mis:safeDistance")
    safe_dist.text = "2"
    
    # 添加无人机信息
    drone_info = SubElement(wayline_ext, "mis:droneInfo")
    drone_type = SubElement(drone_info, "mis:droneType")
    drone_type.text = "P4R"
    drone_height = SubElement(drone_info, "mis:droneHeight")
    use_abs = SubElement(drone_info, "mis:useAbsolute")
    use_abs.text = "true"
    has_takeoff = SubElement(drone_info, "mis:hasTakeoffHeight")
    has_takeoff.text = "false"
    takeoff_height = SubElement(drone_info, "mis:takeoffHeight")
    takeoff_height.text = "0.0"
    
    # 创建LineString
    line_string = SubElement(wayline, "LineString")
    tessellate = SubElement(line_string, "tessellate")
    tessellate.text = "1"
    line_alt_mode = SubElement(line_string, "altitudeMode")
    line_alt_mode.text = "absolute"
    line_coords = SubElement(line_string, "coordinates")
    
    # 生成连续的航线坐标
    line_coords.text = " ".join([f"{lon},{lat},{alt}" for lon, lat, alt in waypoint_coords])
    
    print(f"Generated {len(waypoints)} waypoints")
    
    return root

def generate_facade_waypoints(transformer, flight_direction, num_main_lines, distance, overlap):
    """
    基于外立面坐标系生成航点
    在外立面坐标系中，航点生成更加直观和准确
    """
    facade_points = transformer.facade_points
    waypoints_gps = []
    
    # 在外立面坐标系中确定边界
    x_primes = [p[0] for p in facade_points]
    y_primes = [p[1] for p in facade_points] 
    z_primes = [p[2] for p in facade_points]
    
    min_x_prime = min(x_primes)
    max_x_prime = max(x_primes)
    min_z_prime = min(z_primes)
    max_z_prime = max(z_primes)
    
    # RTK workflow: camera plane is parallel to facade, offset by PHOTO_DISTANCE
    # Flight plane = camera plane - PHOTO_DISTANCE (to facade) + FLIGHT_DISTANCE (from facade)
    avg_y_prime = sum(y_primes) / len(y_primes)
    safe_y_prime = avg_y_prime - PHOTO_DISTANCE + distance  # distance = FLIGHT_DISTANCE
    
    print(f"\n🎯 外立面坐标系航点生成:")
    print(f"  X'范围: {min_x_prime:.2f} ~ {max_x_prime:.2f}m (宽度: {max_x_prime-min_x_prime:.2f}m)")
    print(f"  Z'范围: {min_z_prime:.2f} ~ {max_z_prime:.2f}m (高度: {max_z_prime-min_z_prime:.2f}m)")
    print(f"  RTK offset: camera Y'={avg_y_prime:.2f}m, photo_dist={PHOTO_DISTANCE}m, flight_dist={distance}m")
    print(f"  Y'位置: {safe_y_prime:.2f}m (距离真实外立面 {distance}m)")
    
    # 计算航点间距
    step_size = distance * (1 - overlap)
    
    if flight_direction == "vertical":
        # 垂直航线：沿Z'方向飞行，X'方向分布
        width_span = max_x_prime - min_x_prime
        height_span = max_z_prime - min_z_prime
        
        num_z_points = int(height_span / step_size) + 1
        
        print(f"  垂直航线模式: {num_main_lines}条航线，每条{num_z_points}个点")
        
        for i in range(num_main_lines):
            # 计算当前航线的X'坐标
            x_ratio = i / (num_main_lines - 1) if num_main_lines > 1 else 0
            current_x_prime = min_x_prime + x_ratio * width_span
            
            # 生成该航线上的航点
            for j in range(num_z_points):
                if i % 2 == 0:
                    # 偶数航线：从下到上
                    z_ratio = j / (num_z_points - 1) if num_z_points > 1 else 0
                else:
                    # 奇数航线：从上到下
                    z_ratio = (num_z_points - 1 - j) / (num_z_points - 1) if num_z_points > 1 else 0
                
                current_z_prime = min_z_prime + z_ratio * height_span
                
                # 外立面坐标系中的航点
                facade_waypoint = (current_x_prime, safe_y_prime, current_z_prime)
                
                # 转换为GPS坐标
                gps_waypoint = transformer.facade_to_gps(facade_waypoint)
                waypoints_gps.append(gps_waypoint)
    
    else:
        # 水平航线：沿X'方向飞行，Z'方向分布
        width_span = max_x_prime - min_x_prime
        height_span = max_z_prime - min_z_prime
        
        num_x_points = int(width_span / step_size) + 1
        
        print(f"  水平航线模式: {num_main_lines}条航线，每条{num_x_points}个点")
        
        for i in range(num_main_lines):
            # 计算当前航线的Z'坐标
            z_ratio = i / (num_main_lines - 1) if num_main_lines > 1 else 0
            current_z_prime = min_z_prime + z_ratio * height_span
            
            # 生成该航线上的航点
            for j in range(num_x_points):
                if i % 2 == 0:
                    # 偶数航线：从左到右
                    x_ratio = j / (num_x_points - 1) if num_x_points > 1 else 0
                else:
                    # 奇数航线：从右到左
                    x_ratio = (num_x_points - 1 - j) / (num_x_points - 1) if num_x_points > 1 else 0
                
                current_x_prime = min_x_prime + x_ratio * width_span
                
                # 外立面坐标系中的航点
                facade_waypoint = (current_x_prime, safe_y_prime, current_z_prime)
                
                # 转换为GPS坐标
                gps_waypoint = transformer.facade_to_gps(facade_waypoint)
                waypoints_gps.append(gps_waypoint)
    
    print(f"  生成航点总数: {len(waypoints_gps)}")
    return waypoints_gps


def indent_xml(element, level=0):
    """Add indentation to XML elements"""
    i = "\n" + level * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = i + "  "
        if not element.tail or not element.tail.strip():
            element.tail = i
        for subelement in element:
            indent_xml(subelement, level + 1)
        if not element.tail or not element.tail.strip():
            element.tail = i
    else:
        if level and (not element.tail or not element.tail.strip()):
            element.tail = i


def save_kml_with_indent(root, filename):
    """Save KML file with indentation"""
    indent_xml(root)
    tree = ElementTree(root)
    tree.write(filename, encoding="utf-8", xml_declaration=True)


def main():
    """Main function - use configured photo paths"""
    print("Mavic 3T Flight Path Planner")
    print("="*40)
    print(f"Current parameter configuration:")
    print(f"  Photo distance (RTK prior): {PHOTO_DISTANCE}m")
    print(f"  Flight distance: {FLIGHT_DISTANCE}m")
    print(f"  Overlap rate: {OVERLAP_RATE*100:.0f}%")
    print(f"  Smart planning: {'Enabled' if ENABLE_SMART_PLANNING else 'Disabled'}")
    print(f"  Flight height: Read from image EXIF data")
    print(f"  Camera horizontal FOV: {CAMERA_HFOV}°")
    print(f"  Camera vertical FOV: {CAMERA_VFOV}°")
    print("="*40)
    
    # Check if photo files exist
    existing_photos = []
    for i, photo_path in enumerate(DEFAULT_PHOTOS, 1):
        if Path(photo_path).exists():
            existing_photos.append(photo_path)
            print(f"✓ Photo {i}: {photo_path}")
        else:
            print(f"✗ Photo {i}: {photo_path} (file not found)")
    
    if len(existing_photos) < 4:
        print(f"\nWarning: Only {len(existing_photos)} photos exist, need 4 photos to generate flight path")
        print("Please modify the photo path variables at the top of the script")
        return
    
    print(f"\nUsing {len(existing_photos)} photos to generate flight path...")
    
    # Generate KML file
    root = build_wpml_template(existing_photos, distance=FLIGHT_DISTANCE, overlap=OVERLAP_RATE)
    output_file = "facade_wpml.kml"
    save_kml_with_indent(root, output_file)
    print(f"✓ Saved DJI waypoint KML file: {output_file}")



if __name__ == "__main__":
    # Check if command line arguments are provided
    if len(sys.argv) > 1:
        # Use command line argument mode
        photo_paths = sys.argv[1:]
        if len(photo_paths) < 4:
            print("Error: Need at least 4 photo paths")
            print("Usage: python mavic3T_pp.py photo1.jpg photo2.jpg photo3.jpg photo4.jpg")
            sys.exit(1)
        
        print("Mavic 3T Flight Path Planner")
        print("="*40)
        print(f"Using command line arguments: {len(photo_paths)} photos")
        print(f"Smart planning: {'Enabled' if ENABLE_SMART_PLANNING else 'Disabled'}")
        
        # Check if files exist
        for i, photo_path in enumerate(photo_paths, 1):
            if Path(photo_path).exists():
                print(f"✓ Photo {i}: {photo_path}")
            else:
                print(f"✗ Photo {i}: {photo_path} (file not found)")
                sys.exit(1)
        
        # Generate KML file
        root = build_wpml_template(photo_paths, distance=FLIGHT_DISTANCE, overlap=OVERLAP_RATE)
        output_file = "facade_wpml.kml"
        save_kml_with_indent(root, output_file)
        print(f"✓ Saved DJI waypoint KML file: {output_file}")
    else:
        # Use default photo path mode
        main()

