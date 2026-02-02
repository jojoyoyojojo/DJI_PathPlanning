#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DJI 无人机高度转换工具
用于理解和转换 GPS 椭球高度到 DJI 的 ASL (平均海平面) 高度

高度类型说明：
1. GPS椭球高度 (Ellipsoidal Height) - EXIF中存储的值
   - 相对于WGS84参考椭球面的高度
   - GPS/GNSS直接测量值
   
2. 正高/海拔高度 (Orthometric Height / MSL / ASL)
   - 相对于大地水准面（平均海平面）
   - 日常使用的"海拔"概念
   - DJI航点规划中的"绝对高度"
   
3. 相对高度 (Relative Height / AGL)
   - 相对于起飞点的高度
   - Above Ground Level

转换公式：
  MSL高度 = GPS椭球高度 - 大地水准面差距(Geoid Separation)
"""

import exifread
import pyproj
from typing import Tuple, Optional


def get_geoid_separation(lat: float, lon: float) -> float:
    """
    使用pyproj库计算指定位置的大地水准面差距（N值）
    
    参数：
        lat: 纬度（度）
        lon: 经度（度）
    
    返回：
        大地水准面差距（米），正值表示椭球面在大地水准面之上
    """
    try:
        # 创建WGS84椭球高度到EGM2008正高的转换器
        transformer = pyproj.Transformer.from_crs(
            "EPSG:4979",  # WGS84 3D (lat, lon, ellipsoidal height)
            "EPSG:3855",  # EGM2008 height
            always_xy=False
        )
        
        # 使用0米椭球高度来获取geoid separation
        lat_out, lon_out, geoid_height = transformer.transform(lat, lon, 0.0)
        
        # geoid separation = ellipsoidal_height - orthometric_height
        # 因为我们输入0米椭球高度，所以geoid_separation = 0 - geoid_height
        geoid_separation = -geoid_height
        
        return geoid_separation
    except Exception as e:
        print(f"⚠️  无法计算精确的geoid separation: {e}")
        print(f"使用香港地区的默认值: 6.5米")
        return 6.5  # 香港地区的典型值


def read_image_gps(image_path: str) -> Tuple[float, float, float]:
    """
    从图像EXIF读取GPS坐标和椭球高度
    
    返回：
        (纬度, 经度, GPS椭球高度)
    """
    with open(image_path, 'rb') as f:
        tags = exifread.process_file(f, details=False)
    
    # 解析纬度
    lat_values = tags["GPS GPSLatitude"].values
    lat_ref = tags["GPS GPSLatitudeRef"].printable
    lat = lat_values[0].num / lat_values[0].den + \
          (lat_values[1].num / lat_values[1].den) / 60 + \
          (lat_values[2].num / lat_values[2].den) / 3600
    if lat_ref in ("S", "W"):
        lat = -lat
    
    # 解析经度
    lon_values = tags["GPS GPSLongitude"].values
    lon_ref = tags["GPS GPSLongitudeRef"].printable
    lon = lon_values[0].num / lon_values[0].den + \
          (lon_values[1].num / lon_values[1].den) / 60 + \
          (lon_values[2].num / lon_values[2].den) / 3600
    if lon_ref in ("S", "W"):
        lon = -lon
    
    # 解析高度
    alt_tag = tags.get("GPS GPSAltitude")
    if alt_tag:
        alt = float(alt_tag.values[0].num) / float(alt_tag.values[0].den)
        alt_ref = tags.get("GPS GPSAltitudeRef")
        if alt_ref and int(alt_ref.values[0]) == 1:
            alt = -alt
    else:
        alt = 0.0
    
    return lat, lon, alt


def convert_gps_to_msl(lat: float, lon: float, ellipsoidal_height: float) -> float:
    """
    将GPS椭球高度转换为MSL（平均海平面）高度
    
    参数：
        lat: 纬度（度）
        lon: 经度（度）
        ellipsoidal_height: GPS椭球高度（米）
    
    返回：
        MSL高度（米）- DJI的"绝对高度"
    """
    geoid_sep = get_geoid_separation(lat, lon)
    msl_height = ellipsoidal_height - geoid_sep
    return msl_height


def analyze_image_height(image_path: str, verbose: bool = True) -> dict:
    """
    分析图像的高度信息并进行转换
    
    返回：
        包含各种高度信息的字典
    """
    import os
    filename = os.path.basename(image_path)
    
    # 读取GPS数据
    lat, lon, ellipsoidal_height = read_image_gps(image_path)
    
    # 计算geoid separation
    geoid_sep = get_geoid_separation(lat, lon)
    
    # 转换为MSL高度
    msl_height = ellipsoidal_height - geoid_sep
    
    result = {
        'filename': filename,
        'latitude': lat,
        'longitude': lon,
        'ellipsoidal_height': ellipsoidal_height,
        'geoid_separation': geoid_sep,
        'msl_height': msl_height,
    }
    
    if verbose:
        print(f"\n{'='*70}")
        print(f"文件: {filename}")
        print(f"{'='*70}")
        print(f"位置: {lat:.6f}°N, {lon:.6f}°E")
        print(f"\n高度信息:")
        print(f"  GPS椭球高度 (EXIF原始值):  {ellipsoidal_height:>8.2f} 米")
        print(f"  大地水准面差距 (N):        {geoid_sep:>8.2f} 米")
        print(f"  {'─'*50}")
        print(f"  MSL高度 (DJI绝对高度):     {msl_height:>8.2f} 米 ✓")
        print(f"\n💡 这个 MSL 高度({msl_height:.2f}m)就是 DJI Pilot 中显示的\"绝对高度\"")
    
    return result


if __name__ == "__main__":
    import sys
    
    # 测试图像
    test_images = [
        '/Users/andyliu/Downloads/hkstp_test/11/DJI_0109.JPG',
        '/Users/andyliu/Downloads/hkstp_test/11/DJI_0110.JPG',
        '/Users/andyliu/Downloads/hkstp_test/11/DJI_0111.JPG',
        '/Users/andyliu/Downloads/hkstp_test/11/DJI_0112.JPG',
    ]
    
    print("\n" + "="*70)
    print("DJI 无人机高度转换分析")
    print("="*70)
    
    results = []
    for img_path in test_images:
        result = analyze_image_height(img_path, verbose=True)
        results.append(result)
    
    print("\n" + "="*70)
    print("汇总表格")
    print("="*70)
    print(f"{'文件':<20} {'GPS高度':>10} {'Geoid分离':>12} {'MSL高度':>10}")
    print("─"*70)
    for r in results:
        print(f"{r['filename']:<20} {r['ellipsoidal_height']:>9.2f}m  "
              f"{r['geoid_separation']:>11.2f}m  {r['msl_height']:>9.2f}m")
    
    print("\n" + "="*70)
    print("关键要点")
    print("="*70)
    print("""
✓ EXIF中的GPS高度是 WGS84椭球高度
✓ DJI Pilot中的"绝对高度"是 MSL高度（正高）
✓ 转换公式: MSL = GPS高度 - Geoid分离值
✓ 在香港地区，Geoid分离值约为 6-7米

在您的航点规划程序中：
- 如果使用 heightMode="WGS84"，直接使用GPS椭球高度（EXIF原始值）
- 如果使用 heightMode="EGM96"，应该使用MSL高度（转换后的值）
- 如果使用 heightMode="relativeToStartPoint"，需要计算相对高度
    """)

