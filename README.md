# DJI Mavic 3T 外立面航线规划工具

一个用于生成 DJI 兼容 KMZ 航线文件的 Python 工具，专为外立面摄影设计。通过拍摄建筑外立面的 4 个角点照片，自动生成完整覆盖的飞行路径。

## 功能特点

- 从 EXIF 元数据提取 GPS 坐标
- 通过 4 点平面拟合自动检测外立面朝向
- 生成蛇形航点路径，可配置重叠率
- 输出符合 DJI WPML 1.0.6 标准的 KMZ 文件
- 生成 Google Earth 预览文件
- 支持 RTK 高精度 GPS

## 系统要求

- Python 3.12
- DJI Mavic 3T (无人机类型 67)

## 安装

```bash
# 创建并激活虚拟环境
python3 -m venv drone_env
source drone_env/bin/activate

# 安装依赖
pip install -r requirements.txt
```

## 使用方法

### 从照片生成航线

```bash
# 使用 4 张外立面角点照片
python3 mavic3T_pp_kmz.py photo1.jpg photo2.jpg photo3.jpg photo4.jpg

# 或使用脚本中预设的 PHOTO_PATHS
python3 mavic3T_pp_kmz.py
```

### 输出文件

```
Facade Mission.kmz           # 上传到 DJI Pilot 2
├── wpmz/template.kml        # 编辑器显示 (EGM96 高度)
└── wpmz/waylines.wpml       # 飞行执行 (WGS84 高度)

Facade Mission_preview.kml   # 在 Google Earth 中查看
```

### 验证 KMZ 格式

```bash
python3 verify_kmz.py your_mission.kmz
```

### 高度转换工具

```bash
python3 height_converter.py <image.jpg>
```

## 配置参数

在 `mavic3T_pp_kmz.py` 中编辑参数：

```python
PHOTO_DISTANCE = 5.0    # 拍照时相机到外立面的距离 (米)
FLIGHT_DISTANCE = 5.0   # 期望的飞行距离 (米)
CAMERA_HFOV = 84.0      # 水平视场角 (度)
CAMERA_VFOV = 62.0      # 垂直视场角 (度)
OVERLAP_RATE = 0.65     # 照片重叠率 (0-1)
```

## 工作原理

### 工作流程

1. 在外立面 4 个角点处拍照（开启 RTK GPS）
2. 运行脚本，传入照片路径
3. 将生成的 `.kmz` 上传到 DJI Pilot 2
4. 用 `_preview.kml` 在 Google Earth 中预览航线

### 坐标转换流程

```
GPS (EXIF 中的 WGS84)
  → ENU 坐标 (东-北-天)
  → 外立面局部坐标 (通过平面拟合得到 X'/Y'/Z')
  → 航点网格 (蛇形路径 + 重叠)
  → DJI WPML 格式
```

### RTK 四点检测原理

相机位置定义了一个与实际外立面平行的平面：

```
相机平面 (RTK GPS)         外立面平面           飞行平面
        |                      |                      |
        |<-- PHOTO_DISTANCE -->|<-- FLIGHT_DISTANCE ->|
```

## 技术参考

### 高度标准

| 标准 | 说明 | 使用位置 |
|------|------|----------|
| WGS84 | 椭球高度 (GPS 原始值) | waylines.wpml (执行文件) |
| EGM96 | 正高/海拔高度 | template.kml (编辑器显示) |
| Absolute | WGS84 绝对高度 | Google Earth 预览 |

香港地区大地水准面差距：约 6.3m (WGS84 - EGM96)

### DJI WPML 1.0.6 要求

- 文件必须放在 KMZ 内的 `wpmz/` 子目录
- XML 命名空间：`http://www.dji.com/wpmz/1.0.6`
- 无人机枚举值：67 (Mavic 3T)
- 相机枚举值：52

### KMZ 文件结构

```
mission.kmz (ZIP 格式)
└── wpmz/
    ├── template.kml    # DJI Pilot 编辑器使用
    └── waylines.wpml   # 飞行执行使用
```

### 主要文件

| 文件 | 用途 |
|------|------|
| `mavic3T_pp_kmz.py` | 主程序，包含 FacadeTransformer 类 |
| `mavic3T_pp.py` | 扩展版本，带完整验证 |
| `height_converter.py` | WGS84 ↔ EGM96 高度转换工具 |
| `verify_kmz.py` | KMZ 格式验证工具 |

## 验证方法

### 检查 KMZ 结构

```bash
unzip -l mission.kmz
# 应该显示：
#   wpmz/template.kml
#   wpmz/waylines.wpml
```

### 在 DJI 平台验证

1. 上传 `.kmz` 到 DJI FlySafe 网站
2. 导入到 DJI Pilot 2 应用
3. 检查航点高度是否各不相同（不是全部相同）

### 在 Google Earth 中预览

打开 `*_preview.kml` 可查看：
- 航点位置及正确高度
- 飞行路径连线
- 外立面边界

## 常见问题

### DJI 网站拒绝 KMZ 文件

- 确保文件在 `wpmz/` 子目录内（不是根目录）
- 验证 WPML 版本为 1.0.6
- 检查无人机/相机枚举值

### Google Earth 显示航点贴地

- 使用 `_preview.kml` 文件（不是 template.kml）
- 预览文件使用 `altitudeMode=absolute`

### 所有航点显示相同高度

- 检查 template.kml 中 `useGlobalHeight` 是否设为 `0`
- 每个航点应使用各自的 `ellipsoidHeight`

## 许可证

MIT
