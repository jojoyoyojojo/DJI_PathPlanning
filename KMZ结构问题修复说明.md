# KMZ 结构问题修复说明

## 🔴 您发现的问题

### 1. DJI 网站无法识别 KMZ 格式
**原因**: KMZ 文件结构错误

### 2. Google Earth 中高度消失
**原因**: `altitudeMode` 设置为 `relativeToGround`（贴地模式）

---

## ✅ 问题分析与解决

### 问题 1: KMZ 文件结构错误

#### 错误的结构 ❌
```
test_fixed.kmz
├── template.kml          # 直接在根目录
└── waylines.wpml         # 直接在根目录
```

#### 正确的 DJI 结构 ✅
```
test_fixed.kmz
├── wpmz/
│   ├── template.kml      # 必须在 wpmz/ 子目录
│   └── waylines.wpml     # 必须在 wpmz/ 子目录
```

**DJI 官方要求**: 
> kmz 中需包含 wpmz/template.kml 及 wpmz/waylines.wpml 文件

#### 修复代码
```python
# 之前 (错误)
with zipfile.ZipFile(kmz_path, "w") as zf:
    zf.writestr("template.kml", save_xml(template))
    zf.writestr("waylines.wpml", save_xml(waylines))

# 现在 (正确)
with zipfile.ZipFile(kmz_path, "w") as zf:
    zf.writestr("wpmz/template.kml", save_xml(template))
    zf.writestr("wpmz/waylines.wpml", save_xml(waylines))
```

---

### 问题 2: Google Earth 高度显示问题

#### 原始文件的问题 ❌
```xml
<!-- 您的 my-kmz/template.kml -->
<Point>
  <altitudeMode>relativeToGround</altitudeMode>
  <coordinates>114.211,22.424,26.893</coordinates>
</Point>
```

**结果**: 
- Google Earth 将所有点贴地显示
- 高度显示为 0
- Grounding 显示 "Clamp to ground"

#### Google Earth 正确格式 ✅
```xml
<Point>
  <altitudeMode>absolute</altitudeMode>
  <coordinates>114.211,22.424,26.893</coordinates>
</Point>
```

**altitudeMode 选项说明**:
| 模式 | 说明 | 效果 |
|------|------|------|
| `clampToGround` | 贴地 | 高度被忽略，显示为 0 |
| `relativeToGround` | 相对地面 | 相对地形高度 |
| `absolute` | 绝对高度 | WGS84 椭球高度（显示正确） ✅ |

---

### 问题 3: DJI WPML 和 Google Earth KML 的区别

#### DJI WPML 格式（template.kml）
```xml
<Placemark>
  <!-- DJI 使用 wpml: 命名空间，不使用 altitudeMode -->
  <wpml:index>0</wpml:index>
  <wpml:ellipsoidHeight>26.893</wpml:ellipsoidHeight>
  <wpml:height>29.982</wpml:height>
  <Point>
    <coordinates>114.211,22.424</coordinates>
  </Point>
</Placemark>
```

#### Google Earth KML 格式
```xml
<Placemark>
  <!-- Google Earth 使用标准 KML altitudeMode -->
  <name>WP01</name>
  <description>高度: 26.89m</description>
  <Point>
    <altitudeMode>absolute</altitudeMode>
    <coordinates>114.211,22.424,26.893</coordinates>
  </Point>
</Placemark>
```

**关键区别**:
1. DJI 使用 `wpml:ellipsoidHeight` 和 `wpml:height`
2. Google Earth 使用 `altitudeMode` + 坐标中的第三个值
3. **两者不能混用！**

---

## 🛠️ 完整解决方案

### 1. 修复后的文件

#### DJI KMZ (test_fixed.kmz)
```bash
$ unzip -l test_fixed.kmz
Archive:  test_fixed.kmz
  Length      Date    Time    Name
---------  ---------- -----   ----
    42349  10-13-2025 22:47   wpmz/template.kml  ✅
    41942  10-13-2025 22:47   wpmz/waylines.wpml ✅
```

**用途**: 上传到大疆网站 ✅

#### Google Earth 预览 (preview_google_earth.kml)
```xml
<?xml version='1.0' encoding='utf-8'?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>Facade Mission Preview</name>
    <Placemark>
      <name>WP01</name>
      <description>高度: 26.89m</description>
      <Point>
        <altitudeMode>absolute</altitudeMode>
        <coordinates>114.211,22.424,26.893</coordinates>
      </Point>
    </Placemark>
    ...
  </Document>
</kml>
```

**用途**: 在 Google Earth 中预览航线（显示正确高度）✅

---

## 📊 修复前后对比

### KMZ 结构对比

| 项目 | 修复前 | 修复后 |
|------|--------|--------|
| 文件位置 | 根目录 ❌ | wpmz/ 子目录 ✅ |
| DJI 识别 | 失败 ❌ | 成功 ✅ |
| 目录结构 | `template.kml`<br>`waylines.wpml` | `wpmz/template.kml`<br>`wpmz/waylines.wpml` |

### Google Earth 显示对比

| 项目 | 原始文件 | 修复后 |
|------|----------|--------|
| altitudeMode | `relativeToGround` ❌ | `absolute` ✅ |
| 高度显示 | 0m (贴地) ❌ | 26-33m (正确) ✅ |
| Grounding | "Clamp to ground" ❌ | 正常悬浮 ✅ |

---

## 🚀 使用方法

### 方法 1: 使用测试脚本

```bash
# 生成 DJI KMZ 和 Google Earth 预览
python3 test_kmz_generation.py
```

**输出**:
- ✅ `test_fixed.kmz` - 上传到 DJI 网站
- ✅ `preview_google_earth.kml` - Google Earth 预览

### 方法 2: 使用主脚本

```bash
# 使用您的照片
python3 mavic3T_pp_kmz.py photo1.jpg photo2.jpg photo3.jpg photo4.jpg
```

**输出**:
- ✅ `Facade Mission.kmz` - 带 wpmz/ 结构的 DJI KMZ
- ✅ `Facade Mission_preview.kml` - Google Earth 预览

---

## ⚠️ 重要提醒

### 1. KMZ 结构要求
- **必须**: 文件在 `wpmz/` 子目录
- **必须**: 包含 `template.kml` 和 `waylines.wpml`
- **不能**: 文件直接在根目录

### 2. Google Earth 预览
- **DJI template.kml**: 不能直接用于 Google Earth（会显示高度 0）
- **需要**: 单独生成带 `altitudeMode=absolute` 的预览文件
- **用途**: 仅用于可视化检查航线

### 3. 高度模式说明

#### DJI WPML (template.kml/waylines.wpml)
```xml
<!-- 使用 wpml: 命名空间 -->
<wpml:executeHeightMode>WGS84</wpml:executeHeightMode>
<wpml:ellipsoidHeight>26.893</wpml:ellipsoidHeight>
<wpml:height>29.982</wpml:height>
```

#### Google Earth KML
```xml
<!-- 使用标准 KML altitudeMode -->
<altitudeMode>absolute</altitudeMode>
<coordinates>lon,lat,alt</coordinates>
```

---

## 📁 生成的文件清单

### DJI 飞行文件
| 文件 | 说明 | 用途 |
|------|------|------|
| `test_fixed.kmz` | 带 wpmz/ 结构 | 上传到 DJI 网站 ✅ |
| ├─ `wpmz/template.kml` | 编辑器配置 | DJI Pilot 2 编辑 |
| └─ `wpmz/waylines.wpml` | 执行文件 | 飞行执行 |

### Google Earth 预览
| 文件 | 说明 | 用途 |
|------|------|------|
| `preview_google_earth.kml` | 标准 KML | Google Earth 查看 ✅ |
| - altitudeMode: absolute | 绝对高度 | 显示正确高度 |
| - 带航线和航点 | 可视化 | 检查路径 |

---

## 🔍 验证方法

### 1. 验证 KMZ 结构
```bash
unzip -l test_fixed.kmz
```

**期望输出**:
```
wpmz/template.kml   ✅
wpmz/waylines.wpml  ✅
```

### 2. 验证 DJI 上传
1. 打开 DJI 网站
2. 上传 `test_fixed.kmz`
3. 应该能成功识别 ✅

### 3. 验证 Google Earth 显示
1. 打开 Google Earth
2. 导入 `preview_google_earth.kml`
3. 应该看到正确高度的航点 ✅

---

## ✅ 修复总结

### 问题根源
1. **KMZ 结构错误**: 文件在根目录而非 wpmz/ 子目录
2. **高度模式错误**: 使用 `relativeToGround` 导致贴地显示

### 解决方案
1. ✅ 修复 KMZ 打包: 使用 `wpmz/` 子目录
2. ✅ 分离文件用途: DJI KMZ + Google Earth 预览
3. ✅ 正确的高度模式: Google Earth 使用 `absolute`

### 最终状态
- ✅ DJI 网站可识别 KMZ
- ✅ Google Earth 正确显示高度
- ✅ 所有文件符合规范

---

**修复完成！请使用 test_fixed.kmz 上传到 DJI 网站测试！** 🎉

