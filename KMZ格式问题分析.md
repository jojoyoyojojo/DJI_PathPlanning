# DJI KMZ 格式问题分析与修复

## 问题概述

您生成的 KMZ 文件无法上传到大疆网站，主要原因是格式不符合大疆 WPML 1.0.6 规范。

## 主要问题清单

### 1. **WPML 版本过旧** ❌
- **您的代码**: `xmlns:wpml="http://www.dji.com/wpmz/1.0.2"`
- **官方要求**: `xmlns:wpml="http://www.dji.com/wpmz/1.0.6"`
- **影响**: 大疆网站可能拒绝旧版本的格式

### 2. **template.kml 结构不完整** ❌

#### 缺少必需的元数据字段:
```xml
<!-- 您缺少的字段 -->
<wpml:author>作者信息</wpml:author>
<wpml:createTime>创建时间戳</wpml:createTime>
<wpml:updateTime>更新时间戳</wpml:updateTime>
```

#### missionConfig 不完整:
- ✅ 有: `flyToWaylineMode`, `finishAction`, `exitOnRCLost`
- ❌ 缺少: `takeOffRefPoint`, `takeOffRefPointAGLHeight`, `takeOffSecurityHeight`, `globalTransitionalSpeed`, `globalRTHHeight`
- ❌ 错误: `droneInfo` 应使用枚举值而非字符串

**您的代码:**
```xml
<wpml:droneInfo>
  <wpml:droneType>M3T</wpml:droneType>
  <wpml:useAbsolute>true</wpml:useAbsolute>
</wpml:droneInfo>
```

**正确格式:**
```xml
<wpml:droneInfo>
  <wpml:droneEnumValue>67</wpml:droneEnumValue>
  <wpml:droneSubEnumValue>1</wpml:droneSubEnumValue>
</wpml:droneInfo>
```

#### Folder 结构不符合规范:
- ❌ 缺少: `wpml:templateType`, `wpml:waylineCoordinateSysParam`, `wpml:globalHeight`
- ❌ 缺少: `wpml:globalWaypointHeadingParam`, `wpml:globalWaypointTurnMode`

#### Placemark 结构问题:
您使用了旧的 `mis:` 命名空间:
```xml
<ExtendedData xmlns:mis="www.dji.com">
  <mis:useWaylineAltitude>false</mis:useWaylineAltitude>
  <mis:turnMode>Auto</mis:turnMode>
  <mis:heading>0</mis:heading>
  <mis:gimbalPitch>0</mis:gimbalPitch>
  <mis:actions>ShootPhoto</mis:actions>
</ExtendedData>
```

**应该使用 `wpml:` 元素:**
```xml
<wpml:index>0</wpml:index>
<wpml:ellipsoidHeight>82.7367740002872</wpml:ellipsoidHeight>
<wpml:height>86.226421139</wpml:height>
<wpml:waypointSpeed>10</wpml:waypointSpeed>
<wpml:waypointHeadingParam>...</wpml:waypointHeadingParam>
<wpml:waypointTurnParam>...</wpml:waypointTurnParam>
<wpml:useGlobalHeight>1</wpml:useGlobalHeight>
<wpml:actionGroup>...</wpml:actionGroup>
```

### 3. **waylines.wpml 结构错误** ❌

#### 您使用了错误的结构:
```xml
<wpml:waylineSet>
  <wpml:wayline>
    <wpml:headingMode>UsePointSetting</wpml:headingMode>
    <wpml:executeHeightMode>WGS84</wpml:executeHeightMode>
    <wpml:points>
      <wpml:point>...</wpml:point>
    </wpml:points>
  </wpml:wayline>
</wpml:waylineSet>
```

**正确的结构应该是:**
```xml
<Folder>
  <wpml:templateId>0</wpml:templateId>
  <wpml:executeHeightMode>WGS84</wpml:executeHeightMode>
  <wpml:waylineId>0</wpml:waylineId>
  <wpml:distance>0</wpml:distance>
  <wpml:duration>0</wpml:duration>
  <wpml:autoFlightSpeed>10</wpml:autoFlightSpeed>
  <Placemark>
    <Point>
      <coordinates>lon,lat</coordinates>
    </Point>
    <wpml:index>0</wpml:index>
    <wpml:executeHeight>82.7367740002872</wpml:executeHeight>
    <wpml:waypointSpeed>10</wpml:waypointSpeed>
    ...
  </Placemark>
</Folder>
```

### 4. **Actions 动作结构错误** ❌

#### 您的简化版本:
```xml
<wpml:actions>
  <wpml:action>
    <wpml:actionType>ShootPhoto</wpml:actionType>
  </wpml:action>
</wpml:actions>
```

**正确的 actionGroup 结构:**
```xml
<wpml:actionGroup>
  <wpml:actionGroupId>0</wpml:actionGroupId>
  <wpml:actionGroupStartIndex>0</wpml:actionGroupStartIndex>
  <wpml:actionGroupEndIndex>0</wpml:actionGroupEndIndex>
  <wpml:actionGroupMode>sequence</wpml:actionGroupMode>
  <wpml:actionTrigger>
    <wpml:actionTriggerType>reachPoint</wpml:actionTriggerType>
  </wpml:actionTrigger>
  <wpml:action>
    <wpml:actionId>0</wpml:actionId>
    <wpml:actionActuatorFunc>rotateYaw</wpml:actionActuatorFunc>
    <wpml:actionActuatorFuncParam>
      <wpml:aircraftHeading>0</wpml:aircraftHeading>
      <wpml:aircraftPathMode>counterClockwise</wpml:aircraftPathMode>
    </wpml:actionActuatorFuncParam>
  </wpml:action>
  <wpml:action>
    <wpml:actionId>1</wpml:actionId>
    <wpml:actionActuatorFunc>gimbalRotate</wpml:actionActuatorFunc>
    <wpml:actionActuatorFuncParam>
      <wpml:gimbalHeadingYawBase>north</wpml:gimbalHeadingYawBase>
      <wpml:gimbalRotateMode>absoluteAngle</wpml:gimbalRotateMode>
      <wpml:gimbalPitchRotateEnable>1</wpml:gimbalPitchRotateEnable>
      <wpml:gimbalPitchRotateAngle>0</wpml:gimbalPitchRotateAngle>
      ...
    </wpml:actionActuatorFuncParam>
  </wpml:action>
</wpml:actionGroup>
```

### 5. **缺少必需字段** ❌

#### template.kml 缺少:
- `wpml:waypointGimbalHeadingParam` - 每个航点的云台航向参数
- `wpml:isRisky` - 风险标记
- `wpml:payloadParam` - 负载参数（在 Folder 级别）

#### waylines.wpml 缺少:
- `wpml:waypointHeadingAngleEnable` - 航向角启用标记
- `wpml:waypointGimbalHeadingParam` - 云台航向参数  
- `wpml:waypointWorkType` - 航点工作类型

## 修复内容

我已经修复了您的 `mavic3T_pp_kmz.py` 代码，主要改动包括:

### 1. 更新 WPML 版本到 1.0.6
```python
root.set("xmlns:wpml","http://www.dji.com/wpmz/1.0.6")
```

### 2. 完善 template.kml 结构
- 添加 `wpml:author`, `wpml:createTime`, `wpml:updateTime`
- 使用枚举值配置无人机和相机
- 添加完整的 Folder 参数
- 使用 `wpml:` 元素替代 `mis:` ExtendedData
- 实现完整的 actionGroup 结构

### 3. 修正 waylines.wpml 结构
- 使用 Folder + Placemark 结构替代 waylineSet
- 添加所有必需字段
- 实现正确的 actionGroup

### 4. 修复 KMZ 打包
- 移除了错误的 wpmz 子目录
- 直接在根目录存放 template.kml 和 waylines.wpml

## 关键参数说明

### 无人机和相机枚举值:
- **M3T 无人机**: `droneEnumValue=67`, `droneSubEnumValue=1`
- **M3T 相机**: `payloadEnumValue=52`, `payloadSubEnumValue=0`

### 高度模式:
- **execution (waylines.wpml)**: `WGS84` 或 `relativeToStartPoint`
- **editor (template.kml)**: `EGM96` 或 `relativeToStartPoint`

## 下一步操作

1. ✅ 代码已修复完成
2. ⏳ 运行脚本生成新的 KMZ 文件
3. ⏳ 上传到大疆网站验证

## 参考文档

- [DJI WPML template.kml 规范](https://developer.dji.com/doc/cloud-api-tutorial/cn/api-reference/dji-wpml/template-kml.html)
- [DJI WPML waylines.wpml 规范](https://developer.dji.com/doc/cloud-api-tutorial/cn/api-reference/dji-wpml/waylines-wpml.html)

## 注意事项

1. **枚举值**: 必须使用正确的设备枚举值，不能使用字符串名称
2. **高度字段**: template.kml 需要同时提供 `ellipsoidHeight` 和 `height`
3. **动作结构**: 必须使用 actionGroup，不能用简化的 actions
4. **命名空间**: 统一使用 `wpml:` 前缀，避免混用 `mis:`
5. **文件结构**: KMZ 中文件应在根目录，不要放在子目录中

