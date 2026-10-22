# Lichtblick 1.29.1 监看布局字段依据

准备日期：2026-10-13。布局文件为 [monitor_layout.json](monitor_layout.json)，基于本机 Lichtblick GUI 导出的 [真实默认布局](../验证记录/Lichtblick安装_2026-10-13/default_layout_export.json) 修改。保留原导出的 `3D!18i6zy7`、`Image!3mnp456`、`RawMessagesVirtual!os6rgs` 三个面板 ID，以及原有 grid、camera 基础结构和 layout 分割树格式。采用客户端正常导入功能，未直接改写AppData或应用数据库。**已在Windows客户端GUI成功导入**。

## 已核对的配置字段

字段以官方仓库的 **v1.29.1** 标签源码为依据，源码行号采用 GitHub 的一基行号：

| 配置位置 | 本布局值 | 依据与含义 |
|---|---|---|
| 3D 面板顶层 `followTf` | `map` | `IRenderer.ts:113–119` 为渲染/显示坐标系字段；`CameraStateSettings.ts:286–291` 明确把界面的 general/followTf 写入顶层，而非保存成 general 子对象 |
| 顶层 `followMode` | `follow-none` | `IRenderer.ts:81` 的合法枚举；`CameraStateSettings.ts:222–225` 将 Fixed 对应到该值 |
| `cameraState.perspective` | false | 正交模式；`CameraStateSettings.ts:512–524` 将相机放在 +Z、沿Z旋转，实现二维俯视 |
| `cameraState.phi` / `thetaOffset` | 0 / 0 | phi、theta都是度制，`:490–491` 转弧度；`:513–517` 在二维模式锁定polar角并设置正交相机。phi=0配合正交模式，未沿用默认60°倾角 |
| `cameraState.distance` | 8 | 正交垂直显示跨度，`:518–521` 使用distance/2计算边界；是显示缩放，不是机体距离命令 |
| `topics[话题].visible` | 地图、scan、两路径、Marker、位姿=true；两障碍点云=false | `settings.ts:16–22` 的 BaseSettings，以及 `IRenderer.ts:178–183` 的 topics/layers partial结构；只显式启用已有观测数据 |
| `scene.transforms` | editable=false、showLabel=true、axisScale=0.5 | `IRenderer.ts:139–155`；仅改变本地显示 |
| `transforms[frame:名称].visible` | map/odom/base_link/laser_link=true | `FrameAxes.ts:209–212` 实际索引带 `frame:` 前缀，不猜测裸frame名称键 |
| Image 面板 `imageMode.imageTopic` | `/monitor/camera/image/compressed` | `IRenderer.ts:93–111` 的 ImageModeConfig；本版本 Image 复用3D renderer的配置结构，没有另猜旧版cameraTopic字段 |
| Raw 面板 `topicPath` | `/mavros/state` | 真实默认GUI导出已有该键；沿用全部Raw默认配置，只替换为空的话题路径 |

官方源码链接：[IRenderer.ts](https://raw.githubusercontent.com/Lichtblick-Suite/lichtblick/v1.29.1/packages/suite-base/src/panels/ThreeDeeRender/IRenderer.ts)、[CameraStateSettings.ts](https://raw.githubusercontent.com/Lichtblick-Suite/lichtblick/v1.29.1/packages/suite-base/src/panels/ThreeDeeRender/renderables/CameraStateSettings.ts)、[settings.ts](https://raw.githubusercontent.com/Lichtblick-Suite/lichtblick/v1.29.1/packages/suite-base/src/panels/ThreeDeeRender/settings.ts)、[FrameAxes.ts](https://raw.githubusercontent.com/Lichtblick-Suite/lichtblick/v1.29.1/packages/suite-base/src/panels/ThreeDeeRender/renderables/FrameAxes.ts)。

## 面板与只读约束

左侧65%为地图/scan/TF/路径的正交俯视3D面板；右侧35%上下为一路JPEG图像（55%）和MAVROS完整state消息（45%）。面板引用全部来自实际导出，未新增Publish、Teleop、服务或参数控制面板，`userNodes`为空。

3D与Image默认导出都包含非可选的 `publish` 配置结构（`IRenderer.ts:162–176`）。为了保持兼容，本布局保留结构，但把pose、point、poseEstimate三个目的话题分别改成 `/monitor/disabled/pose`、`/monitor/disabled/point`、`/monitor/disabled/pose_estimate`。这些名字不在桥白名单内；配套桥又设置 `capabilities=[]`、`client_topic_whitelist=[]`。这不声称移除了客户端内置发布按钮；真正只读限制由桥配置承担，桥握手和拒绝行为尚未实测。

图像默认只选preview JPEG，与 `bridge_readonly.launch` 的 `jpeg_source=preview`、`camera_preview.launch` 的压缩出口一致；未选择mission/SSD第二路。相机原图/CameraInfo不在桥白名单，本布局不强行要求它们。map缺失或TF未连接时3D可能显示frame不存在；相机尚未启动时Image为空。这些是当前未接ROS时的预期状态，不等于已经收到数据。

## 本次静态核验与后续验收

[layout_validation_2026-10-13.json](layout_validation_2026-10-13.json) 记录本次JSON解析、三面板叶子引用、无重复ID、仅显示面板、话题逐项匹配preview桥白名单、保留publish话题被排除及空capabilities检查。布局SHA256：`ab4763338748b5c2dd651c2c9d04899cced56302f1c5f9c429617fec8ad601ac`。

上述JSON保留生成时的静态检查快照，其中 `guiImportValidated=false` 指导入前阶段。随后已通过GUI导入，见[客户端GUI核验记录](../验证记录/Lichtblick安装_2026-10-13/gui_verification_2026-10-13.json)：Layouts中出现monitor_layout；左侧为俯视网格，右上为preview JPEG，右下为MAVROS state。客户端显示No data source，图像提示话题不存在，state等待消息。Display frame=map/Follow mode=Fixed由配置与字段核验支持，没有另行打开设置控件验证。

当前ROS连接、真实地图/TF/视频刷新和延迟均未验收；SHELL确认Nano正在返修，后续按[接续步骤](../项目资料/Lichtblick安装与回传接续_2026-10-13.md)自行完成。布局导入不启动Nano数据源，不能为填充空图自行启动飞行任务。
