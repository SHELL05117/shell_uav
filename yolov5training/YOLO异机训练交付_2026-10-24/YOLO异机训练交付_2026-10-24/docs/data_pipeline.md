# 数据准备记录：VOC → YOLO、核查与划分

原包 `data/TargetDelection1.1.zip`（866,460,635 字节，SHA256 `6a9d4537ce546ae5bb0c429f565edaf3f0e44c6e0bd628b7faf5a061c9666b41`）只作只读来源。
派生数据在本机 `work/dataset/`，不覆盖原图与 XML，不把原包写进任何仓库。

## 1. 全量核查（180/180）

脚本 `scripts/01_extract_audit.py`，报告 `reports/audit_pre_conversion.json`。

| 检查 | 结果 |
|---|---|
| ZIP 条目 | 362 = 2 目录 + 180 JPG + 180 XML；解压声明 876,192,894 字节 |
| 完整像素解码 | 180/180 成功（PIL `Image.load()` 全解码，不是只看文件头） |
| EXIF 方向 | 180 张全部为 1（无旋转需要补偿） |
| XML↔JPEG 配对 | 全部配对，`<filename>`、`<width>/<height>` 与 JPEG 一致 |
| 目标数量 | 每张恰好 1 个 `<object>`，共 180 框，无空标注 |
| 类别 | 全部属于固定六类；人员/有毒/易燃/易爆/干扰1/干扰2 各 30 |
| 框几何 | 全部整数坐标、`xmax>xmin`、`ymax>ymin`、在图像范围内 |
| 完全重复图片 | 无（SHA256 唯一） |
| 近似重复 | dHash（16×16，256 bit）汉明距离 ≤12 的图片对为 0 |
| 尺寸分布 | 4284×5712：155；5712×4284：17；3024×4032：8 |
| 拍摄时间 | EXIF 中无 `DateTimeOriginal`（元数据被剥离） |
| 文件名 | IMG_2682–IMG_2863 连续，缺 2772、2819 两帧；每类恰好 30 张连续块 |

## 2. 坐标约定（证据与处理）

- XML 具备 labelImg 的 Pascal VOC 输出指纹：根节点 `verified="yes"`、`<folder>JPEGImages</folder>`、相对 `<path>../JPEGImages/xxx.jpg</path>`、每对象 `pose/truncated/difficult`。
- 坐标全部为整数，`xmin/ymin ≥ 0`，`xmax/ymax ≤ W/H`，没有任何框贴到 0 或等于 W/H 的边界。
- labelImg 保存的就是它内部的 0 基像素位置，因此**不做统一减 1**；按 0 基连续边界换算：
  `xc=(x1+x2)/(2W)`，`yc=(y1+y2)/(2H)`，`w=(x2-x1)/W`，`h=(y2-y1)/H`。
- 1 像素包含/排他口径的差异在 4284 px 宽度上对归一化值影响 < 0.0003，包内几何证据无法进一步分辨；该不确定性已写入 `split_manifest.json` 的 `coordinate_convention`。

## 3. 回画审查

- 6 张类别拼版图（每类 30 图）：`reports/overlays/0_person_marker_sheet.jpg` … `5_distractor_2_sheet.jpg`。
- 每类 1 张原图放大对照：`reports/overlays/zoom_review_6.jpg`。
- 结论：180 个框与图案对齐良好，框紧贴三角形/人头图案，未见类别错配；个别框把三角形尖角裁掉几个像素，属标注尺度的正常误差，未修改原标注。
- 场地背景里还有**未标注的编号红框方块**（1/2/4/7 等）。它们不是六类图案，按背景保留；这也解释了为什么部分图片里模型会出现低置信度的“候选框”。

## 4. 六类固定映射

| YOLO ID | VOC 原类名 | 机器名 | 业务语义 | 训练集/验证集/测试集 |
|---:|---|---|---|---|
| 0 | 人员 | person_marker | person，允许任务 | 18 / 6 / 6 |
| 1 | 识别项-有毒 | hazard_toxic | hazard | 18 / 6 / 6 |
| 2 | 识别项-易燃 | hazard_flammable | hazard | 18 / 6 / 6 |
| 3 | 识别项-易爆 | hazard_explosive | hazard | 18 / 6 / 6 |
| 4 | 干扰项1 | distractor_1 | 忽略，禁止触发 | 18 / 6 / 6 |
| 5 | 干扰项2 | distractor_2 | 忽略，禁止触发 | 18 / 6 / 6 |

不添加背景类（`nc: 6`，`background_class: false`）。

## 5. 划分策略与限制

- 脚本 `scripts/02_build_dataset.py`，清单 `reports/split_manifest.json`（含每张图与 XML 的 SHA256、类别、划分）。
- 事实：每类 30 张是**一整段连续采集**（文件名连续、无时间戳、无 EXIF 拍摄时间），六类各自只有一个采集块。若要“采集组完全独立”，只能把整类分到某一侧，无法保证每类都出现在训练/验证/测试里。
- 因此采用**类内按采集顺序**的 18/6/6 划分：每类前 18 张训练、接着 6 张验证、最后 6 张测试。它降低了“同一时刻的连续帧跨集合”的风险，但**不是采集组独立划分**，`split_manifest.json` 明确写了 `group_independent: false`。
- 因此本轮全部指标只能算**试验结果**，不能当作飞行验收；真实泛化需要补采（不同场地/高度/机载相机/空场景/多目标）。

## 6. 标定集（量化校准）

- 脚本 `scripts/03_make_calib.py`，输出 `work/dataset/calib/`：48 张（每类 8 张），**只来自训练集**。
- 校准图先按 YOLOv5 的 letterbox 规则（640×640，灰 114 填充）生成，与推理时的预处理一致。
- 测试集与验证集不参与校准。
