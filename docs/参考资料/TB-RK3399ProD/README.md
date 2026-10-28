# TB-RK3399ProD资料索引

[项目目录](../../../logs/低空消防无人机工程日志.md)｜[当天日志](../../../logs/2026-10-26_DWA四类部署续验与I2C分口.md)｜[PCA9685与TFmini接线表](../../项目资料/RK板_PCA9685与TFmini接线表_2026-10-24.md)

**本轮后续补充：** SHELL已反馈I²C7为1.8V，选择分口试用与PCA板载5V作为候选；实际门限／上拉和负载仍待核验。芯片手册第60页的[1.8V GPIO输入电压依据](../../验证记录/RK板接口电平依据_2026-10-26/README.md)已补充，不能默认该接口容忍3.3V。下方原始资料文件不改变。

SHELL于2026-10-26要求查找RK3399ProD文档。本目录保存本轮取得的原始PDF，不修改正文、图纸或文件名。原理图沿用项目既有副本，不重复下载；来源、页数及SHA256见[来源与校验](来源与校验.json)。

## 1. 接线优先阅读

| 资料 | 本地入口 | 内容与版本 |
|---|---|---|
| 主板硬件规格书 | [Toybrick TB-RK3399ProD Specification V1.0](1629270722351_4011.pdf) | 原厂署名，Oneyac保存的PDF；正文标称22页，文件实际23页，末页为分销商信息。第20页定位40pin位置，第21页表2-4列脚号，第22页为网络／RS232说明 |
| 主板原理图 | [既有2018-12-17原理图](../../验证记录/RK板资料核查_2026-10-21/TB-RK3399ProD_原理图_20181217.pdf) | Rockchip原图，Rev V1.0，36个PDF页；最后一页为GPIO／J9702／I²C电平转换，图纸自身页号97，不等同PDF页码 |
| 元件位号图 | [SMT_REF_TB-RK3399ProD_33101_20181217YWQ.pdf](SMT_REF_TB-RK3399ProD_33101_20181217YWQ.pdf) | 2页，ref_top／ref_bottom；文件名日期2018-12-17。用于在PCB上找J9702、芯片和元件，不是电气连接原理图 |
| GPIO原理图图片 | [既有第36页PNG](../../验证记录/RK板资料核查_2026-10-21/TB-RK3399ProD_原理图_P36_GPIO.png) | 接线时便于放大查看I²C6／I²C7、3.3V／5V／12V与连接器脚号 |

位号图ref_top中，J9702顶端内侧40／外侧39、底端内侧2／外侧1，与SHELL本轮IMG_1726近照定位一致；它仍是历史图，不能据此声称V1.2全部电路与其完全相同。

## 2. 芯片与部署资料

| 资料 | 入口 | 用途 |
|---|---|---|
| RK3399Pro芯片Datasheet | [Rockchip RK3399Pro Datasheet V1.0_20181109.pdf](Rockchip%20RK3399Pro%20Datasheet%20V1.0_20181109.pdf) | Rev1.0，2018年11月，68页；用于芯片能力、封装／信号和电气规格。芯片焊球编号不能当成主板40针脚号 |
| 已有RKNN API中文手册 | [Rockchip_RK3399Pro_User_Guide_RKNN_API_V1.7.5_CN.pdf](../../../yolov5training/rknn/npu/RK3399Pro_npu/rknn-api/doc/Rockchip_RK3399Pro_User_Guide_RKNN_API_V1.7.5_CN.pdf) | 项目既有旧RKNN1.7.5接口参考；本轮仅提供入口，没有重做全文审查 |
| 本项目部署依赖清单 | [Ubuntu18.04部署依赖](../../项目资料/RK3399ProD_Ubuntu18.04部署依赖清单_2026-10-21.md) | 项目工程记录，须结合后续日志与任务状态，不能当成厂家对所有系统的兼容承诺 |

## 3. 厂家与原文件来源

- [Toybrick官方中文硬件规格书网页](https://t.rock-chips.com/wiki.php?filename=%E6%9D%BF%E7%BA%A7%E6%8C%87%E5%8D%97%2FTB-RK3399ProD)：查接口位置和表2-4；官网表中27脚误写为第二个26，查该处时以原理图为准。
- [Toybrick官方英文硬件规格书网页](https://t.rock-chips.com/en/wiki.php?filename=Board_Guide%2FTB-RK3399ProD)。
- [Toybrick官方本板下载页](https://t.rock-chips.com/portal.php?catid=11&mod=list&product_id=4)：列出原理图、位号图、孔位信息、芯片手册、TRM及软件资料；部分下载需要进入网盘，本轮没有取得可确认对应实板V1.2的新版完整硬件包。
- [Seeed原厂资料保存目录](https://github.com/SeeedDocument/Outsourcing/tree/master/TB-RK3399Pro)：本轮通过GitHub连接器取得目录元数据，下载芯片Datasheet和位号图，并核对大小及Git blob SHA1；既有原理图也与目录blob匹配。连接器不支持二进制下载，PDF原文件通过目录给出的公开raw地址取得。
- [主板规格书原PDF下载](https://uploadcdn.oneyac.com/upload/document/1629270722351_4011.pdf)：Oneyac分销商保存的原厂署名规格书，未将分销商网页描述当作电气设计依据。

## 4. 当前接线需要区分的事项

- 实板照片丝印为`TB-RK3399ProD_33101_V1.2`；已取得历史原理图为2018-12-17／V1.0。当前仅确认照片脚号方向与公开表／位号图一致，真实电源及总线电平仍待测。
- 主板有两路引出的I²C：I²C6为32脚SCL／34脚SDA；I²C7为14脚SCL／16脚SDA。历史原理图显示前者外侧3.3V、后者1.8V，不能直接互换接3.3V模块。
- 本轮未取得板载5V对外保证电流或本机舵机峰值电流。V+使用独立5V是当前接线选择，不表示已证明RK的5V供电不足，也不是PCA模块绝对禁止由主板5V供电。
- 资料查找、PDF解析／视觉核对和文件哈希不等于接线、供电负载、测距或舵机动作验收。
