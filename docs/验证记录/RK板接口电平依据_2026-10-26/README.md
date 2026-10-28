# RK板接口电平依据补充

[当天日志](../../../logs/2026-10-26_DWA四类部署续验与I2C分口.md)｜[分口接线说明](../../项目资料/RK板分口接线与I2C7测量_2026-10-26.md)

本目录保存原厂芯片手册的页面渲染与解释，不是实板测量报告。

- 来源文件：[Rockchip RK3399Pro Datasheet V1.0_20181109.pdf](../../参考资料/TB-RK3399ProD/Rockchip%20RK3399Pro%20Datasheet%20V1.0_20181109.pdf)，Rev1.0，2018年11月；原文件SHA256为`9f706fabbbca25488b3db8a238858fe7f8fb7a9d00e434f58cd874eef588e2e9`。
- 来源地址：[Seeed保存的Rockchip原始PDF](https://github.com/SeeedDocument/Outsourcing/blob/master/TB-RK3399Pro/Rockchip%20RK3399Pro%20Datasheet%20V1.0_20181109.pdf)。
- [PDF第60页原样渲染](RK3399Pro_Datasheet_P60_1V8_GPIO.png)，已视觉核对表格行列。属于表3-3 DC Characteristics的续表。
- `Digital GPIO @1.8V`的`Input High Voltage / Vih`为Min **1.17V**、Max **2.1V**。这不是“绝对最大额定值2.1V”的表述，也不表示某一实板接口已做过耐压实验。
- 本项目历史主板原理图显示I²C7的`I2C7_SCL_1V8`／`I2C7_SDA_1V8`经22Ω到J9702的14／16脚，没有I²C6那样的3.3V转换。SHELL现反馈I²C7实测为1.8V，但未提供逐线原始读数和量法，不能补造这些内容。

据此不能默认1.8V接口允许外设上拉到3.3V。TFmini识别1.8V的输入门限与其SDA／SCL上拉／输出方式需要分别核对；能够读到数据不等于双向电气兼容和长期稳定已经验收。
