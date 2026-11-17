# xr-head-tracking

QVR/CDSP 头部跟踪与姿态接口。

输入消费 xr-device-io 唯一的原厂800Hz请求路径。`XR_SEE_IMU_SOCKET` 使用
XSL4 握手，以 SCM_RIGHTS 交接同一对4096字节真实 DMA 缓冲区，分别注册
CDSP 的原厂 gyro ring0 和 accel ring5。原厂 `adsp_imu`/QDI 线程自主推进 HRP；
相机 RPC 调用者只提交相机，AP 不重写104字节输入、不喂逐样本 RPC，也不积分原始陀螺。

独立读取线程持续消费真实采样元数据，按已发布 OEM 槽与精确 QTimer 纳秒 epoch
配对后发布240字节头姿接口。原厂 VR 模式跳过传感器 timestamp_offset；AP 用
实测时钟元组换算输出时刻，相机直接使用 QCFT 原始 QTimer SOF，按相同625/12换算和原厂曝光数学对齐，避免 BOOTTIME 往返。
未就绪或未配对不生成姿态。有限 DMA 同步/一致性读取保留原厂环没有 generation
的边界；双份数据一致不代表序列锁。原厂标定、质量与状态保留，不添加运动阈值。

该原生共享缓冲实现已完成离线源码准备，跨 DSP 导入、QDI 自主唤醒、相机 RPC
期间 HRP 的持续推进和佩戴效果仍须目标设备验证。历史实现由 Git 保留。

构建入口和依赖见下方；工程 Wiki/Blog 与跨仓库规划集中在 [openkylin-xr-docs](https://github.com/bitsk/openkylin-xr-docs)。

## 构建

相邻检出所需组件，并指定已准备的 xr-kernel 检出：

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_C_COMPILER=aarch64-linux-gnu-gcc \
  -DCMAKE_CXX_COMPILER=aarch64-linux-gnu-g++ \
  -DXR_KERNEL_UAPI_DIR=/usr/include/xr-kernel
cmake --build build -j4
```

头部插件把统一采集作为库链接到唯一生产者，控制器和调试端消费其接口；这次不新增竞争采集进程。第三方 HexagonRPC 代码保留 COPYING，SDK/DSP 固件在设备运行时另行导入。

## 原厂运行资源

DSP runtime 和型号默认 config6 已归档到独立公开 [xr-vendor-assets](https://github.com/bitsk/xr-vendor-assets) Release。通过 xr-image-tools 的 `tools/vendor.py fetch` 下载并校验后，缓存里的 `qvr/runtime/qcom/` 提供 DSP 文件，`qvr/config/config6.xml` 提供型号默认配置；rootfs 安装目标记录在 vendor.lock.json 中。每台设备的相机校准独立提供，不从其他设备的 persist 复制。正常使用不依赖整份原厂镜像。

`XR_DEVICE` 默认为 `a081`，保留原有型号与 XML 摘要校验。VR700 会话显式设置
`XR_DEVICE=vr700`；生产者要求本机设备树同时具有 `VR700` 型号和
`iqiyi,vr700` 首项 compatible。`A081_QVR_CONFIG6` 与
`A081_QVR_CALIBRATION` 继续作为共享接口字段，分别指向已选定的 trinity 配置变体和
本机私有标定。会话从已导入的私有 profile 传入
`XR_HEAD_CONFIG_SHA256`、`XR_HEAD_CALIBRATION_SHA256`，后者必须是完整的小写
SHA-256；生产者在任何 DSP/采集操作前验证文件摘要。VR700 的 config 摘要还必须
匹配所选模式的内置白名单，不接受 profile 指定的任意修改配置。

VR700 允许 `A081_QVR_CVP_MODE=stock` 或显式 `disabled`，默认仍为 stock。
原厂 tracker、CVP DSP 库和 XML 与已有分支分析的输入逐字节相同；disabled 仅在
`SFConfig` 内增加 `<CvpConfig enable="false" />`，并校验对应的固定摘要。
这会选择原算法已有的非 CVP 初始化分支，不提供模拟的 CVP 服务。
私有标定摘要不会编入公共源码，也不会回退到 MIX 标定。型号接入
保留原有 direct IMU、QCFT 相机、时钟与失效条件；通过文件校验不代表已经完成
VR700 DSP、四路采集、坐标或真实跟踪验收。

## openKylin 打包

本仓库包含独立的 `debian/` 配方。使用目标发行线的干净构建环境运行 `dpkg-buildpackage -us -uc -b`，预先准备 Build-Depends 和离线构建输入。包构建、目标系统安装和设备验收分别记录。

当前迁移不自动启用竞争采集、切换活动运行时或写设备启动分区；已有验证基线继续保留。

构建依赖通过已安装的 `xr-kernel-uapi-dev` 和 `libxr-device-dev` 提供；可用 CMAKE_PREFIX_PATH 选择开发包前缀。

FastRPC 客户端由独立 xr-hexagonrpc 仓库维护，通过 libxr-hexagonrpc-dev 的 XRHexagonRPC::client 导入，避免在跟踪仓库复制第二份实现。
