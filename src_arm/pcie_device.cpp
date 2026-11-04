#include "pcie_device.h"
#include "frame_buffer.h"

#include <QString>
#include <unistd.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cstring>
#include <cerrno>

static inline off_t  page_round_down(off_t x) { return x & ~(getpagesize() - 1); }
static inline size_t page_round_up(size_t x)   { return page_round_down(x + getpagesize() - 1); }

namespace {

constexpr unsigned int kPioDebugStatusOffset = 0x2d0;
constexpr unsigned int kPioDebugStartMask = 1u << 1;

unsigned int fpgaSourceValue(FpgaInputSource source)
{
    return static_cast<unsigned int>(source) & 0x3u;
}

FpgaInputSource decodeFrameSource(unsigned int statusWord)
{
    const unsigned int value = (statusWord >> 3) & 0x3u;
    return value == 2u ? FpgaInputSource::Fiber
         : value == 1u ? FpgaInputSource::HDMI
                       : FpgaInputSource::FMC;
}

unsigned int activeSourceFromSwitchStatus(unsigned int statusWord)
{
    // SOURCE_STATUS: bit[3]=active_source[1], bit[0]=active_source[0].
    return ((statusWord >> 3) & 0x1u) << 1 | (statusWord & 0x1u);
}

unsigned int requestedSourceFromSwitchStatus(unsigned int statusWord)
{
    // SOURCE_STATUS: bit[4]=request[1], bit[1]=request[0].
    return ((statusWord >> 4) & 0x1u) << 1 | ((statusWord >> 1) & 0x1u);
}

const char *fpgaSourceName(unsigned int value)
{
    switch (value) {
    case 0u: return "FMC";
    case 1u: return "HDMI";
    case 2u: return "FIBER";
    default: return "INVALID";
    }
}

} // namespace

// ===================================================================
PcieDevice::PcieDevice(QObject *parent)
    : QObject(parent)
    , m_devFd(-1)
    , m_bar0VirtAddr(nullptr)
    , m_dmaPhysicalBase(0)
{
    memset(&m_cmdOp,     0, sizeof(m_cmdOp));
    memset(&m_bar0CmdOp, 0, sizeof(m_bar0CmdOp));
    memset(&m_devInfo,   0, sizeof(m_devInfo));
    memset(&m_dmaOp,     0, sizeof(m_dmaOp));
    memset(&m_bar0Mem,   0, sizeof(m_bar0Mem));
    m_dmaUserAddr = nullptr;
    m_dmaMapSize  = 0;
}

PcieDevice::~PcieDevice()
{
    closeDevice();
}

// ===================================================================
//  mmap 物理内存 (BAR0 PIO 访问)
// ===================================================================
void *PcieDevice::mapPhysicalMemory(unsigned long physAddr, unsigned int size, DEV_MEM *mem)
{
    if (mem->vaddr != nullptr) {
        qDebug() << "[PcieDevice] BAR0 已映射";
        return mem->vaddr;
    }

    off_t  offset = page_round_down(physAddr);
    size_t len    = page_round_up(size);

    int fd = ::open(MEM_DEVICE_PATH, O_RDWR | O_SYNC);
    if (fd < 0) {
        qDebug() << "[PcieDevice] 打开 /dev/mem 失败:" << strerror(errno);
        return nullptr;
    }

    void *vaddr = mmap(nullptr, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset);
    ::close(fd);

    if (vaddr == MAP_FAILED || vaddr == nullptr) {
        qDebug() << "[PcieDevice] mmap 失败:" << strerror(errno);
        return nullptr;
    }

    mem->len    = len;
    mem->offset = offset;
    mem->vaddr  = vaddr;
    return vaddr;
}

// ===================================================================
//  打开设备 — 严格匹配原始 getDevice() 的 IOCTL 序列
// ===================================================================
bool PcieDevice::openDevice()
{
    if (m_devFd >= 0) return true;

    int pioBarIndex = 0;

    // 1. open
    m_devFd = ::open(PCIE_DRIVER_PATH, O_RDWR);
    if (m_devFd < 0) {
        qDebug() << "[PcieDevice] open 失败:" << strerror(errno);
        return false;
    }

    // 2. 优先直接 read() 驱动导出的 command_operation 快照。
    // 新驱动/旧驱动都在 pango_cdev_read() 里把 command_operation 整体拷给用户态，
    // 比依赖 PCI_READ_DATA_CMD 的局部 config dword 语义更稳。
    if (::read(m_devFd, &m_cmdOp, sizeof(m_cmdOp)) > 0 &&
        m_cmdOp.get_pci_dev_info.vendor_id != 0) {
        m_devInfo = m_cmdOp.get_pci_dev_info;
    } else {
        // 回退旧路径
        if (ioctl(m_devFd, PCI_READ_DATA_CMD, &m_cmdOp) < 0) {
            qDebug() << "[PcieDevice] PCI_READ_DATA_CMD 失败:" << strerror(errno);
            goto fail;
        }
        m_devInfo = m_cmdOp.get_pci_dev_info;
    }

    qDebug().nospace()
        << "[PcieDevice] Vendor=0x" << Qt::hex << qSetFieldWidth(4) << qSetPadChar('0')
        << m_devInfo.vendor_id << Qt::reset
        << " Gen" << Qt::dec << m_devInfo.link_speed
        << " x" << m_devInfo.link_width
        << " MPS=" << m_devInfo.mps;
    qDebug().nospace()
        << "[PcieDevice] BAR0 base=0x" << Qt::hex << m_devInfo.bar[0].bar_base
        << " len=0x" << m_devInfo.bar[0].bar_len
        << " | BAR1 base=0x" << m_devInfo.bar[1].bar_base
        << " len=0x" << m_devInfo.bar[1].bar_len;

    // 3. PCI_SET_CONFIG 在部分驱动里是可选 no-op；失败只记日志，不直接终止。
    if (ioctl(m_devFd, PCI_SET_CONFIG, &m_dmaOp) < 0) {
        qDebug() << "[PcieDevice] PCI_SET_CONFIG 失败(忽略):" << strerror(errno);
    }

    // 4. RTL 中 pio_crtl 的寄存器读写路径挂在 BAR0:
    //    - pio 写: ips2l_pcie_dma.o_bar0_wr_* -> pio_crtl.pio_wr_*
    //    - pio 读: ips2l_pcie_dma.o_bar0_rd_* -> pio_crtl.pio_rd_*
    //    BAR1 则接到 DMA controller 的配置写入口。
    //    因此用户态 PIO 直读直写应优先映射 BAR0；BAR0 不可用时再回退 BAR1。
    pioBarIndex = 0;
    if (m_devInfo.bar[pioBarIndex].bar_base == 0 ||
        m_devInfo.bar[pioBarIndex].bar_len == 0) {
        pioBarIndex = 1;
    }
    if (m_devInfo.bar[pioBarIndex].bar_base == 0 ||
        m_devInfo.bar[pioBarIndex].bar_len == 0) {
        qDebug() << "[PcieDevice] 未获得有效 BAR 信息，无法映射 PIO 寄存器";
        goto fail;
    }

    // 5. mmap PIO BAR 物理地址
    m_bar0VirtAddr = mapPhysicalMemory(
        m_devInfo.bar[pioBarIndex].bar_base,
        BAR0_MAP_SIZE,
        &m_bar0Mem
    );
    if (!m_bar0VirtAddr) {
        qDebug() << "[PcieDevice] PIO BAR mmap 失败";
        goto fail;
    }

    qDebug() << "[PcieDevice] PIO BAR" << pioBarIndex << " 映射:" << Qt::hex
             << reinterpret_cast<quintptr>(m_bar0VirtAddr);
    qDebug() << "[PcieDevice] 设备就绪 (DMA 映射待采集时配置)";
    return true;

fail:
    ::close(m_devFd);
    m_devFd = -1;
    return false;
}

// ===================================================================
//  关闭设备
// ===================================================================
void PcieDevice::closeDevice()
{
    if (m_devFd < 0) return;

    // 先停 FPGA，避免 DMA 释放过程中仍有总线写入
    stopCapture();
    usleep(2000);

    // 先撤销用户态 DMA VMA，再释放驱动里的 coherent buffer
    if (m_dmaUserAddr) {
        munmap(m_dmaUserAddr, m_dmaMapSize);
        m_dmaUserAddr = nullptr;
        m_dmaMapSize = 0;
    }
    m_dmaPhysicalBase = 0;

    if (ioctl(m_devFd, PCI_UMAP_ADDR_CMD, &m_dmaOp) < 0) {
        qDebug() << "[PcieDevice] DMA unmap 失败:" << strerror(errno);
    }

    // 取消 BAR0 映射
    if (m_bar0Mem.vaddr) {
        munmap(m_bar0Mem.vaddr, m_bar0Mem.len);
        m_bar0Mem.vaddr  = nullptr;
        m_bar0Mem.len    = 0;
        m_bar0Mem.offset = 0;
    }
    m_bar0VirtAddr = nullptr;

    ::close(m_devFd);
    m_devFd = -1;
    qDebug() << "[PcieDevice] 设备已关闭";
}

// ===================================================================
//  DMA 配置 + 映射 (匹配原始 on_Key1_clicked: set len → dma_map(1))
// ===================================================================
bool PcieDevice::setupDma()
{
    if (!isOpened()) return false;

    // 同一进程里重复开关设备时，先清掉本对象残留的 DMA 映射状态
    if (m_dmaUserAddr) {
        munmap(m_dmaUserAddr, m_dmaMapSize);
        m_dmaUserAddr = nullptr;
        m_dmaMapSize = 0;
    }
    m_dmaPhysicalBase = 0;
    if (ioctl(m_devFd, PCI_UMAP_ADDR_CMD, &m_dmaOp) < 0 && errno != EINVAL) {
        qDebug() << "[PcieDevice] DMA 预清理失败(忽略):" << strerror(errno);
    }

    m_dmaOp.current_len = (1280 * 2) >> 2;   // 2560 字节 → 640 DW
    m_dmaOp.offset_addr = 0;

    if (ioctl(m_devFd, PCI_MAP_ADDR_CMD, &m_dmaOp) < 0) {
        qDebug() << "[PcieDevice] DMA map 失败:" << strerror(errno);
        return false;
    }
    qDebug() << "[PcieDevice] DMA 已映射, len=" << m_dmaOp.current_len << "DW"
             << "offset_addr=" << m_dmaOp.offset_addr
             << "cmd=" << m_dmaOp.cmd;
    return true;
}

// ===================================================================
// programFpgaDmaBases() — 告诉 FPGA "把帧数据写到 ARM 的哪个物理地址"
//
// 【为什么需要这个函数】
// FPGA 往 ARM 内存写数据(DMA), 但它不知道 ARM 端的物理内存地址
// 就像快递员知道要送包裹, 但不知道你家门牌号
// 这个函数就是告诉 FPGA: "槽0写这里, 槽1写那里, 槽2写..."
//
// 【4个槽位的乒乓缓冲】
// FPGA 有 4 个帧槽位, 轮流使用, 避免覆盖还没读走的帧:
//   槽0: 物理地址 base0 → 存第0,4,8,12...帧
//   槽1: 物理地址 base1 → 存第1,5,9,13...帧
//   槽2: 物理地址 base2 → 存第2,6,10,14...帧
//   槽3: 物理地址 base3 → 存第3,7,11,15...帧
//   每写完一帧, FPGA 自动切换到下一个槽
//   每个槽 1280×720×2 = 1,843,200 字节 (1.84MB)
//   4个槽总共 7,372,800 字节 (7.03MB)
//
// 【那些 0x050、0x054 是什么】
// 它们是 FPGA 内部 "PIO 寄存器" 的地址(字节偏移):
//   槽0: 0x050(低32位) + 0x054(高32位) → 64位物理地址
//   槽1: 0x020(低32位) + 0x024(高32位)
//   槽2: 0x040(低32位) + 0x044(高32位)
//   槽3: 0x030(低32位) + 0x034(高32位)
//   配置触发: 0x060 → 写入1让 FPGA 加载新地址
//
// 【为什么每个地址要分两次写(高32位+低32位)】
// 物理地址是 64 位的(比如 0x000000007eb00000)
// 但 PIO 寄存器是 32 位宽的, 一个寄存器装不下 64 位
// 所以要拆成两半:
//   低32位: base & 0xffffffff  → 写入 0x050
//   高32位: base >> 32          → 写入 0x054
//
// 【pioWriteReg 是怎么工作的】
// 它通过之前 mmap 的 BAR0 空间, 直接写 FPGA 寄存器:
//   *(volatile uint32_t*)(bar0_addr + byteOffset/4) = data;
// 这行代码会触发一条 PCIe Memory Write TLP
// → FPGA 收到后把数据写入对应的内部寄存器
// → pcie_tx_fun 模块读这些寄存器获取 DMA 目标地址
// → 每行数据 DMA 时, 地址 = base_addr + 行号 × 2560
// → 720行写完, frame_done 置 1
//
// 【FPGA 侧对应代码: pio_crtl.v】
// 在 FPGA RTL 中, pio_crtl 模块接收 PIO 写请求:
//   if(pio_wr_en && pio_wr_addr == 9'h050)
//       r_ch0_base_addr[31:0] <= pio_wr_data;  // 存低32位
//   if(pio_wr_en && pio_wr_addr == 9'h054)
//       r_ch0_base_addr[63:32] <= pio_wr_data; // 存高32位
//   ... 四个地址同理 ...
//   if(pio_wr_en && pio_wr_addr == 9'h060)
//       r_set_dma_config_en <= 1; // 告诉 pcie_tx_fun: 地址更新了, 重新加载!
// ===================================================================
bool PcieDevice::programFpgaDmaBases()
{
    if (!isOpened()) return false;

    // 从驱动获取 DMA buffer 的物理地址
    // 驱动在 PCI_MAP_ADDR_CMD 时分配了 dma_alloc_coherent 内存
    // 物理地址存在 dma_info.addr_w.addr 中
    DMA_PHYS_INFO physInfo = {};
    if (ioctl(m_devFd, PCI_GET_DMA_PHYS_ADDR, &physInfo) < 0) {
        qDebug() << "[PcieDevice] PCI_GET_DMA_PHYS_ADDR failed:" << strerror(errno);
        return false;
    }

    // 安全检查: 驱动分配的 buffer 至少要有 4 帧大小
    const unsigned long long expectedSize = 4ULL * FRAME_SIZE_RGB565;
    if (physInfo.phys_addr == 0 || physInfo.size < expectedSize) {
        qDebug() << "[PcieDevice] invalid DMA phys info:"
                 << "addr=" << Qt::hex << physInfo.phys_addr
                 << "size=" << Qt::dec << physInfo.size
                 << "expected>=" << expectedSize;
        return false;
    }
    m_dmaPhysicalBase = static_cast<uint64_t>(physInfo.phys_addr);

    // 计算 4 个槽位的物理地址
    // 驱动分配了一块连续物理内存, 大小 = 4 × 1.84MB = 7.37MB
    // 4 个槽在这块内存中等距排列:
    //   base0 = 起始地址          (例如 0x7eb00000)
    //   base1 = base0 + 1.84MB   (例如 0x7ecc2000)
    //   base2 = base0 + 3.68MB   (例如 0x7ee84000)
    //   base3 = base0 + 5.52MB   (例如 0x7f046000)
    const unsigned long long frameSize = FRAME_SIZE_RGB565;
    const unsigned long long base0 = physInfo.phys_addr;
    const unsigned long long base1 = base0 + frameSize;
    const unsigned long long base2 = base0 + frameSize * 2ULL;
    const unsigned long long base3 = base0 + frameSize * 3ULL;

    qWarning().nospace()
        << "[dma-config] physical_base=0x" << QString::number(base0, 16)
        << " bytes=" << physInfo.size;
    qDebug() << "[PcieDevice] program FPGA DMA bases:"
             << Qt::hex
             << "b0=" << base0
             << "b1=" << base1
             << "b2=" << base2
             << "b3=" << base3;

    // ---- 把 4 个 64 位物理地址写入 FPGA 的 PIO 寄存器 ----

    // 槽0: 寄存器 0x050(低32位) + 0x054(高32位)
    pioWriteReg(0x050, static_cast<unsigned int>(base0 & 0xffffffffULL)); // 低32位
    pioWriteReg(0x054, static_cast<unsigned int>(base0 >> 32));           // 高32位

    // 槽1: 寄存器 0x020(低32位) + 0x024(高32位)
    pioWriteReg(0x020, static_cast<unsigned int>(base1 & 0xffffffffULL));
    pioWriteReg(0x024, static_cast<unsigned int>(base1 >> 32));

    // 槽2: 寄存器 0x040(低32位) + 0x044(高32位)
    pioWriteReg(0x040, static_cast<unsigned int>(base2 & 0xffffffffULL));
    pioWriteReg(0x044, static_cast<unsigned int>(base2 >> 32));

    // 槽3: 寄存器 0x030(低32位) + 0x034(高32位)
    pioWriteReg(0x030, static_cast<unsigned int>(base3 & 0xffffffffULL));
    pioWriteReg(0x034, static_cast<unsigned int>(base3 >> 32));

    // 配置触发: 写 0x060 = 1, 告诉 FPGA "地址更新完了, 用新地址"
    // FPGA 内 pio_crtl.v 收到后会产生一个脉冲 set_dma_config_en
    // pcie_tx_fun 收到这个脉冲后重新加载 4 个基地址
    pioWriteReg(0x060, 0x00000001);

    // PCIe Memory Write 是 posted write。紧跟一次无副作用的 BAR0 Memory Read，
    // 强制前面的地址配置和 DMA_SET_EN 在 startCapture() 前真正到达 FPGA。
    const unsigned int writeCount = pioReadReg(0x1b0);
    const unsigned int lastAddr   = pioReadReg(0x190);
    const unsigned int lastData   = pioReadReg(0x1a0);

    qWarning().nospace()
        << "[dma-config] FPGA base registers written and flushed"
        << " write_count=" << writeCount
        << " last_addr=0x" << QString::number(lastAddr, 16)
        << " last_data=0x" << QString::number(lastData, 16);
    return true;
}

// ===================================================================
// 【帧率提升关键1】tryMmapDna() — 零拷贝的核心入口
//
// 要理解为什么帧率从30变60, 先搞清楚数据怎么到用户程序:
//
// 老办法 (readFrame):
//   程序调 ::read() → 内核把FPGA DMA内存的1.84MB数据拷到用户空间
//   → 内核读uncached内存很慢(~20ms) + 等下一帧(~8ms) = ~30ms
//   → 每秒只能处理 1000/30 ≈ 33 帧
//
// 新办法 (tryMmapDna + readPioStatus):
//   1. mmap() 把 FPGA 的 DMA 内存直接映射到用户空间(零拷贝)
//      → 程序直接读 FPGA 写的那块物理内存, 不经过内核搬运
//   2. readPioStatus() 直接读 FPGA 寄存器(2微秒, 无系统调用)
//      → 不靠 ::read() 等状态, 自己轮询
//   3. commitMappedFrame() 只传8字节指针, 不拷贝数据
//      → 1.84MB 的拷贝延迟完全消失
//
// 效果: 从每帧30ms → 每帧几乎0ms, 从30fps → 60fps
// ===================================================================
bool PcieDevice::tryMmapDma()
{
    if (!isOpened()) return false;

    if (m_dmaUserAddr) {
        munmap(m_dmaUserAddr, m_dmaMapSize);
        m_dmaUserAddr = nullptr;
    }

    m_dmaMapSize = 4 * FRAME_SIZE_RGB565;  // 4帧乒乓缓冲

    // 方案1: 设备文件 mmap — 驱动 pango_cdev_mmap 用 dma_mmap_coherent 直接映射
    void *addr = mmap(nullptr, m_dmaMapSize, PROT_READ, MAP_SHARED, m_devFd, 0);
    if (addr != MAP_FAILED) {
        m_dmaUserAddr = addr;
        qDebug() << "[PcieDevice] mmap device fd DMA SUCCESS"
                 << "userAddr=" << addr << "size=" << m_dmaMapSize;
        return true;
    }
    qDebug() << "[PcieDevice] mmap device fd FAILED:" << strerror(errno);

    // 方案2: 回退 /dev/mem (需要知道物理地址)
    const off_t candidates[] = { 0x4bc00000, 0x7eb00000 };
    for (size_t i = 0; i < sizeof(candidates)/sizeof(candidates[0]); i++) {
        const off_t phys = candidates[i];
        off_t pageOff = phys & ~(getpagesize() - 1);
        size_t mapLen = m_dmaMapSize + (phys - pageOff);

        int memFd = ::open("/dev/mem", O_RDONLY);
        if (memFd < 0) continue;

        addr = mmap(nullptr, mapLen, PROT_READ, MAP_SHARED, memFd, pageOff);
        ::close(memFd);

        if (addr != MAP_FAILED) {
            m_dmaUserAddr = (uint8_t*)addr + (phys - pageOff);
            qDebug() << "[PcieDevice] mmap /dev/mem DMA SUCCESS phys=0x" << Qt::hex << phys;
            return true;
        }
    }

    m_dmaUserAddr = nullptr;
    m_dmaMapSize = 0;
    return false;
}

// ===================================================================
//  FPGA 采集启动/停止 (匹配原始 pio_write)
// ===================================================================
void PcieDevice::startCapture()
{
    (void)startCaptureChecked();
}

bool PcieDevice::startCaptureChecked(unsigned int timeoutMs)
{
    if (!isOpened()) return false;

    // Drain a stale read-to-clear frame notification from a previous run.
    FrameStatus staleStatus;
    (void)readPioStatus(staleStatus);

    pioWriteReg(0x000, 0xFFFFFFE5);
    // PIO_DBG_STATUS 0x2d0 bit 1 is the registered host START flag.  0x1b0 is
    // only a PIO write counter and cannot be used as capture state.
    unsigned int debugStatus = pioReadReg(kPioDebugStatusOffset);
    unsigned int waitedMs = 0;
    while ((debugStatus & kPioDebugStartMask) == 0u && waitedMs < timeoutMs) {
        usleep(1000);
        debugStatus = pioReadReg(kPioDebugStatusOffset);
        ++waitedMs;
    }
    const unsigned int sourceStatus = pioReadReg(0x310);
    qWarning().nospace()
        << "[capture-guard] START "
        << ((debugStatus & kPioDebugStartMask) != 0u ? "confirmed" : "TIMEOUT")
        << " wait_ms=" << waitedMs
        << " debug_2d0=0x" << QString::number(debugStatus, 16)
        << " source_status=0x" << QString::number(sourceStatus, 16);
    return (debugStatus & kPioDebugStartMask) != 0u;
}

void PcieDevice::stopCapture()
{
    (void)stopCaptureChecked();
}

bool PcieDevice::stopCaptureChecked(unsigned int timeoutMs)
{
    if (!isOpened()) return false;
    pioWriteReg(0x000, 0xFFFFFF00);
    unsigned int debugStatus = pioReadReg(kPioDebugStatusOffset);
    unsigned int waitedMs = 0;
    while ((debugStatus & kPioDebugStartMask) != 0u && waitedMs < timeoutMs) {
        usleep(1000);
        debugStatus = pioReadReg(kPioDebugStatusOffset);
        ++waitedMs;
    }
    const bool stopped = (debugStatus & kPioDebugStartMask) == 0u;
    qWarning().nospace()
        << "[capture-guard] STOP " << (stopped ? "confirmed" : "TIMEOUT")
        << " wait_ms=" << waitedMs
        << " debug_2d0=0x" << QString::number(debugStatus, 16);
    return stopped;
}

// ===================================================================
//  PIO 写
// ===================================================================
bool PcieDevice::setFpgaInputSource(FpgaInputSource source)
{
    if (!isOpened() || !m_bar0VirtAddr) return false;

    const unsigned int value = fpgaSourceValue(source);
    if (value > 2u) return false;
#ifdef ARM_FINAL_FPGA_DIAGNOSTICS
    const unsigned int beforeStatus = pioReadReg(0x310);
    qWarning().nospace()
        << "[fpga-source-debug] request=" << fpgaSourceName(value)
        << " before=0x" << QString::number(beforeStatus, 16)
        << " active=" << fpgaSourceName(activeSourceFromSwitchStatus(beforeStatus))
        << " requested=" << fpgaSourceName(requestedSourceFromSwitchStatus(beforeStatus))
        << " busy=" << ((beforeStatus >> 2) & 0x1u)
        << " epoch=" << ((beforeStatus >> 8) & 0xffu);
#endif
    pioWriteReg(0x300, value);
    unsigned int status = pioReadReg(0x310);  // flush the posted write

    // A posted write must be visible in SOURCE_STATUS before the UI changes
    // its local source state.  Retry once to tolerate the BAR read pipeline.
    if (requestedSourceFromSwitchStatus(status) != value) {
        pioWriteReg(0x300, value);
        status = pioReadReg(0x310);
    }

    // Wait until the frame-boundary state machine has changed the real mux,
    // rather than restarting ARM polling while the old source is still active.
    // The FPGA timeout is 50 ms at Gen2 and about 100 ms at Gen1, so 250 ms
    // covers both rates plus the next VSYNC.
    unsigned int waitMs = 0;
    while ((requestedSourceFromSwitchStatus(status) != value
            || activeSourceFromSwitchStatus(status) != value
            || ((status >> 2) & 0x1u) != 0u)
           && waitMs < 250u) {
        usleep(1000);
        status = pioReadReg(0x310);
        ++waitMs;
    }

    const unsigned int active = activeSourceFromSwitchStatus(status);
    const unsigned int requested = requestedSourceFromSwitchStatus(status);
    const bool busy = ((status >> 2) & 0x1u) != 0;
    const bool switched = requested == value && active == value && !busy;
#ifdef ARM_FINAL_FPGA_DIAGNOSTICS
    qWarning().nospace()
        << "[fpga-source-debug] result=" << (switched ? "OK" : "TIMEOUT")
        << " target=" << fpgaSourceName(value)
        << " wait_ms=" << waitMs
        << " final=0x" << QString::number(status, 16)
        << " active=" << fpgaSourceName(active)
        << " requested=" << fpgaSourceName(requested)
        << " busy=" << busy
        << " epoch=" << ((status >> 8) & 0xffu);
#endif
    if (!switched) {
        // Cancel a timed-out request so it cannot take effect later after the
        // UI has resumed under the assumption that the old source is active.
        pioWriteReg(0x300, active <= 2u ? active : 0u);
        (void)pioReadReg(0x310);
        qWarning() << "[fpga-source] switch timeout; request rolled back";
    }
    return switched;
}

bool PcieDevice::rearmFpgaInputSource(FpgaInputSource source)
{
    if (!isOpened() || !m_bar0VirtAddr)
        return false;

    // The caller keeps host capture stopped while this runs. Switching to the
    // opposite input and back advances source_epoch twice and starts the mux
    // and selected DMA controller from a clean boundary. A same-source write
    // is a no-op in the FPGA and can retain stale completion/slot state.
    const FpgaInputSource opposite = source == FpgaInputSource::HDMI
                                   ? FpgaInputSource::FMC
                                   : FpgaInputSource::HDMI;
    qWarning() << "[fpga-source] rearm begin target="
               << fpgaSourceName(fpgaSourceValue(source));
    if (!setFpgaInputSource(opposite)) {
        qWarning() << "[fpga-source] rearm failed on opposite source";
        return false;
    }
    usleep(1000);
    if (!setFpgaInputSource(source)) {
        qWarning() << "[fpga-source] rearm failed restoring target source";
        return false;
    }
    qWarning() << "[fpga-source] rearm complete";
    return true;
}

void PcieDevice::pioWrite32(unsigned int data, int baseAddr, int regAddr)
{
    if (!m_bar0VirtAddr) return;
    *((volatile unsigned int *)m_bar0VirtAddr + baseAddr + regAddr) = data;
}

void PcieDevice::pioWriteReg(unsigned int byteOffset, unsigned int data)
{
    if (!m_bar0VirtAddr) return;
    *((volatile unsigned int *)m_bar0VirtAddr + (byteOffset >> 2)) = data;
    // ARM 上阻止编译器/CPU 将后续 MMIO 操作重排到本次写之前。
    __sync_synchronize();
}

// ===================================================================
//  帧读取 (阻塞模式，驱动返回帧状态码)
// ===================================================================
// ===================================================================
// 【老办法】readFrame() — 帧率只有30fps的瓶颈所在
//
// 问题出在 ::read() 系统调用内部:
//   传了 FRAME_SIZE_RGB565(1.84MB) 作为 maxLen
//   → 内核驱动检测到大buffer, 把整帧数据从DMA内存拷到用户buf
//   → DMA内存是 uncached 的(ARM上读一次50-200ns)
//   → 1.84MB / 4B × 75ns ≈ 35ms 纯拷贝时间
//   加上等 frame_done 中断 ~8ms, 每次 ::read() 花 ~30ms
//
// 这就是为什么之前帧率卡在 30fps 的根本原因:
//   不是FPGA慢, 不是PCIe慢, 是内核搬运数据的方式慢
// ===================================================================
int PcieDevice::readFrame(uint8_t *buf, int maxLen, FrameStatus &status)
{
    status = FrameStatus();

    int rd = (int)::read(m_devFd, buf, maxLen);
    if (rd < 0) {
        // EPERM: FPGA 未启动或 DMA 未就绪，不是致命错误，静默重试
        // 原始代码不检查 read() 返回值符号，只看 bit 标志
        if (errno == EPERM)
            return 0;
        qDebug() << "[PcieDevice] read 错误:" << strerror(errno);
        return -1;
    }

    status.frame_done   = (rd & 0x01) != 0;
    status.buffer_index = (rd >> 1) & 0x03;
    status.source       = decodeFrameSource(static_cast<unsigned int>(rd));
    status.source_epoch = static_cast<uint8_t>((rd >> 8) & 0xff);
    status.valid        = status.frame_done;

    return rd;
}

// 轻量级状态轮询：只传 4 字节请求状态码，不传帧数据
// 实验目的：对比 ::read() 传入大 buffer 和小 buffer 的耗时差异
int PcieDevice::pollFrameStatus(FrameStatus &status)
{
    status = FrameStatus();

    // 只传 4 字节 buffer，测试驱动是否有快速路径
    uint32_t statusWord = 0;
    int rd = (int)::read(m_devFd, &statusWord, sizeof(statusWord));
    if (rd < 0) {
        if (errno == EPERM)
            return 0;
        qDebug() << "[PcieDevice] pollFrameStatus 错误:" << strerror(errno);
        return -1;
    }

    status.frame_done   = (rd & 0x01) != 0;
    status.buffer_index = (rd >> 1) & 0x03;
    status.source       = decodeFrameSource(static_cast<unsigned int>(rd));
    status.source_epoch = static_cast<uint8_t>((rd >> 8) & 0xff);
    status.valid        = status.frame_done;

    return rd;
}

// 最快路径：BAR0 直读 FPGA PIO 寄存器 0x140，无任何 syscall
// pio_crtl.v 寄存器格式: {29'd0, i_wr_index[1:0], r_wr_frame_done}
// 读操作会触发 pio_rd_en 从而自动清除 r_wr_frame_done
// ===================================================================
// 【新办法】readPioStatus() — 帧率60fps的关键
//
// 不调 ::read() 系统调用, 直接用 CPU 读 FPGA 的 BAR0 寄存器
//
// BAR0 是什么:
//   启动时通过 /dev/mem mmap 了 FPGA 的 PCIe BAR0 空间
//   地址 0x140 处的寄存器内容 = {buffer_index, frame_done}
//
// 读这个寄存器就是一条 CPU load 指令:
//   uint32_t val = *(volatile uint32_t*)(bar0_addr + 0x140);
//
// 耗时: ~2微秒 (PCIe Memory Read TLP 往返一次)
//   vs 旧 ::read() 的 ~30000微秒 → 快了 15000 倍
//
// 这就是帧率从30变60的根本原因:
//   不用等内核拷贝数据了, 自己直接看 FPGA 有没有新帧
//   有新帧就直接从 mmap 内存读, 没有就 usleep(500) 等一下再查
// ===================================================================
int PcieDevice::readPioStatus(FrameStatus &status)
{
    status = FrameStatus();

    if (!m_bar0VirtAddr) {
        return -1;
    }

    volatile unsigned int *pioBase = (volatile unsigned int *)m_bar0VirtAddr;
    const unsigned int wordOffset = 0x140 >> 2;  // byte 0x140 → word index 0x50
    // 0x140 is read-to-clear.  Exactly one MMIO read is allowed here: a second
    // read consumes the notification with the first access and then observes
    // frame_done=0, which starves the whole capture loop.
    const uint32_t regVal = pioBase[wordOffset];
    __sync_synchronize();

    status.frame_done   = (regVal & 0x01) != 0;
    status.buffer_index = (regVal >> 1) & 0x03;
    status.source       = decodeFrameSource(regVal);
    status.source_epoch = static_cast<uint8_t>((regVal >> 8) & 0xff);
    status.valid        = status.frame_done;

    // 成功路径只通过 status 传递状态位，不把寄存器原值当错误码返回。
    return 0;
}

unsigned int PcieDevice::pioReadReg(unsigned int byteOffset) const
{
    if (!m_bar0VirtAddr) {
        return 0;
    }

    volatile unsigned int *pioBase = (volatile unsigned int *)m_bar0VirtAddr;
    volatile unsigned int *reg = pioBase + (byteOffset >> 2);

    // 该函数只用于 0x150 及以上、16 字节对齐的无副作用调试寄存器。
    // FPGA 的 PIO 读数据经过寄存/PCIe Completion 流水线，切换地址后的第一次
    // 读取可能仍是上一个地址的返回值。同地址重复读取并取最新值，可刷新流水线。
    // 有读清零副作用的 0x140 只能由 readPioStatus() 单次读取。
    unsigned int value = *reg;
    __sync_synchronize();
    for (int retry = 0; retry < 3; ++retry) {
        const unsigned int next = *reg;
        __sync_synchronize();
        if (next == value) {
            return next;
        }
        value = next;
    }
    return value;
}
