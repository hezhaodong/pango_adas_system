#ifndef PCIE_DEVICE_H
#define PCIE_DEVICE_H

#include <QObject>
#include <QDebug>
#include <stdint.h>
#include <sys/types.h>

// ---- 驱动路径 ----
#define PCIE_DRIVER_PATH    "/dev/pango_pci_driver"
#define MEM_DEVICE_PATH     "/dev/mem"

// ---- IOCTL 命令码 ----
#define PCI_IOCTL_TYPE      'S'
#define PCI_READ_DATA_CMD           _IOWR(PCI_IOCTL_TYPE,  0, int)
#define PCI_WRITE_DATA_CMD          _IOWR(PCI_IOCTL_TYPE,  1, int)
#define PCI_MAP_ADDR_CMD            _IOWR(PCI_IOCTL_TYPE,  2, int)
#define PCI_WRITE_TO_KERNEL_CMD     _IOWR(PCI_IOCTL_TYPE,  3, int)
#define PCI_DMA_READ_CMD            _IOWR(PCI_IOCTL_TYPE,  4, int)
#define PCI_DMA_WRITE_CMD           _IOWR(PCI_IOCTL_TYPE,  5, int)
#define PCI_READ_FROM_KERNEL_CMD    _IOWR(PCI_IOCTL_TYPE,  6, int)
#define PCI_UMAP_ADDR_CMD           _IOWR(PCI_IOCTL_TYPE,  7, int)
#define PCI_GET_DMA_PHYS_ADDR       _IOWR(PCI_IOCTL_TYPE, 10, int)  /* 获取DMA物理地址 */
#define PCI_MAP_BAR0_CMD            _IOWR(PCI_IOCTL_TYPE, 10, int)
#define PCI_SET_CONFIG              _IOWR(PCI_IOCTL_TYPE, 11, int)

// ---- 常量 ----
#define BAR0_MAP_SIZE               0x500
#define DMA_MAX_PACKET_SIZE         4096

// ---- 数据结构 (与原始驱动完全一致) ----
typedef struct {
    unsigned long bar_base;
    unsigned long bar_len;
} BAR_BASE_INFO;

typedef struct {
    unsigned int  vendor_id;
    unsigned int  device_id;
    unsigned int  cmd_reg;
    unsigned int  status_reg;
    unsigned int  revision_id;
    unsigned int  class_prog;
    unsigned int  class_device;
    BAR_BASE_INFO bar[6];
    unsigned int  min_gnt;
    unsigned int  max_lat;
    unsigned int  link_speed;
    unsigned int  link_width;
    unsigned int  mps;
    unsigned int  mrrs;
    unsigned int  data[1024];
} PCI_DEVICE_INFO;

typedef struct {
    unsigned char  w_r;
    unsigned char  step;
    unsigned int   addr;
    unsigned int   data;
    unsigned int   cnt;
    unsigned int   delay;
    PCI_DEVICE_INFO get_pci_dev_info;
} COMMAND_OPERATION;

typedef struct {
    unsigned char read_buf[DMA_MAX_PACKET_SIZE];
    unsigned char write_buf[DMA_MAX_PACKET_SIZE];
} DMA_DATA;

typedef struct {
    unsigned int current_len;
    unsigned int offset_addr;
    unsigned int cmd;
    DMA_DATA     data;
} DMA_OPERATION;

typedef struct {
    unsigned long long phys_addr;    // DMA buffer 物理地址
    unsigned int       size;         // buffer 大小 (bytes)
} DMA_PHYS_INFO;

typedef struct {
    off_t   offset;
    size_t  len;
    void   *vaddr;
} DEV_MEM;

enum class FpgaInputSource : uint8_t {
    FMC   = 0,
    HDMI  = 1,
    Fiber = 2
};

struct FrameStatus {
    bool valid = false;
    int  buffer_index = 0;
    bool frame_done = false;
    FpgaInputSource source = FpgaInputSource::FMC;
    uint8_t source_epoch = 0;
};

// ---- PCIe 设备抽象 ----
class PcieDevice : public QObject
{
    Q_OBJECT

public:
    explicit PcieDevice(QObject *parent = nullptr);
    ~PcieDevice();

    bool    openDevice();
    void    closeDevice();
    bool    isOpened() const { return m_devFd >= 0; }
    int     getFd() const { return m_devFd; }

    bool    setupDma();         // 配置并映射 DMA (在采集前调用)
    bool    programFpgaDmaBases();  // 将4帧DMA物理基地址写入FPGA寄存器
    bool    tryMmapDma();       // 尝试 mmap DMA 缓冲区到用户空间（绕过内核拷贝）
    void   *dmaUserAddr() const { return m_dmaUserAddr; }
    uint64_t dmaPhysicalBase() const { return m_dmaPhysicalBase; }
    bool    hasDmaMmap() const { return m_dmaUserAddr != nullptr; }
    void    startCapture();     // 启动 FPGA 采集
    void    stopCapture();
    bool    startCaptureChecked(unsigned int timeoutMs = 150);
    bool    stopCaptureChecked(unsigned int timeoutMs = 150);
    bool    setFpgaInputSource(FpgaInputSource source);
    bool    rearmFpgaInputSource(FpgaInputSource source);

    int     readFrame(uint8_t *buf, int maxLen, FrameStatus &status);
    int     pollFrameStatus(FrameStatus &status);  // 轻量级：4字节::read()只取状态
    int     readPioStatus(FrameStatus &status);    // 最快：BAR0直读PIO寄存器0x140，无syscall
    unsigned int pioReadReg(unsigned int byteOffset) const;

    void    pioWrite32(unsigned int data, int baseAddr, int regAddr);
    void    pioWriteReg(unsigned int byteOffset, unsigned int data);

    const PCI_DEVICE_INFO& deviceInfo() const { return m_devInfo; }
    int  linkSpeed()  const { return m_devInfo.link_speed; }
    int  linkWidth()  const { return m_devInfo.link_width; }
    int  maxPayload() const { return m_devInfo.mps; }

private:
    void   *mapPhysicalMemory(unsigned long physAddr, unsigned int size, DEV_MEM *mem);

    int             m_devFd;
    COMMAND_OPERATION m_cmdOp;          // 用于 PCI_READ_DATA_CMD
    COMMAND_OPERATION m_bar0CmdOp;      // 用于 PCI_MAP_BAR0_CMD (独立结构!)
    PCI_DEVICE_INFO   m_devInfo;
    DMA_OPERATION     m_dmaOp;
    DEV_MEM           m_bar0Mem;
    void             *m_bar0VirtAddr;
    void             *m_dmaUserAddr;      // mmap'd DMA buffer userspace address
    size_t            m_dmaMapSize;       // mmap'd size
    uint64_t          m_dmaPhysicalBase;  // first of the four contiguous frame slots
};

#endif // PCIE_DEVICE_H
