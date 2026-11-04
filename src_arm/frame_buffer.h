#ifndef FRAME_BUFFER_H
#define FRAME_BUFFER_H

#include <QMutex>
#include <cstdint>
#include <cstring>

// ---- 图像参数 ----
#define FRAME_WIDTH         1280
#define FRAME_HEIGHT        720
#define FRAME_SIZE_RGB565    (FRAME_WIDTH * FRAME_HEIGHT * 2)   // 1,843,200 字节
#define FRAME_SIZE_ARGB32    (FRAME_WIDTH * FRAME_HEIGHT * 4)   // ARGB32: 3,686,400 字节
#define RING_BUFFER_COUNT    3                                  // 三缓冲

// ---- 线程安全的三缓冲帧管理器 ----
// 采集线程: lockWriteBuf() → 写入 → commitFrame()
// UI 线程:  acquireLatestFrame() → 读取
// FPGA 的 4 缓冲索引与 FrameBuffer 的 3 缓冲索引不直接映射,
// FrameBuffer 自行管理写入位置
class FrameBuffer
{
public:
    FrameBuffer()
    {
        for (int i = 0; i < RING_BUFFER_COUNT; i++) {
            m_rgb565Buf[i]  = new uint8_t[FRAME_SIZE_RGB565];
            m_frameSrc[i]   = m_rgb565Buf[i];
            m_frameValid[i] = false;
            m_frameSeq[i]   = 0;
        }
        m_writeIndex    = 0;
        m_totalFrames   = 0;
        m_droppedFrames = 0;
    }

    ~FrameBuffer()
    {
        for (int i = 0; i < RING_BUFFER_COUNT; i++) {
            delete[] m_rgb565Buf[i];
        }
    }

    // ---- 采集线程接口 ----

    // 获取写入槽的 RGB565 缓冲区指针 (采集线程直接写入)
    uint8_t *lockWriteBuffer()
    {
        QMutexLocker lock(&m_mutex);

        // 如果下一次写入会覆盖一个尚未被 UI 取走的槽位，
        // 在真正 memcpy 前先把该槽位从“可读集合”里摘掉，避免 UI 读到正在被覆盖的数据。
        if (m_frameValid[m_writeIndex]) {
            m_frameValid[m_writeIndex] = false;
            m_droppedFrames++;
        }

        m_frameSrc[m_writeIndex] = m_rgb565Buf[m_writeIndex];

        return m_rgb565Buf[m_writeIndex];
    }

    // 提交当前写入槽 (采集线程在写入完成后调用)
    void commitFrame(int frameSeq)
    {
        QMutexLocker lock(&m_mutex);

        m_frameSeq[m_writeIndex]   = frameSeq;
        m_frameSrc[m_writeIndex]   = m_rgb565Buf[m_writeIndex];
        m_frameValid[m_writeIndex] = true;
        m_writeIndex = (m_writeIndex + 1) % RING_BUFFER_COUNT;
        m_totalFrames++;
    }

    // ===================================================================
    // 【帧率提升关键2】commitMappedFrame() — 只传指针, 不拷数据
    //
    // 老办法 commitFrame():
    //   采集线程: memcpy(dst, src, 1.84MB) → 写进 FrameBuffer 的本地拷贝
    //   → 每次拷贝 1.84MB, 即使用户态内存是cached也要 ~1.5ms
    //   → 加上内核里那次拷贝(~20ms), 一共两次, 总延迟 ~21.5ms
    //
    // 新办法 commitMappedFrame():
    //   采集线程: 直接把 DMA 内存的指针存进数组
    //   → 不拷贝, 只存 8 字节指针
    //   → 延迟: 0ms
    //
    // 为什么能这样做?
    //   因为 tryMmapDna() 已经把 FPGA 的 DMA 内存映射到了用户空间
    //   这个内存 FPGA 在写, ARM 在读, 不需要额外拷贝
    //   只要 FPGA 还没覆盖这个槽位(4帧乒乓保证67ms内不会被覆盖)
    //   就可以一直安全地读
    // ===================================================================
    void commitMappedFrame(const uint8_t *srcRgb565, int frameSeq)
    {
        QMutexLocker lock(&m_mutex);

        if (m_frameValid[m_writeIndex]) {
            m_frameValid[m_writeIndex] = false;
            // Mapped frames form a latest-frame cache shared by UI and both
            // AI readers. Replacing old metadata is expected, not a DMA drop.
        }

        m_frameSeq[m_writeIndex]   = frameSeq;
        m_frameSrc[m_writeIndex]   = srcRgb565;
        m_frameValid[m_writeIndex] = true;
        m_writeIndex = (m_writeIndex + 1) % RING_BUFFER_COUNT;
        m_totalFrames++;
    }

    // ---- UI 线程接口 (消耗性读取) ----

    // 获取最新已完成帧, 返回 true 表示有新帧
    bool acquireLatestFrame(uint8_t *dstRgb565, int &frameSeq)
    {
        int latest = -1;
        int latestSeq = -1;
        const uint8_t *src = nullptr;
        {
            QMutexLocker lock(&m_mutex);

            for (int i = 0; i < RING_BUFFER_COUNT; i++) {
                if (m_frameValid[i] && m_frameSeq[i] > latestSeq) {
                    latestSeq = m_frameSeq[i];
                    latest = i;
                }
            }

            if (latest < 0) return false;

            src = m_frameSrc[latest];
            frameSeq = m_frameSeq[latest];
            m_frameValid[latest] = false;
        }

        memcpy(dstRgb565, src, FRAME_SIZE_RGB565);
        return true;
    }

    // 获取最新已完成帧的只读视图, 避免 UI 再做一次整帧 memcpy
    // ===================================================================
    // 【帧率提升关键3】acquireLatestFrameView() — 非消耗性"看一眼"
    //
    // 老办法 acquireLatestFrame():
    //   从 FrameBuffer 内部拷贝 1.84MB 到显示缓冲区
    //   → 又一次全帧 memcpy, ~1.5ms
    //
    // 新办法 acquireLatestFrameView():
    //   只返回指向 DMA 内存的指针, 不拷贝
    //   → 显示线程和 AI 线程能同时读同一块内存
    //   → 读完不删(非消耗性), 下一个读者还能再读
    //   → 延迟: 锁等待 ~1μs
    //
    // 一个槽位能被多个线程同时安全读取的前提:
    //   FPGA 的4槽乒乓保证这个槽位不会被覆盖
    //   直到 FPGA 写到第4个槽之后才会回头覆盖
    //   刚完成槽在 FPGA 开始复用前约有 3 个帧周期，即 60fps 下约 50ms
    //   显示线程只需 ~16ms, AI 线程只需 ~20ms
    //   → 都在 67ms 内完成, 安全
    // ===================================================================
    bool acquireLatestFrameView(const uint8_t *&srcRgb565, int &frameSeq)
    {
        int latest = -1;
        int latestSeq = -1;
        {
            QMutexLocker lock(&m_mutex);

            for (int i = 0; i < RING_BUFFER_COUNT; ++i) {
                if (m_frameValid[i] && m_frameSeq[i] > latestSeq) {
                    latestSeq = m_frameSeq[i];
                    latest = i;
                }
            }

            if (latest < 0) return false;

            srcRgb565 = m_frameSrc[latest];
            frameSeq = m_frameSeq[latest];
        }
        return true;
    }

    // ---- 推理线程接口 (非消耗性读取) ----

    // 拷贝最新帧, 不标记 invalid. 推理线程使用, 不影响 UI 线程取帧
    bool copyLatestFrame(uint8_t *dstRgb565, int &frameSeq,
                         int skipFrameSeq = -1)
    {
        int latest = -1;
        int latestSeq = -1;
        const uint8_t *src = nullptr;
        {
            QMutexLocker lock(&m_mutex);

            for (int i = 0; i < RING_BUFFER_COUNT; i++) {
                if (m_frameValid[i] && m_frameSeq[i] > latestSeq) {
                    latestSeq = m_frameSeq[i];
                    latest = i;
                }
            }

            if (latest < 0) return false;

            // 先比较序号再复制，避免 AI 轮询同一帧时重复搬运 1.84 MB。
            if (m_frameSeq[latest] == skipFrameSeq) return false;

            src = m_frameSrc[latest];
            frameSeq = m_frameSeq[latest];
        }

        memcpy(dstRgb565, src, FRAME_SIZE_RGB565);
        return true;
    }

    // ---- 重置 (新采集会话前调用) ----
    void reset()
    {
        QMutexLocker lock(&m_mutex);
        for (int i = 0; i < RING_BUFFER_COUNT; i++) {
            m_frameValid[i] = false;
            m_frameSeq[i]   = 0;
            m_frameSrc[i]   = m_rgb565Buf[i];
        }
        m_writeIndex    = 0;
        m_totalFrames   = 0;
        m_droppedFrames = 0;
    }

    // ---- 统计 ----
    unsigned int totalFrames()   const { return m_totalFrames; }
    unsigned int droppedFrames() const { return m_droppedFrames; }

private:
    int m_oldestValidIndex() const
    {
        int oldest = -1;
        int oldestSeq = 0x7FFFFFFF;
        for (int i = 0; i < RING_BUFFER_COUNT; i++) {
            if (m_frameValid[i] && m_frameSeq[i] < oldestSeq) {
                oldestSeq = m_frameSeq[i];
                oldest = i;
            }
        }
        return oldest;
    }

    uint8_t *m_rgb565Buf[RING_BUFFER_COUNT];    // RGB565 帧缓冲
    const uint8_t *m_frameSrc[RING_BUFFER_COUNT];
    bool     m_frameValid[RING_BUFFER_COUNT];
    int      m_frameSeq[RING_BUFFER_COUNT];

    int      m_writeIndex;

    unsigned int m_totalFrames;
    unsigned int m_droppedFrames;

    QMutex   m_mutex;
};

#endif // FRAME_BUFFER_H
