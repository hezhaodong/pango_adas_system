#include "frame_capture.h"
#include "image_converter.h"

#include <QDebug>
#include <QString>
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>
#include <vector>
#include <unistd.h>
#include <cerrno>
#include <chrono>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <im2d.h>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace {

const char *captureSourceName(FpgaInputSource source)
{
    switch (source) {
    case FpgaInputSource::FMC:   return "FMC";
    case FpgaInputSource::HDMI:  return "HDMI";
    case FpgaInputSource::Fiber: return "FIBER";
    }
    return "INVALID";
}

// The FPGA DMA buffer is dma_alloc_coherent memory (uncached for the ARM
// CPU).  A plain memcpy() is latency-bound there (~20-25 ms for 1.84 MB on
// RK3568), which caps the capture loop at ~40 fps.  This helper keeps four
// 16-byte loads in flight per iteration to overlap the uncached read latency.
// Selected with CAPTURE_COPY=2.
static void copyRgb565Wide(uint8_t *dst, const uint8_t *src, size_t n)
{
#if defined(__aarch64__)
    const size_t blocks = n / 64;
    for (size_t i = 0; i < blocks; ++i) {
        if (i + 8 < blocks)
            __builtin_prefetch(src + 512, 0, 3);
        const uint8x16_t a = vld1q_u8(src);
        const uint8x16_t b = vld1q_u8(src + 16);
        const uint8x16_t c = vld1q_u8(src + 32);
        const uint8x16_t d = vld1q_u8(src + 48);
        vst1q_u8(dst, a);
        vst1q_u8(dst + 16, b);
        vst1q_u8(dst + 32, c);
        vst1q_u8(dst + 48, d);
        src += 64;
        dst += 64;
    }
    std::memcpy(dst, src, n - blocks * 64);
#else
    std::memcpy(dst, src, n);
#endif
}

static void copyRgb565WideOnCpu(uint8_t *dst, const uint8_t *src,
                                size_t n, int cpu)
{
    // Keep optional multi-copy helpers on normal scheduling so a diagnostic
    // CAPTURE_COPY=3/4 run cannot starve the Qt/GL thread.
    struct sched_param normalParam;
    std::memset(&normalParam, 0, sizeof(normalParam));
    (void)pthread_setschedparam(pthread_self(), SCHED_OTHER, &normalParam);

    const long onlineCpus = sysconf(_SC_NPROCESSORS_ONLN);
    if (cpu >= 0 && cpu < onlineCpus && cpu < CPU_SETSIZE) {
        cpu_set_t cpuSet;
        CPU_ZERO(&cpuSet);
        CPU_SET(cpu, &cpuSet);
        (void)pthread_setaffinity_np(pthread_self(), sizeof(cpuSet), &cpuSet);
    }
    copyRgb565Wide(dst, src, n);
}

// dma_alloc_coherent() is intentionally uncached on this RK3568 kernel.  One
// A55 core needs about 13 ms to read a 720p RGB565 frame, just over the
// 12.5-ms budget at 80 fps.  Four independent readers expose more outstanding
// DDR transactions and write disjoint cache lines in the cached destination.
// The caller still waits for completion before publishing the frame, so UI/AI
// consumers never observe a partially copied image.
static void copyRgb565FourWay(uint8_t *dst, const uint8_t *src, size_t n)
{
    const size_t quarter = (n / 4u) & ~static_cast<size_t>(63u);
    const size_t offset1 = quarter;
    const size_t offset2 = quarter * 2u;
    const size_t offset3 = quarter * 3u;

    // The capture thread is pinned to CPU3. pthread affinity is inherited, so
    // explicitly move the three helpers to CPU0..2; otherwise four threads
    // would still serialize on CPU3 and provide no speed-up.
    std::thread worker1(copyRgb565WideOnCpu, dst + offset1, src + offset1,
                        offset2 - offset1, 0);
    std::thread worker2(copyRgb565WideOnCpu, dst + offset2, src + offset2,
                        offset3 - offset2, 1);
    std::thread worker3(copyRgb565WideOnCpu, dst + offset3, src + offset3,
                        n - offset3, 2);
    copyRgb565Wide(dst, src, offset1);
    worker1.join();
    worker2.join();
    worker3.join();
}

static void copyRgb565ThreeWay(uint8_t *dst, const uint8_t *src, size_t n)
{
    const size_t third = (n / 3u) & ~static_cast<size_t>(63u);
    const size_t offset1 = third;
    const size_t offset2 = third * 2u;

    // CPU2 remains free for Qt and the Mali userspace driver.  The capture
    // thread handles the first third on its pinned CPU3.
    std::thread worker1(copyRgb565WideOnCpu, dst + offset1, src + offset1,
                        offset2 - offset1, 0);
    std::thread worker2(copyRgb565WideOnCpu, dst + offset2, src + offset2,
                        n - offset2, 1);
    copyRgb565Wide(dst, src, offset1);
    worker1.join();
    worker2.join();
}

// RK3568's RGA reads the PCIe DMA slot by physical address and writes the
// cached FrameBuffer directly.  This avoids making an A55 core read 1.84 MB
// of uncached dma_alloc_coherent memory every frame (measured at ~12.1 ms).
// Handles are retained for the lifetime of one capture run so the 120-fps
// path pays only the synchronous RGA blit, not import/release ioctls per frame.
class RgaFrameCopier
{
public:
    RgaFrameCopier()
        : m_physicalBase(0)
    {
        m_sourceHandles.fill(0);
        for (Destination &destination : m_destinations) {
            destination.address = nullptr;
            destination.handle = 0;
        }
    }

    ~RgaFrameCopier()
    {
        releaseHandles();
    }

    bool copy(uint64_t physicalBase, int sourceIndex, uint8_t *destination,
              QString &error)
    {
        if (physicalBase == 0 || sourceIndex < 0 || sourceIndex >= RAW_BUF_COUNT
            || destination == nullptr) {
            error = QStringLiteral("invalid DMA physical address, slot, or destination");
            return false;
        }

        if (m_physicalBase != physicalBase) {
            releaseHandles();
            m_physicalBase = physicalBase;
        }

        rga_buffer_handle_t &sourceHandle = m_sourceHandles[sourceIndex];
        if (sourceHandle == 0) {
            const uint64_t sourcePhysical = physicalBase
                + static_cast<uint64_t>(sourceIndex) * FRAME_SIZE_RGB565;
            sourceHandle = importbuffer_physicaladdr(
                sourcePhysical, FRAME_WIDTH, FRAME_HEIGHT, RK_FORMAT_RGB_565);
            if (sourceHandle == 0) {
                error = QStringLiteral("importbuffer_physicaladdr failed");
                return false;
            }
        }

        rga_buffer_handle_t destinationHandle = destinationHandleFor(destination);
        if (destinationHandle == 0) {
            error = QStringLiteral("importbuffer_virtualaddr failed");
            return false;
        }

        const rga_buffer_t sourceBuffer = wrapbuffer_handle(
            sourceHandle, FRAME_WIDTH, FRAME_HEIGHT, RK_FORMAT_RGB_565);
        rga_buffer_t destinationBuffer = wrapbuffer_handle(
            destinationHandle, FRAME_WIDTH, FRAME_HEIGHT, RK_FORMAT_RGB_565);
        const IM_STATUS status = imcopy(sourceBuffer, destinationBuffer, 1);
        if (status <= 0) {
            error = QStringLiteral("imcopy failed: %1")
                        .arg(QString::fromLocal8Bit(imStrError_t(status)));
            return false;
        }
        return true;
    }

private:
    struct Destination {
        uint8_t *address;
        rga_buffer_handle_t handle;
    };

    rga_buffer_handle_t destinationHandleFor(uint8_t *address)
    {
        for (const Destination &destination : m_destinations) {
            if (destination.address == address)
                return destination.handle;
        }
        for (Destination &destination : m_destinations) {
            if (destination.handle != 0)
                continue;
            destination.handle = importbuffer_virtualaddr(
                address, FRAME_WIDTH, FRAME_HEIGHT, RK_FORMAT_RGB_565);
            if (destination.handle != 0)
                destination.address = address;
            return destination.handle;
        }
        return 0;
    }

    void releaseHandles()
    {
        for (rga_buffer_handle_t &handle : m_sourceHandles) {
            if (handle != 0)
                releasebuffer_handle(handle);
            handle = 0;
        }
        for (Destination &destination : m_destinations) {
            if (destination.handle != 0)
                releasebuffer_handle(destination.handle);
            destination.address = nullptr;
            destination.handle = 0;
        }
        m_physicalBase = 0;
    }

    uint64_t m_physicalBase;
    std::array<rga_buffer_handle_t, RAW_BUF_COUNT> m_sourceHandles;
    std::array<Destination, RING_BUFFER_COUNT> m_destinations;
};


struct SeamEstimate {
    int column = 0;
    int score = 0;
    int baseline = 0;
    int coveragePercent = 0;
    bool valid = false;
};

static inline int rgb565Distance(uint16_t a, uint16_t b)
{
    const int ar = (a >> 11) & 0x1f;
    const int ag = (a >> 5) & 0x3f;
    const int ab = a & 0x1f;
    const int br = (b >> 11) & 0x1f;
    const int bg = (b >> 5) & 0x3f;
    const int bb = b & 0x1f;
    return std::abs(ar - br) + std::abs(ag - bg) + std::abs(ab - bb);
}

// A cyclic horizontal row offset creates one discontinuity at the same x in
// almost every row. Real scene edges normally cover only part of the height.
// Sample the image sparsely so calibration is cheap on RK3568.
static SeamEstimate estimateVerticalWrapSeam(const uint8_t *frame)
{
    SeamEstimate result;
    if (!frame) return result;

    constexpr int xMargin = 32;
    constexpr int yMargin = 24;
    constexpr int yStep = 8;
    constexpr int strongEdge = 30;

    const uint16_t *pixels = reinterpret_cast<const uint16_t *>(frame);
    std::vector<int> scores(FRAME_WIDTH, 0);
    std::vector<int> hits(FRAME_WIDTH, 0);
    int sampledRows = 0;

    for (int y = yMargin; y < FRAME_HEIGHT - yMargin; y += yStep) {
        const uint16_t *row = pixels + y * FRAME_WIDTH;
        ++sampledRows;
        for (int x = xMargin; x < FRAME_WIDTH - xMargin; ++x) {
            const int delta = rgb565Distance(row[x - 1], row[x]);
            scores[x] += std::min(delta, 96);
            if (delta >= strongEdge) ++hits[x];
        }
    }

    std::vector<int> population;
    population.reserve(FRAME_WIDTH - 2 * xMargin);
    for (int x = xMargin; x < FRAME_WIDTH - xMargin; ++x)
        population.push_back(scores[x]);
    if (population.empty() || sampledRows == 0) return result;

    auto middle = population.begin() + population.size() / 2;
    std::nth_element(population.begin(), middle, population.end());
    result.baseline = *middle;

    for (int x = xMargin; x < FRAME_WIDTH - xMargin; ++x) {
        if (scores[x] > result.score) {
            result.score = scores[x];
            result.column = x;
            result.coveragePercent = hits[x] * 100 / sampledRows;
        }
    }

    // Require both a full-height edge and a large excess over normal adjacent
    // pixel activity. This avoids rotating a healthy image around an ordinary
    // object edge.
    // The log shows the real moving seam can have only 7% strong-edge coverage
    // while still scoring ~=1427 versus a ~=113 background.  Keep the absolute
    // and relative prominence tests strong, but do not reject that seam merely
    // because foreground content masks most sampled rows.
    result.valid = result.coveragePercent >= 5
                && result.score >= result.baseline + sampledRows * 8
                && result.score >= 800
                && result.score >= std::max(1, result.baseline) * 5;
    return result;
}

static void rotateRgb565FrameLeft(uint8_t *frame, int shiftPixels,
                                  std::vector<uint8_t> &scratch)
{
    const int framePixels = FRAME_WIDTH * FRAME_HEIGHT;
    if (!frame || shiftPixels <= 0 || shiftPixels >= framePixels) return;

    const size_t frameBytes = static_cast<size_t>(FRAME_SIZE_RGB565);
    const size_t shiftBytes = static_cast<size_t>(shiftPixels) * 2u;
    const size_t remainBytes = frameBytes - shiftBytes;
    if (scratch.size() < shiftBytes) scratch.resize(shiftBytes);

    // The measured seam is the number of stale pixels prepended before the
    // real linear-frame origin.  Remove that prefix by rotating the complete
    // RGB565 frame left by the measured column.  This must not be done per
    // row, because the offset crosses row boundaries.
    std::memcpy(scratch.data(), frame, shiftBytes);
    std::memmove(frame, frame + shiftBytes, remainBytes);
    std::memcpy(frame + remainBytes, scratch.data(), shiftBytes);
}

static void rotateRgb565FrameRight(uint8_t *frame, int shiftPixels,
                                   std::vector<uint8_t> &scratch)
{
    const int framePixels = FRAME_WIDTH * FRAME_HEIGHT;
    if (!frame) return;
    shiftPixels %= framePixels;
    if (shiftPixels <= 0) return;

    const size_t frameBytes = static_cast<size_t>(FRAME_SIZE_RGB565);
    const size_t shiftBytes = static_cast<size_t>(shiftPixels) * 2u;
    const size_t remainBytes = frameBytes - shiftBytes;
    if (scratch.size() < shiftBytes) scratch.resize(shiftBytes);

    std::memcpy(scratch.data(), frame + remainBytes, shiftBytes);
    std::memmove(frame + shiftBytes, frame, remainBytes);
    std::memcpy(frame, scratch.data(), shiftBytes);
}

static uint32_t sampledFrameSignature(const uint8_t *frame)
{
    if (!frame) return 0;
    uint32_t hash = 2166136261u;
    for (size_t offset = 0; offset < FRAME_SIZE_RGB565; offset += 256u) {
        for (size_t tap = 0; tap < 256u && offset + tap < FRAME_SIZE_RGB565; tap += 64u) {
            hash ^= frame[offset + tap];
            hash *= 16777619u;
        }
    }
    return hash;
}

} // namespace

FrameCapture::FrameCapture(PcieDevice *dev, FrameBuffer *fb, QObject *parent)
    : QThread(parent)
    , m_device(dev)
    , m_buffer(fb)
    , m_rawBuf4(nullptr)
    , m_running(false)
    , m_stopRequested(false)
    , m_totalFrames(0)
    , m_droppedFrames(0)
    , m_frameSeq(0)
    , m_lastIndex(-1)
    , m_timeout(false)
    , m_inputFrameCount(0)
    , m_processFrameCount(0)
    , m_inputFPS(0.0)
    , m_processFPS(0.0)
{
    m_rawBuf4  = new uint8_t[RAW_BUF_SIZE];
    memset(m_rawBuf4, 0, RAW_BUF_SIZE);
    m_fpsTimer.start();
}

FrameCapture::~FrameCapture()
{
    stopCapture();
    wait(2000);
    delete[] m_rawBuf4;
}

void FrameCapture::stopCapture()
{
    m_stopRequested.store(true, std::memory_order_release);
}

void FrameCapture::run()
{
    m_stopRequested.store(false, std::memory_order_release);
    m_running.store(true, std::memory_order_release);

    // Pin the capture thread to its own core.  Do not use SCHED_FIFO here:
    // an 80-fps uncached snapshot consumes close to Linux's default 950-ms/s
    // RT budget, so the kernel periodically throttles the thread for about
    // 50 ms.  That exact throttle appeared as proc_max_us=60..68 ms and made
    // an otherwise stable 80-fps stream lock near 60 until source restart.
    // A high-priority CFS thread on its dedicated CPU avoids RT throttling.
    // 0x140 is a
    // read-to-clear completion latch: two FPGA completions between reads
    // merge into one and the input frame rate drops (~60*2/3 ~= 40 fps).
    // Default core is 3; override with CAPTURE_CPU (e.g. CAPTURE_CPU=-1).
    {
        const char *cpuEnv = std::getenv("CAPTURE_CPU");
        const int pinCpu = cpuEnv ? std::atoi(cpuEnv) : 3;
        if (pinCpu >= 0 && pinCpu < CPU_SETSIZE) {
            cpu_set_t cpuSet;
            CPU_ZERO(&cpuSet);
            CPU_SET(pinCpu, &cpuSet);
            if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuSet) != 0)
                errno = 0;
        }

        struct sched_param normalParam;
        std::memset(&normalParam, 0, sizeof(normalParam));
        (void)pthread_setschedparam(pthread_self(), SCHED_OTHER, &normalParam);
        errno = 0;
        (void)nice(-10);
        const int appliedPolicy = sched_getscheduler(0);
        errno = 0;
        const int appliedNice = getpriority(PRIO_PROCESS, 0);
        qWarning().nospace()
            << "[capture-thread] cpu=" << pinCpu
            << " policy="
            << (appliedPolicy == SCHED_OTHER ? "SCHED_OTHER" : "UNEXPECTED_RT")
            << " nice=" << appliedNice
            << " (RT throttling disabled)";
    }

    m_timeout       = false;
    m_frameSeq      = 0;
    m_lastIndex     = -1;
    m_inputFrameCount = 0;
    m_processFrameCount = 0;
    m_inputFPS      = 0.0;
    m_processFPS    = 0.0;

    QElapsedTimer frameWatchdog;
    frameWatchdog.start();
    const int FRAME_TIMEOUT_MS = 150;

    QElapsedTimer statsTimer;
    statsTimer.start();
    unsigned int warmupFramesRemaining = 2;
    unsigned int unstableSourceFrames = 0;
    // ---- capture diagnostics (1 Hz aggregate) ----
    unsigned int slotSkipCount = 0;
    qint64 procTotalUs = 0;
    qint64 procMaxUs = 0;
    unsigned int procCount = 0;
    unsigned int lastHwFrameCount = 0;
    std::vector<uint8_t> frameRotateScratch(static_cast<size_t>(FRAME_WIDTH) * 2u);
    // Track the seam as an unwrapped linear-frame phase.  A column-only shift
    // loses one complete row whenever the seam crosses x=1279 -> x=0, which is
    // exactly the residual top-to-bottom rolling seen on FMC.
    int activeFrameShiftPixels = 0;
    int previousSeamColumn = 0;
    int seamVelocityPixels = 0;
    unsigned int invalidSeamFrameCount = 0;
    unsigned int stableSeamFrameCount = 0;
    bool haveSeamMeasurement = false;
    bool fmcPhaseLocked = false;
    // Real-board feedback shows the original left correction still rolls.
    // Start with the opposite sign, then verify it from sparse raw-frame
    // correlation. +1 means rotate left; -1 means rotate right.
    int fmcCorrectionDirection = -1;
    bool fmcDirectionLocked = false;
    constexpr int motionSampleCount = 256;
    constexpr int motionRadius = 64;
    constexpr int motionWindow = motionRadius * 2 + 1;
    std::vector<uint16_t> previousMotionWindows(
        static_cast<size_t>(motionSampleCount * motionWindow));
    bool havePreviousMotionSamples = false;
    uint64_t leftCorrectionScore = 0;
    uint64_t rightCorrectionScore = 0;
    unsigned int motionScoreFrames = 0;
    FpgaInputSource activeFrameSource = FpgaInputSource::FMC;
    uint8_t activeSourceEpoch = 0;
    bool haveSourceEpoch = false;
    // ===================================================================
    // 【帧率提升关键4】采集线程主循环 — 老路 vs 新路
    //
    // 老路 (useMmap=false, 约30fps):
    //   ::read() 阻塞等帧 → 内核拷1.84MB(~20ms) → 用户态再拷1.84MB(~1.5ms)
    //   → 每帧约30ms → 1000/30 = 33fps 上限
    //
    // 新路 (useMmap=true, 60fps):
    //   BAR0 PIO 读状态(~2μs) → 直接读mmap内存(0拷贝) → 传指针(0ms)
    //   → 每帧约0ms → 不受采集限制, 受FPGA输入60fps限制
    // ===================================================================
    const bool useMmap = m_device->hasDmaMmap();

#ifdef ARM_FINAL_FPGA_DIAGNOSTICS
    auto dumpFpgaDebug = [&](const char *tag) {
        static bool hasFpgaCounterBaseline = false;
        static unsigned int lastVcVsCount = 0;
        static unsigned int lastVcTxCount = 0;
        static unsigned int lastDmaReqCount = 0;
        static unsigned int lastDmaDoneCount = 0;
        static unsigned int lastFrameDoneCountDbg = 0;

        // FPGA BAR0 completion path only returns lane 0 of each 128-bit line.
        // Keep every debug register on a 16-byte boundary.
        // 0x2d0 mirrors the legacy 0x144 host-control word on a 16-byte BAR
        // boundary, so RK3568 always receives the correct 32-bit lane.
        const unsigned int pioDebugStatus = m_device->pioReadReg(0x2d0);
        const unsigned int sourceStatus = m_device->pioReadReg(0x310);
        const unsigned int dmaDbgStatus = m_device->pioReadReg(0x150);
        const unsigned int dmaDbgFrame = m_device->pioReadReg(0x160);
        const unsigned int dmaDbgWrCnt = m_device->pioReadReg(0x170);
        const unsigned int dmaDbgTxState = m_device->pioReadReg(0x180);
        const unsigned int pioDbgLastAddr = m_device->pioReadReg(0x190);
        const unsigned int pioDbgLastData = m_device->pioReadReg(0x1a0);
        const unsigned int pioDbgWrCount = m_device->pioReadReg(0x1b0);
        const unsigned int videoDbgInputs = m_device->pioReadReg(0x1c0);
        const unsigned int videoDbgHb = m_device->pioReadReg(0x1d0);
        const unsigned int fifoDbgLevel = m_device->pioReadReg(0x1e0);
        const unsigned int fifoDbgFlags = m_device->pioReadReg(0x1f0);
        const unsigned int txDbgSeen = m_device->pioReadReg(0x200);
        const unsigned int txDbgMaxWr = m_device->pioReadReg(0x210);
        const unsigned int hdmiDbgStatus = m_device->pioReadReg(0x220);
        const unsigned int hdmiDbgSeen = m_device->pioReadReg(0x230);
        const unsigned int vcDbgState = m_device->pioReadReg(0x240);
        const unsigned int vcDbgXy = m_device->pioReadReg(0x250);
        const unsigned int vcDbgVsCount = m_device->pioReadReg(0x260);
        const unsigned int vcDbgTxCount = m_device->pioReadReg(0x270);
        const unsigned int dmaDbgReqCount = m_device->pioReadReg(0x280);
        const unsigned int dmaDbgDoneCount = m_device->pioReadReg(0x290);
        const unsigned int frameDoneDbgCount = m_device->pioReadReg(0x2a0);
        const unsigned int iicDbgStatus = m_device->pioReadReg(0x2b0);
        const unsigned int rxDbgFsm = m_device->pioReadReg(0x2c0);
        const auto hex32 = [](unsigned int value) {
            return QString("0x%1").arg(value, 8, 16, QChar('0'));
        };
        const bool dmaReq = (dmaDbgStatus & 0x1u) != 0;
            const bool dmaCmdRdy = ((dmaDbgStatus >> 1) & 0x1u) != 0;
            const bool dmaTxDone = ((dmaDbgStatus >> 2) & 0x1u) != 0;
            const bool dmaRdEn = ((dmaDbgStatus >> 3) & 0x1u) != 0;
            const bool dmaFifoFullSeen = ((dmaDbgStatus >> 4) & 0x1u) != 0;
            const bool dmaFifoAlmostFullSeen = ((dmaDbgStatus >> 5) & 0x1u) != 0;
            const bool dmaFifoReadEmptySeen = ((dmaDbgStatus >> 6) & 0x1u) != 0;
            const bool dmaLineBacklogSeen = ((dmaDbgStatus >> 7) & 0x1u) != 0;
            const unsigned int dmaFifoHighWater = (dmaDbgStatus >> 9) & 0xffffu;
            const bool dmaFifoDiagValid = ((dmaDbgStatus >> 31) & 0x1u) != 0;
        const unsigned int dbgFrameCnt = dmaDbgFrame & 0xffffu;
        const unsigned int dbgWrIndex = (dmaDbgFrame >> 16) & 0x3u;
        const unsigned int dbgWrCnt = dmaDbgWrCnt & 0xffffu;
        const bool dbgStart = (dmaDbgTxState & 0x1u) != 0;
        const bool dbgStartSync = ((dmaDbgTxState >> 1) & 0x1u) != 0;
        const bool dbgVideoVs = ((dmaDbgTxState >> 2) & 0x1u) != 0;
        const bool dbgVideoDe = ((dmaDbgTxState >> 3) & 0x1u) != 0;
        const bool dbgVideoVsRst = ((dmaDbgTxState >> 4) & 0x1u) != 0;
        const bool dbgLineReq = ((dmaDbgTxState >> 5) & 0x1u) != 0;
        const bool dbgPciLineReq = ((dmaDbgTxState >> 6) & 0x1u) != 0;
        const bool dbgDmaReq = ((dmaDbgTxState >> 7) & 0x1u) != 0;
        const bool dbgDmaCmdRdy = ((dmaDbgTxState >> 8) & 0x1u) != 0;
        const bool dbgDmaTxDone = ((dmaDbgTxState >> 9) & 0x1u) != 0;
        const bool dbgDmaRdEn = ((dmaDbgTxState >> 10) & 0x1u) != 0;
        const bool dbgVideoRstN = ((videoDbgInputs >> 4) & 0x1u) != 0;
        const bool dbgRawVs = ((videoDbgInputs >> 3) & 0x1u) != 0;
        const bool dbgRawDe = ((videoDbgInputs >> 2) & 0x1u) != 0;
        const bool dbgCrtlVs = ((videoDbgInputs >> 1) & 0x1u) != 0;
        const bool dbgCrtlDe = (videoDbgInputs & 0x1u) != 0;
        const unsigned int dbgVideoHb = videoDbgHb & 0xffffu;
        const unsigned int dbgFifoLevel = fifoDbgLevel & 0xffffu;
        const bool dbgFifoOverflowSeen = (fifoDbgFlags & 0x1u) != 0;
        const bool dbgFifoRdEmpty = ((fifoDbgFlags >> 1) & 0x1u) != 0;
        const bool dbgFifoAlmostEmpty = ((fifoDbgFlags >> 2) & 0x1u) != 0;
        const bool dbgFifoAlmostFull = ((fifoDbgFlags >> 3) & 0x1u) != 0;
        const bool dbgSeenVideoDe = (txDbgSeen & 0x1u) != 0;
        const bool dbgSeenVideoVs = ((txDbgSeen >> 1) & 0x1u) != 0;
        const bool dbgSeenLineReq = ((txDbgSeen >> 2) & 0x1u) != 0;
        const bool dbgSeenPciLineReq = ((txDbgSeen >> 3) & 0x1u) != 0;
        const bool dbgSeenDmaReq = ((txDbgSeen >> 4) & 0x1u) != 0;
        const unsigned int dbgMaxWrCnt = txDbgMaxWr & 0xffffu;
        const bool dbgHdmiRgbNonzero = (hdmiDbgStatus & 0x1u) != 0;
        const bool dbgHdmiRawDe = ((hdmiDbgStatus >> 1) & 0x1u) != 0;
        const bool dbgHdmiRawHs = ((hdmiDbgStatus >> 2) & 0x1u) != 0;
        const bool dbgHdmiRawVs = ((hdmiDbgStatus >> 3) & 0x1u) != 0;
        const bool dbgHdmiRstnOut = ((hdmiDbgStatus >> 4) & 0x1u) != 0;
        const bool dbgHdmiRxRstN = ((hdmiDbgStatus >> 5) & 0x1u) != 0;
        const bool dbgHdmiRxInitDone = ((hdmiDbgStatus >> 6) & 0x1u) != 0;
        const bool dbgHdmiInitOver = ((hdmiDbgStatus >> 7) & 0x1u) != 0;
        const bool dbgHdmiLocked = ((hdmiDbgStatus >> 8) & 0x1u) != 0;
        const bool dbgHdmiStartFlag = ((hdmiDbgStatus >> 9) & 0x1u) != 0;
        const bool dbgHdmiDeD0 = ((hdmiDbgStatus >> 10) & 0x1u) != 0;
        const bool dbgHdmiVsD0 = ((hdmiDbgStatus >> 11) & 0x1u) != 0;
        const bool dbgHdmiRgbChange = ((hdmiDbgStatus >> 12) & 0x1u) != 0;
        const bool dbgSeenRawHs = (hdmiDbgSeen & 0x1u) != 0;
        const bool dbgSeenRawVs2 = ((hdmiDbgSeen >> 1) & 0x1u) != 0;
        const bool dbgSeenRawDe2 = ((hdmiDbgSeen >> 2) & 0x1u) != 0;
        const bool dbgSeenRgbNonzero = ((hdmiDbgSeen >> 3) & 0x1u) != 0;
        const bool dbgSeenRgbChange = ((hdmiDbgSeen >> 4) & 0x1u) != 0;
        const bool dbgSeenRxInitDone = ((hdmiDbgSeen >> 5) & 0x1u) != 0;
        const bool dbgSeenInitOver = ((hdmiDbgSeen >> 6) & 0x1u) != 0;
        const unsigned int dbgVcState = (vcDbgState >> 6) & 0x7u;
        const bool dbgVcStart = ((vcDbgState >> 5) & 0x1u) != 0;
        const bool dbgVcVsRise = ((vcDbgState >> 4) & 0x1u) != 0;
        const bool dbgVcRawVs = ((vcDbgState >> 3) & 0x1u) != 0;
        const bool dbgVcRawDe = ((vcDbgState >> 2) & 0x1u) != 0;
        const bool dbgVcOutVs = ((vcDbgState >> 1) & 0x1u) != 0;
        const bool dbgVcOutDe = (vcDbgState & 0x1u) != 0;
        const unsigned int dbgVcY = (vcDbgXy >> 8) & 0xffu;
        const unsigned int dbgVcX = vcDbgXy & 0xffu;
        const unsigned int dbgCfgHeartbeat = (iicDbgStatus >> 16) & 0xffffu;
        const unsigned int dbgIicDataOut = (iicDbgStatus >> 8) & 0xffu;
        const bool dbgMsRstn = ((iicDbgStatus >> 7) & 0x1u) != 0;
        const bool dbgTxInitOver = ((iicDbgStatus >> 6) & 0x1u) != 0;
        const bool dbgRxInitOver = ((iicDbgStatus >> 5) & 0x1u) != 0;
        const bool dbgIicBusy = ((iicDbgStatus >> 4) & 0x1u) != 0;
        const bool dbgIicTrig = ((iicDbgStatus >> 3) & 0x1u) != 0;
        const bool dbgIicByteOver = ((iicDbgStatus >> 2) & 0x1u) != 0;
        const bool dbgIicSda = ((iicDbgStatus >> 1) & 0x1u) != 0;
        const bool dbgIicScl = (iicDbgStatus & 0x1u) != 0;
        const bool dbgIicTrigSeen = ((rxDbgFsm >> 31) & 0x1u) != 0;
        const bool dbgIicBusySeen = ((rxDbgFsm >> 30) & 0x1u) != 0;
        const bool dbgIicByteSeen = ((rxDbgFsm >> 29) & 0x1u) != 0;
        const bool dbgIicSclToggleSeen = ((rxDbgFsm >> 28) & 0x1u) != 0;
        const bool dbgIicSdaLowSeen = ((rxDbgFsm >> 27) & 0x1u) != 0;
        const unsigned int dbgRxDriCnt = (rxDbgFsm >> 7) & 0x1ffu;
        const unsigned int dbgRxState = rxDbgFsm & 0x7fu;
        const bool dbgPioFrameDone = (pioDebugStatus & 0x1u) != 0;
        const bool dbgPioStarted = ((pioDebugStatus >> 1) & 0x1u) != 0;
        const bool dbgPioDmaConfigured = ((pioDebugStatus >> 2) & 0x1u) != 0;
        const unsigned int dbgPioDmaAddressMask = (pioDebugStatus >> 3) & 0xffu;
        const unsigned int activeSourceCode =
            (((sourceStatus >> 3) & 0x1u) << 1) | (sourceStatus & 0x1u);
        const unsigned int requestedSourceCode =
            (((sourceStatus >> 4) & 0x1u) << 1) | ((sourceStatus >> 1) & 0x1u);
        const bool activeHdmi = activeSourceCode == 1u;
        const bool sourceSwitchBusy = ((sourceStatus >> 2) & 0x1u) != 0;
        const auto sourceCodeName = [](unsigned int sourceCode) {
            return sourceCode == 2u ? "FIBER"
                 : sourceCode == 1u ? "HDMI"
                 : sourceCode == 0u ? "FMC" : "INVALID";
        };
        const bool deepDebugPresent = hdmiDbgStatus != 0u || hdmiDbgSeen != 0u
                                   || iicDbgStatus != 0u || rxDbgFsm != 0u;
        const auto rxStateName = [](unsigned int state) {
            switch (state) {
            case 0x01: return "IDLE";
            case 0x02: return "CONECT";
            case 0x04: return "INIT";
            case 0x08: return "WAIT";
            case 0x10: return "STA_RD";
            case 0x20: return "SETING";
            case 0x40: return "RD_BAK";
            default: return "UNKNOWN";
            }
        };
        const unsigned int deltaVcVsCount =
            hasFpgaCounterBaseline ? (vcDbgVsCount - lastVcVsCount) : 0u;
        const unsigned int deltaVcTxCount =
            hasFpgaCounterBaseline ? (vcDbgTxCount - lastVcTxCount) : 0u;
        const unsigned int deltaDmaReqCount =
            hasFpgaCounterBaseline ? (dmaDbgReqCount - lastDmaReqCount) : 0u;
        const unsigned int deltaDmaDoneCount =
            hasFpgaCounterBaseline ? (dmaDbgDoneCount - lastDmaDoneCount) : 0u;
        const unsigned int deltaFrameDoneDbgCount =
            hasFpgaCounterBaseline ? (frameDoneDbgCount - lastFrameDoneCountDbg) : 0u;
        lastVcVsCount = vcDbgVsCount;
        lastVcTxCount = vcDbgTxCount;
        lastDmaReqCount = dmaDbgReqCount;
        lastDmaDoneCount = dmaDbgDoneCount;
        lastFrameDoneCountDbg = frameDoneDbgCount;
        hasFpgaCounterBaseline = true;
        qWarning().nospace() << "[fpga-debug] tag=" << tag
            << " pio_debug=" << hex32(pioDebugStatus)
            << " source_status=" << hex32(sourceStatus)
            << " active=" << sourceCodeName(activeSourceCode)
            << " requested=" << sourceCodeName(requestedSourceCode)
            << " busy=" << sourceSwitchBusy
            << " epoch=" << ((sourceStatus >> 8) & 0xffu);
        qWarning().nospace()
            << "[fpga-host-control] start=" << dbgPioStarted
            << " dma_configured=" << dbgPioDmaConfigured
            << " dma_address_mask=0x" << QString::number(dbgPioDmaAddressMask, 16)
            << " frame_done_latched=" << dbgPioFrameDone
            << " deep_debug_present=" << deepDebugPresent;
        qWarning().nospace()
            << "[fpga-raw]"
            << " dma=" << hex32(dmaDbgStatus)
            << " frame=" << hex32(dmaDbgFrame)
            << " wr_cnt=" << hex32(dmaDbgWrCnt)
            << " tx=" << hex32(dmaDbgTxState)
            << " video=" << hex32(videoDbgInputs)
            << " hb=" << hex32(videoDbgHb)
            << " fifo_level=" << hex32(fifoDbgLevel)
            << " fifo_flags=" << hex32(fifoDbgFlags)
            << " seen=" << hex32(txDbgSeen)
            << " hdmi=" << hex32(hdmiDbgStatus)
            << " hdmi_seen=" << hex32(hdmiDbgSeen)
            << " vc_state=" << hex32(vcDbgState)
            << " vc_xy=" << hex32(vcDbgXy)
            << " vc_vs=" << hex32(vcDbgVsCount)
            << " vc_tx=" << hex32(vcDbgTxCount)
            << " dma_req=" << hex32(dmaDbgReqCount)
            << " dma_done=" << hex32(dmaDbgDoneCount)
            << " frame_done=" << hex32(frameDoneDbgCount)
            << " iic=" << hex32(iicDbgStatus)
            << " rx_fsm=" << hex32(rxDbgFsm);
        qDebug().nospace()
            << "[fpga-dma] req=" << dmaReq
            << " cmd_rdy=" << dmaCmdRdy
            << " tx_done=" << dmaTxDone
            << " rd_en=" << dmaRdEn
            << " frame_cnt=" << dbgFrameCnt
            << " wr_index=" << dbgWrIndex;
        qWarning().nospace()
            << "[fpga-fifo-health] valid=" << dmaFifoDiagValid
            << " full=" << dmaFifoFullSeen
            << " almost_full=" << dmaFifoAlmostFullSeen
            << " read_empty=" << dmaFifoReadEmptySeen
            << " line_backlog=" << dmaLineBacklogSeen
            << " high_water_pixels=" << dmaFifoHighWater;
        qDebug().nospace()
            << "[fpga-tx] start=" << dbgStart
            << " start_sync=" << dbgStartSync
            << " vs=" << dbgVideoVs
            << " de=" << dbgVideoDe
            << " vs_rst=" << dbgVideoVsRst
            << " line_req=" << dbgLineReq
            << " pci_line_req=" << dbgPciLineReq
            << " dma_req=" << dbgDmaReq
            << " dma_cmd_rdy=" << dbgDmaCmdRdy
            << " dma_tx_done=" << dbgDmaTxDone
            << " dma_rd_en=" << dbgDmaRdEn
            << " wr_cnt=" << dbgWrCnt;
        qDebug().nospace()
            << "[fpga-pio] wr_count=" << pioDbgWrCount
            << " last_addr=0x" << QString::number(pioDbgLastAddr, 16)
            << " last_data=0x" << QString::number(pioDbgLastData, 16);
        qDebug().nospace()
            << "[fpga-video] rst_n=" << dbgVideoRstN
            << " raw_vs=" << dbgRawVs
            << " raw_de=" << dbgRawDe
            << " crtl_vs=" << dbgCrtlVs
            << " crtl_de=" << dbgCrtlDe
            << " hb=" << dbgVideoHb;
        qDebug().nospace()
            << "[fpga-fifo] level=" << dbgFifoLevel
            << " overflow_seen=" << dbgFifoOverflowSeen
            << " rd_empty=" << dbgFifoRdEmpty
            << " almost_empty=" << dbgFifoAlmostEmpty
            << " almost_full=" << dbgFifoAlmostFull;
        qDebug().nospace()
            << "[fpga-seen] de=" << dbgSeenVideoDe
            << " vs=" << dbgSeenVideoVs
            << " line_req=" << dbgSeenLineReq
            << " pci_line_req=" << dbgSeenPciLineReq
            << " dma_req=" << dbgSeenDmaReq
            << " max_wr_cnt=" << dbgMaxWrCnt;
        qWarning().nospace()
            << "[hdmi-status] locked=" << dbgHdmiLocked
            << " start=" << dbgHdmiStartFlag
            << " rstn_out=" << dbgHdmiRstnOut
            << " rx_rst_n=" << dbgHdmiRxRstN
            << " rx_init_done=" << dbgHdmiRxInitDone
            << " init_over=" << dbgHdmiInitOver
            << " raw_vs=" << dbgHdmiRawVs
            << " raw_hs=" << dbgHdmiRawHs
            << " raw_de=" << dbgHdmiRawDe
            << " de_d0=" << dbgHdmiDeD0
            << " vs_d0=" << dbgHdmiVsD0
            << " rgb_nonzero=" << dbgHdmiRgbNonzero
            << " rgb_change=" << dbgHdmiRgbChange;
        qWarning().nospace()
            << "[hdmi-seen2] raw_hs=" << dbgSeenRawHs
            << " raw_vs=" << dbgSeenRawVs2
            << " raw_de=" << dbgSeenRawDe2
            << " rgb_nonzero=" << dbgSeenRgbNonzero
            << " rgb_change=" << dbgSeenRgbChange
            << " rx_init_done=" << dbgSeenRxInitDone
            << " init_over=" << dbgSeenInitOver;
        qWarning().nospace()
            << "[video-crtl] state=" << dbgVcState
            << " start=" << dbgVcStart
            << " vs_rise=" << dbgVcVsRise
            << " raw_vs=" << dbgVcRawVs
            << " raw_de=" << dbgVcRawDe
            << " out_vs=" << dbgVcOutVs
            << " out_de=" << dbgVcOutDe
            << " x=" << dbgVcX
            << " y=" << dbgVcY;
        qWarning().nospace()
            << "[fpga-count] vc_vs=" << vcDbgVsCount
            << " (+" << deltaVcVsCount << ")"
            << " vc_tx=" << vcDbgTxCount
            << " (+" << deltaVcTxCount << ")"
            << " dma_req=" << dmaDbgReqCount
            << " (+" << deltaDmaReqCount << ")"
            << " dma_done=" << dmaDbgDoneCount
            << " (+" << deltaDmaDoneCount << ")"
            << " frame_done=" << frameDoneDbgCount
            << " (+" << deltaFrameDoneDbgCount << ")";
        qWarning().nospace()
            << "[ms72xx-i2c] cfg_hb=" << dbgCfgHeartbeat
            << " rstn=" << dbgMsRstn
            << " rx_init=" << dbgRxInitOver
            << " tx_init=" << dbgTxInitOver
            << " state=" << rxStateName(dbgRxState)
            << "(0x" << QString::number(dbgRxState, 16) << ")"
            << " dri_cnt=" << dbgRxDriCnt
            << " data_out=0x" << QString::number(dbgIicDataOut, 16)
            << " busy=" << dbgIicBusy
            << " trig=" << dbgIicTrig
            << " byte_over=" << dbgIicByteOver
            << " scl=" << dbgIicScl
            << " sda=" << dbgIicSda
            << " seen(trig/busy/byte/scl_toggle/sda_low)="
            << dbgIicTrigSeen << "/" << dbgIicBusySeen << "/"
            << dbgIicByteSeen << "/" << dbgIicSclToggleSeen << "/"
            << dbgIicSdaLowSeen;

        QString blockedAt;
        if (activeSourceCode != requestedSourceCode || sourceSwitchBusy) {
            blockedAt = "source-switch-not-complete";
        } else if (!deepDebugPresent) {
            // SOURCE_STATUS bits 4:3 are reserved in the current FPGA image.
            // Without its optional deep map, only the missing frame_done is
            // known; do not report invented HDMI/FMC-valid states.
            blockedAt = "no-frame-done-deep-debug-not-implemented";
        } else if (activeHdmi && !dbgHdmiRxInitDone
                   && dbgRxState == 0x10u) {
            blockedAt = "ms7200-sta-rd-waiting-for-valid-hdmi-timing";
        } else if (activeHdmi && !dbgHdmiRxInitDone) {
            blockedAt = "ms7200-receiver-initialization-not-complete";
        } else if (!dbgPioStarted) {
            blockedAt = "capture-start-command-not-latched";
        } else if (!dbgPioDmaConfigured || dbgPioDmaAddressMask != 0xffu) {
            blockedAt = "dma-base-addresses-not-fully-configured";
        } else if (!dbgHdmiLocked || !dbgHdmiRstnOut || !dbgHdmiRxRstN) {
            blockedAt = "hdmi-clock-or-reset";
        } else if (!dbgMsRstn) {
            blockedAt = "ms72xx-reset-not-released";
        } else if (!dbgIicTrigSeen || !dbgIicSclToggleSeen) {
            blockedAt = "ms7200-i2c-not-running";
        } else if (dbgRxState == 0x02 && dbgIicDataOut != 0x5a) {
            blockedAt = "ms7200-device-id-not-read";
        } else if (dbgRxState == 0x04) {
            blockedAt = "ms7200-register-init-in-progress";
        } else if (dbgRxState == 0x10 && !dbgSeenRawVs2 && !dbgSeenRawDe2) {
            blockedAt = "hdmi-source-clock-not-detected";
        } else if (!dbgSeenRawVs2 || !dbgSeenRawDe2) {
            blockedAt = "hdmi-no-vs-or-de";
        } else if (vcDbgVsCount == 0) {
            blockedAt = "video-crtl-no-vs-edge";
        } else if (vcDbgTxCount == 0) {
            blockedAt = "video-crtl-not-entered-tx";
        } else if (dmaDbgReqCount == 0) {
            blockedAt = "no-complete-line-for-dma-request";
        } else if (dmaDbgDoneCount == 0) {
            blockedAt = "pcie-dma-no-completion";
        } else if (dbgFifoOverflowSeen) {
            blockedAt = "video-fifo-overflow";
        } else if (frameDoneDbgCount == 0) {
            blockedAt = "dma-runs-but-frame-never-reaches-720-lines";
        } else {
            blockedAt = "none-pipeline-running";
        }
        qWarning().nospace()
            << "[fpga-diagnosis] blocked_at=" << blockedAt
            << " (stable repeated BAR0 debug reads)";

        qWarning() << "[rk-system-recovery] press R to release DMA, reload the RK PCIe driver, reset/rescan the endpoint, and restart";
    };
    // One snapshot is taken only when the frame watchdog first expires.  The
    // many BAR reads can perturb the read-to-clear 0x140 path, so never poll
    // this continuously while video is healthy.
    (void)dumpFpgaDebug;
#endif

    RgaFrameCopier rgaFrameCopier;
    bool rgaCopyDisabled = false;
    const char *copyEnv = std::getenv("CAPTURE_COPY");
    const int requestedCopyMode = copyEnv ? std::atoi(copyEnv) : 5;
    if (requestedCopyMode == 5)
        qWarning() << "[rga-copy] RGA hardware DMA-to-cache snapshot enabled";

    while (!m_stopRequested.load(std::memory_order_acquire)) {

        // ---- 获取帧状态 + 数据 ----
        FrameStatus status;
        int rd;
        if (useMmap) {
            // 最快路径: BAR0直读PIO寄存器0x140取状态, 帧数据从mmap'd DMA buffer直读
            // 完全绕过内核驱动, 无任何syscall
            rd = m_device->readPioStatus(status);
        } else {
            // 旧路径: 1.8MB大buffer, 内核搬运帧数据到m_rawBuf4
            rd = m_device->readFrame(m_rawBuf4, FRAME_SIZE_RGB565, status);
        }
        if (rd < 0) {
            emit captureError(QString("read: %1").arg(strerror(errno)));
            break;
        }

        // ---- 帧有效性 ----
        if (!status.frame_done) {
            // 无论旧路径还是 MMAP 直读路径，只要 FPGA 没有宣告新帧完成，
            // 就不能继续把 DMA 缓冲区内容当作新帧提交，否则会反复显示旧数据。
            usleep(500);

            if (frameWatchdog.elapsed() > FRAME_TIMEOUT_MS && !m_timeout) {
                m_timeout = true;
#ifdef ARM_FINAL_FPGA_DIAGNOSTICS
                dumpFpgaDebug("frame-timeout");
#endif
                emit frameTimeout();
            }
            continue;
        }

        // 帧到
        frameWatchdog.restart();
        if (m_timeout) {
            m_timeout = false;
            emit frameResumed();
        }
        // A source epoch change starts a new independent frame sequence.  Drop
        // every slot from the previous source and recalibrate only if the new
        // source is FMC.
        if (!haveSourceEpoch || status.source_epoch != activeSourceEpoch
            || status.source != activeFrameSource) {
            activeFrameSource = status.source;
            activeSourceEpoch = status.source_epoch;
            haveSourceEpoch = true;
            qWarning().nospace()
                << "[capture-source] source=" << captureSourceName(activeFrameSource)
                << " code=" << static_cast<unsigned int>(activeFrameSource)
                << " epoch=" << static_cast<unsigned int>(activeSourceEpoch);
            m_lastIndex = -1;
            warmupFramesRemaining = 2;
            activeFrameShiftPixels = 0;
            previousSeamColumn = 0;
            seamVelocityPixels = 0;
            invalidSeamFrameCount = 0;
            stableSeamFrameCount = 0;
            haveSeamMeasurement = false;
            fmcPhaseLocked = false;
            fmcCorrectionDirection = -1;
            fmcDirectionLocked = false;
            havePreviousMotionSamples = false;
            leftCorrectionScore = 0;
            rightCorrectionScore = 0;
            motionScoreFrames = 0;
        }

        if (status.buffer_index == m_lastIndex) {
            usleep(100);
            continue;
        }
        if (m_lastIndex >= 0 && warmupFramesRemaining == 0) {
            // FPGA cycles through 4 DMA slots; a jump of >1 slot means two
            // frame completions were merged into one 0x140 notification.
            const int slotDelta = (status.buffer_index - m_lastIndex + 4) % 4;
            if (slotDelta != 1)
                slotSkipCount += static_cast<unsigned int>(slotDelta - 1);
        }
        m_lastIndex = status.buffer_index;
        m_inputFrameCount++;

        // The first status after a restart may describe a frame that was
        // already in flight before software synchronized to VSYNC.
        if (warmupFramesRemaining > 0) {
            --warmupFramesRemaining;
            continue;
        }

        const auto procStart = std::chrono::steady_clock::now();

        // pcie_tx_fun now exports r_dma_index with the completion pulse, so
        // 0x140 identifies the freshly completed slot while FPGA moves on to
        // the next one. Reading an extra-deferred completion is unsafe at HDMI
        // 60 Hz when ARM misses notifications: that older slot can be reused
        // while the 1.84 MB snapshot is copied.
        const int sourceIndex = status.buffer_index;

        // ---- 直接写入 RGB565 环形缓冲 ----
        uint8_t *src;
        if (useMmap) {
            // 从 mmap'd DMA 缓冲区直读 (uncached → cached)
            src = (uint8_t*)m_device->dmaUserAddr() + sourceIndex * FRAME_SIZE_RGB565;
        } else {
            // 从内核填充的 m_rawBuf4 读取 (cached → cached)
            src = m_rawBuf4 + sourceIndex * FRAME_SIZE_RGB565;
        }
        uint8_t *dst = nullptr;
        const bool directMappedHighRate = useMmap
                                       && activeFrameSource != FpgaInputSource::FMC;

        if (directMappedHighRate) {
            // Runtime modes: 0 publishes uncached DMA directly, 2 performs a
            // single-core wide copy, 3/4 are diagnostic CPU copies, and 5
            // uses RK3568 RGA for the physical-DMA-to-cached snapshot.
            const int copyMode = requestedCopyMode;
            if (copyMode == 0) {
                // Zero-copy publish: no memcpy in the capture thread.  Input
                // should reach 60 fps here, but UI/AI consumers then read the
                // uncached DMA buffer directly and get slow again.
                __sync_synchronize();
                m_buffer->commitMappedFrame(src, m_frameSeq);
            } else {
                // The capture thread does the single uncached DMA->cached
                // copy so UI/AI consumers never read the whole 1.84 MB from
                // uncached coherent memory on the UI thread.
                dst = m_buffer->lockWriteBuffer();
                __sync_synchronize();
                bool copiedByRga = false;
                if (copyMode == 5 && !rgaCopyDisabled) {
                    QString rgaError;
                    copiedByRga = rgaFrameCopier.copy(
                        m_device->dmaPhysicalBase(), sourceIndex, dst, rgaError);
                    if (!copiedByRga) {
                        rgaCopyDisabled = true;
                        qWarning().nospace()
                            << "[rga-copy] disabled: " << rgaError
                            << "; falling back to CAPTURE_COPY=2";
                    }
                }
                if (!copiedByRga && copyMode == 3)
                    copyRgb565ThreeWay(dst, src, FRAME_SIZE_RGB565);
                else if (!copiedByRga && copyMode == 4)
                    copyRgb565FourWay(dst, src, FRAME_SIZE_RGB565);
                else if (!copiedByRga)
                    copyRgb565Wide(dst, src, FRAME_SIZE_RGB565);
                __sync_synchronize();
                m_buffer->commitFrame(m_frameSeq);
            }
        } else {
        // Snapshot even mmap-backed DMA frames.  The additional 1.84 MB copy
        // is required for FMC because correction modifies the frame in-place.
        dst = m_buffer->lockWriteBuffer();
        __sync_synchronize();
        const uint32_t sourceSignatureBefore = sampledFrameSignature(src);
        copyRgb565Wide(dst, src, FRAME_SIZE_RGB565);
        __sync_synchronize();
        const uint32_t sourceSignatureAfter = sampledFrameSignature(src);
        const uint32_t snapshotSignature = sampledFrameSignature(dst);

        if (sourceSignatureBefore != sourceSignatureAfter
            || snapshotSignature != sourceSignatureAfter) {
            ++unstableSourceFrames;
            if (unstableSourceFrames <= 8 || (unstableSourceFrames % 60u) == 0u) {
                qWarning().nospace()
                    << "[dma-integrity] DROP unstable source"
                    << " event_idx=" << status.buffer_index
                    << " read_idx=" << sourceIndex
                    << " before=0x" << QString::number(sourceSignatureBefore, 16)
                    << " after=0x" << QString::number(sourceSignatureAfter, 16)
                    << " copy=0x" << QString::number(snapshotSignature, 16);
            }
            continue;
        }

        SeamEstimate seam;
        if (activeFrameSource == FpgaInputSource::FMC
            && (!fmcPhaseLocked || (m_frameSeq % 60u) == 0u))
            seam = estimateVerticalWrapSeam(dst);

        if (activeFrameSource == FpgaInputSource::FMC) {
            const int framePixels = FRAME_WIDTH * FRAME_HEIGHT;
            auto normalizeFrameShift = [&](int shift) {
                shift %= framePixels;
                return shift < 0 ? shift + framePixels : shift;
            };
            auto wrappedColumnDelta = [](int to, int from) {
                int delta = to - from;
                if (delta > FRAME_WIDTH / 2) delta -= FRAME_WIDTH;
                if (delta < -FRAME_WIDTH / 2) delta += FRAME_WIDTH;
                return delta;
            };

            // Advance once per captured frame. A valid measurement below will
            // remove the small prediction error without losing accumulated
            // whole-row phase.
            if (haveSeamMeasurement && !fmcPhaseLocked)
                activeFrameShiftPixels = normalizeFrameShift(
                    activeFrameShiftPixels + seamVelocityPixels);

            const int predictedColumn = activeFrameShiftPixels % FRAME_WIDTH;
            const int predictionError = wrappedColumnDelta(seam.column, predictedColumn);
            const bool measurementUsable = !fmcPhaseLocked && seam.valid
                                        && (!haveSeamMeasurement
                                            || std::abs(predictionError) <= 96);

            if (measurementUsable) {
                if (haveSeamMeasurement) {
                    const int measuredDelta = wrappedColumnDelta(
                        seam.column, previousSeamColumn);
                    const int framesSinceMeasurement =
                        static_cast<int>(invalidSeamFrameCount) + 1;
                    const int rounding = framesSinceMeasurement / 2;
                    const int measuredVelocity = measuredDelta >= 0
                        ? (measuredDelta + rounding) / framesSinceMeasurement
                        : (measuredDelta - rounding) / framesSinceMeasurement;
                    if (std::abs(measuredVelocity) <= 64)
                        seamVelocityPixels = measuredVelocity;
                    if (std::abs(measuredDelta) <= 2)
                        ++stableSeamFrameCount;
                    else
                        stableSeamFrameCount = 0;
                    activeFrameShiftPixels = normalizeFrameShift(
                        activeFrameShiftPixels + predictionError);

                    if (stableSeamFrameCount >= 5) {
                        // Match the original eglf5.8 behaviour: startup phase
                        // may move briefly, but once it is stable the software
                        // freezes the correction instead of following scene
                        // edges forever.
                        fmcPhaseLocked = true;
                        seamVelocityPixels = 0;
                        if (!fmcDirectionLocked) {
                            fmcCorrectionDirection = -1;
                            fmcDirectionLocked = true;
                        }
                    }
                } else {
                    activeFrameShiftPixels = seam.column;
                    haveSeamMeasurement = true;
                    stableSeamFrameCount = 0;
                }
                previousSeamColumn = seam.column;
                invalidSeamFrameCount = 0;
            } else if (haveSeamMeasurement && !fmcPhaseLocked) {
                ++invalidSeamFrameCount;
                stableSeamFrameCount = 0;
            }

            // Determine correction sign from adjacent raw frames. If the
            // current raw frame best matches previous[i-d], the raw stream
            // moved right and must be corrected left; previous[i+d] implies
            // the opposite. Sparse sampling keeps this calibration cheap.
            if (!fmcDirectionLocked) {
                const uint16_t *rawPixels = reinterpret_cast<const uint16_t *>(dst);
                const int motionDelta = seamVelocityPixels;
                if (havePreviousMotionSamples
                    && motionDelta != 0
                    && std::abs(motionDelta) <= motionRadius) {
                    uint64_t frameLeftScore = 0;
                    uint64_t frameRightScore = 0;
                    for (int sample = 0; sample < motionSampleCount; ++sample) {
                        const int anchor = motionRadius
                            + sample * (framePixels - 2 * motionRadius) / motionSampleCount;
                        const size_t base = static_cast<size_t>(sample * motionWindow);
                        const uint16_t current = rawPixels[anchor];
                        frameLeftScore += static_cast<uint64_t>(rgb565Distance(
                            current, previousMotionWindows[base + motionRadius - motionDelta]));
                        frameRightScore += static_cast<uint64_t>(rgb565Distance(
                            current, previousMotionWindows[base + motionRadius + motionDelta]));
                    }
                    leftCorrectionScore += frameLeftScore;
                    rightCorrectionScore += frameRightScore;
                    ++motionScoreFrames;

                    if (motionScoreFrames >= 6) {
                        // Only override the real-board default (right) when
                        // left correlation is clearly better by at least 5%.
                        fmcCorrectionDirection =
                            leftCorrectionScore * 100u < rightCorrectionScore * 95u ? 1 : -1;
                        fmcDirectionLocked = true;
                    }
                }

                for (int sample = 0; sample < motionSampleCount; ++sample) {
                    const int anchor = motionRadius
                        + sample * (framePixels - 2 * motionRadius) / motionSampleCount;
                    const size_t base = static_cast<size_t>(sample * motionWindow);
                    for (int offset = -motionRadius; offset <= motionRadius; ++offset)
                        previousMotionWindows[base + motionRadius + offset] =
                            rawPixels[anchor + offset];
                }
                havePreviousMotionSamples = true;
            }
        }
        const int fmcFrameShift = haveSeamMeasurement
                                ? activeFrameShiftPixels : 0;
        if (activeFrameSource == FpgaInputSource::FMC && fmcFrameShift > 0) {
            if (fmcCorrectionDirection > 0)
                rotateRgb565FrameLeft(dst, fmcFrameShift, frameRotateScratch);
            else
                rotateRgb565FrameRight(dst, fmcFrameShift, frameRotateScratch);
        }

        m_buffer->commitFrame(m_frameSeq);
        }

        const qint64 captureReadyUs =
            std::chrono::duration_cast<std::chrono::microseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        emit frameReady(m_frameSeq, captureReadyUs);

        const qint64 procUs = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - procStart).count();
        procTotalUs += procUs;
        procMaxUs = std::max(procMaxUs, procUs);
        ++procCount;

        // ---- 统计 ----
        m_totalFrames++;
        m_processFrameCount++;
        m_frameSeq++;

        qint64 elapsed = statsTimer.elapsed();
        if (elapsed >= 1000) {
            const unsigned int inputFramesThisSec = m_inputFrameCount;
            const unsigned int processedFramesThisSec = m_processFrameCount;
            m_inputFPS = inputFramesThisSec * 1000.0 / elapsed;
            m_processFPS = processedFramesThisSec * 1000.0 / elapsed;

            emit statsUpdated(inputFramesThisSec,
                              processedFramesThisSec,
                              m_buffer->droppedFrames(),
                              m_inputFPS,
                              m_processFPS);

            // 1 Hz capture diagnostics. hw_delta is the FPGA 0x2a0 frame
            // counter delta (0 until the matching FPGA change is rebuilt).
            const unsigned int hwFrameNow = m_device->pioReadReg(0x2a0);
            const unsigned int fifoHealth = m_device->pioReadReg(0x150);
            qWarning().nospace()
                << "[capture-rate] input=" << inputFramesThisSec
                << " source=" << captureSourceName(activeFrameSource)
                << " processed=" << processedFramesThisSec
                << " slot_skips=" << slotSkipCount
                << " completed_est=" << (inputFramesThisSec + slotSkipCount)
                << " hw_delta="
                << (lastHwFrameCount ? hwFrameNow - lastHwFrameCount : 0u)
                << " copy_mode=" << requestedCopyMode
                << " proc_avg_us=" << (procCount ? procTotalUs / procCount : 0)
                << " proc_max_us=" << procMaxUs
                << " fifo_diag_valid=" << ((fifoHealth >> 31) & 0x1u)
                << " fifo_full=" << ((fifoHealth >> 4) & 0x1u)
                << " fifo_almost_full=" << ((fifoHealth >> 5) & 0x1u)
                << " fifo_read_empty=" << ((fifoHealth >> 6) & 0x1u)
                << " line_backlog=" << ((fifoHealth >> 7) & 0x1u)
                << " fifo_high_water_pixels=" << ((fifoHealth >> 9) & 0xffffu);
            lastHwFrameCount = hwFrameNow;
            slotSkipCount = 0;
            procTotalUs = 0;
            procMaxUs = 0;
            procCount = 0;

            m_inputFrameCount = 0;
            m_processFrameCount = 0;
            statsTimer.restart();
        }
    }

    m_running.store(false, std::memory_order_release);
}
