/* fb_probe.c — read the ACTIVE DRM framebuffer and report whether anything was drawn.
 *
 * Distinguishes two very different causes of a black screen:
 *   (a) the app IS rendering, but the content is black  -> content/UI problem
 *   (b) the framebuffer was never painted              -> Qt/EGLFS problem
 *
 * Uses the dumb-buffer mmap path: the framebuffer the app created is a dumb buffer,
 * so we can mmap the currently-scanning-out fb and sample it.
 *
 * Build: gcc -O2 -I/usr/include/libdrm -o fb_probe fb_probe.c -ldrm
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

/* parse "fb=NNN" from the active planes of the first CRTC we can read */
static int find_active_fb(int fd, uint32_t *out_fb)
{
    drmModeRes *res = drmModeGetResources(fd);
    if (!res) return -1;
    int found = -1;
    for (int i = 0; i < res->count_crtcs && found < 0; i++) {
        drmModeCrtc *c = drmModeGetCrtc(fd, res->crtcs[i]);
        if (!c) continue;
        if (c->buffer_id) { *out_fb = c->buffer_id; found = 0; }
        drmModeFreeCrtc(c);
    }
    drmModeFreeResources(res);
    return found;
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);
    int fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    if (fd < 0) { perror("open card0"); return 1; }

    uint32_t fbid = 0;
    if (find_active_fb(fd, &fbid) != 0 || !fbid) {
        printf("没有活动的 framebuffer (CRTC 上没有 fb)\n");
        close(fd);
        return 1;
    }
    printf("活动 framebuffer id = %u\n", fbid);

    drmModeFB2 *fb2 = drmModeGetFB2(fd, fbid);
    uint32_t w = 0, h = 0, pitch = 0, handle = 0;
    uint64_t modifier = 0;
    if (fb2) {
        w = fb2->width; h = fb2->height; pitch = fb2->pitches[0];
        handle = fb2->handles[0]; modifier = fb2->modifier;
        printf("FB2: %ux%u pitch=%u fourcc=0x%08x modifier=0x%llx\n",
               w, h, pitch, fb2->pixel_format, (unsigned long long)modifier);
        drmModeFreeFB2(fb2);
    }
    if (!handle) {
        drmModeFB *fb = drmModeGetFB(fd, fbid);
        if (!fb) { printf("拿不到 FB 信息\n"); close(fd); return 1; }
        w = fb->width; h = fb->height; pitch = fb->pitch; handle = fb->handle;
        printf("FB: %ux%u pitch=%u bpp=%u\n", w, h, pitch, fb->bpp);
        drmModeFreeFB(fb);
    }

    struct drm_mode_map_dumb mreq;
    memset(&mreq, 0, sizeof(mreq));
    mreq.handle = handle;
    if (ioctl(fd, DRM_IOCTL_MODE_MAP_DUMB, &mreq) != 0) {
        printf("MAP_DUMB 失败: %s\n", strerror(errno));
        printf("(说明这个 fb 不是 dumb buffer, 无法直接 mmap 读取)\n");
        close(fd);
        return 1;
    }

    size_t maplen = (size_t)pitch * h;
    uint8_t *m = mmap(NULL, maplen, PROT_READ, MAP_SHARED, fd, mreq.offset);
    if (m == MAP_FAILED) { perror("mmap"); close(fd); return 1; }
    printf("已 mmap %zu 字节\n", maplen);

    /* 采样统计: 完全黑 / 有内容 */
    long nonzero = 0, total = 0, sum = 0;
    long hist[8] = {0};
    for (uint32_t y = 0; y < h; y += 7) {
        uint32_t *row = (uint32_t *)(m + (size_t)y * pitch);
        for (uint32_t x = 0; x < w; x += 7) {
            uint32_t px = row[x];
            int r = (px >> 16) & 0xFF, g = (px >> 8) & 0xFF, b = px & 0xFF;
            int lum = (299 * r + 587 * g + 114 * b) / 1000;
            if (px & 0x00FFFFFF) nonzero++;
            sum += lum; total++;
            hist[lum >> 5]++;
        }
    }
    printf("采样 %ld 点: 非黑像素 %ld (%.2f%%), 平均亮度 %.1f\n",
           total, nonzero, 100.0 * nonzero / total, (double)sum / total);
    for (int i = 0; i < 8; i++)
        printf("  亮度 %3d-%3d: %5.1f%%\n", i * 32, i * 32 + 31, 100.0 * hist[i] / total);

    if (nonzero == 0)
        printf("=> 帧缓冲全黑: app 没有绘制任何内容 (Qt/EGLFS 问题)\n");
    else
        printf("=> 帧缓冲里有内容: app 确实在绘制\n");

    munmap(m, maplen);
    close(fd);
    return 0;
}
