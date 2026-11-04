/* keyinject.c — 通过 /dev/uinput 创建一个虚拟键盘并发送按键
 *
 * 用途: 上位机是"点按钮才开采集"的，但没有物理键盘时无法操作。
 *       本工具创建一个虚拟输入设备并注入按键事件，让 app 以为有人在敲键盘。
 *
 * 编译: gcc -O2 -o keyinject keyinject.c
 * 用法: ./keyinject <key> [count] [delay_ms]
 *         key: enter | space | tab | up | down | left | right | esc
 *       例: ./keyinject tab 6 300      # 按 6 次 Tab
 *           ./keyinject enter 2 500    # 按 2 次回车
 *
 * 说明: 需要内核已加载 uinput 模块 (`modprobe uinput`)。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/input.h>
#include <linux/uinput.h>

static int lookup(const char *name)
{
    if (!strcmp(name, "enter"))  return KEY_ENTER;
    if (!strcmp(name, "space"))  return KEY_SPACE;
    if (!strcmp(name, "tab"))    return KEY_TAB;
    if (!strcmp(name, "esc"))    return KEY_ESC;
    if (!strcmp(name, "up"))     return KEY_UP;
    if (!strcmp(name, "down"))   return KEY_DOWN;
    if (!strcmp(name, "left"))   return KEY_LEFT;
    if (!strcmp(name, "right"))  return KEY_RIGHT;
    if (!strcmp(name, "a"))      return KEY_A;
    if (!strcmp(name, "q"))      return KEY_Q;
    if (!strcmp(name, "m"))      return KEY_M;
    if (!strcmp(name, "v"))      return KEY_V;
    if (!strcmp(name, "1"))      return KEY_1;
    if (!strcmp(name, "2"))      return KEY_2;
    if (!strcmp(name, "3"))      return KEY_3;
    if (!strcmp(name, "4"))      return KEY_4;
    if (!strcmp(name, "5"))      return KEY_5;
    if (!strcmp(name, "6"))      return KEY_6;
    if (!strcmp(name, "7"))      return KEY_7;
    if (!strcmp(name, "8"))      return KEY_8;
    if (!strcmp(name, "r"))      return KEY_R;
    if (!strcmp(name, "h"))      return KEY_H;
    return -1;
}

static void emit(int fd, int type, int code, int val)
{
    struct input_event ev;
    memset(&ev, 0, sizeof(ev));
    ev.type = type; ev.code = code; ev.value = val;
    if (write(fd, &ev, sizeof(ev)) < 0) perror("write ev");
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        fprintf(stderr, "usage: %s <key> [count] [delay_ms]\n"
                        "  keys: enter space tab esc up down left right a q\n", argv[0]);
        return 2;
    }
    int code = lookup(argv[1]);
    if (code < 0) { fprintf(stderr, "unknown key: %s\n", argv[1]); return 2; }
    int count = (argc > 2) ? atoi(argv[2]) : 1;
    int delay = (argc > 3) ? atoi(argv[3]) : 300;

    int fd = open("/dev/uinput", O_WRONLY | O_NONBLOCK);
    if (fd < 0) {
        perror("open /dev/uinput (试试 modprobe uinput)");
        return 1;
    }

    ioctl(fd, UI_SET_EVBIT, EV_KEY);
    ioctl(fd, UI_SET_EVBIT, EV_SYN);
    /* 注册常用按键 */
    int keys[] = { KEY_ENTER, KEY_KPENTER, KEY_SPACE, KEY_TAB, KEY_ESC,
                   KEY_UP, KEY_DOWN, KEY_LEFT, KEY_RIGHT, KEY_A, KEY_Q, KEY_M, KEY_V,
                  KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_R, KEY_H };
    for (unsigned i = 0; i < sizeof(keys)/sizeof(keys[0]); i++)
        ioctl(fd, UI_SET_KEYBIT, keys[i]);

    struct uinput_setup usetup;
    memset(&usetup, 0, sizeof(usetup));
    usetup.id.bustype = BUS_USB;
    usetup.id.vendor  = 0x1234;
    usetup.id.product = 0x5678;
    strcpy(usetup.name, "dsh-virtual-keyboard");
    if (ioctl(fd, UI_DEV_SETUP, &usetup) < 0) perror("UI_DEV_SETUP");
    if (ioctl(fd, UI_DEV_CREATE) < 0) { perror("UI_DEV_CREATE"); return 1; }

    printf("虚拟键盘已创建: dsh-virtual-keyboard\n");
    sleep(2);   /* 等 udev/Qt 发现新设备 */

    for (int i = 0; i < count; i++) {
        emit(fd, EV_KEY, code, 1);
        emit(fd, EV_SYN, SYN_REPORT, 0);
        usleep(delay * 1000 / 2);
        emit(fd, EV_KEY, code, 0);
        emit(fd, EV_SYN, SYN_REPORT, 0);
        printf("  发送 %s (%d/%d)\n", argv[1], i + 1, count);
        fflush(stdout);
        usleep(delay * 1000);
    }

    sleep(1);
    ioctl(fd, UI_DEV_DESTROY);
    close(fd);
    printf("虚拟键盘已销毁\n");
    return 0;
}
