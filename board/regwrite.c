#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    if (argc < 3) { printf("usage: regwrite <phys> <value>\n"); return 2; }
    unsigned long base = strtoul(argv[1], 0, 0);
    unsigned int val  = (unsigned int) strtoul(argv[2], 0, 0);
    int fd = open("/dev/mem", O_RDWR | O_SYNC);
    if (fd < 0) { perror("open /dev/mem"); return 1; }
    unsigned long page = base & ~0xFFFUL;
    unsigned long off  = base - page;
    size_t sz = 0x1000;
    volatile unsigned char *m = (volatile unsigned char *)mmap(0, sz, PROT_READ | PROT_WRITE,
                                                              MAP_SHARED, fd, page);
    if (m == MAP_FAILED) { perror("mmap"); close(fd); return 1; }
    *(volatile unsigned int *)(m + off) = val;
    printf("write 0x%03lX = 0x%08X -> now 0x%08X\n", off, val,
           *(volatile unsigned int *)(m + off));
    munmap((void *)m, sz);
    close(fd);
    return 0;
}
