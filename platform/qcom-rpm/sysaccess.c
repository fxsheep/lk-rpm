#include <sys/types.h>
#include <kernel/spinlock.h>
#include <platform/sysaccess.h>

static uint8_t current_page_select;
static spin_lock_t lock = SPIN_LOCK_INITIAL_VALUE;

void update_page_select(uint8_t page_select) {
    current_page_select = page_select;
    writel(page_select, RPM_PAGE_SELECT);
    return;
}

void sys_access_begin(uintptr_t addr) {
    uint8_t page_select;
    
    spin_lock(&lock);
    page_select = addr >> 30;
    update_page_select(page_select);
    return;
}

void sys_access_finish(uintptr_t addr) {
    spin_unlock(&lock);
    return;
}

uintptr_t sys_addr_xlate(uintptr_t addr) {
    if (addr >= 0x200000 && addr < 0x300000) {
        return addr - 0x200000;
    }
    else {
        return (addr & 0x3FFFFFFF) + 0x60000000;
    }
}

uint32_t sys_readl(uintptr_t addr) {
    uint32_t value;

    sys_access_begin(addr);
    value = readl(sys_addr_xlate(addr));
    sys_access_finish(addr);
    return value;
}

uint8_t sys_readb(uintptr_t addr) {
    uint8_t value;

    sys_access_begin(addr);
    value = readb(sys_addr_xlate(addr));
    sys_access_finish(addr);
    return value;
}

void sys_writel(uint32_t value, uintptr_t addr) {
    sys_access_begin(addr);
    writel(value, sys_addr_xlate(addr));
    sys_access_finish(addr);
    return;
}

void sys_writeb(uint8_t value, uintptr_t addr) {
    sys_access_begin(addr);
    writeb(value, sys_addr_xlate(addr));
    sys_access_finish(addr);
    return;
}
