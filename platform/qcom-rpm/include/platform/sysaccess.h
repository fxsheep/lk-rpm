#pragma once

#define RPM_PAGE_SELECT 0x80070

uint32_t sys_readl(uintptr_t addr);
uint8_t sys_readb(uintptr_t addr);
void sys_writel(uint32_t value, uintptr_t addr);
void sys_writeb(uint8_t value, uintptr_t addr);