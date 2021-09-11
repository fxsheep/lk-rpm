#include <lk/debug.h>
#include <lk/compiler.h>
#include <arch/arm/cm.h>

static void rpm_dummy_irq(void) {
    arm_cm_irq_entry();
    panic("unhandled irq");
}

#define RPM_IRQ(name,num) \
void name##_IRQHandler(void) __WEAK_ALIAS("rpm_dummy_irq");
#include <platform/irqinfo.h>
#undef RPM_IRQ

const void* const __SECTION(".text.boot.vectab2") vectab2[] = {
#define RPM_IRQ(name,num) [name##_IRQn] = name##_IRQHandler,
#include <platform/irqinfo.h>
#undef RPM_IRQ
};