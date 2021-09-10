#include <lk/debug.h>
#include <lk/compiler.h>
#include <arch/arm/cm.h>

static void rpm_dummy_irq(void) {
    arm_cm_irq_entry();
    panic("unhandled irq");
}

#define DEFAULT_HANDLER(x) \
    void rpm_##x##_irq(void) __WEAK_ALIAS("rpm_dummy_irq")

#define DEFIRQ(n) DEFAULT_HANDLER(n);
#include <platform/defirq.h>
#undef DEFIRQ

#define VECTAB_ENTRY(x) rpm_##x##_irq

const void *const __SECTION(".text.boot.vectab2") vectab2[] = {
#define DEFIRQ(n) VECTAB_ENTRY(n),
#include <platform/defirq.h>
#undef DEFIRQ
};
