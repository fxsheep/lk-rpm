#pragma once

#define RPM_F1_QTMR_V1_F1_BASE             (0x84000)

#define QTMR_BASE                          RPM_F1_QTMR_V1_F1_BASE

#define MSM_IOMAP_BASE_SYS                 (0x0U)

#define PERIPH_SS_BASE                     (MSM_IOMAP_BASE_SYS + 0x07800000)

/* UART */
#define BLSP1_UART0_BASE                   (PERIPH_SS_BASE + 0x000AF000)
#define BLSP1_UART1_BASE                   (PERIPH_SS_BASE + 0x000B0000)
#define MSM_USB_BASE                       (PERIPH_SS_BASE + 0x000DB000)