/****************************************************************************
 * arch/arm/src/efm32/hardware/efm32_vcmp.h
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: 2014 Silicon Laboratories, Inc.
 * SPDX-FileCopyrightText: 2014 Pierre-noel Bouteville . All rights reserved.
 * SPDX-FileCopyrightText: 2014 Gregory Nutt. All rights reserved.
 * SPDX-FileContributor: Pierre-noel Bouteville <pnb990@gmail.com>
 * SPDX-FileContributor: Gregory Nutt <gnutt@nuttx.org>
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Laboratories, Inc.
 * has no obligation to support this Software. Silicon Laboratories, Inc. is
 * providing the Software "AS IS", with no express or implied warranties of
 * any kind, including, but not limited to, any implied warranties of
 * merchantability or fitness for any particular purpose or warranties
 * against infringement of any proprietary rights of a third party.
 *
 * Silicon Laboratories, Inc. will not be liable for any consequential,
 * incidental, or special damages, or any other relief, or for any claim by
 * any third party, arising from your use of this Software.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32_VCMP_H
#define __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32_VCMP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "hardware/efm32_memorymap.h"

#if !defined(CONFIG_EFM32_EFM32GG)
#  warning This is the EFM32GG header file; Review/modification needed for this architecture
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* VCMP Register Offsets ****************************************************/

#define EFM32_VCMP_CTRL_OFFSET              0x0000  /* Control Register */
#define EFM32_VCMP_INPUTSEL_OFFSET          0x0004  /* Input Selection Register */
#define EFM32_VCMP_STATUS_OFFSET            0x0008  /* Status Register */
#define EFM32_VCMP_IEN_OFFSET               0x000c  /* Interrupt Enable Register */
#define EFM32_VCMP_IF_OFFSET                0x0010  /* Interrupt Flag Register */
#define EFM32_VCMP_IFS_OFFSET               0x0014  /* Interrupt Flag Set Register */
#define EFM32_VCMP_IFC_OFFSET               0x0018  /* Interrupt Flag Clear Register */

/* VCMP Register Addresses **************************************************/

#define EFM32_VCMP_CTRL                     (EFM32_VCMP_BASE+EFM32_VCMP_CTRL_OFFSET)
#define EFM32_VCMP_INPUTSEL                 (EFM32_VCMP_BASE+EFM32_VCMP_INPUTSEL_OFFSET)
#define EFM32_VCMP_STATUS                   (EFM32_VCMP_BASE+EFM32_VCMP_STATUS_OFFSET)
#define EFM32_VCMP_IEN                      (EFM32_VCMP_BASE+EFM32_VCMP_IEN_OFFSET)
#define EFM32_VCMP_IF                       (EFM32_VCMP_BASE+EFM32_VCMP_IF_OFFSET)
#define EFM32_VCMP_IFS                      (EFM32_VCMP_BASE+EFM32_VCMP_IFS_OFFSET)
#define EFM32_VCMP_IFC                      (EFM32_VCMP_BASE+EFM32_VCMP_IFC_OFFSET)

/* VCMP Register Bit Field Definitions **************************************/

/* Bit fields for VCMP CTRL */

#define _VCMP_CTRL_RESETVALUE               0x47000000UL                         /* Default value for VCMP_CTRL */
#define _VCMP_CTRL_MASK                     0x4F030715UL                         /* Mask for VCMP_CTRL */

#define VCMP_CTRL_EN                        (0x1UL << 0)                         /* Voltage Supply Comparator Enable */
#define _VCMP_CTRL_EN_SHIFT                 0                                    /* Shift value for VCMP_EN */
#define _VCMP_CTRL_EN_MASK                  0x1UL                                /* Bit mask for VCMP_EN */
#define _VCMP_CTRL_EN_DEFAULT               0x00000000UL                         /* Mode DEFAULT for VCMP_CTRL */
#define VCMP_CTRL_EN_DEFAULT                (_VCMP_CTRL_EN_DEFAULT << 0)         /* Shifted mode DEFAULT for VCMP_CTRL */
#define VCMP_CTRL_INACTVAL                  (0x1UL << 2)                         /* Inactive Value */
#define _VCMP_CTRL_INACTVAL_SHIFT           2                                    /* Shift value for VCMP_INACTVAL */
#define _VCMP_CTRL_INACTVAL_MASK            0x4UL                                /* Bit mask for VCMP_INACTVAL */
#define _VCMP_CTRL_INACTVAL_DEFAULT         0x00000000UL                         /* Mode DEFAULT for VCMP_CTRL */
#define VCMP_CTRL_INACTVAL_DEFAULT          (_VCMP_CTRL_INACTVAL_DEFAULT << 2)   /* Shifted mode DEFAULT for VCMP_CTRL */
#define VCMP_CTRL_HYSTEN                    (0x1UL << 4)                         /* Hysteresis Enable */
#define _VCMP_CTRL_HYSTEN_SHIFT             4                                    /* Shift value for VCMP_HYSTEN */
#define _VCMP_CTRL_HYSTEN_MASK              0x10UL                               /* Bit mask for VCMP_HYSTEN */
#define _VCMP_CTRL_HYSTEN_DEFAULT           0x00000000UL                         /* Mode DEFAULT for VCMP_CTRL */
#define VCMP_CTRL_HYSTEN_DEFAULT            (_VCMP_CTRL_HYSTEN_DEFAULT << 4)     /* Shifted mode DEFAULT for VCMP_CTRL */
#define _VCMP_CTRL_WARMTIME_SHIFT           8                                    /* Shift value for VCMP_WARMTIME */
#define _VCMP_CTRL_WARMTIME_MASK            0x700UL                              /* Bit mask for VCMP_WARMTIME */
#define _VCMP_CTRL_WARMTIME_DEFAULT         0x00000000UL                         /* Mode DEFAULT for VCMP_CTRL */
#define _VCMP_CTRL_WARMTIME_4CYCLES         0x00000000UL                         /* Mode 4CYCLES for VCMP_CTRL */
#define _VCMP_CTRL_WARMTIME_8CYCLES         0x00000001UL                         /* Mode 8CYCLES for VCMP_CTRL */
#define _VCMP_CTRL_WARMTIME_16CYCLES        0x00000002UL                         /* Mode 16CYCLES for VCMP_CTRL */
#define _VCMP_CTRL_WARMTIME_32CYCLES        0x00000003UL                         /* Mode 32CYCLES for VCMP_CTRL */
#define _VCMP_CTRL_WARMTIME_64CYCLES        0x00000004UL                         /* Mode 64CYCLES for VCMP_CTRL */
#define _VCMP_CTRL_WARMTIME_128CYCLES       0x00000005UL                         /* Mode 128CYCLES for VCMP_CTRL */
#define _VCMP_CTRL_WARMTIME_256CYCLES       0x00000006UL                         /* Mode 256CYCLES for VCMP_CTRL */
#define _VCMP_CTRL_WARMTIME_512CYCLES       0x00000007UL                         /* Mode 512CYCLES for VCMP_CTRL */
#define VCMP_CTRL_WARMTIME_DEFAULT          (_VCMP_CTRL_WARMTIME_DEFAULT << 8)   /* Shifted mode DEFAULT for VCMP_CTRL */
#define VCMP_CTRL_WARMTIME_4CYCLES          (_VCMP_CTRL_WARMTIME_4CYCLES << 8)   /* Shifted mode 4CYCLES for VCMP_CTRL */
#define VCMP_CTRL_WARMTIME_8CYCLES          (_VCMP_CTRL_WARMTIME_8CYCLES << 8)   /* Shifted mode 8CYCLES for VCMP_CTRL */
#define VCMP_CTRL_WARMTIME_16CYCLES         (_VCMP_CTRL_WARMTIME_16CYCLES << 8)  /* Shifted mode 16CYCLES for VCMP_CTRL */
#define VCMP_CTRL_WARMTIME_32CYCLES         (_VCMP_CTRL_WARMTIME_32CYCLES << 8)  /* Shifted mode 32CYCLES for VCMP_CTRL */
#define VCMP_CTRL_WARMTIME_64CYCLES         (_VCMP_CTRL_WARMTIME_64CYCLES << 8)  /* Shifted mode 64CYCLES for VCMP_CTRL */
#define VCMP_CTRL_WARMTIME_128CYCLES        (_VCMP_CTRL_WARMTIME_128CYCLES << 8) /* Shifted mode 128CYCLES for VCMP_CTRL */
#define VCMP_CTRL_WARMTIME_256CYCLES        (_VCMP_CTRL_WARMTIME_256CYCLES << 8) /* Shifted mode 256CYCLES for VCMP_CTRL */
#define VCMP_CTRL_WARMTIME_512CYCLES        (_VCMP_CTRL_WARMTIME_512CYCLES << 8) /* Shifted mode 512CYCLES for VCMP_CTRL */
#define VCMP_CTRL_IRISE                     (0x1UL << 16)                        /* Rising Edge Interrupt Sense */
#define _VCMP_CTRL_IRISE_SHIFT              16                                   /* Shift value for VCMP_IRISE */
#define _VCMP_CTRL_IRISE_MASK               0x10000UL                            /* Bit mask for VCMP_IRISE */
#define _VCMP_CTRL_IRISE_DEFAULT            0x00000000UL                         /* Mode DEFAULT for VCMP_CTRL */
#define VCMP_CTRL_IRISE_DEFAULT             (_VCMP_CTRL_IRISE_DEFAULT << 16)     /* Shifted mode DEFAULT for VCMP_CTRL */
#define VCMP_CTRL_IFALL                     (0x1UL << 17)                        /* Falling Edge Interrupt Sense */
#define _VCMP_CTRL_IFALL_SHIFT              17                                   /* Shift value for VCMP_IFALL */
#define _VCMP_CTRL_IFALL_MASK               0x20000UL                            /* Bit mask for VCMP_IFALL */
#define _VCMP_CTRL_IFALL_DEFAULT            0x00000000UL                         /* Mode DEFAULT for VCMP_CTRL */
#define VCMP_CTRL_IFALL_DEFAULT             (_VCMP_CTRL_IFALL_DEFAULT << 17)     /* Shifted mode DEFAULT for VCMP_CTRL */
#define _VCMP_CTRL_BIASPROG_SHIFT           24                                   /* Shift value for VCMP_BIASPROG */
#define _VCMP_CTRL_BIASPROG_MASK            0xF000000UL                          /* Bit mask for VCMP_BIASPROG */
#define _VCMP_CTRL_BIASPROG_DEFAULT         0x00000007UL                         /* Mode DEFAULT for VCMP_CTRL */
#define VCMP_CTRL_BIASPROG_DEFAULT          (_VCMP_CTRL_BIASPROG_DEFAULT << 24)  /* Shifted mode DEFAULT for VCMP_CTRL */
#define VCMP_CTRL_HALFBIAS                  (0x1UL << 30)                        /* Half Bias Current */
#define _VCMP_CTRL_HALFBIAS_SHIFT           30                                   /* Shift value for VCMP_HALFBIAS */
#define _VCMP_CTRL_HALFBIAS_MASK            0x40000000UL                         /* Bit mask for VCMP_HALFBIAS */
#define _VCMP_CTRL_HALFBIAS_DEFAULT         0x00000001UL                         /* Mode DEFAULT for VCMP_CTRL */
#define VCMP_CTRL_HALFBIAS_DEFAULT          (_VCMP_CTRL_HALFBIAS_DEFAULT << 30)  /* Shifted mode DEFAULT for VCMP_CTRL */

/* Bit fields for VCMP INPUTSEL */

#define _VCMP_INPUTSEL_RESETVALUE           0x00000000UL                            /* Default value for VCMP_INPUTSEL */
#define _VCMP_INPUTSEL_MASK                 0x0000013FUL                            /* Mask for VCMP_INPUTSEL */

#define _VCMP_INPUTSEL_TRIGLEVEL_SHIFT      0                                       /* Shift value for VCMP_TRIGLEVEL */
#define _VCMP_INPUTSEL_TRIGLEVEL_MASK       0x3FUL                                  /* Bit mask for VCMP_TRIGLEVEL */
#define _VCMP_INPUTSEL_TRIGLEVEL_DEFAULT    0x00000000UL                            /* Mode DEFAULT for VCMP_INPUTSEL */
#define VCMP_INPUTSEL_TRIGLEVEL_DEFAULT     (_VCMP_INPUTSEL_TRIGLEVEL_DEFAULT << 0) /* Shifted mode DEFAULT for VCMP_INPUTSEL */
#define VCMP_INPUTSEL_LPREF                 (0x1UL << 8)                            /* Low Power Reference */
#define _VCMP_INPUTSEL_LPREF_SHIFT          8                                       /* Shift value for VCMP_LPREF */
#define _VCMP_INPUTSEL_LPREF_MASK           0x100UL                                 /* Bit mask for VCMP_LPREF */
#define _VCMP_INPUTSEL_LPREF_DEFAULT        0x00000000UL                            /* Mode DEFAULT for VCMP_INPUTSEL */
#define VCMP_INPUTSEL_LPREF_DEFAULT         (_VCMP_INPUTSEL_LPREF_DEFAULT << 8)     /* Shifted mode DEFAULT for VCMP_INPUTSEL */

/* Bit fields for VCMP STATUS */

#define _VCMP_STATUS_RESETVALUE             0x00000000UL                        /* Default value for VCMP_STATUS */
#define _VCMP_STATUS_MASK                   0x00000003UL                        /* Mask for VCMP_STATUS */

#define VCMP_STATUS_VCMPACT                 (0x1UL << 0)                        /* Voltage Supply Comparator Active */
#define _VCMP_STATUS_VCMPACT_SHIFT          0                                   /* Shift value for VCMP_VCMPACT */
#define _VCMP_STATUS_VCMPACT_MASK           0x1UL                               /* Bit mask for VCMP_VCMPACT */
#define _VCMP_STATUS_VCMPACT_DEFAULT        0x00000000UL                        /* Mode DEFAULT for VCMP_STATUS */
#define VCMP_STATUS_VCMPACT_DEFAULT         (_VCMP_STATUS_VCMPACT_DEFAULT << 0) /* Shifted mode DEFAULT for VCMP_STATUS */
#define VCMP_STATUS_VCMPOUT                 (0x1UL << 1)                        /* Voltage Supply Comparator Output */
#define _VCMP_STATUS_VCMPOUT_SHIFT          1                                   /* Shift value for VCMP_VCMPOUT */
#define _VCMP_STATUS_VCMPOUT_MASK           0x2UL                               /* Bit mask for VCMP_VCMPOUT */
#define _VCMP_STATUS_VCMPOUT_DEFAULT        0x00000000UL                        /* Mode DEFAULT for VCMP_STATUS */
#define VCMP_STATUS_VCMPOUT_DEFAULT         (_VCMP_STATUS_VCMPOUT_DEFAULT << 1) /* Shifted mode DEFAULT for VCMP_STATUS */

/* Bit fields for VCMP IEN */

#define _VCMP_IEN_RESETVALUE                0x00000000UL                    /* Default value for VCMP_IEN */
#define _VCMP_IEN_MASK                      0x00000003UL                    /* Mask for VCMP_IEN */

#define VCMP_IEN_EDGE                       (0x1UL << 0)                    /* Edge Trigger Interrupt Enable */
#define _VCMP_IEN_EDGE_SHIFT                0                               /* Shift value for VCMP_EDGE */
#define _VCMP_IEN_EDGE_MASK                 0x1UL                           /* Bit mask for VCMP_EDGE */
#define _VCMP_IEN_EDGE_DEFAULT              0x00000000UL                    /* Mode DEFAULT for VCMP_IEN */
#define VCMP_IEN_EDGE_DEFAULT               (_VCMP_IEN_EDGE_DEFAULT << 0)   /* Shifted mode DEFAULT for VCMP_IEN */
#define VCMP_IEN_WARMUP                     (0x1UL << 1)                    /* Warm-up Interrupt Enable */
#define _VCMP_IEN_WARMUP_SHIFT              1                               /* Shift value for VCMP_WARMUP */
#define _VCMP_IEN_WARMUP_MASK               0x2UL                           /* Bit mask for VCMP_WARMUP */
#define _VCMP_IEN_WARMUP_DEFAULT            0x00000000UL                    /* Mode DEFAULT for VCMP_IEN */
#define VCMP_IEN_WARMUP_DEFAULT             (_VCMP_IEN_WARMUP_DEFAULT << 1) /* Shifted mode DEFAULT for VCMP_IEN */

/* Bit fields for VCMP IF */

#define _VCMP_IF_RESETVALUE                 0x00000000UL                   /* Default value for VCMP_IF */
#define _VCMP_IF_MASK                       0x00000003UL                   /* Mask for VCMP_IF */

#define VCMP_IF_EDGE                        (0x1UL << 0)                   /* Edge Triggered Interrupt Flag */
#define _VCMP_IF_EDGE_SHIFT                 0                              /* Shift value for VCMP_EDGE */
#define _VCMP_IF_EDGE_MASK                  0x1UL                          /* Bit mask for VCMP_EDGE */
#define _VCMP_IF_EDGE_DEFAULT               0x00000000UL                   /* Mode DEFAULT for VCMP_IF */
#define VCMP_IF_EDGE_DEFAULT                (_VCMP_IF_EDGE_DEFAULT << 0)   /* Shifted mode DEFAULT for VCMP_IF */
#define VCMP_IF_WARMUP                      (0x1UL << 1)                   /* Warm-up Interrupt Flag */
#define _VCMP_IF_WARMUP_SHIFT               1                              /* Shift value for VCMP_WARMUP */
#define _VCMP_IF_WARMUP_MASK                0x2UL                          /* Bit mask for VCMP_WARMUP */
#define _VCMP_IF_WARMUP_DEFAULT             0x00000000UL                   /* Mode DEFAULT for VCMP_IF */
#define VCMP_IF_WARMUP_DEFAULT              (_VCMP_IF_WARMUP_DEFAULT << 1) /* Shifted mode DEFAULT for VCMP_IF */

/* Bit fields for VCMP IFS */

#define _VCMP_IFS_RESETVALUE                0x00000000UL                    /* Default value for VCMP_IFS */
#define _VCMP_IFS_MASK                      0x00000003UL                    /* Mask for VCMP_IFS */

#define VCMP_IFS_EDGE                       (0x1UL << 0)                    /* Edge Triggered Interrupt Flag Set */
#define _VCMP_IFS_EDGE_SHIFT                0                               /* Shift value for VCMP_EDGE */
#define _VCMP_IFS_EDGE_MASK                 0x1UL                           /* Bit mask for VCMP_EDGE */
#define _VCMP_IFS_EDGE_DEFAULT              0x00000000UL                    /* Mode DEFAULT for VCMP_IFS */
#define VCMP_IFS_EDGE_DEFAULT               (_VCMP_IFS_EDGE_DEFAULT << 0)   /* Shifted mode DEFAULT for VCMP_IFS */
#define VCMP_IFS_WARMUP                     (0x1UL << 1)                    /* Warm-up Interrupt Flag Set */
#define _VCMP_IFS_WARMUP_SHIFT              1                               /* Shift value for VCMP_WARMUP */
#define _VCMP_IFS_WARMUP_MASK               0x2UL                           /* Bit mask for VCMP_WARMUP */
#define _VCMP_IFS_WARMUP_DEFAULT            0x00000000UL                    /* Mode DEFAULT for VCMP_IFS */
#define VCMP_IFS_WARMUP_DEFAULT             (_VCMP_IFS_WARMUP_DEFAULT << 1) /* Shifted mode DEFAULT for VCMP_IFS */

/* Bit fields for VCMP IFC */

#define _VCMP_IFC_RESETVALUE                0x00000000UL                    /* Default value for VCMP_IFC */
#define _VCMP_IFC_MASK                      0x00000003UL                    /* Mask for VCMP_IFC */

#define VCMP_IFC_EDGE                       (0x1UL << 0)                    /* Edge Triggered Interrupt Flag Clear */
#define _VCMP_IFC_EDGE_SHIFT                0                               /* Shift value for VCMP_EDGE */
#define _VCMP_IFC_EDGE_MASK                 0x1UL                           /* Bit mask for VCMP_EDGE */
#define _VCMP_IFC_EDGE_DEFAULT              0x00000000UL                    /* Mode DEFAULT for VCMP_IFC */
#define VCMP_IFC_EDGE_DEFAULT               (_VCMP_IFC_EDGE_DEFAULT << 0)   /* Shifted mode DEFAULT for VCMP_IFC */
#define VCMP_IFC_WARMUP                     (0x1UL << 1)                    /* Warm-up Interrupt Flag Clear */
#define _VCMP_IFC_WARMUP_SHIFT              1                               /* Shift value for VCMP_WARMUP */
#define _VCMP_IFC_WARMUP_MASK               0x2UL                           /* Bit mask for VCMP_WARMUP */
#define _VCMP_IFC_WARMUP_DEFAULT            0x00000000UL                    /* Mode DEFAULT for VCMP_IFC */
#define VCMP_IFC_WARMUP_DEFAULT             (_VCMP_IFC_WARMUP_DEFAULT << 1) /* Shifted mode DEFAULT for VCMP_IFC */

#endif /* __ARCH_ARM_SRC_EFM32_HARDWARE_EFM32_VCMP_H */
