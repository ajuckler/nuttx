/****************************************************************************
 * arch/sparc/src/bm3803/bm3803-timerisr.c
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <time.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <arch/board/board.h>

#include "sparc_internal.h"
#include "bm3803.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* The CPU frequency is given by BOARD_CPU_CLOCK (defined in board.h).  The
 * desired interrupt frequency is given by CONFIG_USEC_PER_TICK.  An unscaled
 * ideal match is given by:
 *
 *   CLOCK = CPU_CLOCK / DIVISOR                      # CPU clocks/sec
 *   MATCH = CLOCK / CLOCKS_PER_SEC                   # CPU clocks/timer tick
 *   MATCH = CPU_CLOCK / DIVISOR / CLOCKS_PER_SEC     # CPU clocks/timer tick
 *
 * But we only have 16-bits of accuracy so we need to pick the smallest
 * divisor using the following brute force calculation:
 */
#define BM3803_TIMER_CLOCK                  1000000
#define MATCH1                              (( 1000000 / CLOCKS_PER_SEC) - 1)

/* Bit 0: enables the timer when set */
#define TIMCTR_ENABLE_COUNTER               (1 << 0)

/* Bit 1: automatically reloaded with the reload value after each underflow */
#define TIMCTR_AUTO_RELOAD                 (1 << 1)

/* Bit 2: Set 1, will load the timer reload register into the timer counter
 * register
 */

#define TIMCTR_LOAD_COUNTER                 (1 << 2)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  bm3803_timerisr
 *
 * Description:
 *   The timer ISR will perform a variety of services for various portions
 *   of the systems.
 *
 ****************************************************************************/

static int bm3803_timerisr(int irq, uint32_t *regs, void *arg)
{
  /* Clear the pending timer interrupt */

  sparc_clrpend_irq(BM3803_IRQ_TIMER1);

  /* Process timer interrupt */

  nxsched_process_timer();
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Function:  up_timer_initialize
 *
 * Description:
 *   This function is called during start-up to initialize the timer
 *   interrupt.  NOTE:  This function depends on setup of OSC32 by
 *   up_clkinitialize().
 *
 ****************************************************************************/

void up_timer_initialize(void)
{
  BM3803_REG.scaler_load = (BOARD_CPU_CLOCK / BM3803_TIMER_CLOCK) - 1;
  BM3803_REG.scaler_cnt  = (BOARD_CPU_CLOCK / BM3803_TIMER_CLOCK) - 1;

  /* Setup timer 1 compare match A to generate a tick interrupt.
   *
   * First, setup the match value for compare match A.
   */

  BM3803_REG.timer_cnt1 = (uint32_t)MATCH1;
  BM3803_REG.timer_load1  = (uint32_t)MATCH1;

  BM3803_REG.timer_ctrl1 = (TIMCTR_ENABLE_COUNTER | TIMCTR_AUTO_RELOAD |
                            TIMCTR_LOAD_COUNTER);

  /* Configure the timer interrupt */

  sparc_clrpend_irq(BM3803_IRQ_TIMER1);
#ifdef CONFIG_ARCH_IRQPRIO
  up_prioritize_irq(BM3803_IRQ_TIMER1, CONFIG_BM3803_TIMER1PRIO);
#endif

  /* Attach the timer interrupt vector */

  irq_attach(BM3803_IRQ_TIMER1, (xcpt_t)bm3803_timerisr, NULL);

  /* And enable the timer interrupt */

  up_enable_irq(BM3803_IRQ_TIMER1);
}
