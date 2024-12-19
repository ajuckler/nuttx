/****************************************************************************
 * include/nuttx/eeprom/spi_xx25xx.h
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

#ifndef __INCLUDE_NUTTX_EEPROM_SPI_XX25XX_H
#define __INCLUDE_NUTTX_EEPROM_SPI_XX25XX_H

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: ee25xx_initialize
 *
 * Description: Bind an EEPROM driver to an SPI bus.
 *
 ****************************************************************************/

struct spi_dev_s;
int ee25xx_initialize(FAR struct spi_dev_s *dev, FAR char *devname,
                      int readonly);

#endif /* __INCLUDE_NUTTX_EEPROM_SPI_XX25XX_H */
