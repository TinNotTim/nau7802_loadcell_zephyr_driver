# Copyright (c) 2024, Tin Chiang.
# SPDX-License-Identifier: Apache-2.0

zephyr_library()
zephyr_library_sources(nau7802_loadcell.c)
zephyr_library_sources_ifdef(CONFIG_NAU7802_LOADCELL_TRIGGER nau7802_loadcell_trigger.c)