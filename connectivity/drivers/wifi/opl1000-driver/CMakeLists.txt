# Copyright (c) 2020 ARM Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

target_sources(mbed-wifi
    INTERFACE
        OPL1000Interface.cpp
        OPL1000/OPL1000.cpp
)

target_include_directories(mbed-wifi
    INTERFACE
        .
        ./OPL1000
)
