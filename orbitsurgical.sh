#!/usr/bin/env bash

# Copyright (c) 2024, The ORBIT-Surgical Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

${IsaacLab_PATH}/isaaclab.sh -p -m pip install -e source/extensions/orbit.surgical.ext
${IsaacLab_PATH}/isaaclab.sh -p -m pip install -e source/extensions/orbit.surgical.assets
${IsaacLab_PATH}/isaaclab.sh -p -m pip install -e source/extensions/orbit.surgical.tasks
