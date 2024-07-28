# Copyright (c) 2024, The ORBIT-Surgical Project Developers.
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause

"""Package containing the additional modules."""

import os
import toml

# Conveniences to other module directories via relative paths
ORBITSURGICAL_EXT_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../"))
"""Path to the extension source directory."""

ORBITSURGICAL_METADATA = toml.load(os.path.join(ORBITSURGICAL_EXT_DIR, "config", "extension.toml"))
"""Extension metadata dictionary parsed from the extension.toml file."""

# Configure the module-level variables
__version__ = ORBITSURGICAL_METADATA["package"]["version"]
