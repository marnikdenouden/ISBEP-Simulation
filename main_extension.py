# Copyright (c) 2020-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import os
import directory
directory.directory_setup()

from omni.isaac.examples.base_sample import BaseSampleExtension
from main import Main

class MainExtension(BaseSampleExtension):
    def on_startup(self, ext_id: str):
        super().on_startup(ext_id)
        super().start_extension(
            menu_name="",
            submenu_name="",
            name="ISBEP Sim",
            title="Exploration Planning Simulation",
            doc_link="https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/tutorial_core_hello_world.html",
            overview="This is ISBEP Sim Code 8 June 2024 version",
            file_path=os.path.abspath(__file__),
            sample=Main(),
        )
        return
