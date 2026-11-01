# Copyright 2026 EdgeAI Robotics Challenge Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Check P2 source quality."""
from pathlib import Path
import subprocess
import sys

from ament_copyright.main import main as copyright_main

ROOT = Path(__file__).parents[1]


def test_flake8():
    # Run outside the ROS process: flake8 forks workers and cannot inherit DDS threads.
    result = subprocess.run(
        [sys.executable, '-c',
         'from ament_flake8.main import main; raise SystemExit(main())',
         '--linelength', '100', str(ROOT)], capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stdout + result.stderr


def test_copyright():
    assert copyright_main(argv=[str(ROOT)]) == 0
