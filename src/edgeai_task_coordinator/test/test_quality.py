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

"""Apply the repository's existing ament quality checks to this package's files."""

from pathlib import Path

from ament_copyright.main import main as copyright_main
from ament_flake8.main import main_with_errors


ROOT = Path(__file__).parents[1]


def test_flake8():
    rc, errors = main_with_errors(argv=['--linelength', '100', str(ROOT)])
    assert rc == 0, '\n'.join(errors)


def test_copyright():
    assert copyright_main(argv=[str(ROOT)]) == 0
