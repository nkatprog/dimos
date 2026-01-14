# Copyright 2026 Dimensional Inc.
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

from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3
from dimos.protocol.pubsub.rospubsub import LCM2ROSMixin


def test_simple_encode():
    mixin = LCM2ROSMixin()
    encoding = mixin.encode(Vector3(1.0, 2.0, 3.0))
    print(encoding)
    decode = mixin.decode(encoding)
    print(decode)


def test_complex_encode():
    mixin = LCM2ROSMixin()
    mixin.encode(PoseStamped())
    ...
