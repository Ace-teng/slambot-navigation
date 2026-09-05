# Copyright 2026 slambot-navigation
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

"""Structural tests for the RTAB-Map 3D mapping launch chain.

These tests parse the launch files and assert on the declared structure only
(bringing up the launch tree does not require the rtabmap_ros / orbbec binaries).
"""

import importlib.util
import os

from launch import LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node

_LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch')
_ENTRY = os.path.join(_LAUNCH, 'rtabmap_slam.launch.py')
_INCLUDE = os.path.join(_LAUNCH, 'include', 'rtabmap.launch.py')


def _load_module(path):
    path = os.path.abspath(path)
    name = '_'.join(c if c.isalnum() else '_' for c in path) + '_mod'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _as_text(sub):
    """Flatten a launch substitution (or sequence of them) to plain text."""
    if isinstance(sub, (list, tuple)):
        return ''.join(_as_text(x) for x in sub)
    if hasattr(sub, 'text'):
        return sub.text
    for attr in ('var_name', 'variable_name'):
        if hasattr(sub, attr):
            return '${%s}' % getattr(sub, attr)
    return str(sub)


def _nodes(ld):
    return [e for e in ld.entities if isinstance(e, Node)]


def _node_params(node):
    """Reconstruct the parameter dict handed to a Node."""
    params = {}
    for container in getattr(node, '_Node__parameters', []):
        if isinstance(container, dict):
            for key, value in container.items():
                params[_as_text(key)] = value
    return params


def _node_remaps(node):
    remaps = []
    for source, target in getattr(node, '_Node__remappings', []):
        remaps.append((_as_text(source), _as_text(target)))
    return remaps


def _rtabmap_node(ld):
    nodes = [n for n in _nodes(ld) if n.node_package == 'rtabmap_slam']
    assert len(nodes) == 1
    return nodes[0]


def test_entry_launches_only_rtabmap(monkeypatch):
    """3D entry must not start the 2D slam_toolbox alongside RTAB-Map."""
    monkeypatch.setenv('need_compile', 'False')
    monkeypatch.setenv('MASTER', '')
    monkeypatch.setenv('HOST', '/')

    # Source-level guard: the entry must no longer include the 2D SLAM file.
    # (Include location strings are launch-version dependent, so don't parse them.)
    with open(_ENTRY, encoding='utf-8') as fh:
        source = fh.read()
    assert 'slam_base' not in source
    assert 'include/rtabmap.launch.py' in source

    module = _load_module(_ENTRY)
    top = module.launch_setup(LaunchContext())
    groups = [a for a in top if isinstance(a, GroupAction)]
    assert len(groups) == 1
    members = list(getattr(groups[0], '_GroupAction__actions', []))
    timers = [m for m in members if isinstance(m, TimerAction)]
    includes = [m for m in members if isinstance(m, IncludeLaunchDescription)]

    # Exactly one base robot include and one deferred launch (RTAB-Map); the old
    # 5s timer that started slam_toolbox is gone.
    assert len(includes) == 1
    assert len(timers) == 1
    deferred = [m for m in timers[0].actions if isinstance(m, IncludeLaunchDescription)]
    assert len(deferred) == 1


def test_mapping_include_launches_sync_and_rtabmap():
    """The mapping include runs rgbd_sync + rtabmap in mapping mode."""
    ld = _load_module(_INCLUDE).generate_launch_description()
    packages = sorted(n.node_package for n in _nodes(ld))
    assert packages == ['rtabmap_slam', 'rtabmap_sync']
    executables = sorted(n.node_executable for n in _nodes(ld))
    assert executables == ['rgbd_sync', 'rtabmap']

    rtabmap = _rtabmap_node(ld)
    params = _node_params(rtabmap)
    # Mapping mode, not localization: no incremental-memory switch.
    assert 'Mem/IncrementalMemory' not in params
    assert params['subscribe_rgbd'] is True
    assert params['subscribe_scan'] is True
    # '-d' clears the DB on start -> always begins a fresh map.
    assert '-d' in rtabmap._Node__arguments


def test_mapping_include_database_path():
    """Mapping session writes to an explicit, overridable database_path."""
    ld = _load_module(_INCLUDE).generate_launch_description()
    args = [e for e in ld.entities if type(e).__name__ == 'DeclareLaunchArgument']
    dbs = [a for a in args if a.name == 'database_path']
    assert len(dbs) == 1
    assert _as_text(dbs[0].default_value) == '~/.ros/rtabmap.db'

    params = _node_params(_rtabmap_node(ld))
    # The rtabmap node must actually carry a database_path parameter; its value is a
    # LaunchConfiguration substitution whose encoding differs across launch_ros builds,
    # so only its presence (plus the declared default above) is asserted here.
    assert 'database_path' in params


def test_mapping_include_subscribes_to_rgb_topics():
    """RGB remaps must point at /depth_cam/rgb/* (orbbec publishes rgb/*)."""
    ld = _load_module(_INCLUDE).generate_launch_description()
    for node in _nodes(ld):
        remaps = _node_remaps(node)
        targets = [t for s, t in remaps]
        assert '/depth_cam/color/' not in ' '.join(targets)
    rtabmap = _rtabmap_node(ld)
    remaps = _node_remaps(rtabmap)
    assert ('rgb/image', '/depth_cam/rgb/image_raw') in remaps
    assert ('rgb/camera_info', '/depth_cam/rgb/camera_info') in remaps
    assert ('depth/image', '/depth_cam/depth/image_raw') in remaps
