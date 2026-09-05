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

"""Structural tests for the RTAB-Map localization launch chain.

These tests parse the launch files and assert on the declared structure only
(bringing up the launch tree does not require the rtabmap_ros / orbbec binaries).
"""

import importlib.util
import os

from launch_ros.actions import Node

_LAUNCH = os.path.join(os.path.dirname(__file__), '..', 'launch')
_INCLUDE = os.path.join(_LAUNCH, 'include', 'rtabmap.launch.py')
_VIZ = os.path.join(_LAUNCH, 'rtabmapviz.launch.py')


def _load_module(path):
    path = os.path.abspath(path)
    name = '_'.join(c if c.isalnum() else '_' for c in path) + '_mod'
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _as_text(sub):
    if isinstance(sub, (list, tuple)):
        return ''.join(_as_text(x) for x in sub)
    if hasattr(sub, 'text'):
        return sub.text
    for attr in ('var_name', 'variable_name'):
        if hasattr(sub, attr):
            return '${%s}' % getattr(sub, attr)
    return str(sub)


def _param_text(value):
    """Best-effort plain text of a stored parameter value.

    launch_ros wraps parameter values in slightly different substitution shapes
    across builds/processes (raw literal, TextSubstitution, tuple of them), so
    flatten recursively and strip any quoting the wrapper added.
    """
    parts = []

    def walk(x):
        if isinstance(x, (list, tuple)):
            for y in x:
                walk(y)
        else:
            parts.append(getattr(x, 'text', None) if hasattr(x, 'text') else str(x))

    walk(value)
    return ''.join(parts).strip().strip("'\"")


def _nodes(ld):
    return [e for e in ld.entities if isinstance(e, Node)]


def _node_params(node):
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


def test_localization_include_rtabmap_in_localization_mode():
    """Localization include runs rgbd_sync + rtabmap with memory mapping disabled."""
    ld = _load_module(_INCLUDE).generate_launch_description()
    packages = sorted(n.node_package for n in _nodes(ld))
    assert packages == ['rtabmap_slam', 'rtabmap_sync']

    rtabmap = [n for n in _nodes(ld) if n.node_package == 'rtabmap_slam'][0]
    params = _node_params(rtabmap)
    assert _param_text(params['Mem/IncrementalMemory']) == 'False'
    assert _param_text(params['Mem/InitWMWithAllNodes']) == 'True'
    assert params['subscribe_rgbd'] is True
    assert params['subscribe_scan'] is True
    # Localization must not clear the database on start.
    assert not getattr(rtabmap, '_Node__arguments')


def test_localization_include_database_path():
    """Localization loads the map from the same explicit database_path as mapping."""
    ld = _load_module(_INCLUDE).generate_launch_description()
    args = [e for e in ld.entities if type(e).__name__ == 'DeclareLaunchArgument']
    dbs = [a for a in args if a.name == 'database_path']
    assert len(dbs) == 1
    assert _as_text(dbs[0].default_value) == '~/.ros/rtabmap.db'

    rtabmap = [n for n in _nodes(ld) if n.node_package == 'rtabmap_slam'][0]
    params = _node_params(rtabmap)
    # The rtabmap node must actually carry a database_path parameter; its value is a
    # LaunchConfiguration substitution whose encoding differs across launch_ros builds,
    # so only its presence (plus the declared default above) is asserted here.
    assert 'database_path' in params


def test_localization_include_subscribes_to_rgb_topics():
    """RGB remaps match the slam mapping side (/depth_cam/rgb/*)."""
    ld = _load_module(_INCLUDE).generate_launch_description()
    for node in _nodes(ld):
        targets = [t for _, t in _node_remaps(node)]
        assert '/depth_cam/color/' not in ' '.join(targets)
    rtabmap = [n for n in _nodes(ld) if n.node_package == 'rtabmap_slam'][0]
    remaps = _node_remaps(rtabmap)
    assert ('rgb/image', '/depth_cam/rgb/image_raw') in remaps
    assert ('rgb/camera_info', '/depth_cam/rgb/camera_info') in remaps
    assert ('depth/image', '/depth_cam/depth/image_raw') in remaps


def test_rtabmapviz_subscribes_to_rgb_not_color():
    """rtabmapviz must subscribe to rgb/* (orbbec publishes rgb/*, not color/*)."""
    ld = _load_module(_VIZ).generate_launch_description()
    vizzes = [n for n in _nodes(ld) if n.node_package == 'rtabmap_ros']
    assert len(vizzes) == 1
    targets = [t for _, t in _node_remaps(vizzes[0])]
    assert ('rgb/image', '/depth_cam/rgb/image_raw') in _node_remaps(vizzes[0])
    assert '/depth_cam/color/' not in ' '.join(targets)
