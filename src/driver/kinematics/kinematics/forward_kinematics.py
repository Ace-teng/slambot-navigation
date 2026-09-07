#!/usr/bin/env python3
# encoding: utf-8
# Placeholder for the real forward kinematics module.
#
# The original implementation was shipped outside this repository and has not
# been recovered from any upstream source. Reproducing it requires the exact
# D-H parameters / link dimensions of the mounted arm, which are not committed
# here. Until a real implementation is supplied, every entry point raises
# NotImplementedError so that callers fail loudly instead of moving the arm on
# the basis of guessed kinematics.

class ForwardKinematics(object):
    def __init__(self, debug=False):
        self._debug = debug

    def get_link(self):
        raise NotImplementedError(
            "kinematics.forward_kinematics.ForwardKinematics is a placeholder: "
            "no D-H model is available in this repository.")

    def set_link(self, base_link, link1, link2, link3, end_effector_link):
        raise NotImplementedError(
            "kinematics.forward_kinematics.ForwardKinematics is a placeholder: "
            "no D-H model is available in this repository.")

    def get_joint_range(self, unit='deg'):
        raise NotImplementedError(
            "kinematics.forward_kinematics.ForwardKinematics is a placeholder: "
            "no D-H model is available in this repository.")

    def set_joint_range(self, j1, j2, j3, j4, j5, unit='deg'):
        raise NotImplementedError(
            "kinematics.forward_kinematics.ForwardKinematics is a placeholder: "
            "no D-H model is available in this repository.")

    def forward(self, *args, **kwargs):
        raise NotImplementedError(
            "kinematics.forward_kinematics.ForwardKinematics is a placeholder: "
            "no D-H model is available in this repository.")
