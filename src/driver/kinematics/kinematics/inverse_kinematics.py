#!/usr/bin/env python3
# encoding: utf-8
# Placeholder for the real inverse kinematics module.
#
# See forward_kinematics.py for why this module is only a placeholder. The
# exact link lengths, joint layout and end-effector frame of the mounted arm
# are required before real IK can be implemented or verified against hardware.

def set_link(base_link, link1, link2, link3, end_effector_link):
    raise NotImplementedError(
        "kinematics.inverse_kinematics is a placeholder: no D-H model is "
        "available in this repository.")

def get_link():
    raise NotImplementedError(
        "kinematics.inverse_kinematics is a placeholder: no D-H model is "
        "available in this repository.")

def set_joint_range(j1, j2, j3, j4, j5, unit='deg'):
    raise NotImplementedError(
        "kinematics.inverse_kinematics is a placeholder: no D-H model is "
        "available in this repository.")

def get_joint_range(unit='deg'):
    raise NotImplementedError(
        "kinematics.inverse_kinematics is a placeholder: no D-H model is "
        "available in this repository.")

def get_ik(position, pitch, joint_range, resolution=1):
    raise NotImplementedError(
        "kinematics.inverse_kinematics is a placeholder: no D-H model is "
        "available in this repository.")

def get_position_ik(*args, **kwargs):
    raise NotImplementedError(
        "kinematics.inverse_kinematics is a placeholder: no D-H model is "
        "available in this repository.")
