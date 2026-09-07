#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2022/11/21
import os
import subprocess

try:
    from ament_index_python.packages import get_package_share_directory
    wav_path = os.path.join(get_package_share_directory('xf_mic_asr_offline'), 'feedback_voice')
except Exception:
    # Fallback for running straight from a source workspace before install.
    wav_path = os.environ.get('VOICE_FEEDBACK_DIR', '/home/ubuntu/ros2_ws/src/xf_mic_asr_offline/feedback_voice')


def get_path(f, language='Chinese'):
    if language == 'Chinese':
        return os.path.join(wav_path, f + '.wav')
    return os.path.join(wav_path, 'english', f + '.wav')


def play(voice, volume=80, language='Chinese'):
    path = get_path(voice, language)
    if not os.path.exists(path):
        print('missing voice file:', path)
        return
    try:
        # Use an argument list (never a shell string) so file names with spaces or
        # shell metacharacters cannot be injected into a command line.
        subprocess.run(['play', '-q', path], check=False)
    except BaseException as e:
        print('error', e)


if __name__ == '__main__':
    play('ok')
    play('running', language="English")
