#!/bin/bash
echo $PWD 

# 가상 환경 활성화
source carla-0913-env/bin/activate

# 현재 작업 디렉토리에 따라 CARLA 버전 설정 
export PYTHONPATH=$PWD/carla_simulator/carla-0.9.13-py3.7-linux-x86_64.egg:$PWD/srunner:PYTHONPATH

echo 'set up carla env!'