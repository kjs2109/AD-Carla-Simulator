import os 
import time 
import shutil
import subprocess 
import threading 

import sys 
sys.path.append(os.path.dirname(os.path.abspath(__file__))) 
from utils.data_visualization import save_log_visualization

lock = threading.Lock()


def check_carla_running():
    """Carla 서버가 실행 중인지 확인"""
    try:
        # Carla 서버 프로세스를 찾습니다.
        result = subprocess.run(['pgrep', '-f', 'CarlaUE4'], stdout=subprocess.PIPE, text=True)
        if result.returncode == 0:
            return True  # Carla 서버가 실행 중
        else:
            return False  # Carla 서버가 실행 x 
    except Exception as e:
        print("Carla 서버 확인 중 오류 발생:", e)
        return False


def start_carla_server():
    """Start the Carla server."""
    try:
        command = ['bash', '-c', f'cd {os.path.dirname(os.path.abspath(__file__))} && ./carla_simulator/CarlaUE4.sh --quality-level=Low -RenderOffScreen'] 
        subprocess.Popen(command)  
        print('서버 실행 중...')
        time.sleep(5) 
    except Exception as e:
        print("Error starting Carla server:", e)


def main(saved_root, test_name, scenario_path, scenario_params):
    test_root = os.path.join(saved_root, test_name)
    count = 1 

    for h in scenario_params['headway_time']: 
        for f in scenario_params['coeff_friction']: 
            for vut_speed in scenario_params['VUT_target_speed']:
                vut_speed_mps = round(vut_speed * 0.278, 2)
                for gvt_speed in scenario_params['GVT_target_speed']: 
                    gvt_speed_mps = round(gvt_speed * 0.278, 2) 

                    if not check_carla_running(): 
                        print("Carla server is not running. Starting the server...")
                        start_carla_server() 

                    # 서버가 실행될 때까지 기다리기
                    while not check_carla_running():
                        print("Waiting for Carla server to start...")
                        time.sleep(2)

                    if gvt_speed == 20: 
                        curr_scenario_path = scenario_path.replace('.xosc', '_280.xosc') 
                    elif gvt_speed == 60: 
                        curr_scenario_path = scenario_path.replace('.xosc', '_180.xosc') 
                    else: 
                        raise ValueError('GVT_target_speed must be 20 or 60.') 
                    
                    scenrio_param_text = f'headway_time:{h},coeff_friction:{f},VUT_target_velo:{vut_speed_mps},GVT_target_velo:{gvt_speed_mps}'
                    print(f'[{count}th scenario] headway_time: {h}, coeff_friction: {f}, VUT_target_velo: {vut_speed}, GVT_target_velo: {gvt_speed}', end=' ')

                    command = [
                        'python', 
                        'scenario_runner.py', 
                        '--openscenario', curr_scenario_path, 
                        '--reloadWorld', 
                        '--openscenarioparams', scenrio_param_text
                    ]

                    try:
                        result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, check=True)
                        print('명령어가 성공적으로 실행되었습니다.') 

                    except subprocess.CalledProcessError as e:
                        print('명령어 실행 중 에러가 발생했습니다.')
                        print('반환 코드:', e.returncode)
                        print('에러 메시지:', e.stderr)

                    with lock: 

                        curr_folder_name = sorted([name for name in os.listdir(os.path.join(test_root, 'temp')) 
                                                    if os.path.isdir(os.path.join(test_root, 'temp', name))])[-1] 
                        scenario_name = f'{h}-{str(f)}-{vut_speed}-{gvt_speed}'.replace('.', '')

                        if not os.path.isdir(os.path.join(test_root, scenario_name)): 
                            os.makedirs(os.path.join(test_root, scenario_name)) 

                        src_path = os.path.join(test_root, 'temp', curr_folder_name, 'log_data.csv')
                        dst_path = os.path.join(test_root, scenario_name, 'log_data.csv')  

                        shutil.copyfile(src_path, dst_path)
                        save_log_visualization(dst_path, scenario_name)

                    # time.sleep(10)
                    count += 1  


if __name__ == '__main__':
    saved_root = '/home/k/AD-Carla-Simulator/saved'
    test_name = 'test'
    scenario_path = './scenario/Euro_NCAP_ACC_CCRm_logical.xosc' 

    # km/h 
    scenario_params = {
        # 'iter_num': [1, 2, 3, 4, 5], 
        'headway_time': [1.6, 1.8, 2.0, 2.2, 2.4],
        'coeff_friction': [0.6, 0.7, 0.8, 0.9, 1.0, 1.1], 
        'VUT_target_speed': [80, 90, 100, 110, 120, 130], 
        'GVT_target_speed': [20, 60] 
    }

    start_time = time.time() 
    main(saved_root, test_name, scenario_path, scenario_params)
    print('실행 시간:', time.time() - start_time)
