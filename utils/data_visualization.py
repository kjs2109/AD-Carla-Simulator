import matplotlib.pyplot as plt
import numpy as np
import pandas as pd 



def convert_to_kmh(velo_df):
    # m/s -> km/h 변환
    velo_kmh_df = velo_df.copy()
    velo_kmh_df['ego_velocity'] = velo_kmh_df['ego_velocity'] * 3.6 
    velo_kmh_df['target_velocity'] = velo_kmh_df['target_velocity'] * 3.6

    return velo_kmh_df


def save_log_visualization(file_path, scenario_name): 

    log_df = pd.read_csv(file_path)
    log_kmh_df = convert_to_kmh(log_df) 

    time = np.linspace(0, 45, len(log_kmh_df["ego_velocity"]))
    log_kmh_df['time'] = time

    # 그래프 그리기
    fig, ax1 = plt.subplots(figsize=(10, 6))

    # 좌측 y축 (속도 그래프)
    ax1.plot(log_kmh_df['time'], log_kmh_df['ego_velocity'], label='Ego Velocity', marker='o', color='b')
    ax1.plot(log_kmh_df['time'], log_kmh_df['target_velocity'], label='Target Velocity', marker='x', color='orange')

    # 좌측 y축 레이블
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Velocity (km/h)')
    ax1.legend(loc='upper left')

    # 1초 단위의 점선 그리드 설정 (x축)
    ax1.set_xticks(np.arange(0, 45, 1), minor=True)  
    ax1.grid(True, which='minor', axis='x', linestyle='--', color='gray', alpha=0.7) 

    # 5초 단위의 실선 그리드 설정 (x축)
    ax1.set_xticks(np.arange(0, 45, 5), minor=False)  
    ax1.grid(True, which='major', axis='x', linestyle='-', color='gray', alpha=0.9)  

    # 가로선(y축) 실선 그리드 설정
    ax1.grid(True, which='major', axis='y', linestyle='-', color='gray', alpha=0.9)  

    # 우측 y축 추가 (거리 그래프)
    ax2 = ax1.twinx()  
    ax2.plot(log_kmh_df['time'], log_kmh_df['ego_target_distance'], label='Distance', color='green', linestyle='--')

    # 우측 y축 레이블 및 범위 설정
    ax2.set_ylabel('Distance (m)')
    ax2.set_ylim(0, 60) 
    ax2.legend(loc='upper right')

    ax1.set_xlim(0, 45)

    # 그래프 제목
    plt.title(f'Ego and Target Vehicle Velocity & Distance Over Time [{scenario_name}]')

    # 그래프 저장
    plt.savefig(file_path.replace('log_data.csv', 'log_data.png'))


def log_visualization(file_path): 
    
    log_df = pd.read_csv(file_path)
    distance_velo_kmh_df = convert_to_kmh(log_df) 

    time = np.linspace(0, 45, len(distance_velo_kmh_df["ego_velocity"]))
    distance_velo_kmh_df['time'] = time

    fig, ax1 = plt.subplots(figsize=(10, 6))


    ax1.plot(distance_velo_kmh_df['time'], distance_velo_kmh_df['ego_velocity'], label='Ego Velocity', color='b', marker='o')
    ax1.plot(distance_velo_kmh_df['time'], distance_velo_kmh_df['target_velocity'], label='Target Velocity', color='orange', marker='x')

    # 좌측 y축 레이블
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Velocity (km/h)')
    ax1.legend(loc='upper left')

    # 그리드 설정 (x축)
    ax1.set_xticks(np.arange(0, 46, 1), minor=True)  
    ax1.grid(True, which='minor', axis='x', linestyle='--', color='gray', alpha=0.7) 

    ax1.set_xticks(np.arange(0, 46, 5), minor=False) 
    ax1.grid(True, which='major', axis='x', linestyle='-', color='gray', alpha=0.9) 

    # 그리드 설정 (y축)
    ax1.grid(True, which='major', axis='y', linestyle='-', color='gray', alpha=0.9)  

    # 우측 y축 추가 (거리 그래프)
    ax2 = ax1.twinx()  
    ax2.plot(distance_velo_kmh_df['time'], distance_velo_kmh_df['ego_target_distance'], label='Distance', color='green', linestyle='--')

    # 우측 y축 레이블 및 범위 설정
    ax2.set_ylabel('Distance (m)')
    ax2.set_ylim(0, 60)  
    ax2.legend(loc='upper right')

    ax1.set_xlim(0, 45)

    plt.title('Ego and Target Vehicle Velocity & Distance Over Time')
    plt.show() 


def log_visualization_with_acc_mode(file_path): 

    log_df = pd.read_csv(file_path)
    distance_velo_kmh_df = convert_to_kmh(log_df) 

    time = np.linspace(0, 45, len(distance_velo_kmh_df["ego_velocity"]))
    distance_velo_kmh_df['time'] = time

    # acc_mode가 0인 부분과 0이 아닌 부분으로 데이터 분리
    acc_mode_0 = distance_velo_kmh_df[distance_velo_kmh_df['acc_mode'] == 0]
    acc_mode_not_0 = distance_velo_kmh_df[distance_velo_kmh_df['acc_mode'] != 0]

    # 그래프 그리기
    fig, ax1 = plt.subplots(figsize=(10, 6))

    # 좌측 y축 (속도 그래프)
    ax1.scatter(acc_mode_not_0['time'], acc_mode_not_0['ego_velocity'], label='Ego Velocity', color='b', marker='o')
    ax1.scatter(acc_mode_0['time'], acc_mode_0['ego_velocity'], label='Ego Velocity (acc_mode=0)', color='r', marker='o')
    ax1.plot(distance_velo_kmh_df['time'], distance_velo_kmh_df['target_velocity'], label='Target Velocity', color='orange', marker='x')

    # 좌측 y축 레이블
    ax1.set_xlabel('Time')
    ax1.set_ylabel('Velocity (km/h)')
    ax1.legend(loc='upper left')

    # 그리드 설정
    # x 축
    ax1.set_xticks(np.arange(0, 46, 1), minor=True) 
    ax1.grid(True, which='minor', axis='x', linestyle='--', color='gray', alpha=0.7) 
    ax1.set_xticks(np.arange(0, 46, 5), minor=False) 
    ax1.grid(True, which='major', axis='x', linestyle='-', color='gray', alpha=0.9) 
    # y 축
    ax1.grid(True, which='major', axis='y', linestyle='-', color='gray', alpha=0.9) 

    # 우측 y축 추가 (거리 그래프)
    ax2 = ax1.twinx()  
    ax2.plot(distance_velo_kmh_df['time'], distance_velo_kmh_df['ego_target_distance'], label='Distance', color='green', linestyle='--')

    # 우측 y축 레이블 및 범위 설정
    ax2.set_ylabel('Distance (m)')
    ax2.set_ylim(0, 60) 
    ax2.legend(loc='upper right')

    ax1.set_xlim(0, 45)

    plt.title('Ego and Target Vehicle Velocity & Distance Over Time')
    plt.show()


def visualize_velocity(file_path, dt):
    
    data = pd.read_csv(file_path)

    time = pd.Series(range(len(data))) * dt  

    # 속도 및 가속도 추출
    velocity = data['ego_velocity']

    # 노이즈 필터링 (이동 평균)
    window_size = int(1 / dt)
    velocity_filtered = velocity.rolling(window=window_size).mean()

    # 플롯 생성
    plt.figure(figsize=(14, 6))

    plt.plot(time, velocity_filtered, label='Acceleration (Original)', color='blue')
    plt.title('Velocity Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend()

    plt.ylim(0, 25)

    # 그리드 설정
    ax = plt.gca()
    max_time = time.iloc[-1]

    # x 축 눈금 및 그리드 설정
    ax.set_xticks(np.arange(0, max_time + dt, 1), minor=True)
    ax.grid(True, which='minor', axis='x', linestyle='--', color='gray', alpha=0.7)
    ax.set_xticks(np.arange(0, max_time + dt, 5), minor=False)
    ax.grid(True, which='major', axis='x', linestyle='-', color='gray', alpha=0.9)

    # X축 레이블을 5초 단위로만 표시
    ax.set_xticklabels([str(int(tick)) if tick % 5 == 0 else '' for tick in ax.get_xticks()])

    plt.tight_layout()
    plt.show()


def visualize_acceleration(file_path, dt):

    data = pd.read_csv(file_path)

    time = pd.Series(range(len(data))) * dt  

    # 속도 및 가속도 추출
    velocity = data['ego_velocity']
    acceleration = data['ego_acceleration']

    # 속도 변화량으로 가속도 계산
    acceleration_from_velocity = velocity.diff() / dt  # 시간 간격 dt로 나눔

    # 노이즈 필터링 (이동 평균)
    window_size = int(1 / dt)
    acceleration_filtered = acceleration.rolling(window=window_size).mean()
    acceleration_velocity_filtered = acceleration_from_velocity.rolling(window=window_size).mean()

    # 플롯 생성
    plt.figure(figsize=(14, 6))

    plt.plot(time, acceleration_filtered, label='Acceleration (Original)', color='blue')
    plt.plot(time, acceleration_velocity_filtered, label='Acceleration (from Velocity)', color='green')
    plt.title('Acceleration Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.legend()

    plt.ylim(-10, 10)

    # 그리드 설정
    ax = plt.gca()
    max_time = time.iloc[-1]

    # x 축 눈금 및 그리드 설정
    ax.set_xticks(np.arange(0, max_time + dt, 1), minor=True)
    ax.grid(True, which='minor', axis='x', linestyle='--', color='gray', alpha=0.7)
    ax.set_xticks(np.arange(0, max_time + dt, 5), minor=False)
    ax.grid(True, which='major', axis='x', linestyle='-', color='gray', alpha=0.9)

    # X축 레이블을 5초 단위로만 표시
    ax.set_xticklabels([str(int(tick)) if tick % 5 == 0 else '' for tick in ax.get_xticks()])

    plt.tight_layout()
    plt.show()


def visualize_jerk(file_path, dt):

    data = pd.read_csv(file_path)

    time = pd.Series(range(len(data))) * dt
    window_size = int(1 / dt)

    # 가속도 데이터 추출
    acceleration = data['ego_acceleration']
    acceleration_filtered = acceleration.rolling(window=window_size).mean()

    # 가가속도 계산 (필터링된 가속도로부터)
    jerk_from_acceleration = acceleration_filtered.diff() / dt  # 시간 간격 dt로 나눔
    jerk_acceleration_filtered = jerk_from_acceleration.rolling(window=window_size).mean()

    # 속도 데이터 추출
    velocity = data['ego_velocity']
    acceleration_from_velocity = velocity.diff() / dt
    acceleration_velocity_filtered = acceleration_from_velocity.rolling(window=window_size).mean()

    # 가가속도 계산 (필터링된 가속도로부터)
    jerk_from_velocity = acceleration_velocity_filtered.diff() / dt
    jerk_velocity_filtered = jerk_from_velocity.rolling(window=window_size).mean()

    # 플롯 생성
    plt.figure(figsize=(14, 6))

    plt.plot(time, jerk_acceleration_filtered, label='Jerk (from Acceleration)', color='blue')
    plt.plot(time, jerk_velocity_filtered, label='Jerk (from Velocity)', color='orange')
    plt.title('Jerk Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Jerk (m/s³)')
    plt.legend()

    plt.ylim(-10, 10)

    # 그리드 설정 
    ax = plt.gca()
    max_time = time.iloc[-1]

    ax.set_xticks(np.arange(0, max_time + dt, 1), minor=True)
    ax.grid(True, which='minor', axis='x', linestyle='--', color='gray', alpha=0.7)
    ax.set_xticks(np.arange(0, max_time + dt, 5), minor=False)
    ax.grid(True, which='major', axis='x', linestyle='-', color='gray', alpha=0.9)

    # X축 레이블을 5초 단위로만 표시
    ax.set_xticklabels([str(int(tick)) if tick % 5 == 0 else '' for tick in ax.get_xticks()])

    plt.tight_layout()
    plt.show() 


def visualize_all(file_path, dt):

    data = pd.read_csv(file_path)

    time = pd.Series(range(len(data))) * dt
    window_size = int(1 / dt)

    # 속도 및 가속도 데이터 추출
    velocity = data['ego_velocity']
    acceleration = data['ego_acceleration']

    # 속도 데이터 필터링
    velocity_filtered = velocity.rolling(window=window_size).mean()

    # 가속도 계산 (속도 변화량으로부터)
    acceleration_from_velocity = velocity.diff() / dt
    acceleration_velocity_filtered = acceleration_from_velocity.rolling(window=window_size).mean()

    # 가속도 데이터 필터링
    acceleration_filtered = acceleration.rolling(window=window_size).mean()

    # 가가속도 계산 (필터링된 가속도로부터)
    jerk_from_acceleration = acceleration_filtered.diff() / dt
    jerk_acceleration_filtered = jerk_from_acceleration.rolling(window=window_size).mean()

    # 가속도로부터 계산한 가가속도 (속도로부터)
    acceleration_velocity_filtered = acceleration_from_velocity.rolling(window=window_size).mean()
    jerk_from_velocity = acceleration_velocity_filtered.diff() / dt
    jerk_velocity_filtered = jerk_from_velocity.rolling(window=window_size).mean()

    # 플롯 생성
    plt.figure(figsize=(14, 12))

    # 첫 번째 서브플롯: 속도
    plt.subplot(3, 1, 1)
    plt.plot(time, velocity_filtered, label='Velocity', color='blue')
    plt.title('Velocity Over Time')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    plt.ylim(0, 25)

    # 그리드 설정
    ax1 = plt.gca()
    max_time = time.iloc[-1]
    ax1.set_xticks(np.arange(0, max_time + dt, 1), minor=True)
    ax1.grid(True, which='minor', axis='x', linestyle='--', color='gray', alpha=0.7)
    ax1.set_xticks(np.arange(0, max_time + dt, 5), minor=False)
    ax1.grid(True, which='major', axis='x', linestyle='-', color='gray', alpha=0.9)
    ax1.set_xticklabels([])  

    # 두 번째 서브플롯: 가속도
    plt.subplot(3, 1, 2)
    plt.plot(time, acceleration_filtered, label='Acceleration (Original)', color='blue')
    plt.plot(time, acceleration_velocity_filtered, label='Acceleration (from Velocity)', color='green')
    plt.title('Acceleration Over Time')
    plt.ylabel('Acceleration (m/s²)')
    plt.legend()
    plt.ylim(-10, 10)

    # 그리드 설정
    ax2 = plt.gca()
    ax2.set_xticks(np.arange(0, max_time + dt, 1), minor=True)
    ax2.grid(True, which='minor', axis='x', linestyle='--', color='gray', alpha=0.7)
    ax2.set_xticks(np.arange(0, max_time + dt, 5), minor=False)
    ax2.grid(True, which='major', axis='x', linestyle='-', color='gray', alpha=0.9)
    ax2.set_xticklabels([])  

    # 세 번째 서브플롯: 가가속도
    plt.subplot(3, 1, 3)
    plt.plot(time, jerk_acceleration_filtered, label='Jerk (from Acceleration)', color='blue')
    plt.plot(time, jerk_velocity_filtered, label='Jerk (from Velocity)', color='orange')
    plt.title('Jerk Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Jerk (m/s³)')
    plt.legend()
    plt.ylim(-10, 10)

    # 그리드 설정
    ax3 = plt.gca()
    ax3.set_xticks(np.arange(0, max_time + dt, 1), minor=True)
    ax3.grid(True, which='minor', axis='x', linestyle='--', color='gray', alpha=0.7)
    ax3.set_xticks(np.arange(0, max_time + dt, 5), minor=False)
    ax3.grid(True, which='major', axis='x', linestyle='-', color='gray', alpha=0.9)
    # X축 레이블을 5초 단위로만 표시
    ax3.set_xticklabels([str(int(tick)) if tick % 5 == 0 else '' for tick in ax3.get_xticks()])

    plt.tight_layout()
    plt.show()


def visualize_all_comparison(file_paths, dt):
    # 파일 수
    num_files = len(file_paths)
    window_size = int(1 / dt)

    # 데이터 프레임과 시간 생성
    velocities = []
    accelerations = []
    times = []

    for file_path in file_paths:
        data = pd.read_csv(file_path)
        time = pd.Series(range(len(data))) * dt
        times.append(time)

        velocities.append(data['ego_velocity'])
        accelerations.append(data['ego_acceleration'])

    # 속도 및 가속도 필터링
    velocity_filtered = [velocity.rolling(window=window_size).mean() for velocity in velocities]
    acceleration_filtered = [acceleration.rolling(window=window_size).mean() for acceleration in accelerations]

    # 가가속도 계산
    jerk_acceleration_filtered = []
    for acc_filtered in acceleration_filtered:
        jerk_from_acceleration = acc_filtered.diff() / dt
        jerk_filtered = jerk_from_acceleration.rolling(window=window_size).mean()
        jerk_acceleration_filtered.append(jerk_filtered)

    # 플롯 생성
    plt.figure(figsize=(14, 18))

    # 첫 번째 서브플롯: 속도
    plt.subplot(3, 1, 1)
    for i in range(num_files):
        plt.plot(times[i], velocity_filtered[i], label=f'Velocity (File {i + 1})')
    plt.title('Velocity Comparison Over Time')
    plt.ylabel('Velocity (m/s)')
    plt.legend()
    plt.ylim(0, 25)

    # 두 번째 서브플롯: 가속도
    plt.subplot(3, 1, 2)
    for i in range(num_files):
        plt.plot(times[i], acceleration_filtered[i], label=f'Acceleration (File {i + 1})')
    plt.title('Acceleration Comparison Over Time')
    plt.ylabel('Acceleration (m/s²)')
    plt.legend()
    plt.ylim(-10, 10)

    # 세 번째 서브플롯: 가가속도
    plt.subplot(3, 1, 3)
    for i in range(num_files):
        plt.plot(times[i], jerk_acceleration_filtered[i], label=f'Jerk from Acceleration (File {i + 1})')
    plt.title('Jerk Comparison Over Time')
    plt.xlabel('Time (s)')
    plt.ylabel('Jerk (m/s³)')
    plt.legend()
    plt.ylim(-15, 15)

    plt.tight_layout()
    plt.show()
