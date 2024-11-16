import os
import glob
import pandas as pd
import seaborn as sns 
import matplotlib.pyplot as plt

def calculate_iae(data, target_distance=30.0, dt=0.02):
    """
    Calculate the Integral of Absolute Error (IAE) 
    """
    if not isinstance(data, pd.DataFrame): 
        data = pd.read_csv(data)

    data['absolute_error'] = abs(data['ego_target_distance'] - target_distance)

    iae = (data['absolute_error'].sum()) * dt

    return iae


def calculate_average_absolute_jerk(data, dt):
    """
    Calculate the Average Absolute Jerk
    """
    if not isinstance(data, pd.DataFrame): 
        data = pd.read_csv(data)

    window_size = int(1 / dt) 

    acceleration = data['ego_acceleration']
    jerk = acceleration.diff() / dt  

    jerk_filtered = jerk.rolling(window=window_size).mean()

    average_absolute_jerk = jerk_filtered.abs().mean()

    return average_absolute_jerk 


def calculate_peak_to_peak_jerk(data, dt): 

    if not isinstance(data, pd.DataFrame): 
        data = pd.read_csv(data)

    window_size = int(1 / dt)
    
    acceleration = data['ego_acceleration']
    jerk = acceleration.diff() / dt  

    jerk_filtered = jerk.rolling(window=window_size).mean()
    
    # Peak-to-Peak Jerk 계산
    peak_to_peak_jerk = jerk_filtered.max() - jerk_filtered.min()
    
    return peak_to_peak_jerk


def calculate_appropriate_headway_proportion(data):
    
    if not isinstance(data, pd.DataFrame):
        data = pd.read_csv(data)
    
    total_rows = len(data)
    headway_in_range = data[(data['headway_time'] >= 1) & (data['headway_time'] < 2.25)]
    proportion = len(headway_in_range) / total_rows
    
    return proportion


def check_fail_case(data, dt=0.02): 

    if not isinstance(data, pd.DataFrame): 
        data = pd.read_csv(data)

    window_size = int(1 / dt)
    acceleration = data['ego_acceleration']
    acceleration_filtered = acceleration.rolling(window=window_size).mean()

    fail_case = data[(data['time_to_collision'] <= 0) | (acceleration_filtered <= -5.0)]
    
    return fail_case


def get_max_deceleration(data, dt=0.02): 
    
    if not isinstance(data, pd.DataFrame): 
        data = pd.read_csv(data)

    window_size = int(1 / dt)
    acceleration = data['ego_acceleration']
    acceleration_filtered = acceleration.rolling(window=window_size).mean()
    acceleration_filtered = acceleration_filtered[acceleration_filtered.notna()] 

    max_deceleration = -min(acceleration_filtered)

    return max_deceleration


def get_score(data, dt=0.02): 

    if not isinstance(data, pd.DataFrame): 
        data = pd.read_csv(data)

    window_size = int(1 / dt)
    acceleration = data['ego_acceleration']
    acceleration_filtered = acceleration.rolling(window=window_size).mean()
    acceleration_filtered = acceleration_filtered[acceleration_filtered.notna()] 

    if (len(acceleration) != 2253) or (min(data['time_to_collision']) <= 0): 
        score = 0 
    elif min(acceleration_filtered) < -5.0: 
        score = 0.5 
    else: 
        score = 1.0 
    
    return score 

def make_analysis_result(exp_folder_path):

    exp_names = os.listdir(exp_folder_path)

    analysis_result = [] 
    for exp_name in exp_names: 
        if exp_name == 'temp': 
            continue 
        
        exp_log_path = os.path.join(exp_folder_path, exp_name, 'log_data.csv')
        exp_df = pd.read_csv(exp_log_path)

        analysis_result.append({
            'exp_name': exp_name, 
            'max_deceleration': get_max_deceleration(exp_log_path, dt=0.02),
            'average_absolute_jerk': calculate_average_absolute_jerk(exp_log_path, dt=0.02), 
            'peak_to_peak_jerk': calculate_peak_to_peak_jerk(exp_log_path, dt=0.02), 
            'score': get_score(exp_log_path, dt=0.02)
        })

    analysis_result_df = pd.DataFrame(analysis_result) 

    return analysis_result_df


# Function to remove outliers using IQR
def remove_outliers(series):
    Q1 = series.quantile(0.25)
    Q3 = series.quantile(0.75)
    IQR = Q3 - Q1
    lower_bound = Q1 - 1.5 * IQR
    upper_bound = Q3 + 1.5 * IQR
    return series[(series >= lower_bound) & (series <= upper_bound)]


def plot_box_plot(df_list, col_name, labels, title, ylabel, xlabel):

    data_to_plot_clean = [remove_outliers(df[col_name]) for df in df_list]

    plt.figure(figsize=(10, 6))
    plt.boxplot(data_to_plot_clean, labels=labels)
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xlabel(xlabel)

    plt.show()


def plot_box_plot_with_mean(df_list, col_name, labels, title, ylabel, xlabel):
    
    data_to_plot_clean = [remove_outliers(df[col_name]) for df in df_list]

    plt.figure(figsize=(10, 6))
    plt.boxplot(data_to_plot_clean, labels=labels, showmeans=False)

    means = [data.mean() for data in data_to_plot_clean]
    plt.plot(range(1, len(means) + 1), means, 'D', color='red', label='Mean')  

    plt.title(title)
    plt.ylabel(ylabel)
    plt.xlabel(xlabel)
    plt.legend()  

    plt.show()


def plot_histogram(df_list, col_name, labels, title, ylabel, xlabel, bins=30):
    plt.figure(figsize=(10, 6))

    for df, label in zip(df_list, labels):
        data = remove_outliers(df[col_name]) 
        plt.hist(data, bins=bins, alpha=0.5, label=label)  
    plt.title(title)
    plt.ylabel(ylabel)
    plt.xlabel(xlabel)
    plt.legend()  # 범례 추가

    plt.show() 



def plot_violin_plot(df_list, col_name, labels, title, ylabel, xlabel):

    data_to_plot_clean = [remove_outliers(df[col_name]) for df in df_list]

    plt.figure(figsize=(10, 6))
    plt.violinplot(data_to_plot_clean, showmeans=True)  

    plt.xticks(ticks=range(1, len(labels) + 1), labels=labels)

    plt.title(title)
    plt.ylabel(ylabel)
    plt.xlabel(xlabel)

    plt.show()  


def plot_box_plot_with_mean_seaborn(df_list, col_name, labels, title, ylabel, xlabel):
    # Seaborn 스타일과 색상 설정
    sns.set_style("whitegrid")
    sns.set_palette("pastel")

    # 이상치를 제거한 데이터를 준비하여 데이터프레임으로 변환
    data_to_plot_clean = [remove_outliers(df[col_name]) for df in df_list]
    combined_data = [(label, value) for data, label in zip(data_to_plot_clean, labels) for value in data]
    df_combined = pd.DataFrame(combined_data, columns=["Group", col_name])

    # 박스 플롯을 생성
    plt.figure(figsize=(12, 8))
    ax = sns.boxplot(x="Group", y=col_name, data=df_combined, showmeans=False, width=0.6)

    # 각 그룹별 평균을 계산하고 그래프에 추가
    means = [data.mean() for data in data_to_plot_clean]
    for i, mean in enumerate(means):
        plt.scatter(i, mean, color='red', marker='D', s=70, label='Mean' if i == 0 else "")

    # 제목과 라벨 설정
    plt.title(title, fontsize=16, fontweight='bold', color='darkblue')
    plt.ylabel(ylabel, fontsize=14)
    plt.xlabel(xlabel, fontsize=14)
    plt.legend(loc='upper right', frameon=True, fontsize=12, shadow=True)

    plt.show()
