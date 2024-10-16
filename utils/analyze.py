import os
import glob
import pandas as pd

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
            'iae': calculate_iae(exp_log_path, dt=0.02),
            'average_absolute_jerk': calculate_average_absolute_jerk(exp_log_path, dt=0.02), 
            'peak_to_peak_jerk': calculate_peak_to_peak_jerk(exp_log_path, dt=0.02), 
            'appropriate_headway_proportion': calculate_appropriate_headway_proportion(exp_log_path), 
        })

    analysis_result_df = pd.DataFrame(analysis_result) 

    return analysis_result_df
