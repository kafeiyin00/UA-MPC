# 文件路径
file_path = "/workspace/ws_rotor_sim/src/RotorOptimalControl/system_evaluation/log/EvaluateComplete_NTU_optimal_10000.txt"  # 替换为您的文件路径

# 读取文件并处理数据
try:
    with open(file_path, 'r') as file:
        # 读取每一行并转换为整数
        numbers = [int(line.strip()) for line in file.readlines()]
    
    # 计算平均值
    if len(numbers) > 0:
        average = sum(numbers) / len(numbers)
        print("文件中的数字平均值为:", average)
    else:
        print("文件为空，无法计算平均值。")

except FileNotFoundError:
    print(f"文件未找到: {file_path}")
except ValueError as e:
    print(f"文件内容格式错误: {e}")