import subprocess
from tqdm import tqdm
import concurrent.futures
import json
import time

def runFunction(params, exe_path, map_path, main_exe_path, param_id):
    param_file_name = f"params/{param_id}.txt"
    modifyParams(param_file_name, params)
    command = [exe_path, "-m", map_path, main_exe_path, "-l", "NONE"]
    result = subprocess.run(command, capture_output=True, text=True)
    time.sleep(0.3)
    return result.stdout, params

def modifyParams(file_name, params):
    with open(file_name, 'w') as file:
        for param in params:
            file.write(str(param) + '\n')

def generateParams(param_ranges):
    if not param_ranges:
        return [[]]
    first_range = param_ranges[0]
    rest_ranges = param_ranges[1:]
    rest_combinations = generateParams(rest_ranges)
    full_combinations = []
    current_value = first_range[0]
    while current_value <= first_range[1]:
        for combination in rest_combinations:
            full_combinations.append([current_value] + combination)
        current_value += first_range[2]
    return full_combinations

def main():
    exe_path = ".\\SemiFinalJudge.exe"
    map_path = "./maps/map1.txt"
    main_exe_path = "./build/Release/main.exe"

    param_ranges = [
        (0.4, 0.6, 0.1),
        (1.5, 1.7, 0.1),
        (80, 120, 10),
        (4.5, 4.5, 1),
        (4.0, 4.0, 1)
    ]

    param_combinations = generateParams(param_ranges)

    results = []
    param_id = 0

    # 设置为0
    with open('id.txt', 'w') as id_file:
        id_file.write(str(param_id))

    with concurrent.futures.ProcessPoolExecutor() as executor:
        futures = [executor.submit(runFunction, params, exe_path, map_path, main_exe_path, param_id + i) for i, params in enumerate(param_combinations)]
        for future in tqdm(concurrent.futures.as_completed(futures), total=len(futures), desc="Processing", unit="comb"):
            output, params = future.result()
            result = json.loads(output)
            results.append((params, result['score']))

    results.sort(key=lambda x: x[1], reverse=True)

    with open('output.txt', 'w') as output_file:
        for params, score in results:
            output_file.write(str(params) + ': ' + str(score) + '\n')

if __name__ == '__main__':
    main()
