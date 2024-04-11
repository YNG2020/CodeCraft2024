import subprocess
from tqdm import tqdm
import json
import time
from concurrent.futures import ThreadPoolExecutor

def runFunction(params, exe_path, map_paths, main_exe_path):
    modifyParams(params)
    total_score = 0
    with ThreadPoolExecutor(max_workers=len(map_paths)) as executor:
        futures = []
        for map_path in map_paths:
            futures.append(executor.submit(runProcess, exe_path, map_path, main_exe_path))
        for future in futures:
            result = future.result()
            output = json.loads(result.stdout)
            total_score += output['score']
    return total_score, params

def runProcess(exe_path, map_path, main_exe_path):
    command = [exe_path, "-m", map_path, main_exe_path, "-l", "NONE"]
    result = subprocess.run(command, capture_output=True, text=True)
    time.sleep(0.3)
    return result

def modifyParams(params):
    with open('param.txt', 'w') as file:
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
    map_paths = ["./maps/map1.txt", "./maps/map2.txt", "./maps/map3.txt"]
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

    for params in tqdm(param_combinations, desc="Processing", unit="comb"):
        score, params = runFunction(params, exe_path, map_paths, main_exe_path)
        results.append((params, score))

    results.sort(key=lambda x: x[1], reverse=True)

    with open('output.txt', 'w') as output_file:
        for params, score in results:
            output_file.write(str(params) + ': ' + str(score) + '\n')

if __name__ == '__main__':
    main()
