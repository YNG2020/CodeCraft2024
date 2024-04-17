import subprocess
from tqdm import tqdm

def runFunction(params, exe_path, map_path, main_exe_path):
    modifyParams('param.txt', params)
    command = [exe_path, "-m", map_path, main_exe_path, "-l", "NONE"]
    result = subprocess.run(command, capture_output=True, text=True)
    return result.stdout

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
    output_file = 'output.txt'
    exe_path = ".\\SemiFinalJudge.exe"
    map_path = "./maps/map1.txt"
    main_exe_path = "./build/Release/main.exe"

    param_ranges = [
        (0.4, 0.6, 0.1),
        (1.5, 1.7, 0.1),
        (100, 120, 10),
        (500, 520, 10),
        (0, 1, 1),
        (4.0, 4.2, 0.1)
    ]

    param_combinations = generateParams(param_ranges)

    with open(output_file, 'w') as output, tqdm(total=len(param_combinations), desc="Processing", unit="comb") as pbar:
        for params in param_combinations:
            result = runFunction(params, exe_path, map_path, main_exe_path)
            output.write(str(params) + ': ' + result + '\n')
            pbar.update(1)

if __name__ == '__main__':
    main()
