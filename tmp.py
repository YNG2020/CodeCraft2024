import tqdm
import json
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed

def runFunction(seed, exe_path, map_path, main_exe_path):
    score = 0
    for path in map_path:
        command = [exe_path, "-m", path, main_exe_path, "-l", "NONE", "-s", str(seed), "-f 0"]
        result = subprocess.run(command, capture_output=True, text=True)
        result = json.loads(result.stdout)
        score += result["score"]
    return score

exe_path = ".\\SemiFinalJudge.exe"
map_path = ["./maps/map1.txt", "./maps/map2.txt", "./maps/map3.txt"]
main_exe_path = "./build/Release/main.exe"


scores = []
with ThreadPoolExecutor() as executor:
    future_to_seed = {executor.submit(runFunction, seed, exe_path, map_path, main_exe_path): seed for seed in range(100)}
    for future in tqdm.tqdm(as_completed(future_to_seed), total=len(future_to_seed), desc="Processing", unit="comb"):
        result = future.result()
        scores.append(result)

avg = sum(scores) / len(scores)
var = sum((x - avg) ** 2 for x in scores) / len(scores)
print(f"Average score: {avg}, Variance: {var}")
