import json
import os
import glob


def read_result():
    result = {}
    for path in glob.glob("result/**/*.json"):
        with open(path, ) as f:
            print(path)
            data = json.load(f)
            solver = data.get("solver", None)
            if solver is None:
                continue
            if solver not in result.keys():
                result[solver] = []

            result[solver].append(
                {"data_set": os.path.splitext(os.path.basename(path))[0].split("_")[1], "L": data["L"],
                 "L_a": data["L_a"], "staff_velocity": data["staff_velocity"],
                 "drone_velocity": data["drone_velocity"], "cus_per_staff": data["cus_per_staff"],
                 "status": data["status"], "optimal": data["Optimal"]})
    with open('result.json', 'w') as json_file:
        json.dump(result, json_file, indent=2)
    return result


if __name__ == '__main__':
    read_result()
