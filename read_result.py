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

            try:
                result[solver].append(
                    {"data_set": os.path.splitext(os.path.basename(path))[0].split("_")[1], "L": data["L"],
                     "L_a": data["L_a"], "staff_velocity": data["staff_velocity"],
                     "drone_velocity": data["drone_velocity"], "staff": f'{len(data.get("B"))}/{data["num_staff"]}',
                     "status": data["status"], "drone_completed_time": max(data["A"].values()),
                     "staff_completed_time": max(data["B"].values()), "optimal": data["Optimal"]})
            except Exception:
                result[solver].append(
                    {"data_set": os.path.splitext(os.path.basename(path))[0].split("_")[1], "L": data["L"],
                     "L_a": data["L_a"], "staff_velocity": data["staff_velocity"],
                     "drone_velocity": data["drone_velocity"], "staff": f'-/{data["num_staff"]}',
                     "status": data["status"], "drone_completed_time": "-",
                     "staff_completed_time": "-", "optimal": "-"})

    with open('result.json', 'w') as json_file:
        json.dump(result, json_file, indent=2)
    return result


if __name__ == '__main__':
    read_result()
