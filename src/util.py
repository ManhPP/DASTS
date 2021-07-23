import os

import numpy as np
import scipy.io
from scipy.spatial.distance import cdist


def load_input(config, data_set=None):
    if data_set is None:
        data_set = config.data_path
    result = {}
    if 'd1' in data_set:
        coordinates_matrix = np.loadtxt(data_set, skiprows=2)[:, :2]
        result["num_cus"] = len(coordinates_matrix)
        coordinates_matrix = np.insert(coordinates_matrix, 0, np.zeros(2), 0)
        dis_matrix = cdist(coordinates_matrix, coordinates_matrix)
    elif 'd2' in data_set:
        mat = scipy.io.loadmat(data_set)
        dis_matrix = mat['tspdist']
        result["num_cus"] = mat['NCus'][0][0]
    else:
        raise Exception("Unknown data")

    result["C"] = [i for i in range(1, result["num_cus"] + 1)]
    staff_velocity = config.params["staff_velocity"]
    drone_velocity = config.params["drone_velocity"]

    result["tau"] = {}
    result["tau_a"] = {}
    for i in range(len(dis_matrix)):
        for j in range(len(dis_matrix)):
            result["tau"][i, j] = float(dis_matrix[i, j] / staff_velocity)
            result["tau_a"][i, j] = float(dis_matrix[i, j] / drone_velocity)

    for i in range(len(dis_matrix)):
        result["tau"][result["num_cus"] + 1, i] = result["tau"][0, i]
        result["tau"][i, result["num_cus"] + 1] = result["tau"][i, 0]

        result["tau_a"][result["num_cus"] + 1, i] = result["tau_a"][0, i]
        result["tau_a"][i, result["num_cus"] + 1] = result["tau_a"][i, 0]

    result["tau"][result["num_cus"] + 1, result["num_cus"] + 1] = 0.0
    result["tau_a"][result["num_cus"] + 1, result["num_cus"] + 1] = 0.0

    result["C1"] = []
    for i in result["C"]:
        if result["tau_a"][0, i] <= config.params["L"]:
            result["C1"].append(i)
    result['data_set'] = os.path.splitext(os.path.basename(data_set))[0]
    return result


def make_dirs(path):
    if not os.path.exists(path):
        os.makedirs(path)


def make_dirs_if_not_present(path):
    """
    creates new directory if not present
    """
    if not os.path.exists(path):
        os.makedirs(path)


if __name__ == '__main__':
    pass
