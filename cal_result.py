import argparse
import glob
from datetime import datetime

from omegaconf import OmegaConf

from src.util import *

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='DASTS')

    parser.add_argument('--config', type=str, default="config_s.yml",
                        help='path of config file')
    args = parser.parse_args()
    config = OmegaConf.load(args.config)
    config.result_folder = os.path.join(config.result_folder, datetime.now().strftime("%m%d%Y%H%M%S"))
    paths = glob.glob(config.data_path)
    for data_set in paths:
        print(data_set)
        inp = load_input(config, data_set)
        if config.params.cus_per_staff > 0:
            config.params.num_staff = int(np.ceil(inp['num_cus'] / config.params.cus_per_staff))

        tau = inp["tau"]
        tau_a = inp["tau_a"]
        num_cus = inp["num_cus"]

        staff_trip = [[0, 4, 1, 5, 2, 6, 3, 7]]
        drone_trip = [[4], [1], [2], [6], [3]]

        wait_time = [0 for _ in range(num_cus + 2)]
        arrive_time = [0 for _ in range(num_cus + 2)]
        leave_time = [0 for _ in range(num_cus + 2)]

        cur_time_drone = 0
        cur_staff_ind = [0 for _ in range(len(staff_trip))]
        for dr in drone_trip:
            update_cus = []
            for i in dr:
                for s, t in enumerate(staff_trip):
                    if i in t:
                        index_of_i = t.index(i)
                        cur_cus_ind = cur_staff_ind[s]
                        cur_cus = t[cur_cus_ind]
                        for j in range(cur_cus_ind, index_of_i):
                            s_cus = t[j]
                            d_cus = t[j + 1]
                            arrive_time[d_cus] = leave_time[s_cus] + tau[s_cus, d_cus]
                            if 0 < j + 1 < index_of_i:
                                leave_time[d_cus] = arrive_time[d_cus]
                            elif j == index_of_i - 1:
                                if dr.index(i) == 0:
                                    cur_time_drone += tau_a[0, i]
                                else:
                                    cur_time_drone += tau_a[dr[index_of_i - 1], i]
                                leave_time[d_cus] = max(cur_time_drone, arrive_time[d_cus])
                                cur_time_drone = leave_time[d_cus]
                            update_cus.append(d_cus)

                        cur_staff_ind[s] = index_of_i
            cur_time_drone = leave_time[update_cus[-1]] + tau_a[update_cus[-1], 0]

            for i in update_cus:
                wait_time[i] = cur_time_drone - arrive_time[i]

        for s, t in enumerate(staff_trip):
            update_cus = []
            cur_cus_ind = cur_staff_ind[s]
            for i in range(cur_cus_ind, len(t) - 1):
                s_cus = t[i]
                d_cus = t[i + 1]
                arrive_time[d_cus] = leave_time[s_cus] + tau[s_cus, d_cus]
                leave_time[d_cus] = arrive_time[d_cus]
                if i < len(t) - 2:
                    update_cus.append(d_cus)
            for i in update_cus:
                wait_time[i] = arrive_time[d_cus] - arrive_time[i]
        print(sum(wait_time))
