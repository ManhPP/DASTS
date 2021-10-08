import argparse
import glob
from datetime import datetime

from omegaconf import OmegaConf

from src.ortools_ip import solve_by_ortools
from src.gurobi_ip import solve_by_gurobi
from src.util import *


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='DASTS')

    parser.add_argument('--config', type=str, default="config.yml",
                        help='path of config file')
    args = parser.parse_args()
    config = OmegaConf.load(args.config)
    config.result_folder = os.path.join(config.result_folder, datetime.now().strftime("%m%d%Y%H%M%S"))
    paths = glob.glob(config.data_path)
    print(paths)
    for data_set in paths:
        print(data_set)
        try:
            inp = load_input(config, data_set)
            if config.params.cus_per_staff > 0:
                config.params.num_staff = int(np.ceil(inp['num_cus']/config.params.cus_per_staff))
            if config.solver.solver == "GUROBI":
                solve_by_gurobi(config, inp)
            else:
                solve_by_ortools(config, inp)
        except Exception as e:
            print("Error: ", e)
