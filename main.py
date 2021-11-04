import argparse
import glob
from datetime import datetime

from omegaconf import OmegaConf

from src.v1.ortools_ip import solve_by_ortools
from src.v1.gurobi_ip_v1 import solve_by_gurobi
from src.v1.cplex_ip import solve_by_cplex
from src.util import *
from src.v2.gurobi_ip_v2 import solve_by_gurobi_v2

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='DASTS')

    parser.add_argument('--config', type=str, default="config.yml",
                        help='path of config file')
    args = parser.parse_args()
    config = OmegaConf.load(args.config)
    config.result_folder = os.path.join(config.result_folder, datetime.now().strftime("%m%d%Y%H%M%S"))

    for data_path in config.data_path.split(","):
        paths = glob.glob(data_path)
        print(paths)
        for data_set in paths:
            print(data_set)
            try:
                inp = load_input(config, data_set)
                if config.params.cus_per_staff > 0:
                    config.params.num_staff = int(np.ceil(inp['num_cus'] / config.params.cus_per_staff))

                if config.ver == 2:
                    solve_by_gurobi_v2(config, inp)
                else:
                    if config.solver.solver == "GUROBI":
                        solve_by_gurobi(config, inp)
                    elif config.solver.solver == "CPLEX":
                        solve_by_cplex(config, inp)
                    else:
                        solve_by_ortools(config, inp)
            except Exception as e:
                print("Error: ", e)
