import json
import os

import networkx as nx
import numpy as np
import scipy.io
from docplex.util.status import JobSolveStatus as cp_status
from gurobipy import GRB
from matplotlib import pyplot as plt
from ortools.linear_solver import pywraplp
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

    if config.params["distance_coeff"] > 0:
        dis_matrix *= config.params["distance_coeff"]

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


def get_variable_value(var, solver):
    if solver == "GUROBI":
        return var.X
    elif solver == "CPLEX":
        return var.solution_value
    else:
        return var.solution_value()


def get_obj_value(model, solver):
    if solver == "GUROBI":
        return model.objVal
    elif solver == "CPLEX":
        return model.objective_value
    else:
        return model.Objective().Value()


def get_runtime(model, solver):
    if solver == "GUROBI":
        return model.getAttr("Runtime")
    elif solver == "CPLEX":
        return -1
    else:
        return model.WallTime()


def get_num_constraint(model, solver):
    if solver == "GUROBI":
        return model.getAttr("NumConstrs")
    elif solver == "CPLEX":
        return model.number_of_constraints
    else:
        return model.NumConstraints()


def get_num_var(model, solver):
    if solver == "GUROBI":
        return model.getAttr("NumVars")
    elif solver == "CPLEX":
        return model.number_of_variables
    else:
        return model.NumVariables()


def get_status(status, solver):
    if solver == "GUROBI":
        return "OPTIMAL" if status == GRB.OPTIMAL else "FEASIBLE"
    elif solver == "CPLEX":
        return "OPTIMAL" if status == cp_status.OPTIMAL_SOLUTION else "FEASIBLE"
    else:
        return "OPTIMAL" if status == pywraplp.Solver.OPTIMAL else "FEASIBLE"


def post_process(model, status, inp, config,
                 x, y, f, g, v, s, t, t_a, T, A, B, C, D, B_a, u):
    num_staff = config.params["num_staff"]
    num_cus = inp["num_cus"]
    cC1 = inp["C1"]
    cC = inp["C"]

    cC11 = cC1[:]
    cC11.append(0)
    cC12 = cC1[:]
    cC12.append(num_cus + 1)

    N1 = inp["C"][:]
    N1.append(0)

    N2 = inp["C"][:]
    N2.append(num_cus + 1)

    N = N2[:]
    N.append(0)

    num_drone_trip = len(cC1)

    result = {"x": {}, "y": {}, "f": {}, "g": {}, "s": {}, "t": {}, "t_a": {}, "T": {}, "v": {},
              "A": {}, "B": {}, "B_a": {}, "C": {}, "D": {}, "u": {}}

    result.update(dict(config.params))
    result.update(dict(config.solver))

    if config.solver.solver == "GUROBI":
        result["model_params"] = dict(config.solver.model_params.gurobi)
    elif config.solver.solver == "CPLEX":
        result["model_params"] = dict(config.solver.model_params.cplex)

    make_dirs_if_not_present(config.result_folder)

    if config.solver.solver == "GUROBI" and not (status == GRB.OPTIMAL or status == GRB.TIME_LIMIT):
        result = {"status": "INFEASIBLE" if status == GRB.INFEASIBLE else status}

    if config.solver.solver == "CPLEX" and not (
            status == cp_status.OPTIMAL_SOLUTION or status == cp_status.FEASIBLE_SOLUTION):
        result = {"status": "INFEASIBLE" if status == cp_status.INFEASIBLE_SOLUTION
                                            or cp_status.INFEASIBLE_OR_UNBOUNDED_SOLUTION else status}

    elif config.solver.solver != "GUROBI" and config.solver.solver != "CPLEX" and not (
            status == pywraplp.Solver.OPTIMAL or status == pywraplp.Solver.FEASIBLE):
        result = {"status": "INFEASIBLE" if status == 2 else status}

    else:
        tech2color = config.tech2color
        color2tech = {v: k for (k, v) in tech2color.items()}
        graph = nx.MultiDiGraph()
        drone_graph = nx.MultiDiGraph()

        print_all = False
        color_set = {}
        for k in range(num_staff):
            for j in N2:
                for i in N1:
                    if i == j:
                        continue
                    if get_variable_value(x[i, j, k], config.solver.solver) > 0 and not print_all:
                        result["x"][f"x[{i},{j},{k}]"] = get_variable_value(x[i, j, k], config.solver.solver)
                        try:
                            if i not in graph.nodes:
                                color_set[i] = tech2color[k]
                            elif k != color2tech[graph.nodes[i]['color']]:
                                color_set[i] = tech2color[-1]
                            if j not in graph.nodes:
                                color_set[j] = tech2color[k]
                            elif k != color2tech[graph.nodes[j]['color']]:
                                color_set[j] = tech2color[-1]
                            graph.add_nodes_from([
                                (i, {"color": color_set[i]}),
                                (j, {"color": color_set[j]})
                            ])
                            graph.add_edge(i, j)
                        except Exception as e:
                            print("Loi khi ve lai do thi duong di cua nhan vien: ", e)
        for r in range(num_drone_trip):
            for j in cC12:
                for i in cC11:
                    if i == j:
                        continue
                    if get_variable_value(y[i, j, r], config.solver.solver) > 0 and not print_all:
                        result["y"][f"y[{i},{j},{r}]"] = get_variable_value(y[i, j, r], config.solver.solver)
                        try:
                            drone_graph.add_nodes_from([i, j])
                            drone_graph.add_edge(i, j, label=r)
                        except Exception as e:
                            print("Loi khi ve lai do thi duong di cua drone: ", e)

        for k in range(num_staff):
            for r in range(num_drone_trip):
                for j in N2:
                    for i in cC1:
                        if i == j:
                            continue
                        if get_variable_value(f[i, j, k, r], config.solver.solver) > 0 and not print_all:
                            result["f"][f"f[{i},{j},{k},{r}]"] = get_variable_value(f[i, j, k, r], config.solver.solver)

        for k in range(num_staff):
            for r in range(num_drone_trip):
                for j in cC1:
                    for i in cC:
                        if get_variable_value(g[i, j, k, r], config.solver.solver) > 0 and not print_all:
                            result["g"][f"g[{i},{j},{k},{r}]"] = get_variable_value(g[i, j, k, r], config.solver.solver)

        for r in range(num_drone_trip):
            for i in cC12:
                if get_variable_value(v[i, r], config.solver.solver) > 0 and not print_all:
                    result["v"][f"v[{i}, {r}]"] = get_variable_value(v[i, r], config.solver.solver)

        for r in range(num_drone_trip):
            for i in cC:
                if get_variable_value(C[i, r], config.solver.solver) > 0 and not print_all:
                    result["C"][f"C[{i}, {r}]"] = get_variable_value(C[i, r], config.solver.solver)

        if config.ver == 1:
            for k in range(num_staff):
                for i in N:
                    if get_variable_value(s[i, k], config.solver.solver) > 0 and not print_all:
                        result["s"][f"s[{i}, {k}]"] = get_variable_value(s[i, k], config.solver.solver)

        for k in range(num_staff):
            for i in cC:
                if get_variable_value(D[i, k], config.solver.solver) > 0 and not print_all:
                    result["D"][f"D[{i}, {k}]"] = get_variable_value(D[i, k], config.solver.solver)

        for i in N:
            if get_variable_value(t[i], config.solver.solver) > 0 and not print_all:
                result["t"][f"t[{i}]"] = get_variable_value(t[i], config.solver.solver)
            if get_variable_value(t_a[i], config.solver.solver) > 0 and not print_all:
                result["t_a"][f"t_a[{i}]"] = get_variable_value(t_a[i], config.solver.solver)
            if get_variable_value(T[i], config.solver.solver) > 0 and not print_all:
                result["T"][f"T[{i}]"] = get_variable_value(T[i], config.solver.solver)

        for r in range(num_drone_trip):
            if get_variable_value(A[r], config.solver.solver) > 0 and not print_all:
                result["A"][f"A[{r}]"] = get_variable_value(A[r], config.solver.solver)

        for k in range(num_staff):
            if get_variable_value(B[k], config.solver.solver) > 0 and not print_all:
                result["B"][f"B[{k}]"] = get_variable_value(B[k], config.solver.solver)
            if get_variable_value(B_a[k], config.solver.solver) > 0 and not print_all:
                result["B_a"][f"B_a[{k}]"] = get_variable_value(B_a[k], config.solver.solver)
            if config.ver == 1 and get_variable_value(u[k], config.solver.solver) > 0 and not print_all:
                result["u"][f"u[{k}]"] = get_variable_value(u[k], config.solver.solver)

        result["Optimal"] = get_obj_value(model, config.solver.solver)
        result["Time"] = get_runtime(model, config.solver.solver)
        result["num_constraint"] = get_num_constraint(model, config.solver.solver)
        result['Number of variables'] = get_num_var(model, config.solver.solver)

        result["status"] = get_status(status, config.solver.solver)

        try:
            pos = nx.spectral_layout(graph, scale=10)
            plt.subplot(121)
            nx.draw_networkx(graph, pos, node_color=color_set.values(), font_size=16, font_color="whitesmoke",
                             node_size=500,
                             alpha=0.9)
            nx.draw_networkx_edge_labels(graph, pos, edge_labels=nx.get_edge_attributes(graph, 'label'), font_size=16)
            drone_pos = nx.spiral_layout(drone_graph, scale=10)
            edge_labels = dict([((u, v,), d['label'])
                                for u, v, d in drone_graph.edges(data=True)])
            plt.subplot(122)
            nx.draw(drone_graph, drone_pos)
            nx.draw_networkx(drone_graph, with_labels=True, pos=drone_pos)
            nx.draw_networkx_edge_labels(drone_graph, drone_pos, edge_labels=edge_labels)
            plt.savefig(os.path.join(config.result_folder, "result_" + inp['data_set'] + ".png"), dpi=1000)
            plt.clf()
        except Exception as e:
            print("Loi khi ghi do thi ra file: ", e)

    with open(os.path.join(config.result_folder, 'result_' + inp['data_set'] + '.json'), 'w') as json_file:
        json.dump(result, json_file, indent=2)


if __name__ == '__main__':
    pass
