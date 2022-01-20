import json
import os

import gurobipy as gp
from gurobipy import GRB

from src.util import make_dirs_if_not_present, get_variable_value, get_obj_value, get_status, get_num_var, \
    get_num_constraint, get_runtime


def solve_by_gurobi_v4(config, inp):
    model = gp.Model("DASTS-GUROBI-V4")

    model.setParam("TimeLimit", config.solver.time_limit)
    model.setParam("IntegralityFocus", 1)

    try:
        special_params = config.solver.model_params.gurobi
        for p_name, p_value in special_params.items():
            model.setParam(p_name, p_value)
    except Exception:
        print("Khong co config bo sung cho mo hinh")

    make_dirs_if_not_present(config.result_folder)

    if config.solver.solver_log:
        model.setParam("LogFile", config.result_folder + "/" + inp['data_set'] + ".log")
    # param
    num_cus = inp["num_cus"]
    num_staff = config.params["num_staff"]
    L = config.params["L"]
    L_a = config.params["L_a"]

    tau = inp["tau"]
    tau_a = inp["tau_a"]

    # set
    cC1 = inp["C1"]
    cC = inp["C"]

    cC11 = cC1[:]
    cC11.append(0)
    cC12 = cC1[:]
    cC12.append(num_cus + 1)
    cC13 = cC11[:]
    cC13.append(num_cus + 1)

    N1 = inp["C"][:]
    N1.append(0)

    N2 = inp["C"][:]
    N2.append(num_cus + 1)

    N = N2[:]
    N.append(0)

    num_drone_trip = len(cC1)
    num_node_level = len(cC)

    M = 1000 * L_a

    # declare val
    x = {}
    for i in N1:
        for j in N2:
            for k in range(num_staff):
                for t in range(num_node_level):
                    if i != j:
                        x[i, j, k, t] = model.addVar(vtype=GRB.BINARY, name=f"x[{i},{j},{k},{t}]")

    for i in cC11:
        for j in cC12:
            for r in range(num_drone_trip):
                if i != j:
                    x[i, j, r] = model.addVar(vtype=GRB.BINARY, name=f"x[{i},{j},{r}]")

    a = {}
    d = {}

    for t in range(num_node_level):
        for k in range(num_staff):
            a[t, k] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"a[{t},{k}]")
            d[t, k] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"d[{t},{k}]")

    T = {}
    for k in range(num_staff):
        T[k] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"T[{k}]")

    b = {}
    for i in cC13:
        for r in range(num_drone_trip):
            b[i, r] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"b[{i},{r}]")

    m = {}
    for k in range(num_staff):
        for r in range(num_drone_trip):
            m[k, r] = model.addVar(vtype=GRB.BINARY, name=f"m[{k},{r}]")
            for i in cC1:
                m[i, k, r] = model.addVar(vtype=GRB.BINARY, name=f"m[{i},{k},{r}]")
                for t in range(num_node_level):
                    m[i, t, k, r] = model.addVar(vtype=GRB.BINARY, name=f"m[{i},{t},{k},{r}]")

    y = {}
    y_a = {}
    z = {}

    for k in range(num_staff):
        for i in cC:
            y[i, k] = model.addVar(vtype=GRB.BINARY, name=f"y[{i},{k}]")

    for i in cC1:
        for r in range(num_drone_trip):
            y_a[i, r] = model.addVar(vtype=GRB.BINARY, name=f"y_a[{i},{r}]")

    for k in range(num_staff):
        for r in range(num_drone_trip):
            for t in range(num_node_level):
                for i in cC:
                    y[i, t, k, r] = model.addVar(vtype=GRB.BINARY, name=f"y[{i},{t},{k},{r}]")
                for i in cC1:
                    z[i, t, k, r] = model.addVar(vtype=GRB.BINARY, name=f"z[{i},{t},{k},{r}]")

    e = {}
    for i in cC:
        e[i] = model.addVar(vtype=GRB.BINARY, name=f"e[{i}]")
        for k in range(num_staff):
            e[i, k] = model.addVar(vtype=GRB.BINARY, name=f"e[{i},{k}]")

    e_a = {}
    for i in cC1:
        for r in range(num_drone_trip):
            e_a[i, r] = model.addVar(vtype=GRB.BINARY, name=f"e_a[{i},{r}]")

    # 1
    for k in range(num_staff):
        for t in range(1, num_node_level):
            for i in cC:
                model.addConstr(gp.quicksum(x[i, j, k, t] for j in N2 if j != i)
                                == gp.quicksum(x[j, i, k, t - 1] for j in N1 if j != i), name=f"#1_{i}-{k}-{t}")
    # 2
    for k in range(num_staff):
        for i in N1:
            model.addConstr(gp.quicksum(x[i, j, k, t] for t in range(num_node_level) for j in N2 if j != i)
                            <= gp.quicksum(x[0, j, k, 0] for j in N2), name=f"#2_{i}-{k}")

    # 3
    for k in range(num_staff - 1):
        model.addConstr(gp.quicksum(x[0, j, k, 0] for j in N2)
                        >= gp.quicksum(x[0, j, k + 1, 0] for j in N2), name=f"#3_{k}")

    # 4
    for j in cC:
        model.addConstr(gp.quicksum(x[i, j, k, t]
                                    for i in N1
                                    for k in range(num_staff)
                                    for t in range(num_node_level) if i != j)
                        == 1, name=f"#4-{j}")

    # 5
    for k in range(num_staff):
        model.addConstr(gp.quicksum(x[0, j, k, 0] for j in N2)
                        == gp.quicksum(x[i, num_cus + 1, k, t] for i in N1
                                       for t in range(num_node_level)), name=f"#5_{k}")

    # 6
    for r in range(num_drone_trip):
        for i in cC1:
            model.addConstr(gp.quicksum(x[i, j, r] for j in cC12 if i != j)
                            == gp.quicksum(x[j, i, r] for j in cC11 if i != j), name=f"#6_{i}-{r}")

    # 7
    for r in range(num_drone_trip):
        for i in cC1:
            model.addConstr(gp.quicksum(x[i, j, r] for j in cC12 if i != j)
                            <= gp.quicksum(x[0, j, r] for j in cC1), name=f"#7_{i}-{r}")

    # 8
    for i in cC1:
        model.addConstr(gp.quicksum(x[i, j, r] for j in cC12 for r in range(num_drone_trip) if i != j) <= 1,
                        name=f"#8_{i}")

    # 9
    for r in range(num_drone_trip - 1):
        model.addConstr(gp.quicksum(x[0, j, r] for j in cC1) >= gp.quicksum(x[0, j, r + 1] for j in cC1),
                        name=f"#9_{r}")

    # 10
    model.addConstr(
        gp.quicksum(
            x[i, j, r] * tau_a[i, j] for r in range(num_drone_trip) for i in cC11 for j in cC12 if i != j) <= L_a,
        name=f"#10")

    # 11
    for r in range(num_drone_trip):
        model.addConstr(
            gp.quicksum(x[i, j, r] * tau_a[i, j] for i in cC11 for j in cC12 if i != j) <= L,
            name=f"#11_{r}")

    # 12
    for r in range(num_drone_trip):
        model.addConstr(gp.quicksum(x[0, j, r] for j in cC1) == gp.quicksum(x[i, num_cus + 1, r] for i in cC1),
                        name=f"#12_{r}")

    # 13
    for k in range(num_staff):
        model.addConstr(d[0, k] == 0, name=f"#13-1_{k}")
        model.addConstr(a[0, k] == 0, name=f"#13-2_{k}")

    # 14
    tmp14 = {}

    for k in range(num_staff):
        for t in range(num_node_level - 1):
            tmp14[k, t] = model.addVar(vtype=GRB.BINARY, name=f"tmp14[{k},{t}]")
            model.addConstr(tmp14[k, t] == gp.quicksum(x[i, j, k, t] for i in N1 for j in N2 if i != j),
                            name=f"#tmp-14_{k}-{t}")
            model.addConstr((tmp14[k, t] == 0) >> (d[t, k] == 0), name=f"#14-1_{k}-{t}")
            model.addConstr((tmp14[k, t] == 0) >> (a[t + 1, k] == 0), name=f"#14-2_{k}-{t}")

    # 15
    for k in range(num_staff):
        for t in range(num_node_level - 1):
            model.addConstr(
                (tmp14[k, t] == 1) >> (a[t + 1, k] == d[t, k] + gp.quicksum(
                    x[i, j, k, t] * tau[i, j] for i in N1 for j in N2 if i != j)), name=f"#15_{k}-{t}")

    # 16
    tmp16 = {}

    for k in range(num_staff):
        for t in range(num_node_level - 1):
            tmp16[k, t] = model.addVar(vtype=GRB.BINARY, name=f"tmp16[{k},{t}]")
            model.addConstr(tmp16[k, t] == gp.quicksum(x[i, num_cus + 1, k, t] for i in N1), name=f"#tmp-16_{k}-{t}")

            model.addConstr((tmp16[k, t] == 1) >> (T[k] == a[t + 1, k]), name=f"#16_{k}-{t}")

    # 17
    for k in range(num_staff):
        model.addConstr(T[k] <= L_a, name=f"#17_{k}")

    # 18
    tmp18 = {}

    for r in range(num_drone_trip):
        for i in cC11:
            tmp18[i, r] = model.addVar(vtype=GRB.BINARY, name=f"tmp18[{i},{r}]")
            model.addConstr(tmp18[i, r] == gp.quicksum(x[i, j, r] for j in cC12 if i != j), name=f"#tmp-18_{i}-{r}")

            model.addConstr((tmp18[i, r] == 0) >> (b[i, r] == 0), name=f"#18_{i}-{r}")

    # 19
    for r in range(num_drone_trip):
        for i in cC11:
            for j in cC12:
                if i != j:
                    model.addConstr((x[i, j, r] == 1) >> (b[j, r] >= b[i, r] + x[i, j, r] * tau_a[i, j]),
                                    name=f"#19_{i}-{j}-{r}")

    # 20
    tmp20 = {}

    for r in range(num_drone_trip - 1):
        tmp20[r] = model.addVar(vtype=GRB.BINARY, name=f"tmp20[{r}]")
        model.addConstr(tmp20[r] == gp.quicksum(x[0, j, r + 1] for j in cC12), name=f"#tmp-20_{r}")
        model.addConstr((tmp20[r] == 1) >> (b[0, r + 1] >= b[num_cus + 1, r]), name=f"#20_{r}")

    # 21
    for i in cC1:
        for k in range(num_staff):
            for r in range(num_drone_trip):
                for t in range(num_node_level):
                    model.addConstr(m[i, t, k, r] <= gp.quicksum(x[i, j, k, t] for j in N2 if i != j),
                                    name=f"#21_{i}-{t}-{k}-{r}")

    # 22
    for i in cC1:
        for k in range(num_staff):
            for r in range(num_drone_trip):
                model.addConstr(m[i, k, r] <= gp.quicksum(m[i, t, k, r] for t in range(num_node_level)),
                                name=f"#22_{i}-{k}-{r}")

    # 23
    for k in range(num_staff):
        for r in range(num_drone_trip):
            model.addConstr(m[k, r] <= gp.quicksum(m[i, k, r] for i in cC1), name=f"#23_{k}-{r}")

    # 24
    for r in range(num_drone_trip - 1):
        model.addConstr(
            gp.quicksum(m[k, r] for k in range(num_staff)) >= gp.quicksum(m[k, r + 1] for k in range(num_staff)),
            name=f"#24_{r}")

    # 25
    for k in range(num_staff):
        for r in range(num_drone_trip):
            for t in range(num_node_level):
                for i in cC1:
                    model.addConstr((m[i, t, k, r] == 1) >> (d[t, k] >= b[i, r]), name=f"#25_{i}-{t}-{k}-{r}")

    # 26
    tmp26 = {}

    for k in range(num_staff):
        for t in range(num_node_level):
            for i in cC1:
                tmp26[t, k] = model.addVar(vtype=GRB.BINARY, name=f"tmp26[{t},{k}]")
                model.addConstr(tmp26[t, k] == gp.quicksum(x[i, j, k, t] for j in N2 if i != j) - gp.quicksum(
                    m[i, t, k, r] for r in range(num_drone_trip)), name=f"#tmp26_{t}-{k}")

                model.addConstr((tmp26[t, k] == 1) >> (d[t, k] == a[t, k]), name=f"#26_{t}-{k}")

    # 27
    tmp27 = {}

    for k in range(num_staff):
        for t in range(num_node_level):
            tmp27[t, k] = model.addVar(vtype=GRB.BINARY, name=f"tmp27[{t},{k}]")
            model.addConstr(tmp27[t, k] == gp.quicksum(m[i, t, k, r] for i in cC1 for r in range(num_drone_trip)),
                            name=f"#tmp27_{t}-{k}")
            model.addConstr(
                (tmp27[t, k] == 1) >> (d[t, k] >= a[t, k]), name=f"#27_{t}-{k}")

    for k in range(num_staff):
        for t in range(num_node_level):
            for r in range(num_drone_trip):
                for i in cC1:
                    # 28
                    model.addConstr((m[i, t, k, r] == 1) >> (b[i, r] - a[t, k] <= M * z[i, t, k, r]),
                                    name=f"#28_{i}-{t}-{k}-{r}")

                    # 29
                    model.addConstr((m[i, t, k, r] == 1) >> (a[t, k] - b[i, r] <= M * (1 - z[i, t, k, r])),
                                    name=f"#29_{i}-{t}-{k}-{r}")

                    # 30
                    model.addConstr((m[i, t, k, r] == 1) >> (d[t, k] <= b[i, r] + M * (1 - z[i, t, k, r])),
                                    name=f"#30_{i}-{t}-{k}-{r}")

                    # 31
                    model.addConstr((m[i, t, k, r] == 1) >> (d[t, k] <= a[t, k] + M * z[i, t, k, r]),
                                    name=f"#31_{i}-{t}-{k}-{r}")

    # 32
    for i in cC:
        if i in cC1:
            model.addConstr(
                gp.quicksum(y[i, k] for k in range(num_staff)) + gp.quicksum(
                    y_a[i, r] for r in range(num_drone_trip)) == 1,
                name=f"#32_{i}")
        else:
            model.addConstr(
                gp.quicksum(y[i, k] for k in range(num_staff)) == 1,
                name=f"#32_{i}")
    # 33
    for i in cC1:
        for r in range(num_drone_trip):
            model.addConstr(
                y_a[i, r] == gp.quicksum(y[i, t, k, r] for k in range(num_staff) for t in range(num_node_level)),
                name=f"#33_{i}-{r}")

    # 34
    for k in range(num_staff):
        for t in range(num_node_level):
            for r in range(num_drone_trip):
                for i in cC1:
                    model.addConstr(y[i, t, k, r] <= gp.quicksum(x[i, j, k, t] for j in N2 if i != j), name=f"#34_{i}-{t}-{k}-{r}")

    # 35
    for k in range(num_staff):
        for t in range(num_node_level):
            for r in range(num_drone_trip):
                for i in cC1:
                    model.addConstr(y[i, t, k, r] <= gp.quicksum(
                        m[j, t_a, k, t] for t_a in range(t, num_node_level) for j in cC if j != i), name=f"#35_{i}-{t}-{k}-{r}")

    # 36
    for k in range(num_staff):
        for t in range(num_node_level):
            for r in range(num_drone_trip):
                for i in cC1:
                    model.addConstr(y[i, t, k, r] >= m[i, t, k, r], name=f"#36_{i}-{t}-{k}-{r}")

    # 37
    for k in range(num_staff):
        for t in range(num_node_level - 1):
            for r in range(num_drone_trip):
                for i in cC1:
                    for j in cC1:
                        if i != j:
                            model.addConstr(
                                gp.quicksum(y[i, t, k, r_a] for r_a in range(r + 1)) + 1 >= y[j, t + 1, k, r] + x[
                                    i, j, k, t], name=f"#37_{i}-{t}-{k}-{r}")

    # 38
    for i in cC:
        model.addConstr(e[i] == gp.quicksum(e[i, k] for k in range(num_staff)) + gp.quicksum(
            e_a[i, r] for r in range(num_drone_trip)), name=f"#38_{i}")

    # 39
    for k in range(num_staff):
        for i in cC:
            model.addConstr((y[i, k] == 1) >> (e[i, k] == T[k]), name=f"#39_{i}-{k}")

    # 40
    for r in range(num_drone_trip):
        for i in cC1:
            model.addConstr((y_a[i, r] == 1) >> (e_a[i, r] == b[num_cus + 1, r]), name=f"#40_{i}-{r}")

    # 41
    model.setObjective(
        gp.quicksum(e[i] for i in cC) - gp.quicksum(a[t, k] for t in range(num_node_level) for k in range(num_staff)),
        GRB.MINIMIZE)

    for r in range(num_drone_trip):
        model.addConstr(x[0, num_cus + 1, r] == 0)

    model.write(config.result_folder + "/" + "model.lp")
    model.write(config.result_folder + "/" + "model.mps")
    model.optimize()

    if model.status == GRB.OPTIMAL:
        print('Optimal objective: %g' % model.objVal)
    elif model.status == GRB.INF_OR_UNBD:
        print('Model is infeasible or unbounded')
    elif model.status == GRB.INFEASIBLE:
        print('Model is infeasible')
    elif model.status == GRB.UNBOUNDED:
        print('Model is unbounded')
    else:
        print('===\nOptimization ended with status %d' % model.status)

    print('Obj: %g' % model.objVal)

    status = model.status
    result = {"x": {}, "a": {}, "d": {}, "T": {}, "b": {}, "m": {}, "y": {}, "y_a": {}, "z": {}, "e": {}, "e_a": {}}

    result.update(dict(config.params))
    result.update(dict(config.solver))

    result["model_params"] = dict(config.solver.model_params.gurobi)

    if not (status == GRB.OPTIMAL or status == GRB.TIME_LIMIT):
        result = {"status": "INFEASIBLE" if status == GRB.INFEASIBLE else status}
    else:
        for i in N1:
            for j in N2:
                for k in range(num_staff):
                    for t in range(num_node_level):
                        if i != j and get_variable_value(x[i, j, k, t], config.solver.solver) > 0:
                            result["x"][f"x[{i},{j},{k},{t}]"] = get_variable_value(x[i, j, k, t], config.solver.solver)

        for i in cC11:
            for j in cC12:
                for r in range(num_drone_trip):
                    if i != j and get_variable_value(x[i, j, r], config.solver.solver) > 0:
                        result["x"][f"x[{i},{j},{r}]"] = get_variable_value(x[i, j, r], config.solver.solver)

        for t in range(num_node_level):
            for k in range(num_staff):
                if get_variable_value(a[t, k], config.solver.solver) > 0:
                    result["a"][f"a[{t},{k}]"] = get_variable_value(a[t, k], config.solver.solver)
                if get_variable_value(d[t, k], config.solver.solver) > 0:
                    result["d"][f"d[{t},{k}]"] = get_variable_value(d[t, k], config.solver.solver)

        for k in range(num_staff):
            if get_variable_value(T[k], config.solver.solver) > 0:
                result["T"][f"T[{k}]"] = get_variable_value(T[k], config.solver.solver)

        for i in cC13:
            for r in range(num_drone_trip):
                if get_variable_value(b[i, r], config.solver.solver) > 0:
                    result["b"][f"b[{i},{r}]"] = get_variable_value(b[i, r], config.solver.solver)

        for k in range(num_staff):
            for r in range(num_drone_trip):
                if get_variable_value(m[k, r], config.solver.solver) > 0:
                    result["m"][f"m[{k},{r}]"] = get_variable_value(m[k, r], config.solver.solver)
                for i in cC1:
                    if get_variable_value(m[i, k, r], config.solver.solver) > 0:
                        result["m"][f"m[{i},{k},{r}]"] = get_variable_value(m[i, k, r], config.solver.solver)
                    for t in range(num_node_level):
                        if get_variable_value(m[i, t, k, r], config.solver.solver) > 0:
                            result["m"][f"m[{i},{t},{k},{r}]"] = get_variable_value(m[i, t, k, r], config.solver.solver)

        for k in range(num_staff):
            for i in cC:
                if get_variable_value(y[i, k], config.solver.solver) > 0:
                    result["y"][f"y[{i},{k}]"] = get_variable_value(y[i, k], config.solver.solver)

        for i in cC1:
            for r in range(num_drone_trip):
                if get_variable_value(y_a[i, r], config.solver.solver) > 0:
                    result["y_a"][f"y_a[{i},{r}]"] = get_variable_value(y_a[i, r], config.solver.solver)

        for k in range(num_staff):
            for r in range(num_drone_trip):
                for t in range(num_node_level):
                    for i in cC:
                        if get_variable_value(y[i, t, k, r], config.solver.solver) > 0:
                            result["y"][f"y[{i},{t},{k},{r}]"] = get_variable_value(y[i, t, k, r], config.solver.solver)
                    for i in cC1:
                        if get_variable_value(z[i, t, k, r], config.solver.solver) > 0:
                            result["z"][f"z[{i},{t},{k},{r}]"] = get_variable_value(z[i, t, k, r], config.solver.solver)

        for i in cC:
            if get_variable_value(e[i], config.solver.solver) > 0:
                result["e"][f"e[{i}]"] = get_variable_value(e[i], config.solver.solver)
            for k in range(num_staff):
                if get_variable_value(e[i, k], config.solver.solver) > 0:
                    result["e"][f"e[{i},{k}]"] = get_variable_value(e[i, k], config.solver.solver)

        for i in cC1:
            for r in range(num_drone_trip):
                if get_variable_value(e_a[i, r], config.solver.solver) > 0:
                    result["e_a"][f"e_a[{i},{r}]"] = get_variable_value(e_a[i, r], config.solver.solver)

        result["Optimal"] = get_obj_value(model, config.solver.solver)
        result["Time"] = get_runtime(model, config.solver.solver)
        result["num_constraint"] = get_num_constraint(model, config.solver.solver)
        result['Number of variables'] = get_num_var(model, config.solver.solver)
        result["status"] = get_status(status, config.solver.solver)

    with open(os.path.join(config.result_folder, 'result_' + inp['data_set'] + '.json'), 'w') as json_file:
        json.dump(result, json_file, indent=2)
