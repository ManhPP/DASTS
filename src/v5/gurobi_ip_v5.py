import gurobipy as gp
from gurobipy import GRB

from src.util import make_dirs_if_not_present


def solve_by_gurobi_v5(config, inp):
    model = gp.Model("DASTS-GUROBI-V5")

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
    M = 1000 * L_a

    num_drone_trip = len(cC1)
    num_node_level = len(cC)

    A_t = [(i, j) for i in N1 for j in N2 if not ((i == 0 and j == num_cus + 1) or (i == j))]
    A_d = [(i, j) for i in cC11 for j in cC12 if not ((i == 0 and j == num_cus + 1) or (i == j))]

    x_tech = {}
    for i, j in A_t:
        for k in range(num_staff):
            for t in range(num_node_level):
                x_tech[i, j, k, t] = model.addVar(vtype=GRB.BINARY, name=f"x_tech[{i},{j},{k},{t}]")

    for i_a in cC:
        for k in range(num_staff):
            for t in range(1, num_node_level):
                model.addConstr(gp.quicksum(x_tech[i, j, k, t] for i, j in A_t if i == i_a)
                                == gp.quicksum(x_tech[j, i, k, t - 1] for j, i in A_t if i == i_a))

    for k in range(num_staff):
        model.addConstr(gp.quicksum(x_tech[i, j, k, t] for i, j in A_t for t in range(num_node_level))
                        <= gp.quicksum(x_tech[0, j, k, 0] for i, j in A_t if i == 0))

    for k in range(num_staff - 1):
        model.addConstr(gp.quicksum(x_tech[0, j, k, 0] for i, j in A_t if i == 0)
                        >= gp.quicksum(x_tech[0, j, k + 1, 0] for i, j in A_t if i == 0))

    model.addConstr(
        gp.quicksum(x_tech[i, j, k, t] for i, j in A_t for k in range(num_staff) for t in range(num_node_level) if
                    j != 0 and j != num_cus + 1) == 1)

    a = {}
    d = {}

    for k in range(num_staff):
        for t in range(num_node_level):
            a[t, k] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"a[{t},{k}]")
            d[t, k] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"d[{t},{k}]")

    for k in range(num_staff):
        model.addConstr(a[0, k] == 0)
        model.addConstr(d[0, k] == 0)

    for k in range(num_staff):
        for t in range(num_node_level):
            model.addConstr(d[t, k] <= M * gp.quicksum(x_tech[i, j, k, t] for i, j in A_t))

            model.addConstr(a[t, k] <= d[t, k] + M * (1 - gp.quicksum(x_tech[i, j, k, t] for i, j in A_t)))

        for t in range(1, num_node_level - 1):
            model.addConstr(a[t + 1, k] <= M * gp.quicksum(x_tech[i, j, k, t] for i, j in A_t))

            model.addConstr(
                a[t + 1, k] + M * (1 - gp.quicksum(x_tech[i, j, k, t] for i, j in A_t)) >= d[t, k] + gp.quicksum(
                    x_tech[i, j, k, t] * tau[i, j] for i, j in A_t))

    total_sample_time = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"total_sample_time")
    total_sample_time = gp.quicksum(a[t, k] for k in range(num_staff) for t in range(1, num_node_level))

    a_tech = {}
    a0_bar = {}
    for k in range(num_staff):
        a_tech[k] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"a_tech[{k}]")
        for t in range(num_node_level - 1):
            a0_bar[t + 1, k] = model.addVar(vtype=GRB.BINARY, name=f"a0_bar[{t + 1},{k}]")

            model.addConstr(a0_bar[t + 1, k] <= M * gp.quicksum(x_tech[i, j, k, t] for i, j in A_t if j == num_cus + 1))
            model.addConstr(a0_bar[t + 1, k] <= a[t + 1, k])
            model.addConstr(a0_bar[t + 1, k] >= a[t + 1, k] - M * (
                    1 - gp.quicksum(x_tech[i, j, k, t] for i, j in A_t if j == num_cus + 1)))

        model.addConstr(a_tech[k] == gp.quicksum(a0_bar[t, k] for t in range(1, num_node_level)))

    x_drone = {}
    for i, j in A_d:
        for r in range(num_drone_trip):
            x_drone[i, j, r] = model.addVar(vtype=GRB.BINARY, name=f"x_drone[{i},{j},{r}]")

    for i_a in cC1:
        for r in range(num_drone_trip):
            model.addConstr(gp.quicksum(x_drone[i, j, r] for i, j in A_d if i == i_a) == gp.quicksum(
                x_drone[j, i, r] for j, i in A_d if i == i_a))

            model.addConstr(gp.quicksum(x_drone[i, j, r] for i, j in A_d if i == i_a) <= gp.quicksum(
                x_drone[i, j, r] for i, j in A_d if i == 0))

        model.addConstr(gp.quicksum(x_drone[i, j, r] for r in range(num_drone_trip) for i, j in A_d if i == i_a) <= 1)

    for r in range(num_drone_trip - 1):
        model.addConstr(gp.quicksum(x_drone[i, j, r] for i, j in A_d if i == 0) >= gp.quicksum(
            x_drone[i, j, r + 1] for i, j in A_d if i == 0))

    model.addConstr(gp.quicksum(x_drone[i, j, r] * tau_a[i, j] for r in range(num_drone_trip) for i, j in A_d) <= L_a)

    for r in range(num_drone_trip):
        model.addConstr(
            gp.quicksum(x_drone[i, j, r] * tau_a[i, j] for i, j in A_d) <= L)

    a_drone = {}
    for r in range(num_drone_trip):
        for i_a in cC13:
            a_drone[i_a, r] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"a_drone[{i_a},{r}]")

            if i_a != num_cus + 1:
                model.addConstr(a_drone[i_a, r] <= M * gp.quicksum(x_drone[i, j, r] for i, j in A_d if i == i_a))

        model.addConstr(a_drone[num_cus + 1, r] <= M * gp.quicksum(
            x_drone[i, j, r] for i, j in A_d if j == num_cus + 1 and i != num_cus + 1))
        for i, j in A_d:
            model.addConstr(
                a_drone[j, r] + M * (1 - x_drone[i, j, r]) >= a_drone[i, r] + x_drone[i, j, r] * tau_a[i, j])

    for r in range(num_drone_trip - 1):
        model.addConstr(
            a_drone[0, r + 1] + M * (1 - gp.quicksum(x_drone[i, j, r] for i, j in A_d if i == 0)) >= a_drone[
                num_cus + 1, r])

    x_sync = {}

    for r in range(num_drone_trip):
        for k in range(num_staff):
            x_sync[k, r] = model.addVar(vtype=GRB.BINARY, name=f"x_sync[{k},{r}]")
            for i in cC1:
                x_sync[i, k, r] = model.addVar(vtype=GRB.BINARY, name=f"x_sync[{i},{k},{r}]")
                for t in range(num_node_level):
                    x_sync[i, t, k, r] = model.addVar(vtype=GRB.BINARY, name=f"x_sync[{i},{t},{k},{r}]")

            model.addConstr(x_sync[k, r] == gp.quicksum(x_sync[i, k, r] for i in cC1))
            for i in cC1:
                model.addConstr(x_sync[i, k, r] == gp.quicksum(x_sync[i, t, k, r] for t in range(1, num_node_level)))

                for t in range(num_node_level):
                    model.addConstr(
                        x_sync[i, t, k, r] <= gp.quicksum(x_tech[i_a, j, k, t] for i_a, j in A_t if i == i_a))

            model.addConstr(gp.quicksum(x_sync[i, t, k, r] for i in cC1 for t in range(num_node_level)) <= 1)
    for r in range(num_drone_trip - 1):
        model.addConstr(gp.quicksum(
            x_sync[i, t, k, r] for i in cC1 for t in range(num_node_level) for k in range(num_staff)) >= gp.quicksum(
            x_sync[i, t, k, r + 1] for i in cC1 for t in range(num_node_level) for k in range(num_staff)))

    y_tech = {}
    y_drone = {}
    y_cus = {}
    y_level = {}
    z = {}

    for i in cC:
        for k in range(num_staff):
            y_tech[i, k] = model.addVar(vtype=GRB.BINARY, name=f"y_tech[{i},{k}]")

    for i in cC1:
        for r in range(num_drone_trip):
            y_drone[i, r] = model.addVar(vtype=GRB.BINARY, name=f"y_drone[{i},{r}]")

    for k in range(num_staff):
        for r in range(num_drone_trip):
            for i in cC:
                y_cus[i, k, r] = model.addVar(vtype=GRB.BINARY, name=f"y_cus[{i},{k},{r}]")
                for t in range(num_node_level):
                    z[i, t, k, r] = model.addVar(vtype=GRB.BINARY, name=f"z[{i}, {t},{k},{r}]")

            for t in range(num_node_level):
                y_level[t, k, r] = model.addVar(vtype=GRB.BINARY, name=f"y_level[{t},{k},{r}]")

    for t in range(num_node_level):
        for k in range(num_staff):
            model.addConstr(d[t, k] >= a[t, k])
            model.addConstr(
                d[t, k] <= a[t, k] + M * gp.quicksum(z[i, t, k, r] for i in cC for r in range(num_drone_trip)))

            for r in range(num_drone_trip):
                for i in cC1:
                    model.addConstr(M * (1 - x_sync[i, t, k, r]) + d[t, k] >= a_drone[i, r])
                    model.addConstr(d[t, k] <= a_drone[i, r] + M * (1 - z[i, t, k, r]))

                    model.addConstr(z[i, t, k, r] <= x_sync[i, t, k, r])
                for i in cC:
                    if i not in cC1:
                        model.addConstr(z[i, t, k, r] == 0)

    y = {}

    for i in cC:
        for t in range(num_node_level):
            for k in range(num_staff):
                for r in range(num_drone_trip):
                    y[i, t, k, r] = model.addVar(vtype=GRB.BINARY, name=f"y[{i},{t},{k},{r}]")

    for i in cC:
        if i in cC1:
            model.addConstr(gp.quicksum(y_tech[i, k] for k in range(num_staff)) + gp.quicksum(
                y_drone[i, r] for r in range(num_drone_trip)) == 1)

        else:
            model.addConstr(gp.quicksum(y_tech[i, k] for k in range(num_staff)) == 1)

    for i in cC1:
        for r in range(num_drone_trip):
            model.addConstr(y_drone[i, r] == gp.quicksum(y_cus[i, k, r] for k in range(num_staff)))
            for k in range(num_staff):
                model.addConstr(y_cus[i, k, r] == gp.quicksum(y[i, t, k, r] for t in range(num_drone_trip)))

    for t in range(num_node_level):
        for r in range(num_drone_trip):
            for k in range(num_staff):
                model.addConstr(y_level[t, k, r] == gp.quicksum(y[i, t, k, r] for i in cC))

    for i in cC:
        for t in range(num_node_level):
            for k in range(num_staff):
                model.addConstr(gp.quicksum(y[i, t, k, r] for r in range(num_drone_trip)) <= 1)

                for r in range(num_drone_trip):
                    model.addConstr(
                        y[i, t, k, r] <= gp.quicksum(x_tech[i_a, j, k, t] for i_a, j in A_t if i == i_a and i_a != j))
                    model.addConstr(y[i, t, k, r] <= gp.quicksum(
                        x_sync[j, t_a, k, r] for j in cC1 for t_a in range(t, num_node_level) if j != i))

                    if i in cC1:
                        model.addConstr(y[i, t, k, r] >= x_sync[i, t, k, r])

                    if t < num_node_level - 1:
                        for j in cC:
                            if i != j:
                                model.addConstr(
                                    gp.quicksum(y[i, t, k, r_a] for r_a in range(r)) + 1 >= y[j, t + 1, k, r] + x_tech[
                                        i, j, k, t])

    a_tech_bar = {}
    a_drone_bar = {}

    for k in range(num_staff):
        for i in cC:
            a_tech_bar[i, k] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"a_tech_bar[{i},{k}]")

            model.addConstr(a_tech_bar[i, k] <= M * y_tech[i, k])
            model.addConstr(a_tech_bar[i, k] <= a_tech[k])
            model.addConstr(a_tech_bar[i, k] >= a_tech[k] - M * (1 - y_tech[i, k]))

    for r in range(num_drone_trip):
        for i in cC1:
            a_drone_bar[i, r] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"a_drone_bar[{i},{r}]")

            model.addConstr(a_drone_bar[i, r] <= M * y_drone[i, r])
            model.addConstr(a_drone_bar[i, r] <= M * a_drone[num_cus + 1, r])
            model.addConstr(a_drone_bar[i, r] >= M * a_drone[num_cus + 1, r] - M * (1 - y_drone[i, r]))

    e = {}
    for i in cC:
        e[i] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"e[{i}]")
        if i in cC1:
            model.addConstr(e[i] == gp.quicksum(a_tech_bar[i, k] for k in range(num_staff)) + gp.quicksum(
                a_drone_bar[i, r] for r in range(num_drone_trip)))

        else:
            model.addConstr(e[i] == gp.quicksum(a_tech_bar[i, k] for k in range(num_staff)))

    model.setObjective(
        gp.quicksum(e[i] for i in cC) - total_sample_time, GRB.MINIMIZE)

    model.write(config.result_folder + "/" + "model.lp")
    model.write(config.result_folder + "/" + "model.mps")
    model.optimize()

    if model.status == GRB.OPTIMAL or model.status == GRB.TIME_LIMIT:
        print('Optimal objective: %g' % model.objVal)
        print('Obj: %g' % model.objVal)
    elif model.status == GRB.INF_OR_UNBD:
        print('Model is infeasible or unbounded')
    elif model.status == GRB.INFEASIBLE:
        print('Model is infeasible')
    elif model.status == GRB.UNBOUNDED:
        print('Model is unbounded')
    else:
        print('===\nOptimization ended with status %d' % model.status)

