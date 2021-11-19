import gurobipy as gp
from gurobipy import GRB

from src.util import post_process, make_dirs_if_not_present


def solve_by_gurobi_v2(config, inp):
    model = gp.Model("DASTS-GUROBI-V2")

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

    N1 = inp["C"][:]
    N1.append(0)

    N2 = inp["C"][:]
    N2.append(num_cus + 1)

    N = N2[:]
    N.append(0)

    num_drone_trip = len(cC1)

    # declare val
    x = {}
    y = {}
    f = {}
    g = {}
    v = {}
    s = {}
    t = {}
    t_a = {}  # apostrophe
    T = {}
    A = {}
    B = {}
    C = {}
    D = {}
    B_a = {}
    u = {}

    for k in range(num_staff):
        for j in N2:
            for i in N1:
                if i != j:
                    x[i, j, k] = model.addVar(vtype=GRB.BINARY, name=f"x[{i},{j},{k}]")

    for r in range(num_drone_trip):
        for j in cC12:
            for i in cC11:
                if i != j:
                    y[i, j, r] = model.addVar(vtype=GRB.BINARY, name=f"y[{i},{j},{r}]")

    for k in range(num_staff):
        for r in range(num_drone_trip):
            for j in N2:
                for i in cC1:
                    if i != j:
                        f[i, j, k, r] = model.addVar(vtype=GRB.BINARY, name=f"f[{i},{j},{k},{r}]")

    for k in range(num_staff):
        for r in range(num_drone_trip):
            for j in cC1:
                for i in cC:
                    g[i, j, k, r] = model.addVar(vtype=GRB.BINARY, name=f"g[{i},{j},{k},{r}]")

    for r in range(num_drone_trip):
        for i in cC12:
            v[i, r] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L, name=f"v[{i},{r}]")

    for r in range(num_drone_trip):
        for i in cC:
            C[i, r] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"C[{i},{r}]")

    for k in range(num_staff):
        for i in cC:
            D[i, k] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"D[{i},{k}]")

    for i in N:
        t[i] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"t[{i}]")
        t_a[i] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"t_a[{i}]")
        T[i] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"T[{i}]")

    for r in range(num_drone_trip):
        A[r] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"A[{r}]")

    for k in range(num_staff):
        B[k] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"B[{k}]")
        B_a[k] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"B_a[{k}]")

    # Obj
    model.setObjective(gp.quicksum(C[i, r] for i in cC for r in range(num_drone_trip)) + gp.quicksum(
        D[i, k] for i in cC for k in range(num_staff)), GRB.MINIMIZE)

    # Constraints

    # 2
    for i in cC1:
        for r in range(num_drone_trip):
            model.addConstr((y[i, num_cus + 1, r] == 1) >> (A[r] == t_a[i] + tau_a[i, num_cus + 1]),
                            name=f"completeTimeDrone_node{i}_trip{r}")
    # 3
    for i in N1:
        for k in range(num_staff):
            model.addConstr((x[i, num_cus + 1, k] == 1) >> (B[k] == t[i] + tau[i, num_cus + 1]),
                            name=f"completeTimeTech_node{i}_tech{k}")

    for k in range(num_staff):
        # 4
        model.addConstr(gp.quicksum(x[0, j, k] for j in N2)
                        == gp.quicksum(x[i, num_cus + 1, k] for i in N1), name=f"inoutDepotTech_tech{k}")
        # 5
        model.addConstr(gp.quicksum(x[0, j, k] for j in N2) <= 1, name=f"outDepotTech_tech{k}")

    for r in range(num_drone_trip):
        # 6
        model.addConstr(gp.quicksum(y[0, j, r] for j in cC1) <= 1, name=f"outDepotDrone_trip{r}")

        # 7
        model.addConstr(gp.quicksum(y[0, j, r] for j in cC1)
                        == gp.quicksum(y[i, num_cus + 1, r] for i in cC1), name=f"inoutDepotDrone_trip{r}")

    for i in cC:
        for k in range(num_staff):
            # 8
            model.addConstr(gp.quicksum(x[i, j, k] for j in N2 if i != j)
                            == gp.quicksum(x[j, i, k] for j in N1 if i != j), name=f"inoutCusTech_node{i}_tech{k}")

    for i in cC:
        # 9
        model.addConstr(gp.quicksum(x[i, j, k] for j in N2 for k in range(num_staff) if j != i) == 1,
                        name=f"cusOnceTech_node{i}")

    for r in range(num_drone_trip):
        # 10
        model.addConstr(v[num_cus+1, r] <= L, name=f"durDrone1_node{num_cus+1}_trip{r}")

    for i in cC1:
        for j in cC1:
            for r in range(num_drone_trip):
                # 111
                if i != j:
                    model.addConstr((y[i, j, r] == 1) >> (v[j, r] == v[i, r] + (t_a[j] - t_a[i])),
                                    name=f"durDrone2_node{i}_node{j}_trip{r}")

    for j in cC1:
        for r in range(num_drone_trip):
            # 12
            model.addConstr((y[0, j, r] == 1) >> (v[j, r] >= tau_a[0, j]), name=f"durDrone3_node{j}_trip{r}")

    for r in range(num_drone_trip):
        for j in cC1:
            # 13
            model.addConstr((y[j, num_cus + 1, r] == 1) >> (v[num_cus + 1, r] == v[j, r] + tau_a[j, num_cus + 1]),
                            name=f"durDrone4_node{j}_trip{r}")

    for r in range(num_drone_trip - 1):
        # 14
        model.addConstr(gp.quicksum(y[0, j, r] for j in cC1)
                        >= gp.quicksum(y[0, j, r + 1] for j in cC1), name=f"seqDrone_trip{r}")

    for i in cC1:
        for r in range(num_drone_trip):
            # 15
            model.addConstr(gp.quicksum(y[j, i, r] for j in cC11 if j != i)
                            == gp.quicksum(y[i, j, r] for j in cC12 if j != i), name=f"inoutCusDrone_node{i}_trip{r}")

    for i in cC1:
        # 16
        model.addConstr(
            gp.quicksum(y[i, j, r] for r in range(num_drone_trip) for j in cC12 if i != j) <= 1,
            name=f"meetTechDroneOut_node{i}")

    for i in cC1:
        for r in range(num_drone_trip):
            # 17
            model.addConstr(
                gp.quicksum(f[i, j, k, r] for j in N2 for k in range(num_staff) if i != j) == gp.quicksum(
                    y[z, i, r] for z in cC11 if z != i), name=f"syn_node{i}_trip{r}")

    for i in cC1:
        for j in N2:
            for k in range(num_staff):
                for r in range(num_drone_trip):
                    # 18
                    if i != j:
                        model.addConstr(f[i, j, k, r] <= x[i, j, k], name=f"syn_node{i}_node{j}_tech{k}_trip{r}")

    tmp_wait_tech_drone = {}
    for i in cC1:
        # 19
        tmp_wait_tech_drone[i] = model.addVar(vtype=GRB.INTEGER, name=f"tmp_wait_tech_drone[{i}]")
        model.addConstr(tmp_wait_tech_drone[i] == gp.quicksum(f[i, j, k, r] for k in range(num_staff)
                                                              for r in range(num_drone_trip)
                                                              for j in N2 if i != j), f"tmpWaitTechDrone_node{i}")
        model.addConstr((tmp_wait_tech_drone[i] == 1) >> (t_a[i] == t[i]), name=f"waitTechDrone_node{i}")

    tmp_time_at_cus_tech1 = {}
    for i in cC:
        for z in N1:
            # 20
            if z != i:
                tmp_time_at_cus_tech1[z, i] = model.addVar(vtype=GRB.INTEGER,
                                                           name=f"tmp_time_at_cus_tech1[{i},{z}]")
                model.addConstr(tmp_time_at_cus_tech1[z, i] == gp.quicksum(x[z, i, k] for k in range(num_staff)),
                                name=f"tmpTimeAtCusTech1_node{i}_node{z}")
                model.addConstr((tmp_time_at_cus_tech1[z, i] == 1) >> (t[i] >= t[z] + tau[z, i]),
                                name=f"timeAtCusTech1_node{i}_node{z}")

    tmp_time_at_cus_tech3 = {}
    for i in cC:
        for j in cC:
            for r in range(num_drone_trip - 1):
                # 21
                if i != j:
                    tmp_time_at_cus_tech3[i, j, r] = model.addVar(vtype=GRB.BINARY,
                                                                  name=f"tmp_time_at_cus_tech3[{i},{j},{r}]")
                    model.addGenConstrAnd(tmp_time_at_cus_tech3[i, j, r], [y[0, i, r], y[0, j, r + 1]],
                                          name=f"tmpTimeAtCusTech3_node{i}_node{j}_trip{r}")
                    model.addConstr((tmp_time_at_cus_tech3[i, j, r] == 1) >> (t[j] >= t[i]),
                                    name=f"timeAtCusTech3_node{i}_node{j}_trip{r}")

    tmp_cus_drone = {}
    for j in cC1:
        for i in cC1:
            # 22
            if i != j:
                tmp_cus_drone[i, j] = model.addVar(vtype=GRB.INTEGER, name=f"tmp_cus_drone[{i},{j}]")
                model.addConstr(tmp_cus_drone[i, j] == gp.quicksum(y[i, j, r] for r in range(num_drone_trip)),
                                name=f"tmpCusDrone_node{i}_node{j}")
                model.addConstr((tmp_cus_drone[i, j] == 1) >> (t_a[j] >= t_a[i] + tau_a[i, j]),
                                name=f"cusDrone_node{i}_node{j}")

    for j in cC1:
        # 23
        model.addConstr((y[0, j, 0] == 1) >> (t_a[j] >= tau_a[0, j]), name=f"cusDrone_node{j}_trip0")

    for j in cC1:
        for r in range(1, num_drone_trip):
            # 24
            model.addConstr((y[0, j, r] == 1) >> (t_a[j] >= A[r - 1] + tau_a[0, j]), name=f"cusDrone_node{j}_trip{r}")

    # 25
    model.addConstr(t[0] == 0, name="leaveDepotTech")

    for i in N1:
        for j in cC:
            # 26
            if i != j:
                model.addConstr((tmp_time_at_cus_tech1[i, j] == 1) >> (T[j] == t[i] + tau[i, j]),
                                name=f"timeGetSample_node{i}_node{j}")

    for i in cC:
        # 27
        model.addConstr(
            gp.quicksum(g[i, j, k, r] for r in range(num_drone_trip) for k in range(num_staff) for j in cC1) <= 1,
            name=f"getSampleNode1_node{i}")

    tmp_get_sample_node2 = {}
    for i in cC:
        for j in cC1:
            # 28
            tmp_get_sample_node2[i, j] = model.addVar(vtype=GRB.INTEGER, name=f"tmp_get_sample_node2[{i},{j}]")
            model.addConstr(tmp_get_sample_node2[i, j] == gp.quicksum(g[i, j, k, r]
                                                                      for r in range(num_drone_trip)
                                                                      for k in range(num_staff)),
                            name=f"tmpGetSampleNode2_node{i}_node{j}")
            model.addConstr((tmp_get_sample_node2[i, j] == 1) >> (t[j] >= t[i]), name=f"getSampleNode2_node{i}_node{j}")

    for i in cC:
        for k in range(num_staff):
            # 29
            model.addConstr(gp.quicksum(g[i, j, k, r] for r in range(num_drone_trip) for j in cC1)
                            <= gp.quicksum(x[i, j, k] for j in N2 if i != j), name=f"getSampleNode3_node{i}_tech{k}")

    tmp_get_sample_node4 = {}
    # 30
    for k in range(num_staff):
        for z in cC:
            tmp_get_sample_node4[z, k] = model.addVar(vtype=GRB.INTEGER,
                                                      name=f"tmp_get_sample_node4[{z},{k}]")
            model.addConstr(tmp_get_sample_node4[z, k] == gp.quicksum(x[z, j, k] for j in N2 if z != j),
                            name=f"tmpGetSampleNode4_node{z}_tech{k}")
            for r in range(num_drone_trip):
                for i in cC1:
                    model.addConstr(
                        (tmp_get_sample_node4[z, k] == 1) >> (gp.quicksum(f[i, j, k, r] for j in N2 if i != j)
                                                              >= g[z, i, k, r]),
                        name=f"getSampleNode4_node{i}_node{z}_tech{k}_trip{r}")

    # 31
    tmp_sample_brought_by_drone1 = {}
    for i in cC:
        for r in range(num_drone_trip):
            tmp_sample_brought_by_drone1[i, r] = model.addVar(vtype=GRB.INTEGER,
                                                              name=f"tmp_sample_brought_by_drone1[{i},{r}]")
            model.addConstr(
                tmp_sample_brought_by_drone1[i, r] == gp.quicksum(g[i, j, k, r] for j in cC1 for k in range(num_staff)),
                name=f"tmpSampleBroughtByDrone1_node{i}_trip{r}")
            model.addConstr((tmp_sample_brought_by_drone1[i, r] == 0) >> (C[i, r] == 0),
                            name=f"sampleBroughtByDrone1_node{i}_trip{r}")

    # 32
    tmp_sample_brought_by_drone2 = {}
    for i in cC:
        for r in range(num_drone_trip):
            tmp_sample_brought_by_drone2[i, r] = model.addVar(vtype=GRB.INTEGER,
                                                              name=f"tmp_sample_brought_by_drone2[{i},{r}]")
            model.addConstr(tmp_sample_brought_by_drone2[i, r] == gp.quicksum(
                g[i, j, k, r] for j in cC1 for k in range(num_staff)), name=f"tmpSampleBroughtByDrone2_node{i}_trip{r}")

            model.addConstr((tmp_sample_brought_by_drone2[i, r] == 1) >> (C[i, r] == A[r] - T[i]),
                            name=f"sampleBroughtByDrone2_node{i}_trip{r}")

    # 33
    tmp_sample_brought_by_technican1 = {}
    for i in cC:
        for k in range(num_staff):
            tmp_sample_brought_by_technican1[i, k] = model.addVar(vtype=GRB.INTEGER,
                                                                  name=f"tmp_sample_brought_by_technican1[{i},{k}]")
            model.addConstr(tmp_sample_brought_by_technican1[i, k] == gp.quicksum(
                g[i, j, k, r] for j in cC1 for r in range(num_drone_trip)),
                            name=f"tmpSampleBroughtByTechnican1_node{i}_tech{k}")

            model.addConstr((tmp_sample_brought_by_technican1[i, k] == 1) >> (D[i, k] == 0),
                            name=f"sampleBroughtByTechnican1_node{i}tech{k}")

    # 34
    tmp_sample_brought_by_technican2 = {}
    for i in cC:
        for k in range(num_staff):
            tmp_sample_brought_by_technican2[i, k] = model.addVar(vtype=GRB.INTEGER,
                                                                  name=f"tmp_sample_brought_by_technican2[{i},{k}]")
            model.addConstr(
                tmp_sample_brought_by_technican2[i, k] == gp.quicksum(x[i, j, k] for j in N2 if j != i) - gp.quicksum(
                    g[i, j, k, r] for j in cC1 for r in range(num_drone_trip)),
                name=f"tmpSampleBroughtByTechnican2_node{i}_tech{k}")
            model.addConstr((tmp_sample_brought_by_technican2[i, k] == 1) >> (D[i, k] == B[k] - T[i]),
                            name=f"sampleBroughtByTechnican2_node{i}tech{k}")

    # 35
    tmp_sample_tech_temp = {}
    for k in range(num_staff):
        tmp_sample_tech_temp[k] = model.addVar(vtype=GRB.INTEGER, name=f"tmp_sample_tech_temp[{k}]")
        model.addGenConstrOr(tmp_sample_tech_temp[k], [tmp_sample_brought_by_technican2[i, k] for i in cC],
                             name=f"tmpSampleTechTemp1_tech{k}")

        model.addConstr((tmp_sample_tech_temp[k] == 1) >> (B_a[k] == B[k]), name=f"sampleTechTemp1_tech{k}")

    for k in range(num_staff):
        # 36
        model.addConstr(B_a[k] <= L_a, name=f"maxTime_tech{k}")

    tmp_max_time = {}
    for i in cC:
        # 37
        tmp_max_time[i] = model.addVar(vtype=GRB.INTEGER, name=f"tmp_max_time[{i}]")
        model.addConstr(tmp_max_time[i] <= gp.quicksum(y[i, num_cus + 1, r] for r in range(num_drone_trip)),
                        name=f"tmpMaxTime1_node{i}")
        model.addConstr(tmp_max_time[i] >= gp.quicksum(y[i, num_cus + 1, r] for r in range(num_drone_trip)),
                        name=f"tmpMaxTime2_node{i}")
        model.addConstr((tmp_max_time[i] == 1) >> (t_a[i] + tau_a[i, num_cus + 1] <= L_a), name=f"maxTime_node{i}")

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

    post_process(model, model.status, inp, config,
                 x, y, f, g, v, s, t, t_a, T, A, B, C, D, B_a, u)
