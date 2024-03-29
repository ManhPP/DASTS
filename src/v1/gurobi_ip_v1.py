import gurobipy as gp
from gurobipy import GRB

from src.util import post_process, make_dirs_if_not_present


def solve_by_gurobi(config, inp):
    model = gp.Model("DASTS-GUROBI-V1")

    model.setParam("TimeLimit", config.solver.time_limit)
    model.setParam("IntegralityFocus", 1)

    try:
        special_params = config.solver.model_params.gurobi
        for p_name, p_value in special_params.items():
            model.setParam(p_name, p_value)
    except Exception:
        print("Khong co config bo sung cho mo hinh")

    make_dirs_if_not_present(config.result_folder)

    model.setParam("LogFile", config.result_folder + "/" + inp['data_set'] + ".log")
    # param
    num_cus = inp["num_cus"]
    num_staff = config.params["num_staff"]
    L = config.params["L"]
    L_a = config.params["L_a"]

    tau = inp["tau"]
    tau_a = inp["tau_a"]

    M = 1000 * L_a

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
        for j in N:
            for i in N:
                x[i, j, k] = model.addVar(vtype=GRB.BINARY, name=f"x[{i},{j},{k}]")

    for r in range(num_drone_trip):
        for j in N:
            for i in N:
                y[i, j, r] = model.addVar(vtype=GRB.BINARY, name=f"y[{i},{j},{r}]")

    for k in range(num_staff):
        for r in range(num_drone_trip):
            for j in N:
                for i in N:
                    f[i, j, k, r] = model.addVar(vtype=GRB.BINARY, name=f"f[{i},{j},{k},{r}]")
                    g[i, j, k, r] = model.addVar(vtype=GRB.BINARY, name=f"g[{i},{j},{k},{r}]")

    for r in range(num_drone_trip):
        for i in N:
            v[i, r] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L, name=f"v[{i},{r}]")
            C[i, r] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"C[{i},{r}]")

    for k in range(num_staff):
        for i in N:
            s[i, k] = model.addVar(vtype=GRB.INTEGER, lb=0, ub=int(num_cus), name=f"s[{i},{k}]")
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
        u[k] = model.addVar(vtype=GRB.BINARY, name=f"u[{k}]")

    # Obj
    model.setObjective(gp.quicksum(C[i, r] for i in cC for r in range(num_drone_trip)) + gp.quicksum(
        D[i, k] for i in cC for k in range(num_staff)), GRB.MINIMIZE)

    # Constraint

    for i in cC1:
        for r in range(num_drone_trip):
            # constraint 2
            model.addConstr(A[r] >= t_a[i] + tau_a[i, num_cus + 1] + M * (y[i, num_cus + 1, r] - 1), name=f"completeTimeDrone1_node{i}_trip{r}")

            # constraint 3
            model.addConstr(A[r] <= t_a[i] + tau_a[i, num_cus + 1] + M * (1 - y[i, num_cus + 1, r]), name=f"completeTimeDrone2_node{i}_trip{r}")

    for i in N1:
        for k in range(num_staff):
            # constraint 4
            model.addConstr(B[k] >= t[i] + tau[i, num_cus + 1] + M * (x[i, num_cus + 1, k] - 1), name=f"completeTimeTech1_node{i}_tech{k}")

            # constraint 5
            model.addConstr(B[k] <= t[i] + tau[i, num_cus + 1] + M * (1 - x[i, num_cus + 1, k]), name=f"completeTimeTech2_node{i}_tech{k}")

    # constraint 6
    model.addConstr(gp.quicksum(x[i, 0, k] for i in N2 for k in range(num_staff)) == 0, name=f"inDepotTech")

    # constraint 7
    model.addConstr(gp.quicksum(x[num_cus + 1, j, k] for j in N1 for k in range(num_staff)) == 0, name=f"outDepotTech")

    # constraint 8
    for k in range(num_staff):
        model.addConstr(gp.quicksum(x[0, j, k] for j in N2)
                        == gp.quicksum(x[i, num_cus + 1, k] for i in N1), name=f"inoutDepotTech_tech{k}")

    # constraint 9
    model.addConstr(gp.quicksum(y[i, 0, r] for i in N2 for r in range(num_drone_trip)) == 0, name=f"inDepotDrone")

    # constraint 10
    model.addConstr(gp.quicksum(y[num_cus + 1, j, r] for j in N1 for r in range(num_drone_trip)) == 0, name=f"outDepotDrone")

    for r in range(num_drone_trip):
        # constraint 11
        model.addConstr(gp.quicksum(y[0, j, r] for j in cC1) <= 1, name=f"outDepotDrone2_trip{r}")

        # constraint 12
        model.addConstr(gp.quicksum(y[0, j, r] for j in cC1)
                        == gp.quicksum(y[i, num_cus + 1, r] for i in cC1), name=f"inoutDepotDrone_trip{r}")

    for i in cC:
        for k in range(num_staff):
            # constraint 13
            model.addConstr(gp.quicksum(x[i, j, k] for j in N2)
                            == gp.quicksum(x[j, i, k] for j in N1), name=f"inoutCusTech_node{i}_tech{k}")

    for i in cC:
        # constraint 14
        model.addConstr(gp.quicksum(x[i, j, k] for j in N2 for k in range(num_staff) if j != i) == 1, name=f"cusOnceTech_node{i}")

        # bổ sung thêm để làm chặt giá trị biến x, không để biến này ngẫu nhiên
        model.addConstr(gp.quicksum(x[i, j, k] for j in N2 for k in range(num_staff) if j == i) == 0, name=f"extCusOnceTech_node{i}")

    for r in range(num_drone_trip):
        for i in N:
            # constraint 15
            model.addConstr(v[i, r] <= L, name=f"durDrone1_node{i}_trip{r}")

    for i in cC1:
        for j in cC1:
            for r in range(num_drone_trip):
                # constraint 16
                model.addConstr(v[j, r] >= v[i, r] + (t_a[j] - t_a[i]) + M * (y[i, j, r] - 1), name=f"durDrone20_node{i}_node{j}_trip{r}")

                # constraint 17
                model.addConstr(v[j, r] <= v[i, r] + (t_a[j] - t_a[i]) + M * (1 - y[i, j, r]), name=f"durDrone21_node{i}_node{j}_trip{r}")

    for j in cC1:
        for r in range(num_drone_trip):
            # constraint 18
            model.addConstr(v[j, r] >= tau_a[0, j] + M * (y[0, j, r] - 1), name=f"durDrone30_node{j}_trip{r}")

            # constraint 19
            model.addConstr(v[j, r] <= tau_a[0, j] + M * (1 - y[0, j, r]), name=f"durDrone31_node{j}_trip{r}")

    # model.addConstr(v[num_cus + 1, 0] == A[0])
    for r in range(num_drone_trip):
        # model.addConstr(v[num_cus + 1, r] <= A[r] - A[r - 1])
        for j in cC1:
            # constraint 20
            # TODO: xemlai
            model.addConstr(v[num_cus + 1, r] <= v[j, r] + tau_a[j, num_cus + 1] + M * (1 - y[j, num_cus + 1, r]), name=f"durDrone4_node{j}_trip{r}")

    # for i in cC:
    #     for j in cC:
    #         model.addConstr(gp.quicksum(y[i, j, r] for r in range(num_drone_trip))
    #                    + gp.quicksum(x[i, j, k] for k in range(num_staff))
    #                    <= 1)

    for r in range(num_drone_trip - 1):
        # constraint 21
        model.addConstr(gp.quicksum(y[0, j, r] for j in cC1)
                        >= gp.quicksum(y[0, j, r + 1] for j in cC1), name=f"seqDrone_trip{r}")

    for i in cC1:
        for r in range(num_drone_trip):
            # constraint 22
            model.addConstr(gp.quicksum(y[j, i, r] for j in cC11 if j != i)
                            == gp.quicksum(y[i, j, r] for j in cC12 if j != i), name=f"inoutCusDrone_node{i}_trip{r}")

    for i in cC1:
        # constraint 23
        model.addConstr(
            gp.quicksum(y[i, j, r] for r in range(num_drone_trip) for j in cC12 if i != j) <= 1, name=f"meetTechDroneOut_node{i}")

        # bo sung lam chat gia tri bien y
        model.addConstr(
            gp.quicksum(y[i, j, r] for r in range(num_drone_trip) for j in cC if i == j) == 0, name=f"extMeetTechDroneOut_node{i}")

    for k in range(num_staff):
        for r in range(num_drone_trip):
            # model.addConstr(gp.quicksum(f[i, j, k, r] for i in cC1 for j in N2) <= 1)
            # bo sung lam chat gia tri bien f
            model.addConstr(gp.quicksum(f[i, j, k, r] for i in N1 for j in N2 if i not in cC1) == 0, name=f"extMeetTechDroneOut_tech{k}_trip{r}")

    for j in cC:
        for k in range(num_staff):
            for i in N1:
                # constraint 24
                model.addConstr(s[j, k] >= s[i, k] + 1 - M * (
                        1 - x[i, j, k] + gp.quicksum(f[i, j, k, r] for r in range(num_drone_trip))), name=f"sampleAt1_node{i}_node{j}_tech{k}")
                # constraint 25
                model.addConstr(s[j, k] <= s[i, k] + 1 + M * (
                        1 - x[i, j, k] + gp.quicksum(f[i, j, k, r] for r in range(num_drone_trip))), name=f"sampleAt2_node{i}_node{j}_tech{k})")

    for k in range(num_staff):
        for i in N1:
            # constraint 26
            model.addConstr(s[num_cus + 1, k]
                            >= s[i, k] - M * (1 - x[i, num_cus + 1, k]
                                              + gp.quicksum(f[i, num_cus + 1, k, r]
                                                            for r in range(num_drone_trip)
                                                            for i in N1)), name=f"sampleAt3_node{i}_tech{k})")

            # constraint 27
            model.addConstr(s[num_cus + 1, k]
                            <= s[i, k] + M * (1 - x[i, num_cus + 1, k]
                                              + gp.quicksum(f[i, num_cus + 1, k, r]
                                                            for r in range(num_drone_trip)
                                                            for i in N1)), name=f"sampleAt4_node{i}_tech{k})")

    for j in cC:
        for k in range(num_staff):
            # constraint 28
            model.addConstr(
                s[j, k] >= 1 - M * (1 - gp.quicksum(f[i, j, k, r] for r in range(num_drone_trip) for i in cC1)), name=f"sampleAt5_node{j}_tech{k})")

            # constraint 29
            model.addConstr(
                s[j, k] <= 1 + M * (1 - gp.quicksum(f[i, j, k, r] for r in range(num_drone_trip) for i in cC1)), name=f"sampleAt6_node{j}_tech{k})")

            # constraint 30
            model.addConstr(s[j, k] <= M * gp.quicksum(x[j, i, k] for i in N2), name=f"sampleAt7_node{j}_tech{k})")

    # bo sung 2 rang buoc lam chat bien s
    for k in range(num_staff):
        model.addConstr(s[0, k] == 0, name=f"extS_node0_tech{k}")
        model.addConstr(
            s[num_cus + 1, k] <= M * (
                    1 - gp.quicksum(f[i, num_cus + 1, k, r] for r in range(num_drone_trip) for i in cC1)), name=f"extS_node{num_cus+1}_tech{k})")

    for i in cC1:
        for r in range(num_drone_trip):
            # constraint 31
            model.addConstr(
                gp.quicksum(f[i, j, k, r] for j in N2 for k in range(num_staff)) == gp.quicksum(
                    y[z, i, r] for z in cC11), name=f"syn_node{i}_trip{r}")

    for i in cC1:
        for j in N2:
            for k in range(num_staff):
                for r in range(num_drone_trip):
                    # constraint 32
                    model.addConstr(f[i, j, k, r] <= x[i, j, k], name=f"syn_node{i}_node{j}_tech{k}_trip{r}")

    for i in cC1:
        # constraint 33
        model.addConstr(t_a[i] >= t[i] - M * (
                1 - gp.quicksum(f[i, j, k, r] for k in range(num_staff) for r in range(num_drone_trip) for j in N2)), name=f"waitTechDrone1_node{i}")

    for i in cC1:
        # constraint 34
        model.addConstr(t_a[i] <= t[i] + M * (
                1 - gp.quicksum(f[i, j, k, r] for k in range(num_staff) for r in range(num_drone_trip) for j in N2)), name=f"waitTechDrone2_node{i}")

    for i in cC:
        for z in N1:
            # constraint 35
            model.addConstr(t[i] >= t[z] + tau[z, i] - M * (1 - gp.quicksum(x[z, i, k] for k in range(num_staff))), name=f"timeAtCusTech1_node{i}_node{z}")

    for i in cC:
        for z in N1:
            # constraint 36
            model.addConstr(
                t[i] <= t[z] + tau[z, i] + M * (1 - gp.quicksum(x[z, i, k] for k in range(num_staff)) + gp.quicksum(
                    f[i, j, k, r] for k in range(num_staff) for r in range(num_drone_trip) for j in N2)), name=f"timeAtCusTech2_node{i}_node{z}")

    for i in cC:
        for j in cC:
            for r in range(num_drone_trip - 1):
                # constraint 37
                model.addConstr(t[j] >= t[i] - M * (2 - y[0, i, r] - y[0, j, r + 1]), name=f"timeAtCusTech3_node{i}_node{j}_trip{r}")

    for j in cC1:
        for i in cC1:
            # constraint 38
            model.addConstr(
                t_a[j] >= t_a[i] + tau_a[i, j] - M * (1 - gp.quicksum(y[i, j, r] for r in range(num_drone_trip))), name=f"cusDrone_node{i}_node{j}")

    for j in cC1:
        # constraint 39
        model.addConstr(t_a[j] >= tau_a[0, j] - M * (
                1 - y[0, j, 0]), name=f"cusDrone_node{j}")
        for r in range(1, num_drone_trip):
            # constraint 40
            model.addConstr(t_a[j] >= A[r - 1] + tau_a[0, j] - M * (
                    1 - y[0, j, r]), name=f"cusDrone_node{j}_trip{r}")

    # constraint 41
    model.addConstr(t[0] == 0, name="leaveDepotTech")

    for i in N1:
        for j in cC:
            # constraint 42
            model.addConstr(T[j] >= t[i] + tau[i, j] - M * (1 - gp.quicksum(x[i, j, k] for k in range(num_staff))), name=f"timeGetSample1_node{i}_node{j}")

    for i in N1:
        for j in cC:
            # constraint 43
            model.addConstr(T[j] <= t[i] + tau[i, j] + M * (1 - gp.quicksum(x[i, j, k] for k in range(num_staff))), name=f"timeGetSample2_node{i}_node{j}")

    for i in cC:
        # constraint 44
        model.addConstr(
            gp.quicksum(g[i, j, k, r] for r in range(num_drone_trip) for k in range(num_staff) for j in cC) <= 1, name=f"getSampleNode1_node{i}")

    for i in cC:
        for j in cC:
            # constraint 45
            model.addConstr(t[j] >= t[i] - M * (1 - gp.quicksum(g[i, j, k, r]
                                                                for r in range(num_drone_trip)
                                                                for k in range(num_staff))), name=f"getSampleNode2_node{i}_node{j}")

    for i in cC:
        for k in range(num_staff):
            # constraint 46
            model.addConstr(gp.quicksum(g[i, j, k, r] for r in range(num_drone_trip) for j in cC)
                            <= gp.quicksum(x[i, j, k] for j in N2), name=f"getSampleNode3_node{i}_tech{k}")

    for k in range(num_staff):
        for r in range(num_drone_trip):
            for i in N1:
                for z in N1:
                    # constraint 47
                    model.addConstr(gp.quicksum(f[i, j, k, r] for j in N2)
                                    >= g[z, i, k, r] - M * (1 - gp.quicksum(x[z, j, k] for j in N2)), name=f"getSampleNode4_node{i}_ndoe{z}_tech{k}_trip{r}")

    for r in range(num_drone_trip):
        for j in cC:
            for k in range(num_staff):
                # constraint 48
                model.addConstr(
                    gp.quicksum(g[i, j, k, r] for i in cC) >= s[j, k] - M * (
                            1 - gp.quicksum(f[j, i, k, r] for i in N2)), name=f"sampleAt8_node{j}_tech{k}_trip{r}")

    for r in range(num_drone_trip):
        for j in cC:
            for k in range(num_staff):
                # constraint 49
                model.addConstr(
                    gp.quicksum(g[i, j, k, r] for i in cC) <= s[j, k] + M * (
                            1 - gp.quicksum(f[j, i, k, r] for i in N2)), name=f"sampleAt9_node{j}_tech{k}_trip{r}")

    for i in cC:
        for r in range(num_drone_trip):
            # constraint 51
            model.addConstr(C[i, r] <= M * gp.quicksum(g[i, j, k, r] for j in cC for k in range(num_staff)), name=f"sampleBroughtByDrone1_node{i}_trip{r}")

    for i in cC:
        for r in range(num_drone_trip):
            # constraint 52
            model.addConstr(
                C[i, r] >= A[r] - T[i] - M * (1 - gp.quicksum(g[i, j, k, r] for j in cC for k in range(num_staff))), name=f"sampleBroughtByDrone2_node{i}_trip{r}")

    for i in cC:
        for r in range(num_drone_trip):
            # constraint 53
            model.addConstr(
                C[i, r] <= A[r] - T[i] + M * (1 - gp.quicksum(g[i, j, k, r] for j in cC for k in range(num_staff))), name=f"sampleBroughtByDrone3_node{i}_trip{r}")

    for i in cC:
        for k in range(num_staff):
            # constraint 55
            model.addConstr(D[i, k] <= M * (1 - gp.quicksum(g[i, j, k, r] for j in cC for r in range(num_drone_trip))), name=f"sampleBroughtByTechnican1_node{i}tech{k}")

    for i in cC:
        for k in range(num_staff):
            # constraint 56
            model.addConstr(
                D[i, k] >= B[k] - T[i] - M * (
                        1 - gp.quicksum(x[i, j, k] for j in N2) + gp.quicksum(
                    g[i, j, k, r] for j in cC for r in range(num_drone_trip))), name=f"sampleBroughtByTechnican2_node{i}tech{k}")

    for i in cC:
        for k in range(num_staff):
            # constraint 57
            model.addConstr(
                D[i, k] <= B[k] - T[i] + M * (
                        1 - gp.quicksum(x[i, j, k] for j in N2) + gp.quicksum(
                    g[i, j, k, r] for j in cC for r in range(num_drone_trip))), name=f"sampleBroughtByTechnican3_node{i}tech{k}")

    for k in range(num_staff):
        # constraint 60
        model.addConstr(B_a[k] >= B[k] - M * (1 - u[k]), name=f"sampleTechTemp1_tech{k}")

        # constraint 61
        model.addConstr(B_a[k] <= B[k] + M * (1 - u[k]), name=f"sampleTechTemp2_tech{k}")

    for k in range(num_staff):
        # constraint 62
        model.addConstr(B_a[k] <= L_a, name=f"maxTime1_tech{k}")

    for i in cC:
        # constraint 63
        model.addConstr(t_a[i] + tau_a[i, num_cus + 1] <= L_a + M * (
                1 - gp.quicksum(y[i, num_cus + 1, r] for r in range(num_drone_trip))), name=f"maxTime2_node{i}")

    for k in range(num_staff):
        # constraint 58
        model.addConstr(u[k] <= s[num_cus + 1, k], name=f"techBringSampleToLab1_tech{k}")

    for k in range(num_staff):
        # constraint 59
        model.addConstr(num_cus * u[k] >= s[num_cus + 1, k], name=f"techBringSampleToLab2_tech{k}")

    model.optimize()
    model.write("model.lp")

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
