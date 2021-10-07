import gurobipy as gp
from gurobipy import GRB


def gp_main(config, inp):
    model = gp.Model("dasts")
    model.setParam("TimeLimit", config.solver.time_limit)
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
            v[i, r] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L, name=f"v[{i}, {r}]")
            C[i, r] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"C[{i}, {r}]")

    for k in range(num_staff):
        for i in N:
            s[i, k] = model.addVar(vtype=GRB.INTEGER, lb=0, ub=int(num_cus), name=f"s[{i}, {k}]")
            D[i, k] = model.addVar(vtype=GRB.CONTINUOUS, lb=0, ub=L_a, name=f"D[{i}, {k}]")

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

    model.setObjective(gp.quicksum(C[i, r] for i in cC for r in range(num_drone_trip)) + gp.quicksum(
        D[i, k] for i in cC for k in range(num_staff)), GRB.MINIMIZE)

    # constraint
    for i in cC1:
        for r in range(num_drone_trip):
            model.addConstr(A[r] >= t_a[i] + tau_a[i, num_cus + 1] + M * (y[i, num_cus + 1, r] - 1))
            model.addConstr(A[r] <= t_a[i] + tau_a[i, num_cus + 1] + M * (1 - y[i, num_cus + 1, r]))

    for i in N1:
        for k in range(num_staff):
            model.addConstr(B[k] >= t[i] + tau[i, num_cus + 1] + M * (x[i, num_cus + 1, k] - 1))
            model.addConstr(B[k] <= t[i] + tau[i, num_cus + 1] + M * (1 - x[i, num_cus + 1, k]))

    model.addConstr(gp.quicksum(x[i, 0, k] for i in N2 for k in range(num_staff)) == 0)

    model.addConstr(gp.quicksum(x[num_cus + 1, j, k] for j in N1 for k in range(num_staff)) == 0)

    for k in range(num_staff):
        model.addConstr(gp.quicksum(x[0, j, k] for j in N2)
                        == gp.quicksum(x[i, num_cus + 1, k] for i in N1))

    model.addConstr(gp.quicksum(y[i, 0, r] for i in N2 for r in range(num_drone_trip)) == 0)

    model.addConstr(gp.quicksum(y[num_cus + 1, j, r] for j in N1 for r in range(num_drone_trip)) == 0)

    for r in range(num_drone_trip):
        model.addConstr(gp.quicksum(y[0, j, r] for j in cC1) <= 1)

        model.addConstr(gp.quicksum(y[0, j, r] for j in cC1)
                        == gp.quicksum(y[i, num_cus + 1, r] for i in cC1))

    for i in cC:
        for k in range(num_staff):
            model.addConstr(gp.quicksum(x[i, j, k] for j in N2)
                            == gp.quicksum(x[j, i, k] for j in N1))

    for i in cC:
        model.addConstr(gp.quicksum(x[i, j, k] for j in N2 for k in range(num_staff) if j != i) == 1)
        model.addConstr(gp.quicksum(x[i, j, k] for j in N2 for k in range(num_staff) if j == i) == 0)

    for r in range(num_drone_trip):
        for i in N:
            model.addConstr(v[i, r] <= L)

    for i in cC1:
        for j in cC1:
            for r in range(num_drone_trip):
                model.addConstr(v[j, r] >= v[i, r] + (t_a[j] - t_a[i]) + M * (y[i, j, r] - 1))
                model.addConstr(v[j, r] <= v[i, r] + (t_a[j] - t_a[i]) + M * (1 - y[i, j, r]))

    for j in cC1:
        for r in range(num_drone_trip):
            model.addConstr(v[j, r] >= tau_a[0, j] + M * (y[0, j, r] - 1))
            model.addConstr(v[j, r] <= tau_a[0, j] + M * (1 - y[0, j, r]))

    model.addConstr(v[num_cus + 1, 0] == A[0])
    for r in range(1, num_drone_trip):
        model.addConstr(v[num_cus + 1, r] == A[r] - A[r - 1])
        # model.addConstr(v[num_cus + 1, r] <= v[j, r] + tau_a[j, num_cus + 1] + M * (1 - y[j, num_cus + 1, r]))

    # for i in cC:
    #     for j in cC:
    #         model.addConstr(gp.quicksum(y[i, j, r] for r in range(num_drone_trip))
    #                    + gp.quicksum(x[i, j, k] for k in range(num_staff))
    #                    <= 1)

    for r in range(num_drone_trip - 1):
        model.addConstr(gp.quicksum(y[0, j, r] for j in cC1)
                        >= gp.quicksum(y[0, j, r + 1] for j in cC1))

    for i in cC1:
        for r in range(num_drone_trip):
            model.addConstr(gp.quicksum(y[j, i, r] for j in cC11 if j != i)
                            == gp.quicksum(y[i, j, r] for j in cC12 if j != i))

    for i in cC1:
        model.addConstr(
            gp.quicksum(y[i, j, r] for r in range(num_drone_trip) for j in cC12 if i != j) <= 1)

        model.addConstr(
            gp.quicksum(y[i, j, r] for r in range(num_drone_trip) for j in cC if i == j) == 0)

    for k in range(num_staff):
        for r in range(num_drone_trip):
            # model.addConstr(gp.quicksum(f[i, j, k, r] for i in cC1 for j in N2) <= 1)
            model.addConstr(gp.quicksum(f[i, j, k, r] for i in N1 for j in N2 if i not in cC1) == 0)

    for j in cC:
        for k in range(num_staff):
            for i in N1:
                model.addConstr(s[j, k] >= s[i, k] + 1 - M * (
                        1 - x[i, j, k] + gp.quicksum(f[i, j, k, r] for r in range(num_drone_trip))))
                model.addConstr(s[j, k] <= s[i, k] + 1 + M * (
                        1 - x[i, j, k] + gp.quicksum(f[i, j, k, r] for r in range(num_drone_trip))))

    for k in range(num_staff):
        for i in N1:
            model.addConstr(s[num_cus + 1, k]
                            >= s[i, k] - M * (1 - x[i, num_cus + 1, k]
                                              + gp.quicksum(f[i, num_cus + 1, k, r]
                                                            for r in range(num_drone_trip)
                                                            for i in N1)))
            model.addConstr(s[num_cus + 1, k]
                            <= s[i, k] + M * (1 - x[i, num_cus + 1, k]
                                              + gp.quicksum(f[i, num_cus + 1, k, r]
                                                            for r in range(num_drone_trip)
                                                            for i in N1)))

    for j in cC:
        for k in range(num_staff):
            model.addConstr(
                s[j, k] >= 1 - M * (1 - gp.quicksum(f[i, j, k, r] for r in range(num_drone_trip) for i in cC1)))
            model.addConstr(
                s[j, k] <= 1 + M * (1 - gp.quicksum(f[i, j, k, r] for r in range(num_drone_trip) for i in cC1)))
            model.addConstr(s[j, k] <= M * gp.quicksum(x[j, i, k] for i in N2))

    for k in range(num_staff):
        model.addConstr(s[0, k] == 0)
        model.addConstr(
            s[num_cus + 1, k] <= M * (
                    1 - gp.quicksum(f[i, num_cus + 1, k, r] for r in range(num_drone_trip) for i in cC1)))

    for i in cC1:
        for r in range(num_drone_trip):
            model.addConstr(
                gp.quicksum(f[i, j, k, r] for j in N2 for k in range(num_staff)) == gp.quicksum(
                    y[z, i, r] for z in cC11))

    for i in cC1:
        for j in N2:
            for k in range(num_staff):
                for r in range(num_drone_trip):
                    model.addConstr(f[i, j, k, r] <= x[i, j, k])

    for i in cC1:
        model.addConstr(t_a[i] >= t[i] - M * (
                1 - gp.quicksum(f[i, j, k, r] for k in range(num_staff) for r in range(num_drone_trip) for j in N2)))

    for i in cC1:
        model.addConstr(t_a[i] <= t[i] + M * (
                1 - gp.quicksum(f[i, j, k, r] for k in range(num_staff) for r in range(num_drone_trip) for j in N2)))

    for i in cC:
        for z in N1:
            model.addConstr(t[i] >= t[z] + tau[z, i] - M * (1 - gp.quicksum(x[z, i, k] for k in range(num_staff))))

    for i in cC:
        for z in N1:
            model.addConstr(
                t[i] <= t[z] + tau[z, i] + M * (1 - gp.quicksum(x[z, i, k] for k in range(num_staff)) + gp.quicksum(
                    f[i, j, k, r] for k in range(num_staff) for r in range(num_drone_trip) for j in N2)))

    for i in cC:
        for j in cC:
            for r in range(num_drone_trip - 1):
                model.addConstr(t[j] >= t[i] - M * (2 - y[0, i, r] - y[0, j, r + 1]))

    for j in cC1:
        for i in cC1:
            model.addConstr(
                t_a[j] >= t_a[i] + tau_a[i, j] - M * (1 - gp.quicksum(y[i, j, r] for r in range(num_drone_trip))))

    for j in cC1:
        model.addConstr(t_a[j] >= tau_a[0, j] - M * (
                1 - y[0, j, 0]))
        for r in range(1, num_drone_trip):
            model.addConstr(t_a[j] >= A[r - 1] + tau_a[0, j] - M * (
                    1 - y[0, j, r]))

    model.addConstr(t[0] == 0)

    for i in N1:
        for j in cC:
            model.addConstr(T[j] >= t[i] + tau[i, j] - M * (1 - gp.quicksum(x[i, j, k] for k in range(num_staff))))

    for i in N1:
        for j in cC:
            model.addConstr(T[j] <= t[i] + tau[i, j] + M * (1 - gp.quicksum(x[i, j, k] for k in range(num_staff))))

    for i in cC:
        model.addConstr(
            gp.quicksum(g[i, j, k, r] for r in range(num_drone_trip) for k in range(num_staff) for j in cC) <= 1)

    for i in cC:
        for j in cC:
            model.addConstr(t[j] >= t[i] - M * (1 - gp.quicksum(g[i, j, k, r]
                                                                for r in range(num_drone_trip)
                                                                for k in range(num_staff))))

    for i in cC:
        for k in range(num_staff):
            model.addConstr(gp.quicksum(g[i, j, k, r] for r in range(num_drone_trip) for j in cC)
                            <= gp.quicksum(x[i, j, k] for j in N2))

    for k in range(num_staff):
        for r in range(num_drone_trip):
            for i in N1:
                for z in N1:
                    model.addConstr(gp.quicksum(f[i, j, k, r] for j in N2)
                                    >= g[z, i, k, r] - M * (1 - gp.quicksum(x[z, j, k] for j in N2)))

    for r in range(num_drone_trip):
        for j in cC:
            for k in range(num_staff):
                model.addConstr(
                    gp.quicksum(g[i, j, k, r] for i in cC) >= s[j, k] - M * (
                                1 - gp.quicksum(f[j, i, k, r] for i in N2)))

    for r in range(num_drone_trip):
        for j in cC:
            for k in range(num_staff):
                model.addConstr(
                    gp.quicksum(g[i, j, k, r] for i in cC) <= s[j, k] + M * (
                                1 - gp.quicksum(f[j, i, k, r] for i in N2)))

    for i in cC:
        for r in range(num_drone_trip):
            model.addConstr(C[i, r] <= M * gp.quicksum(g[i, j, k, r] for j in cC for k in range(num_staff)))

    for i in cC:
        for r in range(num_drone_trip):
            model.addConstr(
                C[i, r] >= A[r] - T[i] - M * (1 - gp.quicksum(g[i, j, k, r] for j in cC for k in range(num_staff))))

    for i in cC:
        for r in range(num_drone_trip):
            model.addConstr(
                C[i, r] <= A[r] - T[i] + M * (1 - gp.quicksum(g[i, j, k, r] for j in cC for k in range(num_staff))))

    for i in cC:
        for k in range(num_staff):
            model.addConstr(D[i, k] <= M * (1 - gp.quicksum(g[i, j, k, r] for j in cC for r in range(num_drone_trip))))

    for i in cC:
        for k in range(num_staff):
            model.addConstr(
                D[i, k] >= B[k] - T[i] - M * (
                        1 - gp.quicksum(x[i, j, k] for j in N2) + gp.quicksum(
                    g[i, j, k, r] for j in cC for r in range(num_drone_trip))))

    for i in cC:
        for k in range(num_staff):
            model.addConstr(
                D[i, k] <= B[k] - T[i] + M * (
                        1 - gp.quicksum(x[i, j, k] for j in N2) + gp.quicksum(
                    g[i, j, k, r] for j in cC for r in range(num_drone_trip))))

    for k in range(num_staff):
        model.addConstr(B_a[k] >= B[k] - M * (1 - u[k]))
        model.addConstr(B_a[k] <= B[k] + M * (1 - u[k]))

    for k in range(num_staff):
        model.addConstr(B_a[k] <= L_a)

    for i in cC:
        model.addConstr(t_a[i] + tau_a[i, num_cus + 1] <= L_a + M * (
                1 - gp.quicksum(y[i, num_cus + 1, r] for r in range(num_drone_trip))))

    for k in range(num_staff):
        model.addConstr(u[k] <= s[num_cus + 1, k])

    for k in range(num_staff):
        model.addConstr(num_cus * u[k] >= s[num_cus + 1, k])

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
