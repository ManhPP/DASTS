from ortools.linear_solver import pywraplp
from src.util import post_process


def solve_by_ortools(config, inp):
    # solver = pywraplp.Solver('ip',
    #                          pywraplp.Solver.GUROBI_MIXED_INTEGER_PROGRAMMING)
    solver = pywraplp.Solver.CreateSolver(config.solver.solver)

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
                x[i, j, k] = solver.BoolVar(f"x[{i},{j},{k}]")

    for r in range(num_drone_trip):
        for j in N:
            for i in N:
                y[i, j, r] = solver.BoolVar(f"y[{i},{j},{r}]")

    for k in range(num_staff):
        for r in range(num_drone_trip):
            for j in N:
                for i in N:
                    f[i, j, k, r] = solver.BoolVar(f"f[{i},{j},{k},{r}]")
                    g[i, j, k, r] = solver.BoolVar(f"g[{i},{j},{k},{r}]")

    for r in range(num_drone_trip):
        for i in N:
            v[i, r] = solver.NumVar(0, L, f"v[{i}, {r}]")
            C[i, r] = solver.NumVar(0, L_a, f"C[{i}, {r}]")

    for k in range(num_staff):
        for i in N:
            s[i, k] = solver.IntVar(0, int(num_cus), f"s[{i}, {k}]")
            D[i, k] = solver.NumVar(0, L_a, f"D[{i}, {k}]")

    for i in N:
        t[i] = solver.NumVar(0, L_a, f"t[{i}]")
        t_a[i] = solver.NumVar(0, L_a, f"t_a[{i}]")
        T[i] = solver.NumVar(0, L_a, f"T[{i}]")

    for r in range(num_drone_trip):
        A[r] = solver.NumVar(0, L_a, f"A[{r}]")

    for k in range(num_staff):
        B[k] = solver.NumVar(0, L_a, f"B[{k}]")
        B_a[k] = solver.NumVar(0, L_a, f"B_a[{k}]")
        u[k] = solver.BoolVar(f"u[{k}]")

    # obj
    solver.Minimize(solver.Sum(C[i, r] for i in cC for r in range(num_drone_trip)) + solver.Sum(
        D[i, k] for i in cC for k in range(num_staff)))

    # constraint
    for i in cC1:
        for r in range(num_drone_trip):
            solver.Add(A[r] >= t_a[i] + tau_a[i, num_cus + 1] + M * (y[i, num_cus + 1, r] - 1))
            solver.Add(A[r] <= t_a[i] + tau_a[i, num_cus + 1] + M * (1 - y[i, num_cus + 1, r]))

    for i in N1:
        for k in range(num_staff):
            solver.Add(B[k] >= t[i] + tau[i, num_cus + 1] + M * (x[i, num_cus + 1, k] - 1))
            solver.Add(B[k] <= t[i] + tau[i, num_cus + 1] + M * (1 - x[i, num_cus + 1, k]))

    solver.Add(solver.Sum(x[i, 0, k] for i in N2 for k in range(num_staff)) == 0)

    solver.Add(solver.Sum(x[num_cus + 1, j, k] for j in N1 for k in range(num_staff)) == 0)

    for k in range(num_staff):
        solver.Add(solver.Sum(x[0, j, k] for j in N2)
                   == solver.Sum(x[i, num_cus + 1, k] for i in N1))

    solver.Add(solver.Sum(y[i, 0, r] for i in N2 for r in range(num_drone_trip)) == 0)

    solver.Add(solver.Sum(y[num_cus + 1, j, r] for j in N1 for r in range(num_drone_trip)) == 0)

    for r in range(num_drone_trip):
        solver.Add(solver.Sum(y[0, j, r] for j in cC1) <= 1)

        solver.Add(solver.Sum(y[0, j, r] for j in cC1)
                   == solver.Sum(y[i, num_cus + 1, r] for i in cC1))

    for i in cC:
        for k in range(num_staff):
            solver.Add(solver.Sum(x[i, j, k] for j in N2)
                       == solver.Sum(x[j, i, k] for j in N1))

    for i in cC:
        solver.Add(solver.Sum(x[i, j, k] for j in N2 for k in range(num_staff) if j != i) == 1)
        solver.Add(solver.Sum(x[i, j, k] for j in N2 for k in range(num_staff) if j == i) == 0)

    for r in range(num_drone_trip):
        for i in N:
            solver.Add(v[i, r] <= L)

    for i in cC1:
        for j in cC1:
            for r in range(num_drone_trip):
                solver.Add(v[j, r] >= v[i, r] + (t_a[j] - t_a[i]) + M * (y[i, j, r] - 1))
                solver.Add(v[j, r] <= v[i, r] + (t_a[j] - t_a[i]) + M * (1 - y[i, j, r]))

    for j in cC1:
        for r in range(num_drone_trip):
            solver.Add(v[j, r] >= tau_a[0, j] + M * (y[0, j, r] - 1))
            solver.Add(v[j, r] <= tau_a[0, j] + M * (1 - y[0, j, r]))

    # solver.Add(v[num_cus + 1, 0] == A[0])
    for r in range(1, num_drone_trip):
        # solver.Add(v[num_cus + 1, r] == A[r] - A[r-1])
        for j in cC1:
            solver.Add(v[num_cus + 1, r] <= v[j, r] + tau_a[j, num_cus + 1] + M * (1 - y[j, num_cus + 1, r]))

    # for i in cC:
    #     for j in cC:
    #         solver.Add(solver.Sum(y[i, j, r] for r in range(num_drone_trip))
    #                    + solver.Sum(x[i, j, k] for k in range(num_staff))
    #                    <= 1)

    for r in range(num_drone_trip - 1):
        solver.Add(solver.Sum(y[0, j, r] for j in cC1)
                   >= solver.Sum(y[0, j, r + 1] for j in cC1))

    for i in cC1:
        for r in range(num_drone_trip):
            solver.Add(solver.Sum(y[j, i, r] for j in cC11 if j != i)
                       == solver.Sum(y[i, j, r] for j in cC12 if j != i))

    for i in cC1:
        solver.Add(
            solver.Sum(y[i, j, r] for r in range(num_drone_trip) for j in cC12 if i != j) <= 1)

        solver.Add(
            solver.Sum(y[i, j, r] for r in range(num_drone_trip) for j in cC if i == j) == 0)

    for k in range(num_staff):
        for r in range(num_drone_trip):
            # solver.Add(solver.Sum(f[i, j, k, r] for i in cC1 for j in N2) <= 1)
            solver.Add(solver.Sum(f[i, j, k, r] for i in N1 for j in N2 if i not in cC1) == 0)

    for j in cC:
        for k in range(num_staff):
            for i in N1:
                solver.Add(s[j, k] >= s[i, k] + 1 - M * (
                        1 - x[i, j, k] + solver.Sum(f[i, j, k, r] for r in range(num_drone_trip))))
                solver.Add(s[j, k] <= s[i, k] + 1 + M * (
                        1 - x[i, j, k] + solver.Sum(f[i, j, k, r] for r in range(num_drone_trip))))

    for k in range(num_staff):
        for i in N1:
            solver.Add(s[num_cus + 1, k]
                       >= s[i, k] - M * (1 - x[i, num_cus + 1, k]
                                         + solver.Sum(f[i, num_cus + 1, k, r]
                                                      for r in range(num_drone_trip)
                                                      for i in N1)))
            solver.Add(s[num_cus + 1, k]
                       <= s[i, k] + M * (1 - x[i, num_cus + 1, k]
                                         + solver.Sum(f[i, num_cus + 1, k, r]
                                                      for r in range(num_drone_trip)
                                                      for i in N1)))

    for j in cC:
        for k in range(num_staff):
            solver.Add(s[j, k] >= 1 - M * (1 - solver.Sum(f[i, j, k, r] for r in range(num_drone_trip) for i in cC1)))
            solver.Add(s[j, k] <= 1 + M * (1 - solver.Sum(f[i, j, k, r] for r in range(num_drone_trip) for i in cC1)))
            solver.Add(s[j, k] <= M * solver.Sum(x[j, i, k] for i in N2))

    for k in range(num_staff):
        solver.Add(s[0, k] == 0)
        solver.Add(
            s[num_cus + 1, k] <= M * (
                    1 - solver.Sum(f[i, num_cus + 1, k, r] for r in range(num_drone_trip) for i in cC1)))

    for i in cC1:
        for r in range(num_drone_trip):
            solver.Add(
                solver.Sum(f[i, j, k, r] for j in N2 for k in range(num_staff)) == solver.Sum(y[z, i, r] for z in cC11))

    for i in cC1:
        for j in N2:
            for k in range(num_staff):
                for r in range(num_drone_trip):
                    solver.Add(f[i, j, k, r] <= x[i, j, k])

    for i in cC1:
        solver.Add(t_a[i] >= t[i] - M * (
                1 - solver.Sum(f[i, j, k, r] for k in range(num_staff) for r in range(num_drone_trip) for j in N2)))

    for i in cC1:
        solver.Add(t_a[i] <= t[i] + M * (
                1 - solver.Sum(f[i, j, k, r] for k in range(num_staff) for r in range(num_drone_trip) for j in N2)))

    for i in cC:
        for z in N1:
            solver.Add(t[i] >= t[z] + tau[z, i] - M * (1 - solver.Sum(x[z, i, k] for k in range(num_staff))))

    for i in cC:
        for z in N1:
            solver.Add(
                t[i] <= t[z] + tau[z, i] + M * (1 - solver.Sum(x[z, i, k] for k in range(num_staff)) + solver.Sum(
                    f[i, j, k, r] for k in range(num_staff) for r in range(num_drone_trip) for j in N2)))

    for i in cC:
        for j in cC:
            for r in range(num_drone_trip - 1):
                solver.Add(t[j] >= t[i] - M * (2 - y[0, i, r] - y[0, j, r + 1]))

    for j in cC1:
        for i in cC1:
            solver.Add(t_a[j] >= t_a[i] + tau_a[i, j] - M * (1 - solver.Sum(y[i, j, r] for r in range(num_drone_trip))))

    for j in cC1:
        solver.Add(t_a[j] >= tau_a[0, j] - M * (
                1 - y[0, j, 0]))
        for r in range(1, num_drone_trip):
            solver.Add(t_a[j] >= A[r-1] + tau_a[0, j] - M * (
                    1 - y[0, j, r]))

    solver.Add(t[0] == 0)

    for i in N1:
        for j in cC:
            solver.Add(T[j] >= t[i] + tau[i, j] - M * (1 - solver.Sum(x[i, j, k] for k in range(num_staff))))

    for i in N1:
        for j in cC:
            solver.Add(T[j] <= t[i] + tau[i, j] + M * (1 - solver.Sum(x[i, j, k] for k in range(num_staff))))

    for i in cC:
        solver.Add(solver.Sum(g[i, j, k, r] for r in range(num_drone_trip) for k in range(num_staff) for j in cC) <= 1)

    for i in cC:
        for j in cC:
            solver.Add(t[j] >= t[i] - M * (1 - solver.Sum(g[i, j, k, r]
                                                          for r in range(num_drone_trip)
                                                          for k in range(num_staff))))

    for i in cC:
        for k in range(num_staff):
            solver.Add(solver.Sum(g[i, j, k, r] for r in range(num_drone_trip) for j in cC)
                       <= solver.Sum(x[i, j, k] for j in N2))

    for k in range(num_staff):
        for r in range(num_drone_trip):
            for i in N1:
                for z in N1:
                    solver.Add(solver.Sum(f[i, j, k, r] for j in N2)
                               >= g[z, i, k, r] - M * (1 - solver.Sum(x[z, j, k] for j in N2)))

    for r in range(num_drone_trip):
        for j in cC:
            for k in range(num_staff):
                solver.Add(
                    solver.Sum(g[i, j, k, r] for i in cC) >= s[j, k] - M * (1 - solver.Sum(f[j, i, k, r] for i in N2)))

    for r in range(num_drone_trip):
        for j in cC:
            for k in range(num_staff):
                solver.Add(
                    solver.Sum(g[i, j, k, r] for i in cC) <= s[j, k] + M * (1 - solver.Sum(f[j, i, k, r] for i in N2)))

    for i in cC:
        for r in range(num_drone_trip):
            solver.Add(C[i, r] <= M * solver.Sum(g[i, j, k, r] for j in cC for k in range(num_staff)))

    for i in cC:
        for r in range(num_drone_trip):
            solver.Add(
                C[i, r] >= A[r] - T[i] - M * (1 - solver.Sum(g[i, j, k, r] for j in cC for k in range(num_staff))))

    for i in cC:
        for r in range(num_drone_trip):
            solver.Add(
                C[i, r] <= A[r] - T[i] + M * (1 - solver.Sum(g[i, j, k, r] for j in cC for k in range(num_staff))))

    for i in cC:
        for k in range(num_staff):
            solver.Add(D[i, k] <= M * (1 - solver.Sum(g[i, j, k, r] for j in cC for r in range(num_drone_trip))))

    for i in cC:
        for k in range(num_staff):
            solver.Add(
                D[i, k] >= B[k] - T[i] - M * (
                        1 - solver.Sum(x[i, j, k] for j in N2) + solver.Sum(
                    g[i, j, k, r] for j in cC for r in range(num_drone_trip))))

    for i in cC:
        for k in range(num_staff):
            solver.Add(
                D[i, k] <= B[k] - T[i] + M * (
                        1 - solver.Sum(x[i, j, k] for j in N2) + solver.Sum(
                    g[i, j, k, r] for j in cC for r in range(num_drone_trip))))

    for k in range(num_staff):
        solver.Add(B_a[k] >= B[k] - M * (1 - u[k]))
        solver.Add(B_a[k] <= B[k] + M * (1 - u[k]))

    for k in range(num_staff):
        solver.Add(B_a[k] <= L_a)

    for i in cC:
        solver.Add(t_a[i] + tau_a[i, num_cus + 1] <= L_a + M * (
                1 - solver.Sum(y[i, num_cus + 1, r] for r in range(num_drone_trip))))

    for k in range(num_staff):
        solver.Add(u[k] <= s[num_cus + 1, k])

    for k in range(num_staff):
        solver.Add(num_cus * u[k] >= s[num_cus + 1, k])

    if config.solver.time_limit > 0:
        solver.set_time_limit(config.solver.time_limit*1000)
    if config.solver.num_worker > 0:
        solver.SetNumThreads(config.solver.num_worker)

    print('Number of variables = %d' % solver.NumVariables())
    print('Number of constraints =', solver.NumConstraints())
    result_status = solver.Solve()
    print(result_status)

    post_process(solver, result_status, inp, config,
                 x, y, f, g, v, s, t, t_a, T, A, B, C, D, B_a, u)
