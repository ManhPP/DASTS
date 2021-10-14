from docplex.mp.model import Model

from src.util import post_process


def solve_by_cplex(config, inp):
    model = Model("DASTS-CPLEX")

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
                x[i, j, k] = model.binary_var(f"x[{i},{j},{k}]")

    for r in range(num_drone_trip):
        for j in N:
            for i in N:
                y[i, j, r] = model.binary_var(f"y[{i},{j},{r}]")

    for k in range(num_staff):
        for r in range(num_drone_trip):
            for j in N:
                for i in N:
                    f[i, j, k, r] = model.binary_var(f"f[{i},{j},{k},{r}]")
                    g[i, j, k, r] = model.binary_var(f"g[{i},{j},{k},{r}]")

    for r in range(num_drone_trip):
        for i in N:
            v[i, r] = model.continuous_var(0, L, f"v[{i}, {r}]")
            C[i, r] = model.continuous_var(0, L_a, f"C[{i}, {r}]")

    for k in range(num_staff):
        for i in N:
            s[i, k] = model.integer_var(0, int(num_cus), f"s[{i}, {k}]")
            D[i, k] = model.continuous_var(0, L_a, f"D[{i}, {k}]")

    for i in N:
        t[i] = model.continuous_var(0, L_a, f"t[{i}]")
        t_a[i] = model.continuous_var(0, L_a, f"t_a[{i}]")
        T[i] = model.continuous_var(0, L_a, f"T[{i}]")

    for r in range(num_drone_trip):
        A[r] = model.continuous_var(0, L_a, f"A[{r}]")

    for k in range(num_staff):
        B[k] = model.continuous_var(0, L_a, f"B[{k}]")
        B_a[k] = model.continuous_var(0, L_a, f"B_a[{k}]")
        u[k] = model.binary_var(f"u[{k}]")

    # obj
    model.minimize(model.sum(C[i, r] for i in cC for r in range(num_drone_trip)) + model.sum(
        D[i, k] for i in cC for k in range(num_staff)))

    # constraint
    for i in cC1:
        for r in range(num_drone_trip):
            model.add_constraint(A[r] >= t_a[i] + tau_a[i, num_cus + 1] + M * (y[i, num_cus + 1, r] - 1))
            model.add_constraint(A[r] <= t_a[i] + tau_a[i, num_cus + 1] + M * (1 - y[i, num_cus + 1, r]))

    for i in N1:
        for k in range(num_staff):
            model.add_constraint(B[k] >= t[i] + tau[i, num_cus + 1] + M * (x[i, num_cus + 1, k] - 1))
            model.add_constraint(B[k] <= t[i] + tau[i, num_cus + 1] + M * (1 - x[i, num_cus + 1, k]))

    model.add_constraint(model.sum(x[i, 0, k] for i in N2 for k in range(num_staff)) == 0)

    model.add_constraint(model.sum(x[num_cus + 1, j, k] for j in N1 for k in range(num_staff)) == 0)

    for k in range(num_staff):
        model.add_constraint(model.sum(x[0, j, k] for j in N2)
                             == model.sum(x[i, num_cus + 1, k] for i in N1))

    model.add_constraint(model.sum(y[i, 0, r] for i in N2 for r in range(num_drone_trip)) == 0)

    model.add_constraint(model.sum(y[num_cus + 1, j, r] for j in N1 for r in range(num_drone_trip)) == 0)

    for r in range(num_drone_trip):
        model.add_constraint(model.sum(y[0, j, r] for j in cC1) <= 1)

        model.add_constraint(model.sum(y[0, j, r] for j in cC1)
                             == model.sum(y[i, num_cus + 1, r] for i in cC1))

    for i in cC:
        for k in range(num_staff):
            model.add_constraint(model.sum(x[i, j, k] for j in N2)
                                 == model.sum(x[j, i, k] for j in N1))

    for i in cC:
        model.add_constraint(model.sum(x[i, j, k] for j in N2 for k in range(num_staff) if j != i) == 1)
        model.add_constraint(model.sum(x[i, j, k] for j in N2 for k in range(num_staff) if j == i) == 0)

    for r in range(num_drone_trip):
        for i in N:
            model.add_constraint(v[i, r] <= L)

    for i in cC1:
        for j in cC1:
            for r in range(num_drone_trip):
                model.add_constraint(v[j, r] >= v[i, r] + (t_a[j] - t_a[i]) + M * (y[i, j, r] - 1))
                model.add_constraint(v[j, r] <= v[i, r] + (t_a[j] - t_a[i]) + M * (1 - y[i, j, r]))

    for j in cC1:
        for r in range(num_drone_trip):
            model.add_constraint(v[j, r] >= tau_a[0, j] + M * (y[0, j, r] - 1))
            model.add_constraint(v[j, r] <= tau_a[0, j] + M * (1 - y[0, j, r]))

    # model.add_constraint(v[num_cus + 1, 0] == A[0])
    for r in range(1, num_drone_trip):
        # model.add_constraint(v[num_cus + 1, r] == A[r] - A[r - 1])
        for j in cC1:
            model.add_constraint(v[num_cus + 1, r] <= v[j, r] + tau_a[j, num_cus + 1] + M * (1 - y[j, num_cus + 1, r]))

    # for i in cC:
    #     for j in cC:
    #         model.add_constraint(model.sum(y[i, j, r] for r in range(num_drone_trip))
    #                    + model.sum(x[i, j, k] for k in range(num_staff))
    #                    <= 1)

    for r in range(num_drone_trip - 1):
        model.add_constraint(model.sum(y[0, j, r] for j in cC1)
                             >= model.sum(y[0, j, r + 1] for j in cC1))

    for i in cC1:
        for r in range(num_drone_trip):
            model.add_constraint(model.sum(y[j, i, r] for j in cC11 if j != i)
                                 == model.sum(y[i, j, r] for j in cC12 if j != i))

    for i in cC1:
        model.add_constraint(
            model.sum(y[i, j, r] for r in range(num_drone_trip) for j in cC12 if i != j) <= 1)

        model.add_constraint(
            model.sum(y[i, j, r] for r in range(num_drone_trip) for j in cC if i == j) == 0)

    for k in range(num_staff):
        for r in range(num_drone_trip):
            # model.add_constraint(model.sum(f[i, j, k, r] for i in cC1 for j in N2) <= 1)
            model.add_constraint(model.sum(f[i, j, k, r] for i in N1 for j in N2 if i not in cC1) == 0)

    for j in cC:
        for k in range(num_staff):
            for i in N1:
                model.add_constraint(s[j, k] >= s[i, k] + 1 - M * (
                        1 - x[i, j, k] + model.sum(f[i, j, k, r] for r in range(num_drone_trip))))
                model.add_constraint(s[j, k] <= s[i, k] + 1 + M * (
                        1 - x[i, j, k] + model.sum(f[i, j, k, r] for r in range(num_drone_trip))))

    for k in range(num_staff):
        for i in N1:
            model.add_constraint(s[num_cus + 1, k]
                                 >= s[i, k] - M * (1 - x[i, num_cus + 1, k]
                                                   + model.sum(f[i, num_cus + 1, k, r]
                                                               for r in range(num_drone_trip)
                                                               for i in N1)))
            model.add_constraint(s[num_cus + 1, k]
                                 <= s[i, k] + M * (1 - x[i, num_cus + 1, k]
                                                   + model.sum(f[i, num_cus + 1, k, r]
                                                               for r in range(num_drone_trip)
                                                               for i in N1)))

    for j in cC:
        for k in range(num_staff):
            model.add_constraint(
                s[j, k] >= 1 - M * (1 - model.sum(f[i, j, k, r] for r in range(num_drone_trip) for i in cC1)))
            model.add_constraint(
                s[j, k] <= 1 + M * (1 - model.sum(f[i, j, k, r] for r in range(num_drone_trip) for i in cC1)))
            model.add_constraint(s[j, k] <= M * model.sum(x[j, i, k] for i in N2))

    for k in range(num_staff):
        model.add_constraint(s[0, k] == 0)
        model.add_constraint(
            s[num_cus + 1, k] <= M * (
                    1 - model.sum(f[i, num_cus + 1, k, r] for r in range(num_drone_trip) for i in cC1)))

    for i in cC1:
        for r in range(num_drone_trip):
            model.add_constraint(
                model.sum(f[i, j, k, r] for j in N2 for k in range(num_staff)) == model.sum(y[z, i, r] for z in cC11))

    for i in cC1:
        for j in N2:
            for k in range(num_staff):
                for r in range(num_drone_trip):
                    model.add_constraint(f[i, j, k, r] <= x[i, j, k])

    for i in cC1:
        model.add_constraint(t_a[i] >= t[i] - M * (
                1 - model.sum(f[i, j, k, r] for k in range(num_staff) for r in range(num_drone_trip) for j in N2)))

    for i in cC1:
        model.add_constraint(t_a[i] <= t[i] + M * (
                1 - model.sum(f[i, j, k, r] for k in range(num_staff) for r in range(num_drone_trip) for j in N2)))

    for i in cC:
        for z in N1:
            model.add_constraint(t[i] >= t[z] + tau[z, i] - M * (1 - model.sum(x[z, i, k] for k in range(num_staff))))

    for i in cC:
        for z in N1:
            model.add_constraint(
                t[i] <= t[z] + tau[z, i] + M * (1 - model.sum(x[z, i, k] for k in range(num_staff)) + model.sum(
                    f[i, j, k, r] for k in range(num_staff) for r in range(num_drone_trip) for j in N2)))

    for i in cC:
        for j in cC:
            for r in range(num_drone_trip - 1):
                model.add_constraint(t[j] >= t[i] - M * (2 - y[0, i, r] - y[0, j, r + 1]))

    for j in cC1:
        for i in cC1:
            model.add_constraint(
                t_a[j] >= t_a[i] + tau_a[i, j] - M * (1 - model.sum(y[i, j, r] for r in range(num_drone_trip))))

    for j in cC1:
        model.add_constraint(t_a[j] >= tau_a[0, j] - M * (
                1 - y[0, j, 0]))
        for r in range(1, num_drone_trip):
            model.add_constraint(t_a[j] >= A[r - 1] + tau_a[0, j] - M * (
                    1 - y[0, j, r]))

    model.add_constraint(t[0] == 0)

    for i in N1:
        for j in cC:
            model.add_constraint(T[j] >= t[i] + tau[i, j] - M * (1 - model.sum(x[i, j, k] for k in range(num_staff))))

    for i in N1:
        for j in cC:
            model.add_constraint(T[j] <= t[i] + tau[i, j] + M * (1 - model.sum(x[i, j, k] for k in range(num_staff))))

    for i in cC:
        model.add_constraint(
            model.sum(g[i, j, k, r] for r in range(num_drone_trip) for k in range(num_staff) for j in cC) <= 1)

    for i in cC:
        for j in cC:
            model.add_constraint(t[j] >= t[i] - M * (1 - model.sum(g[i, j, k, r]
                                                                   for r in range(num_drone_trip)
                                                                   for k in range(num_staff))))

    for i in cC:
        for k in range(num_staff):
            model.add_constraint(model.sum(g[i, j, k, r] for r in range(num_drone_trip) for j in cC)
                                 <= model.sum(x[i, j, k] for j in N2))

    for k in range(num_staff):
        for r in range(num_drone_trip):
            for i in N1:
                for z in N1:
                    model.add_constraint(model.sum(f[i, j, k, r] for j in N2)
                                         >= g[z, i, k, r] - M * (1 - model.sum(x[z, j, k] for j in N2)))

    for r in range(num_drone_trip):
        for j in cC:
            for k in range(num_staff):
                model.add_constraint(
                    model.sum(g[i, j, k, r] for i in cC) >= s[j, k] - M * (1 - model.sum(f[j, i, k, r] for i in N2)))

    for r in range(num_drone_trip):
        for j in cC:
            for k in range(num_staff):
                model.add_constraint(
                    model.sum(g[i, j, k, r] for i in cC) <= s[j, k] + M * (1 - model.sum(f[j, i, k, r] for i in N2)))

    for i in cC:
        for r in range(num_drone_trip):
            model.add_constraint(C[i, r] <= M * model.sum(g[i, j, k, r] for j in cC for k in range(num_staff)))

    for i in cC:
        for r in range(num_drone_trip):
            model.add_constraint(
                C[i, r] >= A[r] - T[i] - M * (1 - model.sum(g[i, j, k, r] for j in cC for k in range(num_staff))))

    for i in cC:
        for r in range(num_drone_trip):
            model.add_constraint(
                C[i, r] <= A[r] - T[i] + M * (1 - model.sum(g[i, j, k, r] for j in cC for k in range(num_staff))))

    for i in cC:
        for k in range(num_staff):
            model.add_constraint(
                D[i, k] <= M * (1 - model.sum(g[i, j, k, r] for j in cC for r in range(num_drone_trip))))

    for i in cC:
        for k in range(num_staff):
            model.add_constraint(
                D[i, k] >= B[k] - T[i] - M * (
                        1 - model.sum(x[i, j, k] for j in N2) + model.sum(
                    g[i, j, k, r] for j in cC for r in range(num_drone_trip))))

    for i in cC:
        for k in range(num_staff):
            model.add_constraint(
                D[i, k] <= B[k] - T[i] + M * (
                        1 - model.sum(x[i, j, k] for j in N2) + model.sum(
                    g[i, j, k, r] for j in cC for r in range(num_drone_trip))))

    for k in range(num_staff):
        model.add_constraint(B_a[k] >= B[k] - M * (1 - u[k]))
        model.add_constraint(B_a[k] <= B[k] + M * (1 - u[k]))

    for k in range(num_staff):
        model.add_constraint(B_a[k] <= L_a)

    for i in cC:
        model.add_constraint(t_a[i] + tau_a[i, num_cus + 1] <= L_a + M * (
                1 - model.sum(y[i, num_cus + 1, r] for r in range(num_drone_trip))))

    for k in range(num_staff):
        model.add_constraint(u[k] <= s[num_cus + 1, k])

    for k in range(num_staff):
        model.add_constraint(num_cus * u[k] >= s[num_cus + 1, k])

    model.set_time_limit(config.solver.time_limit)

    try:
        special_params = config.solver.model_params.cplex
        for p_id, p_value in special_params.items():
            model.get_parameter_from_id(p_id).set(p_value)
    except Exception:
        print("Khong co config bo sung cho mo hinh")

    model.print_information()
    model.solve()

    # model.print_solution()
    # print(result_status)
    #
    post_process(model, model.get_solve_status(), inp, config, num_staff, num_drone_trip, N,
                 x, y, f, g, v, s, t, t_a, T, A, B, C, D, B_a, u)
