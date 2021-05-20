from typing import Dict, List

import numpy as np
import pandas as pd
import pulp
from matplotlib import pyplot as plt
from scipy.spatial import distance_matrix


class ILP:
    """
    Integer Linear Program for Vehicle Routing Problem
    """

    def __init__(
        self,
        M: int = 50,
        N: int = 10,
        max_demand: int = 10,
        capacity_slack: float = 0,
        random_seed: int = None,
    ):
        self.M = M
        self.N = N
        # set random seed
        if random_seed is not None:
            np.random.seed(seed=random_seed)

        # generate M+N random locations on a unit square
        coordinates = np.random.uniform(size=2 * (self.M + self.N))
        coordinates = np.vstack(
            (coordinates[: (self.M + self.N)], coordinates[(self.M + self.N) :])
        ).T
        # compute distance matrix
        self.D = distance_matrix(coordinates, coordinates)
        # generate demand per customer at random
        self.d = np.random.choice(np.arange(1, max_demand + 1), size=self.M + self.N)
        self.d[M:] = 0
        # calculate capacity given the capacity_slack level
        self.C = np.ceil(self.d.sum() * (1 + capacity_slack) / self.N)
        self.solution = None

    def solve(self) -> None:

        opt_model = pulp.LpProblem(name="VRP")

        # set of all locations
        L = np.arange(self.M + self.N)
        # set of customers
        C = L[: self.M]
        # set of teams
        T = L[self.M :]

        # decision variables x_i_j_k
        # indicating if arc (i, j) is travelled by team k
        x = {
            (i, j, k): pulp.LpVariable(cat=pulp.LpBinary, name=f"x_{i}_{j}_{k}")
            for i in L
            for j in L
            for k in T
        }

        # decision variables u_i
        # indicating the rank of customer i in the route
        u = {
            i: pulp.LpVariable(cat=pulp.LpBinary, name=f"u_{i}")
            for i in C
        }

        # Each customer location should have an inflow arc
        constraints_1 = {
            j: opt_model.addConstraint(
                pulp.LpConstraint(
                    e=pulp.lpSum(x[i, j, k] for i in C for k in T),
                    sense=pulp.LpConstraintEQ,
                    rhs=1,
                    name=f"1_constraint_{j}",
                )
            )
            for j in C
        }

        # Each customer location should have an outflow arc
        constraints_2 = {
            i: opt_model.addConstraint(
                pulp.LpConstraint(
                    e=pulp.lpSum(x[i, j, k] for j in C for k in T),
                    sense=pulp.LpConstraintEQ,
                    rhs=1,
                    name=f"2_constraint_{i}",
                )
            )
            for i in C
        }

        # Control an inflow of the team's base
        constraints_3 = {
            (j, k): opt_model.addConstraint(
                pulp.LpConstraint(
                    e=pulp.lpSum(x[i, j, k] for i in L),
                    sense=pulp.LpConstraintLE,
                    rhs=1,
                    name=f"3_constraint_{j}_{k}",
                )
            )
            for j in T
            for k in [j]
        }

        # Forbid teams visiting other bases
        constraints_4 = {
            (j, k): opt_model.addConstraint(
                pulp.LpConstraint(
                    e=pulp.lpSum(x[i, j, k] for i in L),
                    sense=pulp.LpConstraintEQ,
                    rhs=0,
                    name=f"4_constraint_{j}_{k}",
                )
            )
            for j in T
            for k in [x for x in T if x != j]
        }

        # Control flow of teams
        constraints_5 = {
            (j, k): opt_model.addConstraint(
                pulp.LpConstraint(
                    e=pulp.lpSum(x[i, j, k] for i in L)
                    - pulp.lpSum(x[j, i, k] for i in L),
                    sense=pulp.LpConstraintEQ,
                    rhs=0,
                    name=f"5_constraint_{j}_{k}",
                )
            )
            for j in L
            for k in T
        }

        # No loop arcs
        constraints_6 = {
            (i, k): opt_model.addConstraint(
                pulp.LpConstraint(
                    e=x[i, i, k],
                    sense=pulp.LpConstraintEQ,
                    rhs=0,
                    name=f"6_constraint_{i}_{k}",
                )
            )
            for i in L
            for k in T
        }

        # Capacity constraints
        constraints_7 = {
            k: opt_model.addConstraint(
                pulp.LpConstraint(
                    e=pulp.lpSum(self.d[i] * x[i, j, k] for i in L for j in L),
                    sense=pulp.LpConstraintLE,
                    rhs=self.C,
                    name=f"7_constraint_{k}",
                )
            )
            for k in T
        }

        # Subtours elimination constraints
        constraints_7 = {
            (i, j, k): opt_model.addConstraint(
                pulp.LpConstraint(
                    e=u[i] - u[j] + self.M * x[i, k, k],
                    sense=pulp.LpConstraintLE,
                    rhs=self.M - 1,
                    name=f"8_constraint_{i}_{j}_{k}",
                )
            )
            for i in C
            for j in [x for x in C if x != i]
            for k in T
        }

        # add objective
        objective = pulp.lpSum(
            x[i, j, k] * self.D[i, j] for i in L for j in L for k in T
        )

        # set to maximize the objective
        opt_model.sense = pulp.LpMaximize
        opt_model.setObjective(objective)

        # solving with CBC solver
        pulp.LpSolverDefault.msg = 1
        opt_model.solve()

        return None

    def plot(self):

