from typing import Dict, List

import numpy as np
import pandas as pd
import pulp
from scipy.spatial import distance_matrix
from matplotlib import pyplot as plt


class ILP:
    """
    Integer Linear Program for Vehicle Routing Problem
    """

    def __init__(
        self, M: int = 50, N: int = 10, max_demand: int = 10, capacity_slack: float = 0, random_seed: int = None
    ):
        self.M = M
        self.N = N
        # set random seed
        if random_seed is not None:
            np.random.seed(seed=random_seed)

        # generate M+N random locations on a unit square
        coordinates = np.random.uniform(size=2*(self.M+self.N))
        coordinates = np.vstack((coordinates[:(self.M+self.N)], coordinates[(self.M+self.N):])).T
        # compute distance matrix
        self.D = distance_matrix(coordinates, coordinates)
        # generate demand per customer at random
        self.d = np.random.choice(np.arange(1, max_demand+1), size=self.M)
        # calculate capacity given the capacity_slack level
        self.C = np.ceil(self.d.sum()*(1+capacity_slack)/self.N)
        self.solution = None

    def solve(self) -> None:

        return None
