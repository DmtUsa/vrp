from vrp.vrp_ilp import VRP

if __name__ == "__main__":

    # Create an instance of VRP
    vrp = VRP(M=10, N=3, max_demand=10, capacity_slack=2, random_seed=42)
    # Solve ILP
    vrp.solve()

    # Plot solution
    vrp.plot()

    pass
