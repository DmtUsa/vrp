from vrp.vrp_ilp import VRP

if __name__ == "__main__":

    # Create an instance of VRP
    vrp = VRP(M=5, N=2, max_demand=10, capacity_slack=0, random_seed=42)
    # Solve ILP
    vrp.solve_comodity_flow()

    # Plot solution
    vrp.plot()

    pass
