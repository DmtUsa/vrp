from vrp.ilp import ILP
import time

if __name__ == "__main__":

    # Solve the ILP formulation
    ilp = ILP(random_seed=42)
    start_time = time.time()
    ilp.solve()
    total_time = round(time.time() - start_time)

    pass
