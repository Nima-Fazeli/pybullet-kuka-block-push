import KukaBlockClass
import matplotlib.pyplot as plt
import pdb


def main():
    # choose to use visualization - don't if collecting large amounts of data
    withVis = False
    kb = KukaBlockClass.KukaBlock(withVis)

    # number of simulations to run, use 1 with visualization
    simRuns = 5

    # push direction with respect to the positive y axis
    # used only if random init is off
    theta = 0.00
    withRand = True

    # plotting utility
    axis_name = ['x robot (m)', 'y robot (m)', 'x block (m)', 'y block (m)', 'block rotation (rads)']
    fig, axes = plt.subplots(1, 5, figsize=(18, 5))

    # simulate and plot trajectories
    for simCount in range(simRuns):
        # simulation resets everytime it's called
        traj = kb.simulate(theta, withRandom=withRand)

        # plotting
        for i in range(5):
            axes[i].plot(traj[:, i])

    # add some labels
    for i in range(5):
        axes[i].set_xlabel(axis_name[i])

    plt.show()

if __name__ == "__main__":
    main()
