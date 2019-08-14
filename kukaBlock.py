import KukaBlockClass


def main():
    kb = KukaBlockClass.KukaBlock()

    # push direction with respect to the positive y axis
    theta = 0.0
    traj = kb.simulate(theta)

if __name__ == "__main__":
    main()
