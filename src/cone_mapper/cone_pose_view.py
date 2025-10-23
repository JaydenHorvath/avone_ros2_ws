#!/usr/bin/env python3
import csv
import matplotlib.pyplot as plt

def load_cones(csv_path):
    cones = []
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                x = float(row['x'])
                y = float(row['y'])
                cones.append((x, y))
            except ValueError:
                continue
    return cones


def main():
    csv_path = 'cones_log.csv'  # adjust if needed
    cones = load_cones(csv_path)

    if not cones:
        print(f"No cone data found in {csv_path}")
        return

    xs, ys = zip(*cones)

    plt.figure(figsize=(8, 8))
    plt.scatter(xs, ys, c='orange', edgecolors='k', s=60, label='Cones')

    plt.xlabel("X position [m]")
    plt.ylabel("Y position [m]")
    plt.title("Cone Map Visualisation")
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()
