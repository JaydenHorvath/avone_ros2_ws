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
                color = row.get('colour', 'unknown').strip().lower()
                cones.append((x, y, color))
            except ValueError:
                continue
    return cones


def main():
    csv_path = 'cones_log.csv'  # adjust if needed
    cones = load_cones(csv_path)

    if not cones:
        print(f"No cone data found in {csv_path}")
        return

    # Separate cones by colour
    color_map = {'orange': [], 'blue': [], 'yellow': [], 'unknown': []}

    for x, y, c in cones:
        if c not in color_map:
            c = 'unknown'
        color_map[c].append((x, y))

    plt.figure(figsize=(8, 8))

    # Plot each colour set with distinct styling
    if color_map['orange']:
        xs, ys = zip(*color_map['orange'])
        plt.scatter(xs, ys, c='orange', edgecolors='k', s=60, label='Orange Cones')

    if color_map['blue']:
        xs, ys = zip(*color_map['blue'])
        plt.scatter(xs, ys, c='blue', edgecolors='k', s=60, label='Blue Cones')

    if color_map['yellow']:
        xs, ys = zip(*color_map['yellow'])
        plt.scatter(xs, ys, c='yellow', edgecolors='k', s=60, label='Yellow Cones')

    if color_map['unknown']:
        xs, ys = zip(*color_map['unknown'])
        plt.scatter(xs, ys, c='gray', edgecolors='k', s=60, label='Unknown Cones')

    plt.xlabel("X position [m]")
    plt.ylabel("Y position [m]")
    plt.title("Cone Map Visualisation (with Colour)")
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == '__main__':
    main()

