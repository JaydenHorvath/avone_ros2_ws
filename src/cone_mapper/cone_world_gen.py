#!/usr/bin/env python3
import csv

# ---------------- CONFIG ----------------
CSV_FILE = "cones_log.csv"          # input CSV file
OUTPUT_FILE = "cones_generated.sdf" # output SDF fragment
CONE_URI = "model://blue_cone"      # Gazebo model URI
Z_OFFSET = 0.15                     # cone height above ground (m)
# ----------------------------------------

def main():
    with open(CSV_FILE, newline='') as f:
        reader = csv.DictReader(f)
        cones = [row for row in reader]

    with open(OUTPUT_FILE, "w") as f:
        f.write('<?xml version="1.0"?>\n')
        f.write('<sdf version="1.6">\n')
        f.write('  <world name="cones_from_csv">\n\n')

        for i, cone in enumerate(cones):
            try:
                x = float(cone['x'])
                y = float(cone['y'])
                z = float(cone['z']) if 'z' in cone and cone['z'] else 0.0
            except (KeyError, ValueError):
                continue

            name = f"blue_cone_{i}"

            f.write(f'    <include>\n')
            f.write(f'      <pose>{x:.4f} {y:.4f} {Z_OFFSET:.2f} 0 0 0</pose>\n')
            f.write(f'      <uri>{CONE_URI}</uri>\n')
            f.write(f'      <name>{name}</name>\n')
            f.write(f'    </include>\n\n')

        f.write('  </world>\n')
        f.write('</sdf>\n')

    print(f"✅ Generated {len(cones)} cone includes → {OUTPUT_FILE}")

if __name__ == "__main__":
    main()
