import csv, math

# --- CONFIG ---
origin_lat = -33.807231472666665
origin_lon = 150.86992797166667
input_csv  = "cones_log.csv"
output_kml = "cones.kml"

# --- CONVERT local X/Y (metres) → lat/lon ---
def xy_to_latlon(x, y, lat0, lon0):
    dlat = y / 111320.0
    dlon = x / (111320.0 * math.cos(math.radians(lat0)))
    return lat0 + dlat, lon0 + dlon

# --- LOAD CSV and filter duplicates ---
cones = {}
with open(input_csv) as f:
    reader = csv.DictReader(f)
    for r in reader:
        x, y, z = float(r["x"]), float(r["y"]), float(r["z"])
        key = (round(x, 3), round(y, 3))  # group by approx XY
        if key not in cones or z < cones[key]["z"]:  # keep lowest Z
            cones[key] = {"x": x, "y": y, "z": z}

# --- WRITE KML ---
kml = ['<?xml version="1.0" encoding="UTF-8"?>',
       '<kml xmlns="http://www.opengis.net/kml/2.2">',
       '  <Document>',
       '    <name>AVONE Cones (Filtered)</name>']

for i, (k, cone) in enumerate(cones.items()):
    lat, lon = xy_to_latlon(cone["x"], cone["y"], origin_lat, origin_lon)
    kml.append(f'''    <Placemark>
      <name>Cone {i}</name>
      <Point><coordinates>{lon},{lat},0</coordinates></Point>
    </Placemark>''')

kml.append('  </Document></kml>')

with open(output_kml, "w") as f:
    f.write("\n".join(kml))

print(f"✅ Wrote {len(cones)} unique cones to {output_kml}")
