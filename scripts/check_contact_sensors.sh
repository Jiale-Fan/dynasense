#!/usr/bin/env bash
#
# End-to-end diagnosis of the ContactBPS Gazebo contact sensors.
#
# Run inside the container with the sim up:
#     rosrun dynasense check_contact_sensors.sh [LEG]
#
# The decisive output is section 2: a collision name that a sensor BINDS but
# that is not DECLARED on that sensor's link binds nothing, and the sensor's
# topic then publishes empty `states` forever at its update rate -- which is
# indistinguishable from "no contact happened".
#
# Two things this deliberately does NOT assume about `gz sdf -p` output:
#   * attribute quoting -- it writes name='x', so patterns like name="x" match
#     nothing at all and every check silently comes back empty;
#   * well-formedness -- some builds interleave Warning [...] lines into
#     stdout, so XML parsing can fail outright. There is a line-scan fallback.
set -uo pipefail

LEG=${1:-LF}
URDF=${URDF:-/tmp/contact_bps_check.urdf}
SDF_OUT=${SDF_OUT:-/tmp/contact_bps_check.sdf}
ERR_OUT=/tmp/contact_bps_check.err

echo "### 1. robot description -> $URDF"
python3 - "$URDF" <<'PY'
import sys
import rospy
for name in ("/anymal_description", "/robot_description"):
    try:
        txt = rospy.get_param(name)
    except Exception:
        continue
    with open(sys.argv[1], "w") as fh:
        fh.write(txt)
    print("    from %s, %d bytes" % (name, len(txt)))
    break
else:
    sys.exit("    FAILED: no description on the param server. Is the sim running?")
PY
[ -s "$URDF" ] || exit 1

gz sdf -p "$URDF" > "$SDF_OUT" 2>"$ERR_OUT"
if [ ! -s "$SDF_OUT" ]; then
  echo "    FAILED: gz sdf -p produced nothing. stderr:"
  sed 's/^/      /' "$ERR_OUT" | head -20
  exit 1
fi
echo "    converted -> $SDF_OUT ($(wc -c < "$SDF_OUT") bytes)"

python3 - "$SDF_OUT" "$LEG" <<'PY'
import re
import sys
import xml.etree.ElementTree as ET

sdf_path, leg = sys.argv[1], sys.argv[2]
text = open(sdf_path, "r", errors="replace").read()

lo, hi = text.find("<sdf"), text.rfind("</sdf>")
trimmed = text[lo:hi + len("</sdf>")] if lo != -1 and hi != -1 else text

declared = {}   # collision name -> link it is declared on
sensors = []    # (link, sensor name, [bound collision names])


def by_xml(doc):
    root = ET.fromstring(doc)
    for link in root.iter("link"):
        ln = link.get("name", "?")
        for col in link.findall("collision"):
            if col.get("name"):
                declared[col.get("name")] = ln
        for sensor in link.findall("sensor"):
            if sensor.get("type") != "contact":
                continue
            contact = sensor.find("contact")
            bound = []
            if contact is not None:
                bound = [(c.text or "").strip() for c in contact.findall("collision")]
            sensors.append((ln, sensor.get("name", "?"), bound))


def by_scan(doc):
    re_link = re.compile(r'''<link\s+name=["']([^"']+)["']''')
    re_col_decl = re.compile(r'''<collision\s+name=["']([^"']+)["']''')
    re_col_bind = re.compile(r'''<collision>\s*([^<]+?)\s*</collision>''')
    re_sensor = re.compile(r'''<sensor\s+[^>]*name=["']([^"']+)["']''')
    link, sensor, bound = None, None, []
    for line in doc.splitlines():
        m = re_link.search(line)
        if m:
            link = m.group(1)
            continue
        if "</link>" in line:
            link = None
            continue
        m = re_sensor.search(line)
        if m and "type=" in line and "contact" in line:
            sensor, bound = m.group(1), []
            continue
        if sensor is not None and "</sensor>" in line:
            sensors.append((link or "?", sensor, bound))
            sensor, bound = None, []
            continue
        m = re_col_bind.search(line)
        if m and sensor is not None:
            bound.append(m.group(1))
            continue
        m = re_col_decl.search(line)
        if m and link:
            declared[m.group(1)] = link


try:
    by_xml(trimmed)
    print("    parsed as XML")
except ET.ParseError as exc:
    print("    XML parse failed (%s); falling back to a line scan" % exc)
    line_no = getattr(exc, "position", (0, 0))[0]
    lines = trimmed.splitlines()
    for i in range(max(0, line_no - 3), min(len(lines), line_no + 2)):
        mark = ">>" if i == line_no - 1 else "  "
        print("      %s %5d | %s" % (mark, i + 1, lines[i][:150]))
    declared.clear()
    del sensors[:]
    by_scan(trimmed)

print("")
print("### 2. contact sensors that survived urdf2sdf lumping")
if not sensors:
    print("    (none at all -- no contact sensor reached the SDF)")
for link_name, sensor_name, bound in sorted(sensors):
    if leg not in sensor_name and leg not in link_name:
        continue
    print("    %s   on link %s" % (sensor_name, link_name))
    if not bound:
        print("        !! binds no collision element at all")
    for b in bound:
        where = declared.get(b)
        if where is None:
            print("        !! BINDS NOTHING: %s" % b)
        elif where != link_name:
            print("        !! WRONG LINK: %s declared on %s, sensor on %s" % (b, where, link_name))
        else:
            print("        ok  %s" % b)

print("")
print("### 3. collisions DECLARED on %s_SHANK / %s_THIGH" % (leg, leg))
rows = sorted((v, k) for k, v in declared.items() if v in ("%s_SHANK" % leg, "%s_THIGH" % leg))
if not rows:
    names = sorted(set(declared.values()))
    print("    (none found; links carrying declared collisions: %s)"
          % (", ".join(names[:12]) if names else "<none>"))
for link_name, name in rows:
    print("    %-12s %s" % (link_name, name))

print("")
print("### 4. every bound name that binds nothing, all legs")
bad = sorted(set(b for _, _, bs in sensors for b in bs if b not in declared))
if bad:
    for b in bad:
        print("    !! %s" % b)
else:
    print("    (none: every bound name matches a declared collision)")
PY

echo
echo "### 5. topics and subscribers"
for body in thigh shank foot knee_cylinder; do
  topic="/contacts/${LEG}/${body}"
  if info=$(rostopic info "$topic" 2>/dev/null) && [ -n "$info" ]; then
    subs=$(printf '%s\n' "$info" | sed -n '/Subscribers/,$p' | grep -c '^ \*')
    echo "    $topic  exists, $subs subscriber(s)"
  else
    echo "    $topic  MISSING (no publisher: sensor or plugin was not created)"
  fi
done
