"""
Takes the trjaectory found in input.xml, that is in on format, and writes it to the trajectory_timestamps_renumbered in a 
format that is compatable with the actor trajectory in gazebo.

New trajectory should be pasted into the model.sdf inside the volvo_AVG model, or any other actors.
"""

import re
import pathlib

src  = pathlib.Path("input.xml").read_text().splitlines()
out  = []
tick = 1

number_line = re.compile(r'^(\s*)([-0-9.eE]+(?:\s+[-0-9.eE]+){5})\s*$')
start_time = 0.0

def extract_time(text: str) -> float:
    """
    Pulls the number out of <time>…</time> and returns it as a float.
    """
    return float(re.search(r"<time>(.*?)</time>", text).group(1))

for line in src:
    # --- renumber <time> ----------------------------------------------
    if line.strip().startswith("<time>"):
        # <time>17393978098.0</time>
        # separate out the time 
        time = extract_time(line)
        #convert from milliseconds 
        time /= 1000
        if start_time == 0.0:
            start_time = time
        out.append(f"    <time>{time-start_time}</time>")
        tick += 1
        continue

    # --- fix the pose orientation -------------------------------------
    m = number_line.match(line)
    if m:
        indent, payload = m.groups()
        vals = payload.split()                 # six strings
        # swap: put yaw (old idx 5) in pitch slot (idx 3)
        vals[3], vals[5] = vals[5], vals[3]
        out.append(f"{indent}{' '.join(vals)}")
    else:
        out.append(line)

pathlib.Path("trajectory_timestamps_renumbered.xml").write_text("\n".join(out))
print("Done – wrote trajectory_timestamps_renumbered.xml with", tick-1, "waypoints")
