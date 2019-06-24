#!/usr/bin/env python
import os
import IPython

for name in os.listdir("."):
    if name.startswith("fix_missing"):
        continue
    parts = name.split("_")

    scenario = parts[0]
    strategy = parts[1]
    belief = "_".join(parts[2:7])

    print scenario, strategy, belief

    with open(name, "r") as f:
        contents = f.readlines()

    contents.insert(3, "Belief: " + belief +     "   0.0000 1 0 0.0000 0.0000 0.0000\n")
    contents.insert(3, "Scenario: " + scenario + "   0.0000 1 0 0.0000 0.0000 0.0000\n")
    contents.insert(3, "Strategy: " + strategy + "   0.0000 1 0 0.0000 0.0000 0.0000\n")

    with open(name, "w") as f:
        f.write("".join(contents))
    

    



