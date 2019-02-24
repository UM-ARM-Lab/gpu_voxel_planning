#!/usr/bin/env python
from matplotlib import pyplot as plt
import os
import IPython


logged_filepath = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/selective_densification_timings/"
save_path = logged_filepath


def extract_experiment_info(dirname, filename):
    # print filename.split()
    parts = filename.split()
    return {"scenario": parts[0],
            "strategy": parts[1],
            "data": extract_timing_data(dirname, filename)}

def extract_timing_data(dirname, filename):
    filepath = dirname + filename
    t = []
    l = []
    eps = 0.0001
    with open(filepath, "r") as f:
        # line = f.readline()
        lines = f.readlines()
        for line in lines:
            if not line.startswith("PathLength"):
                continue
            data = line.split()
            if len(data) > 3:
                continue

            #Added time datapoint just before new datapoint, for accurate plotting
            if(len(t) > 0):
                t.append(float(data[1]) - eps)
                l.append(l[-1])
            
            t.append(float(data[1]))
            l.append(float(data[2]))
            # print(data)
    return [t, l]

def load_experiment_files():

    exps = []
    for name in os.listdir(logged_filepath):
        if name.endswith(".png"):
            continue
        # extract_timing_data(logged_filepath, name)
        exps.append(extract_experiment_info(logged_filepath, name))
    return exps


def plot_group(exps):
    scenario = exps[0]["scenario"];
    print "Scenario:", scenario

    # strategies = [x["strategy"] for x in exps]
    exps.sort(key=lambda exp: exp["strategy"])
    
    for exp in exps:
        data = exp["data"]
        plt.plot(data[0], data[1], label=exp["strategy"])
    plt.title(scenario)
    plt.legend()
    plt.xlabel("Time (s)")
    plt.ylabel("Path Length (rad)")
    plt.savefig(save_path + scenario)
    plt.show()
    # print exps
    # print len(exps)
    # print ""


def plot_all(exps):
    scenarios = {x["scenario"] for x in exps}
    for scenario in scenarios:
        scen_exps = []
        for exp in exps:
            if not exp["scenario"] == scenario:
                continue
            scen_exps.append(exp)
        plot_group(scen_exps)
    

if __name__ == "__main__":
    
    print("hi")
    exps = load_experiment_files()
    plot_all(exps)
    



