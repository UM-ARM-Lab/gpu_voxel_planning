#!/usr/bin/env python
from matplotlib import pyplot as plt
import os
import IPython


logged_filepath = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/selective_densification_timings/"
save_path = logged_filepath



strategy_mappings = {"RRT_Strategy":"RRT",
                     "Omniscient_SD_Graph_Search": "Selective Densification",
                     "Omniscient_SD_Graph_Search_": "Selective Densification",
                     "Omniscient_SD_Graph_Search_precomputed": "SD with precomputed edges",
                     "Omniscient_SD_Graph_Search_precomputed_": "SD with precomputed edges",
                     "BIT_Strategy":"BIT*",
                     "Dense_Graph_Search":"Dense Graph",
                     "Dense_Graph_Search_precomputed":"DG with precomputed edges"
                     }

scenario_mappings = {"Bookshelf":"Bookshelf",
                     "Table_with_Box_table_known_visible_cave_known_full_cave_known":"Table",
                     "SlottedWall": "Wall"}

ordering = {"RRT": 1,
            "BIT*":1.1,
            "Dense Graph":1.8,
            "DG with precomputed edges":1.9,
            "Selective Densification":2,
            "SD with precomputed edges":3
            }


linestyles = {"RRT": {"color": "grey", "linewidth":2},
              "BIT*": {"color" : [1, 0, 0], "linewidth":2},
              "Dense Graph":{"color" : [.3, .7, .1], "linewidth":2},
              "DG with precomputed edges":{"color" : [.3, .7, .6], "linewidth":2},
              "Selective Densification": {"color": "blue", "linewidth":4},
              "SD with precomputed edges": {"color": "green", "linewidth":4}}



def extract_experiment_info(dirname, filename):
    # print filename.split()
    parts = filename.split()
    return {"scenario": scenario_mappings[parts[0]],
            "strategy": strategy_mappings[parts[1]],
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

            if float(data[2]) > 1000000:
                continue
            
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
    exps.sort(key=lambda exp: ordering[exp["strategy"]])

    fig = plt.figure()
    ax = plt.subplot(111)
    
    for exp in exps:
        data = exp["data"]
        # ax.plot(data[0], data[1], label=exp["strategy"])
        ax.plot(data[0], data[1], label=exp["strategy"], **linestyles[exp["strategy"]])
    plt.title(scenario)

    ax.set_xlabel("Planning Time (s)")
    ax.set_ylabel("Path Length (rad)")

    box = ax.get_position()
    ax.set_position([box.x0, box.y0, box.width * 0.5, box.height])

    

    handles, labels = ax.get_legend_handles_labels()
    handle_list, label_list = [], []
    for handle, label in zip(handles, labels):
        if label not in label_list:
            handle_list.append(handle)
            label_list.append(label)
    ax.legend(handle_list, label_list, loc='center left', bbox_to_anchor=(1, 0.5))


    
    # ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    
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
    



