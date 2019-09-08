#!/usr/bin/env python
from matplotlib import pyplot as plt
import os
import numpy as np
import IPython
import pandas as pd

proposed_filepath = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/selective_densification_timings/"
baseline_filepath = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/selective_densification_timings_old_bidirectional/"
# save_path = logged_filepath + "figures/"

chosen_cp = 1.0


scenario_mappings = {"Bookshelf":"Bookshelf",
                     "Table_with_Box_table_known_visible_cave_known_full_cave_known":"Table",
                     "SlottedWall": "Wall",
                     "CloseWall": "Obstacle"}



def parse_experiment(dirname, filename):
    filepath = dirname + filename
    with open(filepath, "r") as f:
        lines = f.readlines()

        for line in lines:
            data = line.split()
            if len(data) == 0:
                continue
            
            if data[0].startswith("Scenario:"):
                scenario = scenario_mappings[data[1]]
            if data[0].startswith("Strategy:"):
                strategy = data[1]
                if strategy != "Omniscient_SD_Graph_Search":
                    return None

            # if line.startswith("c_p") and len(data) < 5:
            if line.startswith("c_p"):
                c_p = float(data[-1])
                if c_p != chosen_cp:
                    return None

            if line.startswith("Failed"):
                return None
            
            if line.startswith("Plan"):
                total = float(data[1])

            if line.startswith("lazy_sp a_star forward"):
                astar_forward = float(data[3])
            if line.startswith("lazy_sp a_star reverse"):
                astar_reverse = float(data[3])
            if data[0]=="CheckEdge":
                if line.startswith("CheckEdge From Scratch"):
                    continue
                check_edge = float(data[1])


            if data[0].startswith("~~~~~~~~~~~"):
                break
        fr = pd.DataFrame({"scenario": [scenario],
                           "astar_forward": [astar_forward],
                           "astar_reverse": [astar_reverse],
                           "check_edge": [check_edge],
                           "cp": [c_p],
                           "total_time": [total],
        })

        return fr

def formatpm(df, scenario, field):
    m = df.groupby(["scenario"]).mean()
    s = df.groupby(["scenario"]).std()

    return " & " + ("%.1f" % m[field][scenario]) + " $\\pm$ " + ("%.1f" % s[field][scenario])
    
        
def output_latex(baseline, proposed):
    """
    baseline and proposed should be dataframes
    """
    # bm = baseline.groupby(["scenario"]).mean()
    # bsd = baseline.groupby(["scenario"]).std()
    # pm = proposed.groupby(["scenario"]).mean()
    # psd = proposed.groupby(["scenario"]).std()

    scenario = "Wall"

    def prop(field):
        return formatpm(proposed, scenario, field)
    def base(field):
        return formatpm(baseline, scenario, field)
    
    print("Table for " + scenario)
    print("")

    print("\\begin{table}[]")
    print("\\begin{tabular}{lllll}")
    print(" & Total & Edge Check & Forward & Reverse \\\\")
    print("Proposed" + prop("total_time") + prop("check_edge") +
          prop("astar_forward") + prop("astar_reverse") + "\\\\")
    print("Baseline" + base("total_time") + base("check_edge") +
          base("astar_forward") + base("astar_reverse") + "\\\\")
    print("\\end{tabular}")
    print("\\end{table}")


def parse_experiments(dirname):
    data_frames = []
    for name in os.listdir(dirname):
        if name.endswith(".png"):
            continue
        if name.endswith(".txt"):
            continue
        if os.path.isdir(os.path.join(dirname, name)):
            continue
        fr = parse_experiment(dirname, name)
        if fr is not None:
            data_frames.append(fr)
    return pd.concat(data_frames, ignore_index=True)

def generate_bilazysp_table():
    baseline_df = parse_experiments(baseline_filepath)
    proposed_df = parse_experiments(proposed_filepath)


    output_latex(baseline_df, proposed_df)


if __name__ == "__main__":
    
    print("generating table for bilazysp comparison")
    generate_bilazysp_table()
