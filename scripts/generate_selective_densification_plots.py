#!/usr/bin/env python
from matplotlib import pyplot as plt
import os
import numpy as np
import IPython


logged_filepath = "/home/bradsaund/catkin_ws/src/gpu_voxel_planning/selective_densification_timings/"
save_path = logged_filepath

plotted_cp = 1.0

ordered_scenarios = ["Table", "Bookshelf", "Wall"]



color_cycle = ['#332288', '#88CCEE', '#44AA99', '#117733', '#999933', '#DDCC77', '#CC6677', '#882255', '#AA4499']
color_cycle = ['#e41a1c',
               '#377eb8',
               '#4daf4a',
               '#984ea3',
               '#ff7f00',
               '#ffff33',
               '#a65628',
               '#f781bf']

strategy_mappings = {"RRT_Strategy":"RRT",
                     "Omniscient_SD_Graph_Search": "SD",
                     "Omniscient_SD_Graph_Search_precomputed": "SD-pre",
                     "BIT_Strategy":"BIT*",
                     "Dense_Graph_Search":"DG",
                     "Dense_Graph_Search_precomputed":"DG-pre"
                     }

scenario_mappings = {"Bookshelf":"Bookshelf",
                     "Table_with_Box_table_known_visible_cave_known_full_cave_known":"Table",
                     "SlottedWall": "Wall"}

ordering = {"RRT": 1,
            "BIT*":1.1,
            "DG":1.8,
            "DG-pre":1.9,
            "SD":2,
            "SD-pre":3
            }


linestyles = {"RRT":    {"color": "grey",         "linewidth":2},
              "BIT*":   {"color": color_cycle[3], "linewidth":2},
              "DG":     {"color": "yellow",       "linewidth":2},
              "DG-pre": {"color": "orange",       "linewidth":2},
              "SD":     {"color": "blue",         "linewidth":5},
              "SD-pre": {"color": "green",        "linewidth":5}}



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
    u = []
    c_p = -1
    eps = 0.0001
    col_checks = float('inf')
    a_star = -1

    with open(filepath, "r") as f:
        # line = f.readline()
        lines = f.readlines()
        for line in lines:
            data = line.split()
            if line.startswith("c_p") and len(data) < 5:
                c_p = float(data[2])
                continue

            if line.startswith("SetRobotConfig before smoothing"):
                col_checks = float(data[4])
                continue

            if line.startswith("lazy_sp a_star") and len(data) > 2:
                if(data[2] == "forward" or
                   data[2] == "reverse"):
                    continue
                a_star = float(data[2])
            
            if not line.startswith("PathLength"):
                continue

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
    u = [time + length*4 for (time, length) in zip(t, l)]
            
    return {"time": t,
            "length": l,
            "utility": u,
            "c_p": c_p,
            "col_checks" : col_checks,
            "a_star": a_star}

def load_experiment_files():

    exps = []
    for name in os.listdir(logged_filepath):
        if name.endswith(".png"):
            continue
        if name.endswith(".txt"):
            continue
        # extract_timing_data(logged_filepath, name)
        exps.append(extract_experiment_info(logged_filepath, name))
    return exps



def plot_all_strategies(exps, variant):
    """plots strategies on the same plot
    exps: list of experiement data
    variant: string, either "length" or "utility"
    """
    scenario = exps[0]["scenario"];
    print "Scenario:", scenario

    # strategies = [x["strategy"] for x in exps]
    exps.sort(key=lambda exp: ordering[exp["strategy"]])

    fig = plt.figure()
    ax = plt.subplot(111)
    
    for exp in exps:
        data = exp["data"]

        if(exp["data"]["c_p"] != -1 and
           exp["data"]["c_p"] != plotted_cp):
            continue
            

        if(variant == "length"):
            y = data["length"]
            y_axis_label = "Path Length (rad)"
        if(variant == "utility"):
            y = data["utility"]
            y_axis_label = "Planning + Execution (s)"
            # ax.set_ylim([0, 50])
            ax.set_xlim([0, 100])

        if(len(data["time"]) > 0 and data["time"][0] > 100):
            continue
            
        ax.plot(data["time"], y, label=exp["strategy"], **linestyles[exp["strategy"]])
    # plt.title(scenario)

    ax.set_xlabel("Planning Time (s)", fontsize=22)
    ax.set_ylabel(y_axis_label, fontsize=22)
    ax.xaxis.set_tick_params(labelsize=20)
    ax.yaxis.set_tick_params(labelsize=20)

    box = ax.get_position()
    ax.set_position([box.x0, box.y0 + 0.05, box.width, box.height])

    handles, labels = ax.get_legend_handles_labels()
    handle_list, label_list = [], []
    for handle, label in zip(handles, labels):
        if label not in label_list:
            handle_list.append(handle)
            label_list.append(label)
    # ax.legend(handle_list, label_list, loc='center left', bbox_to_anchor=(1.1, 0.5))
    
    # ax.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.savefig(save_path + scenario + "_" + variant)
    plt.show()


    fig_legend = plt.figure()
    fig_legend.legend(handle_list, label_list)
    fig_legend.savefig(save_path + scenario + "_legend", bbox_inches = 'tight')
    plt.show()
    
    # print exps
    # print len(exps)
    # print ""

    

def plot_without_smoothing(exps):
    exps.sort(key=lambda exp: ordering[exp["strategy"]])
    scenario = exps[0]["scenario"];

    fig = plt.figure()
    ax = plt.subplot(211)
    ax2 = plt.subplot(212)

    for exp in exps:
        # if(exp["strategy"] != "Selective Densification" and
        #    exp["strategy"] != "SD with precomputed edges"):
        if(exp["strategy"] != "SD-pre"):
            continue
        data = exp["data"]
        # x = data["time"][0]
        x = data["c_p"]
        y = data["length"][0]
        ax.scatter(x, y, label=exp["strategy"], **linestyles[exp["strategy"]])
        ax2.scatter(x, data["time"][0], label=exp["strategy"], **linestyles[exp["strategy"]])
        # ax.annotate(exp["data"]["c_p"], (x, y))


    box = ax.get_position()
    ax.set_position([box.x0, box.y0, box.width, box.height])

    box = ax2.get_position()
    ax2.set_position([box.x0, box.y0 + .03, box.width, box.height])


    
    ax.set_xscale("symlog", linthreshx=0.00001)
    ax.set_xlim([-0.00001, 1000])
    # ax.set_xlabel("Planning Time (s)")
    ax.set_ylabel("Path Length (rad)", fontsize=22)
    ax.yaxis.set_label_coords(-0.1, 0.6)
    # ax.title(scenario)

    ax2.set_xscale("symlog", linthreshx=0.00001)
    # ax2.set_yscale("log")
    ax2.set_xlim([-0.00001, 1000])
    ax2.set_ylabel("Planning time (s)", fontsize=22)
    ax2.set_xlabel("c_p", fontsize = 22)

    
    ax2.set_xticks([0, 0.0001, 0.01, 1, 100])
    ax2.xaxis.set_tick_params(labelsize=20)
    ax2.yaxis.set_tick_params(labelsize=20)
    ax2.set_yticks(ax2.get_yticks()[::2])
    ax2.yaxis.set_label_coords(-0.1, 0.4)
    
    ax.set_xticklabels([])
    ax.set_yticks(ax.get_yticks()[::2])
    ax.yaxis.set_tick_params(labelsize=20)
    
    # plt.title(scenario)
    plt.savefig(save_path + scenario + "_cp_sweep")
    plt.show()
        

def plot_group(exps):
    plot_all_strategies(exps, "length")
    plot_all_strategies(exps, "utility")
    plot_without_smoothing(exps)
    


def plot_all(exps):
    scenarios = {x["scenario"] for x in exps}
    for scenario in scenarios:
        scen_exps = []
        for exp in exps:
            if not exp["scenario"] == scenario:
                continue
            scen_exps.append(exp)
        plot_group(scen_exps)

def get_min_times(exps):
    min_times = []
    min_lengths = []
    min_utils = []
    col_checks = []
    a_star = []
    for exp in exps:
        min_time = float("inf")
        min_length = float("inf")
        min_util = float("inf")
        if len(exp["data"]["time"]) > 0:
            min_time = min(exp["data"]["time"])
        if len(exp["data"]["length"]) > 0:
            min_length = min(exp["data"]["length"])
        if len(exp["data"]["utility"]) > 0:
            min_util = min(exp["data"]["utility"])

        if min_time == float("inf"):
            continue
        
        col_checks.append(exp["data"]["col_checks"])
        min_times.append(min_time)
        min_lengths.append(min_length)
        min_utils.append(min_util)
        a_star.append(exp["data"]["a_star"])
    # if exps[0]["strategy"] == "RRT":
    #     IPython.embed()
    return {"min_times" : min_times,
            "min_lengths" : min_lengths,
            "min_utils" : min_utils,
            "col_checks" : col_checks,
            "a_star": a_star}
    

def format_double(num):
    number_format = '{0:.2g}'
    if num>100:
        return '{0:g}'.format(float('{:.2g}'.format(num)))
    return number_format.format(num)

def format_int(num):
    return str(int(num))

def compute_or_dash(l, format_func=format_double):
    """
    returns the minimum value if greater than 0, or a "-" if the list is empty or contains a negative element
    """
    if len(l) == 0:
        return "-"

    if min(l) < 0:
        return "-"
    
    return format_func(l)
    

def print_best_scene_strat_stats(exps, f):
    """
    Takes in experiments all from the same scene/strat and a file f
    Prints the min to line
    """
    r = get_min_times(exps)

    fd = lambda l: format_double(min(l))
    fi = lambda l: format_int(min(l))

    f.write("    & " + compute_or_dash(r["min_times"], fd)
            + " & "  + compute_or_dash(r["min_lengths"], fd)
            + " & "  + compute_or_dash(r["min_utils"], fd)
            + " & "  + compute_or_dash(r["col_checks"], fi)
            + " & "  + compute_or_dash(r["a_star"], fd))
    f.write("\n")

def print_avg_scene_strat_stats(exps, f):
    """
    Takes in experiments all from the same scene/strat and a file f
    Prints the min to line
    """
    r = get_min_times(exps)

    fd = lambda l: format_double(np.mean(l))
    fi = lambda l: format_int(np.mean(l))
        
    f.write("    & "  + compute_or_dash(r["min_times"], fd)
            + " & "  + compute_or_dash(r["min_lengths"], fd)
            + " & "  + compute_or_dash(r["min_utils"], fd)
            + " & "  + compute_or_dash(r["col_checks"], fi)
            + " & "  + compute_or_dash(r["a_star"], fd)
    )
    f.write("\n")




def is_valid(exp, scenario, strat):
    if not exp["scenario"] == scenario:
        return False
    if not exp["strategy"] == strat:
        return False
    if(exp["data"]["c_p"] != -1 and
       exp["data"]["c_p"] != plotted_cp):
        return False
    return True
    
def print_stats(exps):
    # scenarios = {x["scenario"] for x in exps}
    strategies = {x["strategy"] for x in exps}
    filepath = logged_filepath + "stats.txt"
    
    with open(filepath, "w") as f:
        for strat in strategies:
            f.write("best-" + strat + " \n")
            for scenario in ordered_scenarios:
                f.write("  % " + scenario  +"\n")
                matching_exps = [exp for exp in exps if is_valid(exp, scenario, strat)]
                print_best_scene_strat_stats(matching_exps, f)
            f.write("  \\\\ \n")

            f.write("avg-" + strat + " \n")
            for scenario in ordered_scenarios:
                f.write("  % " + scenario  +"\n")
                matching_exps = [exp for exp in exps if is_valid(exp, scenario, strat)]
                print_avg_scene_strat_stats(matching_exps, f)
            f.write("  \\\\ \n")


def print_better_fraction(exps):
    for scenario in ordered_scenarios:
        SD = [exp for exp in exps if is_valid(exp, scenario, "SD-pre")]
        RRTs = [exp for exp in exps if is_valid(exp, scenario, "RRT")]
        BITs = [exp for exp in exps if is_valid(exp, scenario, "BIT*")]

        r_SD = get_min_times(SD)
        r_RRT = get_min_times(RRTs)
        r_BIT = get_min_times(BITs)

        SD_util = min(r_SD["min_utils"])

        better_rrt = sum(util < SD_util for util in r_RRT["min_utils"])
        better_bit = sum(util < SD_util for util in r_BIT["min_utils"])

        print(scenario)
        print("Better RRT: " + str(better_rrt) + "/" + str(len(r_RRT["min_utils"])))
        print("Better BIT: " + str(better_bit) + "/" + str(len(r_BIT["min_utils"])))
        
        
        # IPython.embed()
    

if __name__ == "__main__":
    
    print("hi")
    exps = load_experiment_files()
    # plot_all(exps)
    print_stats(exps)
    print_better_fraction(exps)



