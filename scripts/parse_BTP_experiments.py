#!/usr/bin/env python

import requests
import json
import urllib2
import IPython
import os
import time
from datetime import datetime
from matplotlib import pyplot as plt
from collections import OrderedDict
import matplotlib.dates as mdates
import rospy
import rospkg
import pandas as pd
from collections import OrderedDict
import numpy as np
import seaborn as sns


# Real
experiment_dir = "/experiments_real/"
scenarios_to_parse = ["RealTable", "Refrigerator"]

# Sim
# experiment_dir = "/experiments/"
# scenarios_to_parse = ["Box", "Bookshelf"]


short_scenario = OrderedDict([
    ("Bookshelf", "Bookshelf"),
    ("Table_with_Box_table_known_visible_cave_known_full_cave_unknown", "Box"),
    ("RealEmpty", "RealTable"),
    ("Fridge", "Refrigerator")])

short_belief = OrderedDict([
    ("MoE_0.000000_0.000000_0.000000_0.100000", "MoE Good"),
    ("MoE_0.100000_0.100000_0.100000_0.400000", "MoE Noisy"),
    ("MoEBonkers_0.000000_0.000000_0.000000_0.050000", "MoE Bonkers"),
    ("CHS_0.000000_0.000000_0.000000_0.000000", "CHS"),
    ("Obstacle_0.000000_0.000000_0.000000_0.100000", "Particles Good"),
    ("Obstacle_0.100000_0.100000_0.100000_0.400000", "Particles Noisy"),
    ("Bonkers_0.000000_0.000000_0.000000_0.050000", "Particles Bonkers"),
])

short_strategy = OrderedDict([
    ("ParetoCosta1", "CM 1"),
    ("ParetoCosta10", "CM 10"),
    ("Optimistic", "OFU"),
    ("ORO", "ORO"),
    ("HOP", "MCBE"),
    ("QMDP", "QMDP"),
    ("Thompson", "TS")])

def super_short_belief(belief_string):
    sb = short_belief[belief_string]
    if(sb == "CHS"):
        return "CHS"
    if(sb.startswith("Particles")):
        return "MPF"
    if(sb.startswith("MoE")):
        return "MoE"

def is_selected_strat(strat):
    ss = short_strategy[strat]
    return True
    if ss in ["OFU", "CM 1", "MCBE"]:
        return True
    return False


fmt = "%Y-%m-%dT%H:%M:%S"

class Experiment:
    timestamp = None
    scenario = None
    strategy = None
    belief = None
    label = None
    exec_cost = None
    pareto_weight = None
    succeeded = True
    planning_time = None
    num_collision = None
    num_steps = None

class ExperimentGroup:
    scenario = None
    strategy = None
    belief = None
    exec_costs = None
    avg_exec_cost = None
    planning_times = None
    avg_planning_time = None
    succeeded = None

def find_all(experiments, scenario, strategy, belief):
    return [e for e in experiments
            if e.scenario == scenario
            if e.strategy == strategy
            if e.belief == belief]

def group_experiments(experiments):
    """Groups experiments of the same scenario, strat, belief"""
    grouped = []
    for exp in experiments:
        if len(find_all(grouped, exp.scenario, exp.strategy, exp.belief)):
            print("Skipping " + exp.scenario + ", " + exp.strategy + ", " + exp.belief + ". Already evaluated")
            continue
        eg = ExperimentGroup()
        eg.scenario = exp.scenario
        eg.strategy = exp.strategy
        eg.belief = exp.belief
        eg.succeeded = True
        eg.exec_costs = []
        eg.planning_times = []
        
        g = find_all(experiments, exp.scenario, exp.strategy, exp.belief)
        print(len(g))
        for e in g:
            if(exp.exec_cost is None):
                eg.exec_costs.append(float('inf'))
            else:
                eg.exec_costs.append(exp.exec_cost)
            eg.planning_times.append(exp.planning_time)
            eg.succeeded = eg.succeeded and exp.succeeded

        eg.avg_exec_cost = np.mean(eg.exec_costs)
        eg.avg_planning_time = np.mean(eg.planning_times)
        grouped.append(eg)
        # IPython.embed()
    return grouped

def average_same_xval(xs, ys):
    if len(xs) != len(ys):
        raise "Can't average when xs and ys have different lengths"
        
    avg_xs = []
    avg_ys = []
    for x in xs:
        if x in avg_xs:
            continue
        same_y_costs = [ys[i] for i in range(len(ys)) if xs[i] == x]
        
        
        avg_xs.append(x)
        avg_ys.append(np.mean(same_y_costs))
    return avg_xs, avg_ys
        
    
        

def get_experiment(experiments, scenario, strategy, belief):
    """
    Returns the experiment matching scenario, strategy, belief
    """
    for exp in experiments:
        if short_scenario[exp.scenario] != scenario:
            continue
        if short_belief[exp.belief] != belief:
            continue
        if short_strategy[exp.strategy] != strategy:
            continue
        return exp
    return None


def get_scenarios(experiments):
    """ Returns a set of all scenarios in the experiments"""
    return {exp.scenario for exp in experiments}

def plot_all_data_for_scenarios(all_experiments, save_path):
    for scenario in get_scenarios(all_experiments):
        exps = [exp for exp in all_experiments if exp.scenario == scenario]
        plot_scenario(exps, save_path)
    

def plot_scenario(experiments, save_path):

    """Plots a list of experiments all belonging to the same scenario"""

    experiments.sort(key=lambda e:e.label)

    series = pd.Series([e.avg_exec_cost * e.succeeded for e in experiments])
    ax = series.plot(kind='bar')
    
    ax.set_title(short_scenario[experiments[0].scenario])
    
    x_labels = [e.label for e in experiments]
    ax.set_xticklabels(x_labels)
    plt.tight_layout()
    plt.show()
    ax.get_figure().savefig(save_path + experiments[0].scenario + ".png")


def plot_sorted(all_experiments, save_path):
    hardness_map = {"easy": ["CHS", "Particles Good", "MoE Good"],
                    "medium" : ["CHS", "Particles Noisy", "MoE Noisy"],
                    "hard" : ["CHS", "Particles Bonkers", "MoE Bonkers"]}
    
    for scenario in get_scenarios(all_experiments):
        for hardness in hardness_map:
            exps = [exp for exp in all_experiments
                    if exp.scenario == scenario
                    if short_belief[exp.belief] in hardness_map[hardness]
                    if is_selected_strat(exp.strategy)]
            plot_sorted_single(exps, hardness, save_path)

def plot_sorted_single(experiments, hardness, save_path):
    """Plots a list of experiments all belonging to the same scenario"""

    for e in experiments:
        e.label = super_short_belief(e.belief) + "+" + short_strategy[e.strategy]
        # e.label = e.label.ljust(8)
    order = ["MoE+CM 1",
             "CHS+CM 1",
             "MPF+CM 1",
             # "CHS+MCBE",
             # "MPF+MCBE",
             "MoE+OFU",
             "MoE+TS",
             "CHS+QMDP",
             "MoE+MCBE",
             "CHS+ORO",

             # "CHS+OFU",
             # "MPF+OFU"

    ]

    def custom_sort(exp):
        # order = ["MoE+CM",
        #          "MoE+MCBE",
        #          "MoE+OFU",
        #          "CHS+CM",
        #          "CHS+MCBE",
        #          "CHS+OFU",
        #          "MPF+CM",
        #          "MPF+MCBE",
        #          "MPF+OFU"]
        label = exp.label
        for i in range(len(order)):
            if label.startswith(order[i]):
                return i
        return 100
        

    # experiments.sort(key=lambda e:e.label)
    experiments.sort(key=custom_sort)
    experiments = [e for e in experiments if e.label in order]
    
    costs = [e.avg_exec_cost * e.succeeded for e in experiments]
    planning_times = [e.avg_planning_time * e.succeeded for e in experiments]

    x_labels, costs = average_same_xval([e.label for e in experiments], costs)
    x_labels, planning_times = average_same_xval([e.label for e in experiments], planning_times)

    x_labels = x_labels + ["CHS+RRT"]
    costs = costs + [100]
    planning_times = planning_times + [15*60]
    hue = [float(label == "MoE+CM 1") for label in x_labels ]
    # hue = [c for c in costs]
    clipped_times = [min(p, 100) for p in planning_times]
    clipped_policy = [min(p, 40) for p in costs]

    data = pd.DataFrame({"average policy cost":clipped_policy,
                         "planning time (s)":clipped_times,
                         "method":x_labels,
                         "hue":hue})
    
    # series = pd.Series(costs)
    # ax = series.plot(kind='bar', fontsize=30)
    # IPython.embed()
    sns.set(style="whitegrid", font_scale=2.2)
    ax = sns.barplot(x="method", y="average policy cost", data=data, hue="hue", dodge=False)




    ax.legend_.remove()
    
    ax.set_xticklabels(ax.get_xticklabels(), rotation=90)
    ax.set_title(short_scenario[experiments[0].scenario] + " " + hardness + "\n", fontsize=30)
    # ax.set_xticklabels(x_labels)
    # ax.set_yticks([25, 50])
    ax.set_yticks([20, 40])
    labels = [item.get_text() for item in ax.get_yticklabels()]
    labels[0] = "20"
    labels[1] = ">40"
    if(hardness=="medium" or hardness =="hard"):
        ax.yaxis.set_label_text(" ")
        labels[0] = " "
        labels[1] = "   "
    ax.set_yticklabels(labels)
    ax.xaxis.set_label_text("")

    # plt.axvline(x=2.5, linewidth=1, color='k')
    plt.axvline(x=0.5, linewidth=1, color='k')
    plt.tight_layout()
    plt.show()
    ax.get_figure().savefig(save_path + short_scenario[experiments[0].scenario] + "_" + hardness + ".png")


    ax = sns.barplot(x="method", y="planning time (s)", data=data, hue="hue", dodge=False)

    ax.legend_.remove()
    
    ax.set_xticklabels(ax.get_xticklabels(), rotation=90)
    ax.set_yticks([50, 100])
    labels = [item.get_text() for item in ax.get_yticklabels()]
    labels[0] = "50"
    labels[1] = ">100"
    if(hardness=="medium" or hardness =="hard"):
        ax.yaxis.set_label_text(" ")
        labels[0] = " "
        labels[1] = "   "
    ax.set_yticklabels(labels)

    ax.set_yticklabels(labels)
    ax.xaxis.set_label_text("")

    # plt.axvline(x=2.5, linewidth=1, color='k')
    plt.axvline(x=0.5, linewidth=1, color='k')
    plt.tight_layout()
    plt.show()
    ax.get_figure().savefig(save_path + short_scenario[experiments[0].scenario] + "_" + hardness + "_times.png")

def plot_avg_fig(all_experiments, save_path):
    scenario = "Bookshelf"
    exps = [exp for exp in all_experiments
            if exp.scenario == scenario
            if is_selected_strat(exp.strategy)]
    plot_sorted_single(exps, "all", save_path)



def write_latex(experiments, save_path):
#     \begin{table}[]
#     \centering
#     \begin{tabular}{|c|c|c|c|c|c|c|c|c|c|}
#         \hline
#          & \multicolumn{9}{c|}{Prior Belief} \\
#         \hline
#          & \multicolumn{4}{c|}{MoE} & \multicolumn{4}{c|}{Obstacles} & CHS\\
#         \hline
#          & \multicolumn{2}{c|}{good} & \multicolumn{2}{c|}{bonkers} &
#          \multicolumn{2}{c|}{good} & \multicolumn{2}{c|}{bonkers} & \\
#         \hline
#          & narrow & broad & narrow & broad & narrow & broad & narrow & broad & \\
#         \hline
#         Optimistic & & & & & & & & & \\
#         \hline
#         Thompson & & & & & & & & & \\
#         \hline
#         HOP & & & & & & & & & \\
#         \hline
#         ORO & & & & & & & & & \\
#         \hline
#         Collision Measure & & & & & & & & & \\
#         \hline
#     \end{tabular}
#     \caption{HOP: Hindsight optimization, aka Average over Clairvoyance. ORO: Optimistic Rollout}
#     \label{tab:my_label}
#     \end{table}

    header = """\begin{table}[]
\centering
\begin{tabular}{|c|c|c|c|c|c|c|c|c|c|}
"""

    beliefs = short_belief.values()
    scenarios = short_scenario.values()
    strategies = short_strategy.values()
    
    def gc(scenario, strat, bel):
        exp = get_experiment(experiments, scenario, strat, bel)
        if exp is None:
            return "-"
        if not exp.succeeded:
            return str(float('inf'))
        if exp.avg_exec_cost is None:
            return str(float('inf'))
        return "%5.1f" % exp.avg_exec_cost

    def gt(scenario, strat, bel):
        exp = get_experiment(experiments, scenario, strat, bel)
        if exp is None:
            return "-"
        if not exp.succeeded:
            return str(float('inf'))
        return "%5.1f" % float(exp.avg_planning_time)

    def write_belief_headers(f):
        f.write("& \\multicolumn{3}{c|}{MoE} & CHS & \\multicolumn{3}{c|}{MPF} \n \\\\ \n")
        for bel in beliefs:
            f.write(" & ")
            bel = bel.strip("Particles ")
            bel = bel.strip("MoE ")
            classy_names = {"Good": "Easy", "Noisy": "Med", "Bonk": "Hard", "CHS": "CHS"}
            f.write(classy_names[bel])
        f.write("\\\\")


    def write_cost_line(f, scenario, strat):
        f.write("\\hline\n")
        f.write(strat)
        for bel in beliefs:
            f.write(" & ")
            f.write(gc(scenario, strat, bel))
        f.write("\\\\\n")

    def write_cost_table(scenario):
        with open(save_path + "table_" + scenario + "cost.tex", "w") as f:
            f.write("\\begin{table}[]\n")
            f.write("\\centering\n")
            f.write("\\begin{tabular}{|c|" + "c|" * len(beliefs) + "}\n")
            f.write("\\hline\n")
            write_belief_headers(f)
            for strat in strategies:
                write_cost_line(f, scenario, strat)
            f.write("\\hline\n")
            f.write("\\end{tabular}\n")
            f.write("\\caption{" + scenario + " Policy Cost}\n")
            f.write("\\label{tab:experiment_" + scenario + "_cost}\n")
            f.write("\\end{table}\n")

    def write_time_line(f, scenario, strat):
        f.write("\\hline\n")
        f.write(strat)
        for bel in beliefs:
            f.write(" & ")
            f.write(gt(scenario, strat, bel))
        f.write("\\\\\n")
        
    def write_time_table(scenario):
        with open(save_path + "table_" + scenario + "timings.tex", "w") as f:
            f.write("\\begin{table}[]\n")
            f.write("\\centering\n")
            f.write("\\begin{tabular}{|c|" + "c|" * len(beliefs) + "}\n")
            f.write("\\hline\n")
            write_belief_headers(f)
            for strat in strategies:
                write_time_line(f, scenario, strat)
            f.write("\\hline\n")
            f.write("\\end{tabular}\n")
            f.write("\\caption{" + scenario + " Planning Times}\n")
            f.write("\\label{tab:experiment_" + scenario + "_time}\n")
            f.write("\\end{table}\n")

        

    for scenario in scenarios_to_parse:
        write_cost_table(scenario)
        write_time_table(scenario)

    

def load_file(filepath, filename):
    exp = Experiment()
    with open(filepath + filename) as f:
        exp.timestamp = f.readline()
        line = f.readline()
        while line:
            parts = line.split()
            if len(parts) == 0:
                line = f.readline()
                continue
            
            if parts[0] == "accumulated_cost" and exp.exec_cost is None:
                exp.exec_cost = float(parts[5])
            elif parts[0] == "Strategy:":
                exp.strategy = parts[1]
            elif parts[0] == "Scenario:":
                exp.scenario = parts[1]
            elif parts[0] == "Belief:":
                exp.belief = parts[1]
            elif parts[0] == "Action_Limit_Exceeded":
                exp.succeeded = False
            elif parts[0] == "Failed":
                exp.succeeded = False
                exp.exec_cost = -1
            elif parts[0] == "Bump" and exp.num_collision is None:
                exp.num_collision = parts[2]
            elif len(parts) > 4 and parts[2] == "Planning" and exp.planning_time is None:
                exp.planning_time = float(parts[4])
                exp.num_steps = parts[5]
            line = f.readline()

    print filename
    exp.label =  short_belief[exp.belief] + "_" + short_strategy[exp.strategy]
    # IPython.embed()
    return exp
        

def load_all_files():
    path = rospkg.RosPack().get_path('gpu_voxel_planning') + experiment_dir
    experiments = []
    for name in os.listdir(path):
        if name.endswith(".png") or\
           name.endswith(".pdf") or\
           name.endswith(".tex"):
           continue;
        experiments.append(load_file(path, name))

    experiments = group_experiments(experiments)
        
    # plot_all_data_for_scenarios(experiments, path)
    # plot_sorted(experiments, path) - this is the one currently in the paper
    # plot_avg_fig(experiments, path)
    write_latex(experiments, path)
    


if __name__=='__main__':
    rospy.init_node("plot_experiments")
    load_all_files();
