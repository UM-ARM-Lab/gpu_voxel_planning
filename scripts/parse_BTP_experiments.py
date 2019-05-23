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


short_scenario = OrderedDict([
    ("Bookshelf", "Bookshelf"),
    ("Table_with_Box_table_known_visible_cave_known_full_cave_unknown", "Box")])

short_belief = OrderedDict([
    ("CHS_0.000000_0.000000_0.000000_0.000000", "CHS"),
    ("Obstacle_0.000000_0.000000_0.000000_0.100000", "Good Particles"),
    ("Obstacle_0.100000_0.100000_0.100000_0.400000", "Noisy Particles"),
    ("Bonkers_0.000000_0.000000_0.000000_0.050000", "Bonkers"),
    ("MoE_0.000000_0.000000_0.000000_0.100000", "MoE Good")])

short_strategy = OrderedDict([
    ("Optimistic", "Optimistic"),
    ("ParetoCosta1", "CollisionMeasure a=1"),
    ("ParetoCosta10", "CollisionMeasure a=10"),
    ("ORO", "ORO"),
    ("HOP", "HOP"),
    ("Thompson", "Thompson")])


experiment_dir = "/experiments/"
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

def plot_data(all_experiments, save_path):
    for scenario in get_scenarios(all_experiments):
        exps = [exp for exp in all_experiments if exp.scenario == scenario]
        plot_scenario(exps, save_path)
    

def plot_scenario(experiments, save_path):

    """Plots a list of experiments all belonging to the same scenario"""

    experiments.sort(key=lambda e:e.label)

    series = pd.Series([e.exec_cost * e.succeeded for e in experiments])
    ax = series.plot(kind='bar')
    
    ax.set_title(short_scenario[experiments[0].scenario])
    
    x_labels = [e.label for e in experiments]
    ax.set_xticklabels(x_labels)
    plt.tight_layout()
    plt.show()
    ax.get_figure().savefig(save_path + experiments[0].scenario + ".png")


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
            return float('inf')
        return exp.exec_cost

    def write_belief_headers(f):
        for bel in beliefs:
            f.write(" & ")
            f.write(bel)
        f.write("\\\\")
        

    def write_line(f, scenario, strat):
        f.write("\\hline\n")
        f.write(strat)
        for bel in beliefs:
            f.write(" & ")
            f.write(str(gc(scenario, strat, bel)))
        f.write("\\\\\n")

    def write_table(scenario):
        with open(save_path + "table_" + scenario + ".tex", "w") as f:
            f.write("\\begin{table}[]\n")
            f.write("\\centering\n")
            f.write("\\begin{tabular}{|c|" + "c|" * len(beliefs) + "}\n")
            f.write("\\hline\n")
            write_belief_headers(f)
            for strat in strategies:
                write_line(f, scenario, strat)
            f.write("\\hline\n")
            f.write("\\end{tabular}\n")
            f.write("\\caption{" + scenario + "}\n")
            f.write("\\label{tab:experiment_" + scenario + "}\n")
            f.write("\\end{table}\n")

    write_table("Bookshelf")
    write_table("Box")

    

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
                exp.planning_time = parts[4]
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
    plot_data(experiments, path)
    write_latex(experiments, path)
    


if __name__=='__main__':
    rospy.init_node("plot_experiments")
    load_all_files();
