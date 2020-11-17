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
real_experiment_dir = "/experiments_real/"
# real_scenarios_to_parse = ["RealTable", "Refrigerator"]

# Sim
sim_experiment_dir = "/experiments/"
# sim_scenarios_to_parse = ["Box", "Bookshelf"]


short_scenario = OrderedDict([
    ("Bookshelf", "Bookshelf"),
    ("Table_with_Box_table_known_visible_cave_known_full_cave_unknown", "Box"),
    ("RealEmpty", "RealTable"),
    ("Fridge", "Refrigerator")])

short_belief = OrderedDict([
    ("MoE_0.000000_0.000000_0.000000_0.100000", "MoE Easy"),
    ("MoE_0.100000_0.100000_0.100000_0.400000", "MoE Med"),
    ("MoEBonkers_0.000000_0.000000_0.000000_0.050000", "MoE Hard"),
    ("CHS_0.000000_0.000000_0.000000_0.000000", "CHS"),
    ("Obstacle_0.000000_0.000000_0.000000_0.100000", "MPF Easy"),
    ("Obstacle_0.100000_0.100000_0.100000_0.400000", "MPF Med"),
    ("Bonkers_0.000000_0.000000_0.000000_0.050000", "MPF Hard"),
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
    if(sb.startswith("MPF")):
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

def get_hardness(b):
    b = short_belief[b]
    if(b == "CHS"):
        return "All"
    return b[4:]


def make_dataframe(experiments):
    return pd.DataFrame({"scenario": [short_scenario[e.scenario] for e in experiments],
                         "strategy": [short_strategy[e.strategy] for e in experiments],
                         "full belief":[short_belief[e.belief] for e in experiments],
                         "belief": [super_short_belief(e.belief) for e in experiments],
                         "planning time":[e.planning_time for e in experiments],
                         "policy cost":[e.exec_cost for e in experiments],
                         "success": [e.succeeded for e in experiments],
                         "hardness": [get_hardness(e.belief) for e in experiments]
    })
    

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

def plot(exps, save_path):
    sns.set(style="dark")

    cm = exps[exps["strategy"] == "CM 1"]
    # cm1 = exps[exps["strategy"] == "CM 1"]
    # cm10 = exps[exps["strategy"] == "CM 10"]
    # cm = pd.concat([cm1, cm10])

    # for i in range(cm["policy cost"].values):
    #     if cm["policy cost"].values[i] < 0:
    #         cm["policy cost"].values[i] = 128

    failed_cm = pd.concat([cm[cm["policy cost"] > 20], cm[cm["policy cost"] < 1]])
    failed_cm["policy cost"].values[:] = 19
    # failed_plot = pd.DataFrame({"belief": 
    # IPython.embed()
    # f, (ax_cm1, ax_cm2) = plt.subplots(nrows=2, ncols=1, sharex=True)
    order=["MoE Easy", "CHS", "MPF Easy", "MoE Med", "CHS", "MPF Med",  "MoE Hard", "CHS", "MPF Hard"]

    # ax_cm2 = sns.swarmplot(x="full belief", y="policy cost", data=cm,
    #                        order=order, hue="scenario", split=True)
    # ax_cm2 = sns.swarmplot(x="full belief", y="policy cost", data=failed_cm, ax=ax_cm2,
    #                        order=order, marker="+")
    # sns.swarmplot(x="full belief", y="policy cost", data=cm, ax=ax_cm2,
    #               order = order)

    ax_cm2 = sns.swarmplot(x="scenario", y="policy cost", data=cm,
                           hue="full belief", split=True, size=10)

    

    # ax = sns.swarmplot(x="full belief", y="policy cost", data=cm, ax=ax_cm2,
    #                    order=order)

    # ax_cm1.set_ylim(115, 130)
    ax_cm2.set_ylim(3, 20)

    plt.axvline(2.5, linewidth=1, color='k')
    plt.axvline(5.5, linewidth=1, color='k')
    # ax_cm = sns.violinplot(x="belief", y="policy cost", data=cm)
    # plt.subplots_adjust(wspace=0, hspace=0)
    
    plt.show()


    moe = exps[exps["belief"] == "MoE"]
    ax_moe = sns.swarmplot(x="strategy", y="policy cost", data = moe,
                           order=["CM 1", "CM 10", "TS", "MCBE", "QMDP", "OFU"])
    plt.show()
    # ax_moe = sns.swarmplot(x="strategy", y="policy cost", data = moe,
    #                        order=["CM 1", "CM 10", "TS", "MCBE", "QMDP", "OFU"])
                     

    IPython.embed()

'''
    Remove duplicate elements from list
'''
def removeDuplicates(listofElements):
    
    # Create an empty list to store unique elements
    uniqueList = []
    
    # Iterate over the original list and for each element
    # add it to uniqueList, if its not already there.
    for elem in listofElements:
        if elem not in uniqueList:
            uniqueList.append(elem)
    
    # Return the list of unique elements        
    return uniqueList

def CMvsOFU(exps):
    cm_better = 0
    total = 0
    cm_better_fraction = []
    
    for scenario in exps["scenario"].unique():
        for belief in exps["full belief"].unique():
            t = exps
            t = t[t["scenario"] == scenario]
            t = t[t["full belief"] == belief]
            t = t[t["belief"] == "MoE"]

            if(len(t[t["strategy"] == "CM 1"]["policy cost"].values) == 0):
                continue
            
            
            cm_cost = t[t["strategy"] == "CM 1"]["policy cost"].values[0]
            ofu_cost = t[t["strategy"] == "OFU"]["policy cost"].values[0]

            total += 1
            if cm_cost <= ofu_cost:
                cm_better += 1
            else:
                IPython.embed()

            cm_better_fraction.append(cm_cost / ofu_cost)
    print("CM is better than OFU " + str(cm_better) + " of " + str(total) + " trials")
    IPython.embed()

def CMvsMCBE(exps):
    cm_better = 0
    total = 0
    cm_better_fraction = []
    cm_better_time = []
    cm_planning_time = []
    mcbe_planning_time = []
    
    for scenario in exps["scenario"].unique():
        for belief in exps["full belief"].unique():
            t = exps
            t = t[t["scenario"] == scenario]
            t = t[t["full belief"] == belief]
            # t = t[t["belief"] == "MoE"]

            if(len(t[t["strategy"] == "CM 1"]["policy cost"].values) == 0):
                continue
            
            
            cm_cost = t[t["strategy"] == "CM 1"]["policy cost"].values[0]
            mcbe_cost = t[t["strategy"] == "MCBE"]["policy cost"].values[0]
            cm_time = t[t["strategy"] == "CM 1"]["planning time"].values[0]
            mcbe_time = t[t["strategy"] == "MCBE"]["planning time"].values[0]

            total += 1
            if cm_cost <= mcbe_cost:
                cm_better += 1
            # else:
            #     IPython.embed()

            cm_better_fraction.append(cm_cost / mcbe_cost)
            cm_better_time.append(cm_time/mcbe_time)
            cm_planning_time.append(cm_time)
            mcbe_planning_time.append(mcbe_time)
            
    print("CM is better than MCBE " + str(cm_better) + " of " + str(total) + " trials")
    print("CM plannig time averages " + str(np.mean(cm_planning_time)))
    print("MCBE plannig time averages " + str(np.mean(mcbe_planning_time)))
    IPython.embed()


def MPFvsCHS(exps):

    total = 0
    mpf_easy_better = 0
    mpf_hard_better = 0
    for scenario in exps["scenario"].unique():
        for strategy in exps["strategy"].unique():
            t = exps
            t = t[t["scenario"] == scenario]
            t = t[t["strategy"] == strategy]

            if len(t[t["belief"] == "CHS"]["policy cost"].values) == 0:
                continue

            chs_cost = t[t["belief"] == "CHS"]["policy cost"].values[0]
            mpf_easy_cost = t[t["full belief"] == "MPF Easy"]["policy cost"].values[0]
            if len(t[t["full belief"] == "MPF Hard"]["policy cost"].values) == 0:
                mpf_hard_cost = 9999999
            else:
                mpf_hard_cost = t[t["full belief"] == "MPF Hard"]["policy cost"].values[0]
            if mpf_hard_cost < 0:
                mpf_hard_cost = 9999999

            total += 1
            if mpf_easy_cost < chs_cost:
                mpf_easy_better += 1
            if mpf_hard_cost < chs_cost:
                mpf_hard_better += 1
                IPython.embed()
    print "MPF Easy is better in ", str(mpf_easy_better), " of ", str(total)
    print "MPF Hard is better in ", str(mpf_hard_better), " of ", str(total)
            
def analyze(exps):

    ranks = {"MoE":{1:0, 2:0, 3:0},
             "CHS":{1:0, 2:0, 3:0},
             "MPF":{1:0, 2:0, 3:0},
    }
    
    for scenario in exps["scenario"].unique():
        # for strategy in exps["strategy"].unique():
            for hardness in ["Easy", "Med", "Hard"]:
                t = exps
                t = t[t["scenario"] == scenario]
                t = t[t["strategy"] == "CM 1"]
                g = pd.concat([t[t["hardness"] == "All"], t[t["hardness"] == hardness]])

                ranked = g.sort_values("policy cost")["belief"].values
                ranked = removeDuplicates(ranked)
                if len(ranked) > 3:
                    IPython.embed()
                for i in range(len(ranked)):
                    print ranked[i]
                    ranks[ranked[i]][i+1] += 1
                
    IPython.embed()
                # tchs = 
                # trial = exps[exps

        

def load_all_files():
    sim_path = rospkg.RosPack().get_path('gpu_voxel_planning') + sim_experiment_dir
    real_path = rospkg.RosPack().get_path('gpu_voxel_planning') + real_experiment_dir
    save_path = rospkg.RosPack().get_path('gpu_voxel_planning') + "/figures"
    experiments = []
    for name in os.listdir(sim_path):
        if name.endswith(".png") or\
           name.endswith(".pdf") or\
           name.endswith(".tex"):
           continue;
        experiments.append(load_file(sim_path, name))
    for name in os.listdir(real_path):
        if name.endswith(".png") or\
           name.endswith(".pdf") or\
           name.endswith(".tex"):
           continue;
        experiments.append(load_file(real_path, name))


    experiments = make_dataframe(experiments)

    # plot(experiments, save_path)
    # analyze(experiments)
    # CMvsOFU(experiments)
    CMvsMCBE(experiments)
                                  
    # MPFvsCHS(experiments)


if __name__=='__main__':
    rospy.init_node("plot_experiments")
    load_all_files();
