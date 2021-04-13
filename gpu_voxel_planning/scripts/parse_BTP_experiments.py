#!/usr/bin/env python

import os

from typing import List
from matplotlib import pyplot as plt

import rospkg
import pandas as pd
from collections import OrderedDict
import numpy as np
import seaborn as sns
from pathlib import Path

# Real
# experiment_dir = "/experiments_real/"
# scenarios_to_parse = ["RealTable", "Refrigerator"]

# Sim
experiment_dir = "experiments"
scenarios_to_parse = ["Box", "Bookshelf"]

# Output
OUTDIR = "paper_figures"

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
    if sb == "CHS":
        return "CHS"
    if sb.startswith("Particles"):
        return "MPF"
    if sb.startswith("MoE"):
        return "MoE"


def is_selected_strat(strat):
    ss = short_strategy[strat]
    return True
    if ss in ["OFU", "CM 1", "MCBE"]:
        return True
    return False


fmt = "%Y-%m-%dT%H:%M:%S"


class Experiment:
    def __init__(self):
        self.timestamp = None
        self.scenario = None
        self.strategy = None
        self.belief = None
        self.label = None
        self.exec_cost = None
        self.pareto_weight = None
        self.succeeded = True
        self.planning_time = None
        self.num_collision = None
        self.num_steps = None

    def __str__(self):
        return f"[{self.scenario}, {self.strategy}, {self.belief}]"


class ExperimentGroup:
    def __init__(self, experiments: List[Experiment]):
        if len(experiments) == 0:
            raise RuntimeError("Cannot create an experiment group with 0 experiments")

        self.scenario = experiments[0].scenario
        self.strategy = experiments[0].strategy
        self.belief = experiments[0].belief

        self._validate(experiments)

        self.exec_costs = [e.exec_cost if e.exec_cost is not None else float('inf') for e in experiments]
        self.avg_exec_cost = np.mean(self.exec_costs)
        self.planning_times = [e.planning_time for e in experiments]
        self.avg_planning_time = np.mean(self.planning_times)
        self.succeeded = all(e.succeeded for e in experiments)
        self.label = f"{super_short_belief(self.belief)}+{short_strategy[self.strategy]}"

    def __str__(self):
        return f"[{self.scenario}, {self.strategy}, {self.belief}]"

    def _validate(self, experiments):
        """Confirm all experiments belong to the same group"""
        for exp in experiments:
            if exp.scenario != self.scenario or exp.strategy != self.strategy or exp.belief != self.belief:
                raise RuntimeError(f"Experiment {exp} does not belong in this group {self}")


def find_all(experiments, scenario: str, strategy: str, belief: str):
    """Returns all experiments done using scenario, stratety, belief"""
    return [e for e in experiments
            if e.scenario == scenario
            if e.strategy == strategy
            if e.belief == belief]


def group_experiments(experiments):
    """Groups experiments of the same scenario, strat, belief

    This is needed because multiple trials were conducted for each experiment, and some are across multiple files
    """
    grouped = []
    for exp in experiments:
        if already_in_group := len(find_all(grouped, exp.scenario, exp.strategy, exp.belief)):
            if already_in_group > 1:
                raise RuntimeError(
                    f"Experiment group {exp.scenario}, {exp.strategy}, {exp.belief} appears more than once")
            # print(f"Skipping {exp.scenario}, {exp.strategy}, {exp.belief}. Already evaluated")
            continue

        grouped.append(ExperimentGroup(find_all(experiments, exp.scenario, exp.strategy, exp.belief)))

    return grouped


def average_same_xval(xs, ys):
    if len(xs) != len(ys):
        raise Exception("Can't average when xs and ys have different lengths")

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

    experiments.sort(key=lambda e: e.label)

    series = pd.Series([e.avg_exec_cost * e.succeeded for e in experiments])
    ax = series.plot(kind='bar')

    ax.set_title(short_scenario[experiments[0].scenario])

    x_labels = [e.label for e in experiments]
    ax.set_xticklabels(x_labels)
    plt.tight_layout()
    plt.show()
    ax.get_figure().savefig(save_path + experiments[0].scenario + ".png")


def plot_sorted(all_experiments: List[ExperimentGroup], save_path: Path):
    hardness_map = {"Easy Prior": ["CHS", "Particles Good", "MoE Good"],
                    "Medium Prior": ["CHS", "Particles Noisy", "MoE Noisy"],
                    "Hard Prior": ["CHS", "Particles Bonkers", "MoE Bonkers"]}

    for scenario in get_scenarios(all_experiments):
        for hardness in hardness_map:
            exps = [exp for exp in all_experiments
                    if exp.scenario == scenario
                    if short_belief[exp.belief] in hardness_map[hardness]
                    if is_selected_strat(exp.strategy)]
            bar_plot_by_scenario_and_prior(exps, hardness, save_path)


def bar_plot_by_scenario_and_prior(experiments: List[ExperimentGroup], hardness: str, save_path):
    """Plots a list of experiments all belonging to the same scenario"""

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

    upper_clip_time = 100
    upper_clip_cost = 40

    def custom_sort(exp: ExperimentGroup):
        if exp.label in order:
            return order.index(exp.label)
        return 100

    # experiments.sort(key=lambda e:e.label)
    experiments.sort(key=custom_sort)
    experiments = [e for e in experiments if e.label in order]

    costs = [e.avg_exec_cost * e.succeeded for e in experiments]
    planning_times = [e.avg_planning_time * e.succeeded for e in experiments]

    x_labels, costs = average_same_xval([e.label for e in experiments], costs)
    x_labels, planning_times = average_same_xval([e.label for e in experiments], planning_times)

    # Add baseline experiment, all of which failed
    x_labels = x_labels + ["Baseline [8]"]
    costs = costs + [upper_clip_time] # Add baseline cost
    planning_times = planning_times + [15 * 60] # Add baseline planning time


    hue = [float(label == "MoE+CM 1") for label in x_labels]
    # hue = [c for c in costs]
    clipped_times = [min(p, upper_clip_time) for p in planning_times]
    clipped_policy = [min(p, upper_clip_cost) for p in costs]

    data = pd.DataFrame({"average policy cost (rad)": clipped_policy,
                         "planning time (s)": clipped_times,
                         "method": x_labels,
                         "hue": hue})

    sns.set(style="whitegrid", font_scale=2.2)
    ax = sns.barplot(x="method", y="average policy cost (rad)", data=data, hue="hue", dodge=False)

    ax.legend_.remove()

    ax.set_xticklabels(ax.get_xticklabels(), rotation=90)
    ax.set_title(short_scenario[experiments[0].scenario] + " " + hardness + "\n", fontsize=30)
    # ax.set_xticklabels(x_labels)
    # ax.set_yticks([25, 50])
    ax.set_yticks([20, 40])
    labels = [item.get_text() for item in ax.get_yticklabels()]
    labels[0] = "20"
    labels[1] = ">40"
    if hardness in {"medium", "hard"}:
        ax.yaxis.set_label_text(" ")
        labels[0] = " "
        labels[1] = "   "
    ax.set_yticklabels(labels)
    ax.xaxis.set_label_text("")

    # plt.axvline(x=2.5, linewidth=1, color='k')
    plt.axvline(x=0.5, linewidth=1, color='k')
    plt.tight_layout()
    plt.show()
    filepath = save_path / (short_scenario[experiments[0].scenario] + "_" + hardness.replace(" ", "") + ".png")
    ax.get_figure().savefig(filepath.as_posix())

    ax = sns.barplot(x="method", y="planning time (s)", data=data, hue="hue", dodge=False)

    ax.legend_.remove()

    ax.set_xticklabels(ax.get_xticklabels(), rotation=90)
    ax.set_yticks([50, 100])
    labels = [item.get_text() for item in ax.get_yticklabels()]
    labels[0] = "50"
    labels[1] = ">100"
    if hardness in {"medium" or "hard"}:
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
    filepath = save_path / (short_scenario[experiments[0].scenario] + "_" + hardness.replace(" ", "") + "_times.png")
    ax.get_figure().savefig(filepath.as_posix())


def plot_avg_fig(all_experiments, save_path):
    scenario = "Bookshelf"
    exps = [exp for exp in all_experiments
            if exp.scenario == scenario
            if is_selected_strat(exp.strategy)]
    bar_plot_by_scenario_and_prior(exps, "all", save_path)


# def plot_swarm(all_experiments, save_path):
#     plt.show()
#     ax.get_figure().savefig(save_path + "_swarm.png")


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

    def get_cost(scenario, strategy, bel):
        exp = get_experiment(experiments, scenario, strategy, bel)
        if exp is None:
            return "-"
        if not exp.succeeded:
            return str(float('inf'))
        if exp.avg_exec_cost is None:
            return str(float('inf'))
        return "%5.1f" % exp.avg_exec_cost

    def get_time(scenario, strategy, bel):
        exp = get_experiment(experiments, scenario, strategy, bel)
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
            f.write(get_cost(scenario, strat, bel))
        f.write("\\\\\n")

    def write_cost_table(scenario):
        with (save_path / f"table_{scenario}_cost.tex").open("w") as f:
            f.write("\\begin{table}[]\n")
            f.write("\\centering\n")
            f.write("\\begin{tabular}{|c|" + "c|" * len(beliefs) + "}\n")
            f.write("\\hline\n")
            write_belief_headers(f)
            for strategy in strategies:
                write_cost_line(f, scenario, strategy)
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
            f.write(get_time(scenario, strat, bel))
        f.write("\\\\\n")

    def write_time_table(scenario):
        with (save_path / f"table_{scenario}_timings.tex").open("w") as f:
            f.write("\\begin{table}[]\n")
            f.write("\\centering\n")
            f.write("\\begin{tabular}{|c|" + "c|" * len(beliefs) + "}\n")
            f.write("\\hline\n")
            write_belief_headers(f)
            for strategy in strategies:
                write_time_line(f, scenario, strategy)
            f.write("\\hline\n")
            f.write("\\end{tabular}\n")
            f.write("\\caption{" + scenario + " Planning Times}\n")
            f.write("\\label{tab:experiment_" + scenario + "_time}\n")
            f.write("\\end{table}\n")

    for scenario_to_parse in scenarios_to_parse:
        write_cost_table(scenario_to_parse)
        write_time_table(scenario_to_parse)


def load_file(filepath):
    exp = Experiment()
    with filepath.open() as f:
        exp.timestamp = f.readline()
        while line := f.readline():
            parts = line.split()
            if len(parts) == 0:
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

    print(f'Loaded {filepath.parts[-1]}')
    exp.label = short_belief[exp.belief] + "_" + short_strategy[exp.strategy]
    # IPython.embed()
    return exp


def root_dir():
    return Path(rospkg.RosPack().get_path('gpu_voxel_planning'))


def load_all_files():
    path = root_dir() / experiment_dir
    save_dir = root_dir() / OUTDIR
    if not save_dir.exists():
        save_dir.mkdir()

    experiments = group_experiments([load_file(fp) for fp in path.glob('*')])


    # plot_all_data_for_scenarios(experiments, path)

    # - this is the one currently in the paper
    plot_sorted(experiments, save_dir)

    # plot_avg_fig(experiments, path)
    write_latex(experiments, save_dir)


if __name__ == '__main__':
    # rospy.init_node("plot_experiments")
    load_all_files()
