#! /usr/bin/python

import numpy as np
import itertools
import math
from scipy.special import binom as binomial

g_chss = [
    # "c...",
    # ".c..",
    # "..c.",
    # "...c",
    "cccc",
    # ".cc.",
    # " ..."
]

p_cell_occ = 0.1
world_size = 4


def calc_world_probability(world):
    p = 1.0
    for cell in world:
        if cell == "x":
            p *= p_cell_occ
        else:
            p *= (1-p_cell_occ)
    return p






all_worlds = [''.join(i) for i in itertools.product([' ','x'], repeat = world_size)]

prior_probabilities = {world: calc_world_probability(world) for world in all_worlds}



def is_valid_world(world, collision_sets):
    for collision_set in collision_sets:
        if not is_valid_world_for_single_collision_set(world, collision_set):
            return False
    return True

def is_valid_world_for_single_collision_set(world, col_set):
    # print "Checking validity of ", world, "for colset ", col_set
    col_set_satisfied = True
    
    for col_set_letter, world_letter in zip(col_set, world):
        
        if col_set_letter == "c":
            col_set_satisfied = False
            if world_letter == "x":
                return True

        if col_set_letter == " " and world_letter == "x":
            return False

    return col_set_satisfied


def select_valid_worlds(worlds, col_sets):
    return [world for world in worlds if is_valid_world(world, col_sets)]

def consistent_probability(worlds, probs, col_sets):
    v_worlds = select_valid_worlds(worlds, col_sets)
    return np.sum([probs[world] for world in v_worlds])

def renormalize_probabilities(worlds, probs):
    tot = np.sum([probs[world] for world in worlds])
    valid_probs = {world: probs[world]/tot for world in worlds}
    return valid_probs
    

def apply_collision_sets(worlds, probs, col_sets):
    valid_worlds = select_valid_worlds(worlds, col_sets)
    valid_probs = renormalize_probabilities(valid_worlds, probs)
    return valid_worlds, valid_probs

def calc_entropy(worlds, probs):
    H = -np.sum([probs[world] * math.log(probs[world], 2) for world in worlds])
    return H

def no_col_set():
    p_6_empty = (1-p_cell_occ)**6
    print "p 6 empty: ", p_6_empty

def print_worlds(worlds, probabilities = None):
    for world in worlds:
        p = ""
        if probabilities is not None:
            p = "p = " + str(probabilities[world])
        print "".join(["|", world, "|"]), p
    print


def print_spacer():
    print ""
    print "========================"
    print ""


def print_all_worlds():
    print "All worlds"
    print_worlds(all_worlds, prior_probabilities)
    print "Entropy: ", calc_entropy(all_worlds, prior_probabilities)
    print_spacer()

def print_consistent_worlds():
    print "All worlds with collision sets"
    v_worlds, v_probs = apply_collision_sets(all_worlds, prior_probabilities, g_chss)
    print_worlds(v_worlds, v_probs)

    print "Probability: ", consistent_probability(all_worlds, prior_probabilities, g_chss)
    print "Entropy: ", calc_entropy(v_worlds, v_probs)
    print_spacer()


def calc_entropy_tmp():
    H = 0
    n = world_size
    for k in range(n+1):
        p = p_cell_occ**k * (1-p_cell_occ)**(n-k)
        H -= binomial(n,k) * p * math.log(p, 2)
    return H

def calc_prob_fast(p_occ, prev_chss, new_chs):
    chs_size = new_chs.count("c")
    return 1-(1-p_occ)**chs_size


if __name__ == "__main__":
    print_all_worlds()
    print_consistent_worlds()
    calc_prob_fast(p_cell_occ, None, g_chss)
    print "Fast prob calc", calc_prob_fast(p_cell_occ, None, g_chss[0])
    print calc_entropy_tmp()
        


