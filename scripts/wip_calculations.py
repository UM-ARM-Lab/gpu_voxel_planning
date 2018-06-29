#! /usr/bin/python

import numpy as np
import itertools

collision_sets = ["xx ",
                  " xx"]
p_cell_occ = 0.1
world_size = 3


def calc_world_probability(world):
    p = 1.0
    for cell in world:
        if cell == "x":
            p *= p_cell_occ
        else:
            p *= (1-p_cell_occ)
    return p






all_worlds = [''.join(i) for i in itertools.product([' ','x'], repeat = 3)]

prior_probabilities = {world: calc_world_probability(world) for world in all_worlds}



def is_valid_world(world, collision_sets):
    for collision_set in collision_sets:
        if not is_valid_world_for_single_collision_set(world, collision_set):
            return False
    return True

def is_valid_world_for_single_collision_set(world, col_set):
    # print "Checking validity of ", world
    for col_set_letter, world_letter in zip(col_set, world):
        
        if col_set_letter == "x" and world_letter == "x":
            return True
    return False


def select_valid_worlds(worlds):
    return [world for world in worlds if is_valid_world(world, collision_sets)]


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



if __name__ == "__main__":
    # no_col_set()
    print_worlds(select_valid_worlds(all_worlds))
    print_worlds(all_worlds, prior_probabilities)


