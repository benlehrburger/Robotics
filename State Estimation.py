# Ben Lehrburger
# COSC 081 SA5

import numpy as np

# Probabilistic constants
NO_MOVE = 0.25
MOVE_ONE_FORWARD = 0.50
MOVE_TWO_FORWARD = 0.25
WRONG_MOVE = 0.00
STAY_LAST_CELL = 1.00
SECOND_TO_LAST_STAY = 0.25
SECOND_TO_LAST_ONE_FORWARD = 0.75

# Wrap a Bayes filter object
class BayesFilter:

    def __init__(self, belief, forward=None, backward=None):

        # Starting belief state
        self.belief = belief
        # Forward commands
        self.forwards = forward
        # Backward commands
        self.backwards = backward

    # Estimate the belief state after moving the robot forwards
    def forward(self, belief):

        # Get all locations
        locations = belief.shape[0]
        # Initialize the prime
        belief_prime = np.zeros(locations)

        # For each possible move
        for move in range(locations):

            # Save belief state for no move
            belief_none = belief[move]

            # Save belief state for one move forward
            if move >= 1:
                belief_one = belief[move - 1]
            else:
                belief_one = 0

            # Save belief state for two moves forward
            if move >= 2:
                belief_two = belief[move - 2]
            else:
                belief_two = 0

            # If the robot is anywhere but the last cell
            if move < locations - 1:
                # Calculate its probabilistic location
                belief_prime[
                    move] = NO_MOVE * belief_none + MOVE_ONE_FORWARD * belief_one + MOVE_TWO_FORWARD * belief_two

            # If the robot is in the last cell
            elif move == locations - 1:
                # Calculate its probabilistic location
                belief_prime[
                    move] = STAY_LAST_CELL * belief_none + SECOND_TO_LAST_ONE_FORWARD * belief_one + SECOND_TO_LAST_STAY * belief_two

        return belief_prime

    # Estimate the belief state after moving the robot backwards
    def backward(self, belief):

        # Get all locations
        locations = belief.shape[0]
        # Initialize the prime
        belief_prime = np.zeros(locations)

        # For each possible move
        for move in range(locations):

            # Save belief state for no move
            belief_none = belief[move]

            # Save belief state for one move forward
            if move < locations - 1:
                belief_one = belief[move + 1]
            else:
                belief_one = 0

            # Save belief state for two moves forward
            if move < locations - 2:
                belief_two = belief[move + 2]
            else:
                belief_two = 0

            # If the robot is anywhere but the first cell
            if move > 0:
                # Calculate its probabilistic location
                belief_prime[
                    move] = NO_MOVE * belief_none + MOVE_ONE_FORWARD * belief_one + MOVE_TWO_FORWARD * belief_two

            # If the robot is in the first cell
            elif move == 0:
                # Calculate its probabilistic location
                belief_prime[
                    move] = STAY_LAST_CELL * belief_none + SECOND_TO_LAST_ONE_FORWARD * belief_one + SECOND_TO_LAST_STAY * belief_two

        return belief_prime

    # Compute the probabilistic location using a Bayes filter
    def bayes_filter(self, belief, command):

        # Calculate probabilistic location if command is forwards
        if command == 1:
            return self.forward(belief)

        # Calculate probabilistic location if command is backwards
        if command == -1:
            return self.backward(belief)

    # Execute the filter for an arbitrary number of forwards and backwards commands
    def execute(self):

        # For each forward command
        for move in range(1, self.forwards + 1):
            # Update the belief state
            self.belief = self.bayes_filter(self.belief, 1)
            print("\nThe belief state after " + str(move) + " forward moves is:\n" + str(self.belief))

        # For each backward command
        for move in range(1, self.backwards + 1):
            # Update the belief state
            self.belief = self.bayes_filter(self.belief, -1)
            print("\nThe belief state after " + str(move) + " backward moves is:\n" + str(self.belief))

        print("\nThe final belief state of the robot after " + str(self.forwards) + " moves forward and " + str(
            self.backwards) + " moves backward is:\n" + str(self.belief))

# Starting belief state
belief = np.array([.1, .1, .1, .1, .1, .1, .1, .1, .1, .1])
# Forward moves
forward = 3
# Backward moves
backward = 2

print('The initial belief state is:\n' + str(belief))

discrete_bayes_filter = BayesFilter(belief, forward, backward)
discrete_bayes_filter.execute()

