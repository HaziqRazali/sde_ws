import numpy as np

class WeightedMovingFilter:
    """ Applies a weighted moving average filter to a sequence of data points. """
    def __init__(self, weights, size):
        """
        Initializes the filter.

        Args:
            weights: A numpy array of weights for the moving average.
            data_size: The dimension of the data vectors being filtered.
        """
        self.weights = np.array(weights)
        self.size = size
        self.data_history = []

    def add_data(self, new_data):
        """
        Adds a new data point to the history, discarding the oldest if full.

        Args:
            new_data: The new data vector to add.
        """
        if len(self.data_history) == len(self.weights):
            self.data_history.pop(0)
        self.data_history.append(new_data)

    @property
    def filtered_data(self):
        """
        Computes the filtered data based on the current history.

        Returns:
            The weighted average of the data in the history. Returns a zero
            vector if the history is empty.
        """
        if not self.data_history:
            return np.zeros(self.size)
        
        num_data_points = len(self.data_history)
        active_weights = self.weights[:num_data_points]
        active_weights = active_weights / np.sum(active_weights)

        weighted_sum = np.zeros(self.size)
        for i, data_point in enumerate(reversed(self.data_history)):
            if i < len(active_weights):
                weighted_sum += active_weights[i] * data_point
        return weighted_sum

