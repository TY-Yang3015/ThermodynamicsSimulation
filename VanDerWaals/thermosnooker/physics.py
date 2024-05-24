"""Physics functions."""
import numpy as np


def maxwell(speed: np.ndarray | float,
            kbt: float,
            mass: float = 1.) -> np.ndarray | float:
    """
    Given the Maxwell-Boltzmann speed distribution PDF at a given
    temperature and speed.

    Args:
        speed (np.ndarray | float): Speed of particles
        kbt (float): Boltzmann constant times temperature
        mass (float): mass of the particle

    Returns:
        np.ndarray | float: Maxwell-Boltzmann distribution at given v and T.

    """

    pdf = np.exp(- mass * np.square(speed) / (2 * kbt))
    pdf *= 4 * np.pi * np.square(speed)
    pdf *= (mass / (2 * np.pi * kbt)) ** 1.5

    return pdf
