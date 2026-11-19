"""Measured dimensions and approximate sphere geometry for one-ball grasping."""
import math
import numpy as np
from contact_path import unit


def check_dimensions(diameter, opening, insertion, compression):
    if not all(math.isfinite(v) for v in (diameter, opening, insertion, compression)):
        raise ValueError('Dimensions must be finite')
    if not 0 < diameter <= opening <= .15:
        raise ValueError('Ball must fit inside the measured maximum opening')
    if not 0 < insertion <= .05 or not 0 < compression <= min(.005, diameter / 4):
        raise ValueError('Insertion must be 0..50 mm; compression 0..5 mm and less than a quarter diameter')
    return opening - diameter


def finger_position(opening, measured_maximum, model_half_maximum):
    if not all(math.isfinite(v) for v in (opening, measured_maximum, model_half_maximum)):
        raise ValueError('Invalid opening')
    if not 0 <= opening <= measured_maximum or measured_maximum <= 0 or model_half_maximum <= 0:
        raise ValueError('Opening outside measured range')
    # First approximation only: physical width versus driver opening calibration.
    return model_half_maximum * opening / measured_maximum


def grasp_targets(surface, camera_origin, forward, diameter, clearance, insertion):
    surface, origin = np.asarray(surface, dtype=float), np.asarray(camera_origin, dtype=float)
    if surface.shape != (3,) or origin.shape != (3,) or not np.all(np.isfinite(np.r_[surface,origin])):
        raise ValueError('Invalid observed point or camera origin')
    if not all(math.isfinite(v) and v > 0 for v in (diameter,clearance,insertion)):
        raise ValueError('Invalid target dimensions')
    # Assumes the observed ROI is near the front-center of a sphere.
    center = surface + unit(surface-origin) * diameter/2
    forward = unit(forward)
    pre = center - forward*(diameter/2 + clearance)
    tip = center + forward*insertion
    return center, pre, tip
