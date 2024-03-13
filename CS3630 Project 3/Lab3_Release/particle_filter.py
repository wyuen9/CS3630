from grid import *
from particle import Particle
from utils import *
import setting
import numpy as np
np.random.seed(setting.RANDOM_SEED)
from itertools import product

# Name: Winnie Yuen

def create_random(count, grid):
    """
    Returns a list of <count> random Particles in free space.
    Parameters:
        count: int, the number of random particles to create
        grid: a Grid, passed in to motion_update/measurement_update
    Returns:
        List of Particles with random coordinates in the grid's free space.
    """

    # TODO: implement here
    # -------------------
    particles = []
    while len(particles) < count:
        x, y = grid.random_free_place()
        particles.append(Particle(x,y))
    # -------------------
    return particles
# ------------------------------------------------------------------------

def motion_update(old_particles, odometry_measurement, grid):

    """
    Implements the motion update step in a particle filter. Refer setting.py and utils.py for required functions and noise parameters.
    NOTE: the GUI will crash if you have not implemented this method yet. To get around this, try setting new_particles = old_particles.
    Arguments:
        old_particles: List 
            list of Particles representing the belief before motion update p(x_{t-1} | u_{t-1}) in *global coordinate frame*
        odometry_measurement: Tuple
            noisy estimate of how the robot has moved since last step, (dx, dy, dh) in *local robot coordinate frame*
    Returns: 
        a list of NEW particles representing belief after motion update \tilde{p}(x_{t} | u_{t})

    """
    new_particles = []
    for particle in old_particles:
        # extract the x/y/heading from the particle
        x_g, y_g, h_g = particle.xyh

        # and the change in x/y/heading from the odometry measurement
        dx_r, dy_r, dh_r = odometry_measurement
        new_particle = None

        # TODO: implement here
        # ----------------------------------
        # align odometry_measurement's robot frame coords with particle's global frame coords (heading already aligned)
        noisy_dx = add_gaussian_noise(dx_r, setting.ODOM_TRANS_SIGMA)
        noisy_dy = add_gaussian_noise(dy_r, setting.ODOM_TRANS_SIGMA)
        noisy_dh = add_gaussian_noise(dh_r, setting.ODOM_HEAD_SIGMA)

        # compute estimated new coordinate, using current pose and odometry measurements. Make sure to add noise to simulate the uncertainty in the robot's movement.

        dx_g, dy_g = rotate_point(noisy_dx, noisy_dy, particle.h)
        new_x = x_g + dx_g
        new_y = y_g + dy_g
        new_h = (h_g + noisy_dh) % 360

        # create a new particle with this noisy coordinate
        new_particles.append(Particle(new_x, new_y, new_h))

        # ----------------------------------
    return new_particles
# ------------------------------------------------------------------------

def generate_marker_pairs(robot_marker_list, particle_marker_list):

    """ Pair markers in order of closest distance

        Arguments:

        robot_marker_list -- List of markers observed by the robot
        particle_marker_list -- List of markers observed by the particle
        Returns: List[Tuple] of paired robot and particle markers

    """
    marker_pairs = []

    while len(robot_marker_list) > 0 and len(particle_marker_list) > 0:

        # TODO: implement here
        # ---------------------------------
        # find the (particle marker,robot marker) pair with shortest grid distance
        for r_marker in robot_marker_list:
            closest_particle_marker = min(particle_marker_list, key=lambda p_marker: grid_distance(r_marker[0], r_marker[1], p_marker[0], p_marker[1]))

        # add this pair to marker_pairs and remove markers from corresponding lists

            marker_pairs.append((r_marker, closest_particle_marker))
            particle_marker_list.remove(closest_particle_marker)
            robot_marker_list.remove(r_marker)
        # ----------------------------------

    return marker_pairs



def gaussian_pdf(distDiff, headingDiff, transSig, headSig):
    dist = (distDiff**2) / (2*transSig**2)
    head = (headingDiff**2) / (2*headSig**2)
    return np.exp(-(dist + head))
# ------------------------------------------------------------------------

def marker_likelihood(robot_marker, particle_marker):

    """ Calculate likelihood of reading this marker using Gaussian PDF. The 
        standard deviation of the marker translation and heading distributions 
        can be found in settings.py  
        Arguments:
        robot_marker -- Tuple (x,y,theta) of robot marker pose
        particle_marker -- Tuple (x,y,theta) of particle marker pose
        Returns: float probability

    """

    l = 0.0
    # TODO: implement here
    # ----------------------------------
    # find the distance between the particle marker and robot marker

    distDiff = grid_distance(robot_marker[0], robot_marker[1], particle_marker[0], particle_marker[1])
    # find the difference in heading between the particle marker and robot marker

    headingDiff = diff_heading_deg(robot_marker[2], particle_marker[2])

    # calculate the likelihood of this marker using the gaussian pdf
    
    l = gaussian_pdf(distDiff, headingDiff, setting.MARKER_TRANS_SIGMA, setting.MARKER_HEAD_SIGMA)
    # ----------------------------------
    return l
# ------------------------------------------------------------------------

def particle_likelihood(robot_marker_list, particle_marker_list):

    """ Calculate likelihood of the particle pose being the robot's pose
        Arguments:
        robot_marker_list -- List of markers observed by the robot
        particle_marker_list -- List of markers observed by the particle
        Returns: float probability

    """

    l = 1.0
    marker_pairs = generate_marker_pairs(robot_marker_list, particle_marker_list)
    # TODO: implement here
    # ----------------------------------
    # update the particle likelihood using the likelihood of each marker pair
    # HINT: consider what the likelihood should be if there are no pairs generated
    if len(marker_pairs) == 0:
        return 0
    for robot_marker, particle_marker in marker_pairs:
        l *= marker_likelihood(robot_marker, particle_marker)
    # ----------------------------------
    return l
# ------------------------------------------------------------------------

def measurement_update(particles, measured_marker_list, grid):

    """ Particle filter measurement update
        Arguments:
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before measurement update (but after motion update)
        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information,
                see grid.py and CozGrid for definition
                Can be used to evaluate particles
        Returns: the list of particles represents belief p(x_{t} | u_{t})

                after measurement update

    """
    measured_particles = []
    particle_weights = []
    num_rand_particles = 25

    if len(measured_marker_list) > 0:
        for p in particles:
            x, y = p.xy
            if grid.is_in(x, y) and grid.is_free(x, y):
                robot_marker_list = measured_marker_list.copy()
                particle_marker_list =  p.read_markers(grid)
                l = None

                # TODO: implement here
                # ----------------------------------
                # compute the likelihood of the particle pose being the robot's
                # pose when the particle is in a free space
                l = particle_likelihood(robot_marker_list, particle_marker_list)
                # ----------------------------------
            else:
                # TODO: implement here
                # ----------------------------------
                # compute the likelihood of the particle pose being the robot's pose
                # when the particle is NOT in a free space
                l = 0.0
                # ----------------------------------
            particle_weights.append(l)

    else:
        particle_weights = [1.]*len(particles)
    # TODO: Importance Resampling
    # ----------------------------------
    # if the particle weights are all 0, generate a new list of random particles

    total_weight = sum(particle_weights)

    if total_weight == 0:
        measured_particles = create_random(len(particles), grid)
    # normalize the particle weights
    else:
        normalized_weights = [w / total_weight for w in particle_weights]

    # create a fixed number of random particles and add to measured particles
        random_particles = create_random(num_rand_particles, grid)
        measured_particles = measured_particles + random_particles

    # resample remaining particles using the computed particle weights
        resample_particle = np.random.choice(particles,len(particles) - num_rand_particles,True, normalized_weights)
        measured_particles = resample_particle
    # ----------------------------------
    return measured_particles