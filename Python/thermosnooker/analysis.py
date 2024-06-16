"""Analysis Module."""
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
from thermosnooker.balls import Ball, Container
from thermosnooker.simulations import (SingleBallSimulation, MultiBallSimulation)
from thermosnooker.physics import maxwell
from tqdm.auto import tqdm

sns.set_theme()
sns.set_context("paper")
sns.set(rc={"xtick.bottom": True, "ytick.left": True})
palette = sns.color_palette('pastel')


def task9():
    """
    Task 9.

    In this function, you should test your animation. To do this, create a container
    and ball as directed in the project brief. Create a SingleBallSimulation object from these
    and try running your animation. Ensure that this function returns the balls final position and
    velocity.

    Returns:
        tuple[NDArray[np.float64], NDArray[np.float64]]: The balls final position and velocity
    """
    container = Container(radius=10.)
    ball = Ball(pos=[-5, 0], vel=[1, 0.], radius=1., mass=1.)
    sbs = SingleBallSimulation(container=container, ball=ball)
    sbs.run(10, True, 0.1)

    return sbs.ball().pos(), sbs.ball().vel()


def task10():
    """
    Task 10.

    In this function we shall test your MultiBallSimulation. Create an instance of this class using
    the default values described in the project brief and run the animation for 500 collisions.

    Watch the resulting animation carefully and make sure you aren't seeing errors like balls sticking
    together or escaping the container.
    """
    sim = MultiBallSimulation()
    sim.run(100, True, 0.1)


def task11():
    """
    Task 11.

    In this function we shall be quantitatively checking that the balls aren't escaping or sticking.
    To do this, create the two histograms as directed in the project script. Ensure that these two
    histogram figures are returned.

    Returns:
        tuple[Figure, Firgure]: The histograms (distance from centre, inter-ball spacing).
    """
    sim = MultiBallSimulation()
    sim.run(1000, False)

    positions = np.array([ball.pos() for ball in sim.balls()])
    distance_from_centre = np.linalg.norm(positions - sim.container().pos(), axis=-1)

    inter_ball = [sim.balls()[i].pos() - sim.balls()[j].pos()
                  for i in range(len(sim.balls())) for j in range(len(sim.balls()))]
    inter_ball = np.linalg.norm(inter_ball, axis=-1)
    inter_ball_nonzero = inter_ball[inter_ball != 0]

    fig1, ax1 = plt.subplots()
    ax1.hist(distance_from_centre, bins=50,
             edgecolor='black', linewidth=0.5)
    ax1.set_title('Distance Distribution from Container Center')
    ax1.set_xlabel('Distance from Container Center')

    fig2, ax2 = plt.subplots()
    ax2.hist(inter_ball_nonzero, bins=50,
             edgecolor='black', linewidth=0.5)
    ax2.set_title('Inter-ball Distance Distribution')
    ax2.set_xlabel('Inter-ball Distance')

    return fig1, fig2


def task12():
    """
    Task 12.

    In this function we shall check that the fundamental quantities of energy and momentum are conserved.
    Additionally we shall investigate the pressure evolution of the system. Ensure that the 4 figures
    outlined in the project script are returned.

    Returns:
        tuple[Figure, Figure, Figure, Figure]: matplotlib Figures of the KE, momentum_x, momentum_y ratios
                                               as well as pressure evolution.
    """
    sim = MultiBallSimulation()
    num_collision = 4000

    ke_t = np.zeros(num_collision + 1)
    ke_t[0] = sim.kinetic_energy()

    p_t = np.zeros((num_collision + 1, 2))
    p_t[0] = sim.momentum()

    pressure_t = np.zeros(num_collision + 1)
    pressure_t[0] = 0

    times = np.zeros(num_collision + 1)

    pbar = tqdm(total=num_collision)
    for i in range(num_collision):
        sim.next_collision()

        ke_t[i + 1] = sim.kinetic_energy()
        p_t[i + 1] = sim.momentum()
        pressure_t[i + 1] = sim.pressure()
        times[i + 1] = sim.time()

        pbar.update(1)

    fig1, ax1 = plt.subplots()
    ax1.plot(times, ke_t / ke_t[0])
    ax1.set_title('Total KE Evolution')
    ax1.set_xlabel('Time')
    ax1.set_ylabel('System Total Kinetic Energy')

    fig2, ax2 = plt.subplots()
    ax2.plot(times, p_t[:, 0] / p_t[0, 0])
    ax2.set_title('Momentum x Component Evolution')
    ax2.set_xlabel('Time')
    ax2.set_ylabel(r'System Momentum $x$ Component')

    fig3, ax3 = plt.subplots()
    ax3.plot(times, p_t[:, 1] / p_t[0, 1])
    ax3.set_title('Momentum y Component Evolution')
    ax3.set_xlabel('Time')
    ax3.set_ylabel(r'System Momentum $y$ Component')

    fig4, ax4 = plt.subplots()
    ax4.plot(times, pressure_t)
    ax4.set_title('Pressure Evolution')
    ax4.set_xlabel('Time')
    ax4.set_ylabel('System Pressure')

    return fig1, fig2, fig3, fig4


def task13():
    """
    Task 13.

    In this function we investigate how well our simulation reproduces the distributions of the IGL.
    Create the 3 figures directed by the project script, namely:
    1) PT plot
    2) PV plot
    3) PN plot
    Ensure that this function returns the three matplotlib figures.

    Returns:
        tuple[Figure, Figure, Figure]: The 3 requested figures: (PT, PV, PN)
    """
    k_b = 1.380649e-23
    num_of_sim = 100
    velocities = np.linspace(0.1, 300., num_of_sim)
    sim_pressures = np.zeros(num_of_sim)
    idl_pressures = np.zeros(num_of_sim)
    temperatures = np.zeros(num_of_sim)
    for i, velocity in enumerate(velocities):
        print(1, i)
        sim = MultiBallSimulation(b_speed=velocity)
        sim.run(1000, False)
        sim_pressures[i] = sim.pressure()
        temperatures[i] = sim.t_equipartition()

        idl_pressures[i] = len(sim.balls()) * k_b * sim.t_equipartition()
        idl_pressures[i] /= sim.container().volume()

    fig1, ax1 = plt.subplots()
    ax1.plot(temperatures, sim_pressures, label='Simulation')
    ax1.plot(temperatures, idl_pressures, label='Ideal')
    ax1.set_xlabel('T')
    ax1.set_ylabel('P')
    ax1.legend()

    volumes = np.linspace(10., 20., num_of_sim)
    sim_pressures = np.zeros(num_of_sim)
    idl_pressures = np.zeros(num_of_sim)
    temperatures = np.zeros(num_of_sim)
    for i, radius in enumerate(volumes):
        print(2, i)
        sim = MultiBallSimulation(c_radius=radius)
        sim.run(1000, False)
        sim_pressures[i] = sim.pressure()
        temperatures[i] = sim.t_equipartition()

        idl_pressures[i] = len(sim.balls()) * k_b * sim.t_equipartition()
        idl_pressures[i] /= sim.container().volume()

    fig2, ax2 = plt.subplots()
    ax2.plot(volumes ** 2 * np.pi, sim_pressures, label='Simulation')
    ax2.plot(volumes ** 2 * np.pi, idl_pressures, label='Ideal')
    ax2.set_xlabel('V')
    ax2.set_ylabel('P')
    ax2.legend()

    sim = MultiBallSimulation()
    num_of_sim = len(sim.balls())
    number = np.arange(0, num_of_sim, 1).astype(np.int64)
    number += 1
    sim_pressures = np.zeros(num_of_sim)
    idl_pressures = np.zeros(num_of_sim)
    temperatures = np.zeros(num_of_sim)
    for i, number_i in enumerate(number):
        sim = MultiBallSimulation()
        sim._ball_list = sim.balls()[:number_i]
        print(3, number_i, len(sim.balls()))
        sim.run(100, False)
        sim_pressures[i] = sim.pressure()
        temperatures[i] = sim.t_equipartition()

        idl_pressures[i] = number_i * k_b * sim.t_equipartition()
        idl_pressures[i] /= sim.container().volume()

    fig3, ax3 = plt.subplots()
    n_plot = np.insert(number, 0, 0)
    sim_p_plot = np.insert(sim_pressures, 0, 0)
    idl_p_plot = np.insert(idl_pressures, 0, 0)
    ax3.plot(n_plot, sim_p_plot, label='Simulation')
    ax3.plot(n_plot, idl_p_plot, label='Ideal')
    ax3.set_xlabel('N')
    ax3.set_ylabel('P')
    ax3.legend()

    return fig1, fig2, fig3


def task14():
    """
    Task 14.

    In this function we shall be looking at the divergence of our simulation from the IGL. We shall
    quantify the ball radii dependence of this divergence by plotting the temperature ratio defined in
    the project brief.

    Returns:
        Figure: The temperature ratio figure.
    """

    radii = np.linspace(0.01, 1., 1000)
    ratio = np.zeros(radii.shape)
    for i, radius in enumerate(radii):
        print(i)
        sim = MultiBallSimulation(b_radius=radius)
        sim.run(100, False)
        ratio[i] = sim.t_equipartition() / sim.t_ideal()

    fig, axis = plt.subplots()
    axis.plot(radii, ratio, label=r'$T_\text{eq}/T_\text{ideal}$')
    axis.set_xlabel(r'Ball Radius')
    axis.set_ylabel(r'')
    axis.legend()

    return fig


def task15():
    """
    Task 15.

    In this function we shall plot a histogram to investigate how the speeds of the balls evolve from the initial
    value. We shall then compare this to the Maxwell-Boltzmann distribution. Ensure that this function returns
    the created histogram.

    Returns:
        Figure: The speed histogram.
    """
    k_b = 1.38064852e-23
    #sim = MultiBallSimulation(c_radius=10., b_radius=0.001, nrings=3, multi=5)
    sim = MultiBallSimulation()
    sim.run(2000, False)

    fig, axis = plt.subplots()
    bins = 40
    axis.hist(sim.speeds(), density=True, bins=bins, label=r'Simulation')
    speeds = np.linspace(0, np.max(sim.speeds()))
    m_b = maxwell(speeds, k_b * sim.t_equipartition())
    m_b *= np.histogram(sim.speeds(), density=True, bins=bins)[0].max() / np.max(m_b)
    axis.plot(speeds, m_b
              , label=r'Ideal M-Boltzmann distribution', color='red')
    axis.legend(loc='upper right')

    return fig


if __name__ == "__main__":
    # Run task 9 function
    # BALL_POS, BALL_VEL = task9()

    # Run task 10 function
    # task10()

    # Run task 11 function
    # FIG11_BALLCENTRE, FIG11_INTERBALL = task11()

    # Run task 12 function
    # FIG12_KE, FIG12_MOMX, FIG12_MOMY, FIG12_PT = task12()
    # FIG12_KE.savefig('FIG12_KE.png', dpi=100)
    # FIG12_MOMX.savefig('FIG12_MOMX.png', dpi=100)
    # FIG12_MOMY.savefig('FIG12_MOMY.png', dpi=100)
    # FIG12_PT.savefig('FIG12_PT.png', dpi=100)

    # Run task 13 function
    #FIG13_PT, FIG13_PV, FIG13_PN = task13()
    #FIG13_PT.savefig('fig13_pt.png')
    #FIG13_PV.savefig('fig13_pv.png')
    #FIG13_PN.savefig('fig13_pn.png')

    # Run task 14 function
    #FIG14 = task14()
    #FIG14.savefig('fig14.png')

    # Run task 15 function
    FIG15 = task15()
    FIG15.savefig('fig15.png')



