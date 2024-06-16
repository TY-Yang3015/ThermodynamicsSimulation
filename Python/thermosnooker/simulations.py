"""Simulation Classes."""
import matplotlib.pyplot as plt
# import matplotlib
import numpy as np
from thermosnooker.balls import Ball, Container
from tqdm.auto import tqdm

# matplotlib.use('TkAgg')


class Simulation:
    """
    This is the simulation bass class. Only the run() method is implemented.

    Attributes:
        no public attribute.

    Args:
        N/A

    Examples:
        N/A

    """

    def next_collision(self) -> None:
        """
        Function to simulate next collision. Not implemented.

        Args:
            N/A

        Returns:
            None

        Raises:
            NotImplementedError: Base class should not be called directly.

        """
        raise NotImplementedError('next_collision() needs to be implemented in derived classes')

        return None

    def setup_figure(self) -> None:
        """
        Function to set up the animation figure. Not implemented.

        Args:
            N/A

        Returns:
            None

        Raises:
            NotImplementedError: Base class should not be called directly.

        Examples:
            N/A

        """
        raise NotImplementedError('setup_figure() needs to be implemented in derived classes')

        return None

    def run(self, num_collisions: int
            , animate: bool = False,
            pause_time: float = 0.001) -> None:
        """
        Function to execute the simulation.

        Args:
            num_collisions (int): number of collisions in the system to simulate.
            animate (bool): whether to show the animation of collisions. Defaults to False.
            pause_time (float): time to pause for each frame of animation. Defaults to 0.001.

        Returns:
            None

        """
        if animate:
            self.setup_figure()

        pbar = tqdm(range(num_collisions))
        for _ in range(num_collisions):
            self.next_collision()
            pbar.update(1)
            if animate:
                plt.gcf().canvas.draw_idle()
                plt.pause(pause_time)
        if animate:
            plt.show(block=False)
        plt.close()

        return None


class SingleBallSimulation(Simulation):
    """
    Class to simulate a single ball in an ideal container. Inherits from Simulation.

    Attributes:
        No public attribute.

    Args:
        container (Container): the Container class instance for the simulation.
        ball (Ball): the Ball class instance for the simulation.

    Examples:
        >>> test_ball = Ball()
        >>> test_container = Container()
        >>> simulation = SingleBallSimulation(test_container, test_ball)
        >>> simulation.run(10)

    """

    def __init__(self, container: Container, ball: Ball):
        """
        Initialise the class with input container and ball.

        Args:
            container (Container): the Container class instance for the simulation.
            ball (Ball): the Ball class instance for the simulation.
        """
        super().__init__()
        self._container = container
        self._ball_list = [ball]

    def next_collision(self) -> None:
        """
        Implemented next_collision method for single ball simulation. The method will set a
        default dt_next variable with np.inf and update it with the current smallest value.
        The smallest dt_next will be recorded and the corresponding collision will be executed.
        Relevant status of the container and ball will be updated at the end.

        Note this method do not handle simultaneous collisions among balls.

        Args:
            N/A

        Returns:
            None

        Examples:
            >>> test_ball = Ball()
            >>> test_container = Container()
            >>> simulation = SingleBallSimulation(test_container, test_ball)
            >>> simulation.next_collision()

        """
        dt_next = np.inf
        collision_balls = [[]]
        for ball in self._ball_list:
            for i in range(len(self._ball_list)):
                if ball is not self._ball_list[i]:
                    dt_current = ball.time_to_collision(self._ball_list[i])
                    if isinstance(dt_current, type(None)):
                        dt_current = np.inf
                    if dt_current < dt_next:
                        dt_next = dt_current
                        collision_balls = [[ball, self._ball_list[i]]]

        for ball in self._ball_list:
            dt_current = ball.time_to_collision(self._container)
            if dt_current < dt_next:
                dt_next = dt_current
                collision_balls = [[ball, self._container]]

            elif dt_current == dt_next:
                collision_balls.append([ball, self._container])

        for ball in self._ball_list:
            ball.move(dt_next)
        self._container.move(dt_next)

        for collision in collision_balls:
            collision[0].collide(collision[1])

        return None

    def setup_figure(self) -> None:
        """
        Setup figure for animation by plotting the patches.

        Returns:
            None

        """
        fig = plt.figure(figsize=(8, 8), linewidth=1)
        axis = plt.axes(xlim=(-self._container.radius(),
                              self._container.radius()),
                        ylim=(-self._container.radius(),
                              self._container.radius()))
        for ball in self._ball_list:
            axis.add_patch(ball.patch())
        axis.add_patch(self._container.patch())
        plt.gcf().canvas.draw_idle()
        plt.show(block=False)

        return None

    def container(self) -> Container:
        """
        Accessor method to get the container.

        Args:
            N/A

        Returns:
            Container: the container used for simulation with current status.

        Examples:
            >>> test_ball = Ball()
            >>> test_container = Container()
            >>> simulation = SingleBallSimulation(test_container, test_ball)
            >>> result_container = simulation.container()

        """
        return self._container

    def ball(self) -> Ball:
        """
        Accessor method to get the ball.

        Args:
            N/A

        Returns:
            Ball: the ball used for simulation with current status.

        Examples:
            >>> test_ball = Ball()
            >>> test_container = Container()
            >>> simulation = SingleBallSimulation(test_container, test_ball)
            >>> result_ball = simulation.ball()

        """
        return self._ball_list[0]


class MultiBallSimulation(Simulation):
    """
    Class to simulate many balls in an ideal container. Inherits from Simulation.

    Attributes:
        No public attribute.

    Args:
        c_radius (float): radius of the container. Defaults to 10.
        b_radius (float): radius of the balls. Defaults to 1.
        b_speed (float): speed of the balls. Defaults to 10.
        b_mass (float): mass of the balls. Defaults to 1.
        rmax (float): maximum radius of the ball rings. Defaults to 8.
        nrings (int): number of ball rings. Defaults to 3.
        multi (int): factor by which number of balls in each ring is increased.
        Defaults to 6.

    Examples:
        >>> sim = MultiBallSimulation()

    """

    def __init__(self, c_radius: float = 10.,
                 b_radius: float = 1.,
                 b_speed: float = 10.,
                 b_mass: float = 1.,
                 rmax: float = 8.,
                 nrings: int = 3,
                 multi: int = 6):
        """
        Initialise the class with the ball and container parameters using initialisers
        implemented in the worksheet.

        Args:
            c_radius (float): radius of the container. Defaults to 10.
            b_radius (float): radius of the balls. Defaults to 1.
            b_speed (float): speed of the balls. Defaults to 10.
            b_mass (float): mass of the balls. Defaults to 1.
            rmax (float): maximum radius of the ball rings. Defaults to 8.
            nrings (int): number of ball rings. Defaults to 3.
            multi (int): factor by which number of balls in each ring is increased. Defaults to 6.

        """
        super().__init__()
        container = Container(radius=c_radius)
        self._container = container
        self._ball_list = []
        self._time = 0
        self._kb = 1.380649e-23

        positions = self.__position_initialiser(rmax, nrings, multi)
        velocities = self.__velocity_initialiser(len(positions), b_speed)
        for i, position_i in enumerate(positions):
            self._ball_list.append(Ball(position_i, velocities[i]
                                        , b_radius, b_mass))

    def __position_initialiser(self, rmax: float, nrings: int, multi: int) -> np.ndarray:
        """
        A helper function to initialise the ball positions. This is a protected
        function that should not be called directly.

        Args:
            rmax (float): maximum radius of the ball rings. Defaults to 8.
            nrings (int): number of ball rings. Defaults to 3.
            multi (int): factor by which number of balls in each ring is increased. Defaults to 6.

        Returns:
            np.ndarray: positions of the initial balls.

        """
        number_of_balls = (multi + nrings * multi) * nrings / 2
        position_list = np.zeros((int(number_of_balls + 1), 2))

        radial_increment = rmax / nrings
        k = 1
        for i in range(nrings + 1):
            if i == 0:
                position_list[i] = np.array([0., 0.])
            else:
                angular_increment = 2 * np.pi / (multi * i)
                radius = radial_increment * i
                for j in range(multi * i):
                    theta = angular_increment * j
                    position_list[k] \
                        = np.array([radius * np.cos(theta), radius * np.sin(theta)])
                    k += 1

        return position_list

    def __velocity_initialiser(self, number_of_balls: int, speed: float) -> np.ndarray:
        """
        Initialises the velocity of the balls by sampling from a uniform distribution.

        Args:
            number_of_balls (int): number of balls.
            speed (float): speed of the balls.

        Returns:
            np.ndarray: initial velocity of the balls.

        """
        velocity_list = np.zeros((number_of_balls, 2))
        angles = np.random.uniform(0, 2*np.pi, number_of_balls)
        velocity_list[:, 0] = speed * np.cos(angles)
        velocity_list[:, 1] = speed * np.sin(angles)

        return velocity_list

    def balls(self) -> list:
        """
        Accessor method to get the balls.

        Args:
            N/A

        Returns:
            list: a list of balls used for simulation with current status.

        Examples:
            >>> sim = MultiBallSimulation()
            >>> test_balls =  sim.balls()

        """
        return self._ball_list

    def next_collision(self) -> None:
        """
        Implemented next_collision method for multi-ball simulation. The method will set a
        default dt_next variable with np.inf and update it with the current smallest value.
        The smallest dt_next will be recorded and the corresponding collision will be executed.
        Relevant status of the container and ball will be updated at the end.

        This method handle simultaneous collisions among balls and with container.

        Args:
            N/A

        Returns:
            N/A

        Examples:
            >>> sim = MultiBallSimulation()
            >>> sim.next_collision()

        """
        dt_next = np.inf
        collision_balls = [[]]
        for ball in self._ball_list:
            for i in range(len(self._ball_list)):
                if ball is not self.balls()[i]:
                    dt_current = ball.time_to_collision(self.balls()[i])
                    if dt_current is None:
                        dt_current = np.inf
                    if dt_current < dt_next:
                        dt_next = dt_current
                        collision_balls = [[ball, self.balls()[i]]]

                    elif ((dt_current == dt_next) &
                          ([self._ball_list[i], ball] not in collision_balls)):
                        collision_balls.append([ball, self._ball_list[i]])

        for ball in self._ball_list:
            dt_current = ball.time_to_collision(self._container)
            if dt_current < dt_next:
                dt_next = dt_current
                collision_balls = [[ball, self._container]]

            elif dt_current == dt_next:
                collision_balls.append([ball, self._container])

        for ball in self._ball_list:
            ball.move(dt_next)
        self._container.move(dt_next)

        for collision in collision_balls:
            collision[0].collide(collision[1])

        self._time += dt_next

        return None

    def setup_figure(self) -> None:
        """
        Setup figure for animation by plotting the patches.

        Args:
            N/A

        Returns:
            N/A

        """
        fig = plt.figure(figsize=(8, 8), linewidth=1)
        axis = plt.axes(xlim=(-self._container.radius(),
                              self._container.radius()),
                        ylim=(-self._container.radius(),
                              self._container.radius()))
        for ball in self._ball_list:
            axis.add_patch(ball.patch())
        axis.add_patch(self._container.patch())
        plt.gcf().canvas.draw_idle()
        plt.show(block=False)

        return None

    def container(self) -> Container:
        """
        Accessor method to get the container.

        Args:
            N/A

        Returns:
            Container: the container used for simulation with current status.

        """
        return self._container

    def kinetic_energy(self) -> float:
        """
        Accessor method to get the kinetic energy of the entire system (include container)

        Args:
            N/A

        Returns:
            float: the kinetic energy of the entire system

        """
        velocity_list = np.array([ball.vel() for ball in self._ball_list])
        k_e = np.array([np.dot(vi, vi) for vi in velocity_list]).sum()
        k_e *= 0.5 * self._ball_list[0].mass()

        k_e += (0.5 * self.container().mass()
                * np.dot(self.container().vel(), self.container().vel()))
        return k_e

    def momentum(self) -> np.ndarray:
        """
        Accessor method to get the total momentum of the system (exclude container)

        Args:
            N/A

        Returns:
            np.ndarray: the total momentum of the system

        """
        velocity_list = np.array([ball.vel() for ball in self._ball_list])
        momenta = (self._ball_list[0].mass() * velocity_list).sum(axis=0)

        momenta += self.container().mass() * self.container().vel()
        return momenta

    def time(self) -> float:
        """
        Accessor method to get the total evolution time

        Args:
            N/A

        Returns:
            float: the total evolution time

        """
        return self._time

    def pressure(self) -> float:
        """
        Accessor method to get the system pressure

        Args:
            N/A

        Returns:
            float: the total pressure on the container

        """
        momenta = np.abs(self._container.dp_tot()) / self.time()
        momenta /= self._container.surface_area()
        return momenta

    def t_equipartition(self) -> float:
        """
        Accessor method to get the temperature according to the equipartition theorem

        Args:
            N/A

        Returns:
            float: the system temperature according to the equipartition theorem

        """
        vels = np.array([np.dot(ball.vel(), ball.vel())
                         for ball in self.balls()])
        kes = vels * 0.5 * self.balls()[0].mass()
        t_eq = np.mean(kes) / self._kb

        return t_eq

    def t_ideal(self) -> float:
        """
        Accessor method to get the temperature according to the ideal gas law

        Args:
            N/A

        Returns:
            float: the system temperature according to the ideal gas law

        """
        p_v = self.pressure() * self.container().volume()
        n_k = len(self.balls()) * self._kb
        return p_v / n_k

    def speeds(self) -> np.ndarray:
        """
        Accessor method to get a list of speed

        Args:
            N/A

        Returns:
            np.ndarray: the system temperature according to the ideal gas law

        """
        return np.array([np.linalg.norm(ball.vel())
                         for ball in self._ball_list])
