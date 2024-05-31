"""Ball and Container classes."""
import numpy as np
import matplotlib.pyplot as plt


class Ball:
    """
    A rigid ball to simulate gas particle in rigid-ball gas model.

    Attributes:
        No public attribute.

    Args:
        pos (np.ndarray): Position of the ball
        vel (np.ndarray): Velocity of the ball
        radius (float): Radius of the ball
        mass (float): Mass of the ball

    Examples:
        >>> test_ball = Ball()

    """
    def __init__(self, pos: np.ndarray = np.array([0, 0]),
                 vel: np.ndarray = np.array([1, 0]),
                 radius: float = 1.,
                 mass: float = 1.):
        """
        Initialize the ball.

        Args:
            pos (np.ndarray): Position of the ball
            vel (np.ndarray): Velocity of the ball
            radius (float): Radius of the ball
            mass (float): Mass of the ball

        """
        self._pos = np.array(pos, dtype=np.float64)
        self._vel = np.array(vel, dtype=np.float64)
        self._radius = float(radius)
        self._mass = float(mass)
        self.__sanitise_input(self._pos, self._vel, self._radius, self._mass)
        self._patch = plt.Circle((pos[0], pos[1]), radius
                                 , facecolor='red',
                                 linewidth=1, edgecolor='black'
                                 )

    def __sanitise_input(self,
                         pos: np.ndarray,
                         vel: np.ndarray,
                         radius: float,
                         mass: float) -> None:
        """
        Check the type and shape of initial input. This is a
        protected member of the class.

        Args:
            pos (np.ndarray): Position of the ball
            vel (np.ndarray): Velocity of the ball
            radius (float): Radius of the ball
            mass (float): Mass of the ball

        Raises:
            ValueError: if pos and vel are not of length 2.
            ValueError: if radius and mass are not float.

        Returns:
            None

        """
        if ((len(pos) != 2) | (len(vel) != 2)
                | (isinstance(radius, float) is False)
                | (isinstance(mass, float) is False)):
            raise ValueError("check input arguments")

        return None

    def pos(self) -> np.ndarray:
        """
        Accessor method to get the position of the ball

        Args:
            N/A

        Returns:
            np.ndarray: the position of the ball.

        """
        return self._pos

    def radius(self) -> float:
        """
        Accessor method to get the radius of the ball

        Args:
            N/A

        Returns:
            float: the radius of the ball.

        """
        return self._radius

    def mass(self) -> float:
        """
        Accessor method to get the mass of the ball

        Args:
            N/A

        Returns:
            float: the mass of the ball.

        """
        return self._mass

    def vel(self) -> np.ndarray:
        """
        Accessor method to get the velocity of the ball

        Args:
            N/A

        Returns:
            np.ndarray: the velocity of the ball

        """
        return self._vel

    def set_vel(self, vel: np.ndarray) -> None:
        """
        Set the velocity of the ball to a given vector.


        Args:
            vel (np.ndarray): Velocity vector of the ball

        Returns:
            None

        Raises:
            ValueError: if the length of the velocity vector is not 2

        """
        if len(vel) != 2:
            raise ValueError("velocity must have two elements")
        self._vel = np.array(vel, dtype=np.float64)

        return None

    def move(self, dt: float) -> None:
        """
        Move the ball according to the time increment and velocity.

        Args:
            dt (float): The time increment to move the ball.

        Returns:
            None

        """
        self._pos += self._vel * dt
        self._patch.center = self._pos

        return None

    def patch(self) -> plt.Circle:
        """
        Accessor method to the matplotlib patch of the ball.

        Args:
            N/A

        Returns:
            plt.Circle: the graphical patch of the ball.

        """
        return self._patch

    def __determinant(self, r_diff: np.ndarray,
                      v_diff: np.ndarray,
                      rad_diff: float) -> float:
        """
        Determinant of the collision time quadratic equation
        differ by a constant factor. This is a protected member
        of the class.

        Args:
            r_diff (np.ndarray): position difference
            v_diff (np.ndarray): velocity difference
            rad_diff (float): radius sum/difference

        Returns:
            float: the collision time quadratic equation determinant

        """
        # print('r:', r)
        # print('v:', v)
        # print('R:', R)
        return ((np.dot(r_diff, v_diff) ** 2) - np.dot(v_diff, v_diff)
                * (np.dot(r_diff, r_diff) - (rad_diff ** 2)))

    def time_to_collision(self, other: any) -> float | None:
        """
        Hard-coded time to collision according to mechanical
        theory.


        Args:
            other: either a Container class instance or a Ball class instance, representing different collisions.

        Returns:
            float | None: the time to collision. Return None if they do not collide.

        """
        r_diff = self.pos() - other.pos()
        v_diff = self.vel() - other.vel()
        rad_diff = self.radius() + other.radius()
        if isinstance(other, Container) or isinstance(self, Container):
            rad_diff = self.radius() - other.radius()

        det = self.__determinant(r_diff, v_diff, rad_diff)
        if det > 0:
            d_t = -np.sqrt(det)
            if isinstance(other, Container) or isinstance(self, Container):
                d_t *= -1
            d_t -= np.dot(r_diff, v_diff)
            # print('det:', dt)
            d_t *= 1 / np.dot(v_diff, v_diff)

            if d_t > 0:
                return d_t
            else:
                return None
        else:
            return None

    # elif isinstance(other, Container):
    #    r = self.pos()
    #    v = self.vel()
    #    R = self.radius() - other.radius()

    #    dt = np.sqrt(self.__determinant(r, v, R))
    #    dt -= np.dot(r, v)
    #    dt *= 1 / np.dot(v, v)

    #    if dt > 0:
    #        return dt
    #    else:
    #        return np.inf

    # else:
    #    raise ValueError('unknown class object.')

    def collide(self, other: any) -> None:
        """
        Process ball collisions or collision with container.

        Args:
            other: A container instance or a Ball class instance, representing different collisions

        Returns:
            None

        """
        r = self.pos() - other.pos()
        v = self.vel() - other.vel()

        v1 = self.vel().copy()
        v1 -= ((2 * other.mass()
                / (self.mass() + other.mass()))
               * (np.dot(v, r) / np.dot(r, r))) * r

        v2 = other.vel().copy()
        v2 += ((2 * self.mass()
                / (self.mass() + other.mass()))
               * (np.dot(v, r) / np.dot(r, r))) * r

        if isinstance(other, Container):
            dp = v2 - other.vel()
            dp *= other.mass()
            dp = np.linalg.norm(dp)
            other.add_dp(dp)
        elif isinstance(self, Container):
            dp = v1 - self.vel()
            dp *= self.mass()
            dp = np.linalg.norm(dp)
            self.add_dp(dp)

        self.set_vel(v1)
        other.set_vel(v2)

        return None

    # elif isinstance(other, Container):
    #    r_hat = self.pos() / np.linalg.norm(self.pos())#

    #    v_r = np.dot(self.vel(), r_hat) * r_hat
    #   v_theta = self.vel() - v_r

    #    dp = np.abs(2 * self.mass() * v_r)

    #   self.set_vel(v_theta - v_r)
    #    other.add_dp(dp)


class Container(Ball):
    """
    A massive container class, inheriting from Ball

    Args:
        radius (float): The radius of the container
        mass (float): The mass of the container

    """
    def __init__(self, radius: float = 10,
                 mass: float = 1e7):
        """
        Initialize a container.

        Args:
            radius (float): The radius of the container
            mass (float): The mass of the container
        """
        super().__init__([0, 0], [0, 0], radius, mass)
        self._dp_tot = 0.
        self._patch = plt.Circle((self._pos[0], self._pos[1])
                                 , radius, fill=False
                                 , edgecolor='blue', linewidth=1)

    def volume(self) -> float:
        """
        Accessor method to get the 2D volume of the container

        Args:
            N/A

        Returns:
            float: the volume of the container

        """
        return np.pi * self._radius ** 2

    def surface_area(self) -> float:
        """
        Accessor method to get the 2D surface area of the container

        Args:
            N/A

        Returns:
            float: the 2D surface area of the container

        """
        return 2 * np.pi * self._radius

    def dp_tot(self) -> float:
        """
        Accessor method to the total scalar change in momentum

        Args:
            N/A

        Returns:
            float: the total scalar change in momentum

        """
        return self._dp_tot

    def add_dp(self, dp: float) -> None:
        """
        Add a particular value to the total scalar
        change in momentum.

        Args:
            dp (float): The particular change in momentum to add

        Returns:
            None

        """
        self._dp_tot += dp

        return None
