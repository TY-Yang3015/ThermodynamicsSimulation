from DataLoader import DataLoader
from tqdm.auto import tqdm
import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as spi


class ViscosityAnalyser:
    def __init__(self, data_loader: DataLoader):
        self.data = data_loader
        self.kb = 1.38064852e-23

    def extract_viscosity(self, init_frame: int):
        self.init_frame = init_frame
        times = self.data.chrono["time"][init_frame:]
        times -= times[0]
        temps = self.data.macro_dict["t_equipartition"][init_frame:]

        viscosity = np.zeros(len(times))
        p_tensors = self._pressure_tensors(self.init_frame)

        pbar = tqdm(total=len(times))
        pbar.set_description("retrieving viscosity")
        for i, ti in enumerate(times):
            if i == 0:
                pbar.update(1)
                pass
            else:
                integral = spi.trapezoid(y=p_tensors[:i], x=times[:i])
                viscosity[i] = integral ** 2
                viscosity[i] *= ((self.data.sim_spec["containerRadius"] ** 2)
                                 * np.pi / (2. * ti))
                viscosity[i] *= 1 / (self.kb * temps[i])
                pbar.update(1)

        self.viscosity = viscosity
        self.times = times

        np.savetxt(self.data.folder_path + "/viscosity_init_" + str(init_frame) + '.csv'
                   , self.viscosity, delimiter=',')

        return viscosity

    def plot_viscosity(self):
        fig, ax = plt.subplots()
        ax.plot(self.times, self.viscosity)
        print(self.theoretical_viscosity())
        #ax.hlines(self.theoretical_viscosity(), np.min(self.times)
        #          , np.max(self.times), "red")

        return ax

    def _pressure_tensors(self, init_frame: int):
        velocity_list = self.data.ball_vel[init_frame:]

        p_tensor = np.zeros(len(velocity_list))
        pbar = tqdm(total=velocity_list.shape[0])
        pbar.set_description("getting pressure tensors")
        for i in range(len(velocity_list)):
            p_tensor[i] = (velocity_list[i][:, 0] * velocity_list[i][:, 1]).sum()
            pbar.update(1)

        p_tensor *= self.data.sim_spec['ballMass']
        pbar.close()
        pbar.clear()

        return p_tensor

    def theoretical_viscosity(self):
        velocity_list = self.data.ball_vel[self.init_frame:]
        cbar = np.mean(np.linalg.norm(velocity_list, axis=-1))

        m = self.data.sim_spec['ballMass']
        sigma = self.data.sim_spec['ballRadius'] * 4.

        return 0.499 * ((m * cbar) / (np.sqrt(2) * np.pi * (sigma ** 2)))


class VelocityAutocorrelationAnalyser:
    def __init__(self, data_loader: DataLoader):
        self.data = data_loader
        self.time = self.data.chrono["time"]
        self.autocorrelation = None

    def get_velocity_autocorrelation(self, step: int | None = None):
        velocity_list = self.data.ball_vel.copy()
        autocorr = np.zeros(velocity_list.shape[0])

        pbar = tqdm(total=len(velocity_list))
        k = 0
        for i in range(len(velocity_list)):
            autocorr[i] = ((velocity_list[i][:, 0] * velocity_list[i - k][:, 0]) +
                           (velocity_list[i][:, 1] * velocity_list[i - k][:, 1])).sum() / len(velocity_list[i])
            if step is not None:
                if i % step == 0:
                    k = 0
                k += 1
            else:
                k = i
            pbar.update(1)

        pbar.close()
        pbar.clear()

        self.autocorrelation = autocorr

        return autocorr

    def plot_velocity_autocorrelation(self):
        if self.autocorrelation is not None:
            fig, ax = plt.subplots()
            ax.plot(self.time, self.autocorrelation)
            return ax
        else:
            self.get_velocity_autocorrelation(None)
            fig, ax = plt.subplots()
            ax.plot(self.time, self.autocorrelation)
            return ax
