from DataLoader import DataLoader
from tqdm.auto import tqdm
import numpy as np
import matplotlib.pyplot as plt
from numba import njit


@njit(parallel=True)
def core(integrand, times, i):
    integral = np.trapz(y=integrand[:i] * integrand[0], x=times[:i])
    # p_diag_1 = p_tensor_xx - ((1 / 3) * (p_tensor_xx + p_tensor_yy))
    # integral += np.trapz(y=p_diag_1[:i] * p_diag_1[0], x=times[:i])
    # p_diag_2 = p_tensor_yy - ((1 / 3) * (p_tensor_xx + p_tensor_yy))
    # integral += np.trapz(y=p_diag_2[:i] * p_diag_2[0], x=times[:i])
    return integral


class ViscosityAnalyser:
    def __init__(self, data_loader: DataLoader):
        self.data = data_loader
        self.kb = 1.38064852e-23

    def extract_viscosity_gk(self, init_frame: int, integral_length: None | int = None, interval: int = 10):
        self.init_frame = init_frame
        if integral_length is None:
            self.end_frame = None
        else:
            self.end_frame = init_frame + integral_length

        times = self.data.chrono["time"].copy()[self.init_frame:self.end_frame]
        times -= times[0]
        temps = self.data.macro_dict["t_equipartition"][self.init_frame:self.end_frame]

        viscosity = np.zeros(len(times))
        integrand = self._integrand(self.init_frame, self.end_frame, interval)
        # p_tensor_xx = self._pressure_tensors_xx(self.init_frame, self.end_frame)
        # p_tensor_yy = self._pressure_tensors_yy(self.init_frame, self.end_frame)

        pbar = tqdm(total=len(times))
        pbar.set_description("retrieving viscosity")
        for i, ti in enumerate(times):
            if i == 0:
                pbar.update(1)
                pass
            else:
                integral = core(integrand, times, i)
                viscosity[i] = integral
                # viscosity[i] *= ((self.data.sim_spec["containerRadius"] ** 2)
                #                 * np.pi)
                viscosity[i] /= (self.kb * temps[i])
                pbar.update(1)

        self.viscosity = viscosity
        self.times = times

        if self.end_frame is not None:
            np.savetxt(self.data.folder_path + "/viscosity_init_" + str(init_frame) + str(self.end_frame) + '.csv'
                       , self.viscosity, delimiter=',')
        else:
            np.savetxt(self.data.folder_path + "/viscosity_init_" + str(init_frame) + '.csv'
                       , self.viscosity, delimiter=',')

        return viscosity

    def plot_viscosity(self):
        fig, ax = plt.subplots()
        ax.plot(self.times, self.viscosity)
        print(self.theoretical_viscosity())
        # ax.hlines(self.theoretical_viscosity(), np.min(self.times)
        #          , np.max(self.times), "red")

        return ax

    def _integrand(self, init_frame: int, end_frame: int, interval: int = 10):
        velocity_list = self.data.ball_vel[init_frame:]
        integral_length = end_frame - init_frame

        pbar = tqdm(total=int((velocity_list.shape[0] - integral_length) / interval))
        pbar.set_description("getting pressure tensors")

        end = integral_length
        integrand = np.zeros(integral_length)
        p_tensor = np.zeros(integral_length)
        i = 0
        k = 0
        while end < (len(velocity_list)):
            velocity = velocity_list[i:end]
            for j in range(len(velocity)):
                p_tensor[j] = (velocity[j][:, 0] * velocity[j][:, 1]).sum()
            integrand += p_tensor
            end += interval
            i += interval
            k += 1
            pbar.update(1)
        pbar.close()
        pbar.clear()
        integrand /= k

        integrand *= self.data.sim_spec['ballMass']

        return integrand

    def theoretical_viscosity(self):
        velocity_list = self.data.ball_vel[self.init_frame:]
        cbar = np.mean(np.linalg.norm(velocity_list, axis=1))

        m = self.data.sim_spec['ballMass']
        sigma = self.data.sim_spec['ballRadius'] * 4. * 1e-15

        return 0.499 * ((m * cbar) / (np.sqrt(2) * np.pi * (sigma ** 2)))


class DiffusionAnalyser:
    def __init__(self, data_loader: DataLoader):
        self.data = data_loader

    def extract_diffusion_coeff(self, init_frame: int):
        self.init_frame = init_frame
        position_list = self.data.ball_pos[self.init_frame:]
        diffusion_coeff = np.zeros(len(position_list))

        self.times = self.data.chrono["time"][self.init_frame:].copy()
        self.times -= self.times[0]

        pbar = tqdm(total=len(position_list))
        for i in range(len(position_list)):
            if i == 0:
                pbar.update(1)
                pass
            else:
                diffusion_coeff[i] = np.mean(np.linalg.norm((position_list[i] - position_list[0]), axis=1) ** 2)
                diffusion_coeff[i] /= 4
                pbar.update(1)

        self.diffusion_coeff = diffusion_coeff

        return diffusion_coeff

    def plot_diffusion_coeff(self):
        fig, ax = plt.subplots()
        ax.plot(self.times, self.diffusion_coeff)

        return ax


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
