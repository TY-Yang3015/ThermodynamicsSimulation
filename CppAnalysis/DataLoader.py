import numpy as np
import pandas as pd
import glob, os

class DataLoader:
    def __init__(self, folder: str):
        self.folder_path = folder

    def load(self):

        self.sim_spec = self._load_sim_spec()

        if self.sim_spec is 0:
            del self.sim_spec

        self.ball_vel = self._load_raw("/ball_velocities.csv")
        self.ball_pos = self._load_raw("/ball_positions.csv")

        self.container_vel = self._load_raw("/container_velocities.csv")
        self.container_pos = self._load_raw("/container_positions.csv")

        self.macro_dict = self._load_macro()

        self.chrono = self._load_chrono()

        return self
        
    def _load_raw(self, filename):
        file_path = self.folder_path + filename
        data = pd.read_csv(file_path)
        
        raw_data = data.to_numpy()
        
        data = np.zeros((raw_data.shape[0], int(raw_data.shape[1]/2), 2))
        for i in range(raw_data.shape[0]):
            for j in range(raw_data.shape[1]):
                if j % 2 == 1:
                    data[i][j//2][1] = float(raw_data[i][j].strip(']'))
                else:
                    data[i][j//2][0] = float(raw_data[i][j].strip('['))
        return data

    def _load_sim_spec(self):
        directory_path = self.folder_path
        
        pattern = os.path.join(directory_path, '*simulation_spec*.csv')
        
        matching_files = glob.glob(pattern)

        if len(matching_files) == 0:
            print("simulation spec was not registered.")
            return 0
        
        spec = np.loadtxt(*matching_files, dtype=str, delimiter=',')
        
        spec_dict = {}
        for i in range(len(spec[0])):
            spec_dict[spec[0][i]] = float(spec[1][i])

        return spec_dict

    def _load_macro(self):
        macro_file = self.folder_path + '/macro_information.csv'
        
        header = np.loadtxt(macro_file, delimiter=',', dtype=str)[0]
        
        data_dict = {}
        for i in range(len(header)):
            data_dict[header[i]] = np.loadtxt(macro_file, delimiter=',',  skiprows=1).T[i]

        return data_dict

    def _load_chrono(self):
        macro_file = self.folder_path + '/system_event_track.csv'
        
        header = ["time", "collision"]
        
        data_dict = {}
        for i in range(len(header)):
            data_dict[header[i]] = np.loadtxt(macro_file, delimiter=',',  skiprows=1).T[i]

        return data_dict
