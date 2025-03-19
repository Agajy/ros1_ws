#!/usr/bin/env python

import pandas as pd

class ExcelReader:
    
    @staticmethod
    def extract_data_excel_to_array(filename):
        data_frame = pd.read_excel(filename, header=None)
        np_matrix = data_frame.to_numpy()
        return np_matrix