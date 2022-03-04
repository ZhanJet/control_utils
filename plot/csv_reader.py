#!/usr/bin/python3
import csv
import numpy as np

def read_csv(csvfile):
    data = {}
    with open(csvfile, newline='\n') as file:
        csvreader = csv.reader(file)
        #reading first row of field names
        fields = csvreader.__next__()
        print('Field Names\n--------------')
        for field in fields:
            print("%8s"%field, end=', ')
            data[field] = np.array([])
        
        csvreader = csv.reader(file, quoting=csv.QUOTE_NONNUMERIC)
        #reading rows
        for row in csvreader:
            for i, field in enumerate(fields):
                data[field] = np.append(data[field], row[i])

    return data