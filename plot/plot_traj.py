#!/usr/bin/python3
import time
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os
import csv_reader

path = os.getcwd()
data_w = csv_reader.read_csv(csvfile=path + '/data_w.csv')
data_wo = csv_reader.read_csv(csvfile=path + '/data_wo.csv')

## plot
plt.rcParams['font.family'] = 'serif'
plt.rcParams['font.serif'] = ['Times New Roman'] + plt.rcParams['font.serif']

fig, axs = plt.subplots(2, 2, figsize=(6.4, 3.2), sharex=True, tight_layout=True)
fig.subplots_adjust(hspace=0.2)
axs[0,0].tick_params('x', labelbottom=True, labelsize=9)
axs[0,1].tick_params('x', labelbottom=True, labelsize=9)
axs[1,0].tick_params('x', labelbottom=True, labelsize=9)
axs[1,1].tick_params('x', labelbottom=True, labelsize=9)
axs[1,0].set_xlabel('Time (s)', fontsize=9)
axs[1,1].set_xlabel('Time (s)', fontsize=9)
axs[1,0].set_xlim(-0.1, 2.1)
axs[1,0].set_xticks(np.arange(0.0, 2.1, 0.5))

# x_ee
axs[0,1].plot(data_w['Time'], data_w['x_des_w'], 'b-', label=r'$x_{ee}^{d}$', lw=1)
axs[0,1].plot(data_w['Time'], data_w['x_hat'], 'r-', label=r'$x_{ee}^{m}$', lw=1)
axs[0,1].legend(loc='upper left', mode="compact", ncol=1, fontsize=9, frameon=True)
axs[0,1].set_ylabel(r'$P_x$ (m)', fontsize=9)

axs[0,0].plot(data_wo['Time'], data_wo['x_des_w'], 'b-', label=r'$x_{ee}^{d}$', lw=1)
axs[0,0].plot(data_wo['Time'], data_wo['x_hat'],'r-', label=r'$x_{ee}^{m}$', lw=1)
axs[0,0].legend(loc='upper left', mode="compact", ncol=1, fontsize=9, frameon=True)
axs[0,0].set_ylabel(r'$P_x$ (m)', fontsize=9)

# V_ee & V_cmd
axs[1,1].plot(data_w['Time'], data_w['dx_hat'], 'r-', label=r'$V_{ee}^{m}$', lw=1)
axs[1,1].plot(data_w['Time'], data_w['wb_x_vel'], 'b-', label=r'$V_{ee}$', lw=1)
axs[1,1].plot(data_w['Time'], data_w['base_x_vel_flt'], 'g-', label=r'$V_{b}$', lw=1)
axs[1,1].plot(data_w['Time'], data_w['arm_x_vel_syn'], 'm-', label=r'$V_{a}$', lw=1)
axs[1,1].legend(loc='upper left', mode="compact", ncol=1, fontsize=8, frameon=True)
axs[1,1].set_ylabel(r'$V_{x}$ (m/s)', fontsize=9)
axs[1,1].set_ylim(-0.02, 0.16)

axs[1,0].plot(data_wo['Time'], data_wo['dx_hat'],'r-', label=r'$V_{ee}^{m}$', lw=1)
axs[1,0].plot(data_wo['Time'], data_wo['wb_x_vel'], 'b-', label=r'$V_{ee}$', lw=1)
axs[1,0].plot(data_wo['Time'], data_wo['base_x_vel_flt'], 'g-', label=r'$V_{b}$', lw=1)
axs[1,0].plot(data_wo['Time'], data_wo['arm_x_vel_syn'],'m-', label=r'$V_{a}$', lw=1)
axs[1,0].legend(loc='upper left', mode="compact", ncol=1, fontsize=8, frameon=True)
axs[1,0].set_ylabel(r'$V_{x}$ (m/s)', fontsize=9)
axs[1,0].set_ylim(-0.02, 0.16)

plt.show()
