""" Read and plot the experimental data logged as rosbag of whole-body control"""
import math
from pickle import TRUE
import time
from numpy.core.fromnumeric import trace
# import rospy
# import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque

dt = 0.01
# Load data
load_from_file = False
dir = '/home/zhanjet/moying_ws/src/mobile_manipulator_controller/admittance_control/record/'
bag_files = ['obst_avoid_2022-01-20-12-26-20.bag', 'obst_avoid_2022-01-20-12-13-26.bag', 
            'obst_avoid_2022-01-29-20-48-07.bag', 'obst_avoid_2022-01-29-21-14-23.bag']
file_plot = 3
topics_to_plot = ['/admittance_control/ee_pose_world',  #plot_2D
                '/me5/manipulability', '/admittance_control/k_manip', #fig1, manip     
                '/admittance_control/y_wb', '/admittance_control/y_wb_disturb', '/admittance_control/V_wb_delta', #fig2
                '/admittance_control/y_dot_base', '/admittance_control/V_arm_delta', '/admittance_control/V_arm_delta_ns',
                '/admittance_control/factor_ns', '/admittance_control/project', #fig3, delta_V_base=cmd_vel-V_base, y_dot_arm=V_arm_delta_ns
                '/admittance_control/distance', '/obstacle_avoidance/obstacles', '/admittance_control/V_base_avoid'] #fig4

if load_from_file:
    import rosbag_reader
    topic_type_dict = rosbag_reader.read_bag(bagfile=dir+bag_files[file_plot])
    topics = topic_type_dict.keys()
    data = rosbag_reader.read_msg(topics)
    print(data[topics[0]]['t'][0])
    np.save('data_wbc_' + str(file_plot) + '.npy', data)
else:
    data = np.load('data_wbc_' + str(file_plot) + '.npy', allow_pickle='TRUE', encoding='bytes').item()
    topics = data.keys()

# Plot the data in animation
## line up the time series
# t_first = sys.float_info.max
# t_first = float('inf')
t_start = np.finfo(np.float64).max
t_end = np.finfo(np.float64).min
topic_first = topics[0]
for topic in topics:
    ts = data[topic]['t'][0]
    te = data[topic]['t'][-1]
    print(topic)
    print(ts, te, te-ts)
    if t_start > ts:
        t_start = ts
        topic_first = topic
    if t_end < te:
        t_end = te
duration = t_end - t_start
print(t_start)
print(topic_first)
print(duration)
print(data['/obstacle_avoidance/obstacles']['t'][0], data['/obstacle_avoidance/obstacles']['t'][-1])
for topic in topics:
    data[topic]['t'] = np.round(np.array(data[topic]['t']) - t_start, 2)
    print(data[topic]['t'][0], data[topic]['t'][-1])

## V_base_delta = cmd_vel - V_base
topic1 = '/me5/mecanum_velocity_controller/cmd_vel'
topic2 = '/admittance_control/V_base'
topic3 = '/admittance_control/V_base_delta'
ts = max(data[topic1]['t'][0], data[topic2]['t'][0])
te = min(data[topic1]['t'][-1], data[topic2]['t'][-1])
indexs1 = int((ts-data[topic1]['t'][0])/dt)
indexs2 = int((ts-data[topic2]['t'][0])/dt)
indexe1 = int((te-data[topic1]['t'][0])/dt)
indexe2 = int((te-data[topic2]['t'][0])/dt)
data[topic3] = {}
data[topic3]['t'] = data[topic1]['t'][indexs1:indexe1]
data[topic3]['vx'] = data[topic1]['vx'][indexs1:indexe1] - data[topic2]['vx'][indexs2:indexe2]
data[topic3]['vy'] = data[topic1]['vy'][indexs1:indexe1] - data[topic2]['vy'][indexs2:indexe2]

# Plot animation
ani_select = 2 # 1-deviation; 2-splines

from matplotlib.font_manager import FontProperties
font = FontProperties()
font.set_family('serif')
font.set_name('Times New Roman')
font.set_style('italic')
##Postion
topic_pos = '/admittance_control/ee_pose_world'
data[topic_pos]['x'] = data[topic_pos]['x'][:] - data[topic_pos]['x'][0]
data[topic_pos]['y'] = data[topic_pos]['y'][:] - data[topic_pos]['y'][0]
fig_xy, ax_xy = plt.subplots(figsize=(8., 7.))
ax_xy.set_title('End effector deviation', fontsize=20)
ax_xy.set(xlim=(-0.3,0.3), ylim=(-0.3, 0.3))
ax_xy.set(xticks=np.arange(-0.3, 0.3, 0.1), yticks=np.arange(-0.3, 0.3, 0.1))
ax_xy.tick_params(labelsize=18)
ax_xy.set_aspect('equal', adjustable='box')
ax_xy.invert_xaxis()
ax_xy.invert_yaxis()
# ax_xy.set_xlabel('x/m', fontsize=12, fontproperties=font)
ax_xy.set_xlabel(r'$\Delta$x/m', fontsize=24, fontweight='roman')
ax_xy.set_ylabel(r'$\Delta$y/m', fontsize=24, fontweight='roman')
trace = plt.Line2D([], [], color='blue', linestyle=':', linewidth=2)
trace_tail = plt.Line2D([], [], color='red', linewidth=4)
trace_head = plt.Line2D([], [], color='red', marker='o', markersize=10, markeredgecolor='r')
ax_xy.add_line(trace)
ax_xy.add_line(trace_tail)
ax_xy.add_line(trace_head)
margin = 100
ax_xy.grid(True)
fig_xy.tight_layout()
time_template = 'time = %.1fs'
time_text = ax_xy.text(0.55, 0.9, '', transform=ax_xy.transAxes, fontsize=25)

##Demo
fig, axs = plt.subplots(4, 1, figsize=(8, 8), sharex=True)
fig.subplots_adjust(hspace=0.0)
axs[0].tick_params('x', labelbottom=True, labelsize=6)
axs[1].tick_params('x', labelbottom=True, labelsize=6)
axs[2].tick_params('x', labelbottom=True, labelsize=6)
axs[3].set_xlim(0, data[topic_first]['t'][-1]-3.0)
axs[3].set_xlabel('Time / s', fontsize=12)
axs[3].set_ylabel('Obstacles', fontsize=12)
axs[2].set_ylabel('Velocities', fontsize=12)
axs[1].set_ylabel('Deviation', fontsize=12)
axs[0].set_ylabel('Values', fontsize=12)
# plt.show()
#fig1, disturbance
order = 1
ln_ywb_x, = axs[order].plot([], [], 'b-', label=r'$x_{wb}$')
ln_ywb_y, = axs[order].plot([], [], 'b--', label=r'$y_{wb}$')
ln_ywb_dist_x, = axs[order].plot([], [], 'g-', label=r'$x_{dist}$')
ln_ywb_dist_y, = axs[order].plot([], [], 'g--', label=r'$y_{dist}$')
axs[order].set_ylim(-0.2, 0.5)
axs[order].legend(loc='upper left', mode="compact", ncol=2, frameon=True)
#fig2, vel
order = 2
ln_Vwb_delta_x, = axs[order].plot([], [], 'r-', label=r'$V^{x}_{wb}$')
ln_Vwb_delta_y, = axs[order].plot([], [], 'r--', label=r'$V^{y}_{wb}$')
ln_Vb_delta_x, = axs[order].plot([], [], 'g-', label=r'$V^{x}_{base}$')
ln_Vb_delta_y, = axs[order].plot([], [], 'g--', label=r'$V^{y}_{base}$')
ln_Va_delta_x, = axs[order].plot([], [], 'm-', label=r'$^{cp}V^{x}_{arm}$')
ln_Va_delta_y, = axs[order].plot([], [], 'm--', label=r'$^{cp}V^{y}_{arm}$')
ln_Va_ns_x, = axs[order].plot([], [], 'b-', label=r'$V^{x}_{arm}$')
ln_Va_ns_y, = axs[order].plot([], [], 'b--', label=r'$V^{y}_{arm}$')
axs[order].set_ylim(-0.6, 0.6)
legend1 = axs[order].legend(handles=[ln_Vwb_delta_x, ln_Vwb_delta_y, ln_Vb_delta_x, ln_Vb_delta_y], 
                loc='upper left', mode="compact", ncol=2, fontsize=9, frameon=True)
legend2 = axs[order].legend(handles=[ln_Va_delta_x, ln_Va_delta_y, ln_Va_ns_x, ln_Va_ns_y], 
                loc='lower left', mode="compact", ncol=2, fontsize=9, frameon=True)
axs[order].add_artist(legend1)
#fig3, manip, factor_ns
order = 0
twin1 = axs[order].twinx()
ln_manip, = axs[order].plot([], [], 'r-', label=r'$c_{m}$') #manip
axs[order].set_ylim(0.0, 0.06)
axs[order].tick_params(axis='y', labelcolor='r')
ln_kmanip, = twin1.plot([], [], 'b--', label=r'$k_{cm}$') #k_manip
axs[order].axhspan(0.0345, 0.04, facecolor='tab:red', alpha=0.3)
axs[order].text(2., 0.035, r'$c_{m}$ limitation band', fontsize=10)
ln_fns, = twin1.plot([], [], 'b:', label=r'$a_{ns}$') #a_ns
ln_proj, = twin1.plot([], [], 'b-', label=r'$a_{sw}$') #switch of a_ns
twin1.set_ylim(0., 1.1)
twin1.tick_params(axis='y', labelcolor='b')
axs[order].legend(handles=[ln_manip, ln_kmanip, ln_fns, ln_proj], 
                loc='lower left', mode="compact", ncol=2, frameon=True)
# axs[order].legend(handles=[ln_manip, ln_kmanip, ln_fns, ln_proj])
#fig4, obstacles
order = 3
ln_obst_x, = axs[order].plot([], [], 'b-', label=r'$x_{o}$')
ln_obst_y, = axs[order].plot([], [], 'g-', label=r'$y_{o}$')
ln_dis, = axs[order].plot([], [], 'r-', label=r'$d_{o}$')
axs[order].set_ylim(-1.0, 1.2)
axs[order].legend(loc='upper left', frameon=True)

fig.tight_layout()

# plot_init()
t0 = time.time()
t = 0.0
# while t < t_end - t_start:
#     t = time.time() - t0
#     index = int(t/dt)
#     # plot position
#     time_text.set_text(time_template % (t))
#     trace.set_xdata(data[topic_pos]['x'][:index])
#     trace.set_ydata(data[topic_pos]['y'][:index])
#     trace_tail.set_data(data[topic_pos]['x'][index-margin:index],
#                         data[topic_pos]['y'][index-margin:index])
#     trace_head.set_data(data[topic_pos]['x'][index],
#                         data[topic_pos]['y'][index])
#     # plot demo
#     plt.pause(dt)

def set_data(ln, topic, t, xdata, ydata):
    if t >= data[topic]['t'][0] and t <= data[topic]['t'][-1]:
        index = int((t-data[topic]['t'][0])/dt)
        ln.set_data(xdata[:index], ydata[:index])
    elif t > data[topic]['t'][-1]:
        data[topic]['t'] = np.append(data[topic]['t'], t)
        ydata = np.append(ydata, ydata[-1])
        ln.set_data(xdata, ydata)
    else:
        ln.set_data([], [])

def set_xydata(ln, topic, t, xdata, ydata):
    if t >= data[topic]['t'][0] and t <= data[topic]['t'][-1]:
        ln.set_data(xdata, ydata)
    else:
        ln.set_data([], [])

def animate_deviation(i):
    global t0
    t = time.time() - t0
    index = int(t/dt)
    # t = i * dt
    # index = i
    if t > duration:
        t0 = time.time()
    # plot position
    time_text.set_text(time_template % (t))
    # if t >= data[topic_pos]['t'][0] and t <= data[topic_pos]['t'][-1]:
    #     trace.set_data(data[topic_pos]['x'][:index], data[topic_pos]['y'][:index])
    #     trace_tail.set_data(data[topic_pos]['x'][index-margin:index],
    #                         data[topic_pos]['y'][index-margin:index])
    #     trace_head.set_data(data[topic_pos]['x'][index],
    #                         data[topic_pos]['y'][index])
    set_xydata(trace, topic_pos, t, data[topic_pos]['x'][:index], data[topic_pos]['y'][:index])
    set_xydata(trace_tail, topic_pos, t, data[topic_pos]['x'][index-margin:index], data[topic_pos]['y'][index-margin:index])
    set_xydata(trace_head, topic_pos, t, data[topic_pos]['x'][index], data[topic_pos]['y'][index])

    return trace, trace_tail, trace_head, time_text

def animate_splines(i):
    global t0
    t = time.time() - t0
    # index = int(t/dt)
    # t = i * dt
    # index = i
    if t > duration:
        t0 = time.time()
    topic = '/admittance_control/y_wb'
    set_data(ln_ywb_x, topic, t, data[topic]['t'], data[topic]['x'])
    set_data(ln_ywb_y, topic, t, data[topic]['t'], data[topic]['y'])
    topic = '/admittance_control/y_wb_disturb'
    set_data(ln_ywb_dist_x, topic, t, data[topic]['t'], data[topic]['x'])
    set_data(ln_ywb_dist_y, topic, t, data[topic]['t'], data[topic]['y'])
    topic = '/me5/manipulability'
    set_data(ln_manip, topic, t, data[topic]['t'], data[topic]['data'])
    topic = '/admittance_control/k_manip'
    set_data(ln_kmanip, topic, t, data[topic]['t'], data[topic]['data'])
    topic = '/admittance_control/factor_ns'
    set_data(ln_fns, topic, t, data[topic]['t'], data[topic]['data'])
    topic = '/admittance_control/project'
    set_data(ln_proj, topic, t, data[topic]['t'], data[topic]['data'])

    topic = '/obstacle_avoidance/obstacles'
    set_data(ln_obst_x, topic, t, data[topic]['t'], data[topic]['x'])
    set_data(ln_obst_y, topic, t, data[topic]['t'], data[topic]['y'])
    topic = '/admittance_control/distance'
    set_data(ln_dis, topic, t, data[topic]['t'], data[topic]['data'])

    topic = '/admittance_control/V_wb_delta'
    set_data(ln_Vwb_delta_x, topic, t, data[topic]['t'], data[topic]['vx'])
    set_data(ln_Vwb_delta_y, topic, t, data[topic]['t'], data[topic]['vy'])
    # topic = '/admittance_control/y_dot_base' # V_base_delta
    # V_base_delta = cmd_vel - V_base
    topic = '/admittance_control/V_base_delta'
    set_data(ln_Vb_delta_x, topic, t, data[topic]['t'], data[topic]['vx'])
    set_data(ln_Vb_delta_y, topic, t, data[topic]['t'], data[topic]['vy'])
    topic = '/admittance_control/V_arm_delta'
    set_data(ln_Va_delta_x, topic, t, data[topic]['t'], data[topic]['vx'])
    set_data(ln_Va_delta_y, topic, t, data[topic]['t'], data[topic]['vy'])
    topic = '/admittance_control/V_arm_delta_ns'
    set_data(ln_Va_ns_x, topic, t, data[topic]['t'], data[topic]['vx'])
    set_data(ln_Va_ns_y, topic, t, data[topic]['t'], data[topic]['vy'])

    ln_list = [ln_ywb_x, ln_ywb_y, ln_ywb_dist_x, ln_ywb_dist_y, 
            ln_manip, ln_kmanip, ln_fns, ln_proj,
            ln_obst_x, ln_obst_y, ln_dis,
            ln_Vwb_delta_x, ln_Vwb_delta_y, 
            ln_Vb_delta_x, ln_Vb_delta_y,
            ln_Va_delta_x, ln_Va_delta_y,
            ln_Va_ns_x, ln_Va_ns_y]

    return ln_list

if ani_select == 1:
    ani = animation.FuncAnimation(fig_xy, animate_deviation, len(data[topic_pos]['t']), interval=dt*1000, blit=True)
elif ani_select == 2:
    ani = animation.FuncAnimation(fig, animate_splines, len(data[topic_first]['t']), interval=dt*1000, blit=True)

plt.show()
# ani.save('wbc.mov')