import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os
import csv

#@title Import packagess for plotting and creating graphics
import time
import itertools
import numpy as np
from typing import Callable, NamedTuple, Optional, Union, List
import matplotlib.pyplot as plt

xml_path = 'motor_model.xml'
simend = 7
print_camera_config = 0

header = ['timestep', 'angular_velocity', 'torque']
file_name = "test_data.csv"
f = open('data/' + file_name, mode='w', newline='')
writer = csv.writer(f)
writer.writerow(header)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    #pass

    #spring-like position servo
    # set_position_servo(1, 10) # 1: actuator 2: gain
    # data.ctrl[1] = np.pi  #position

    #speed control; velocity servo
    set_velocity_servo(2, 0.5)
    data.ctrl[2] = 3  #velocity

    #position control; position/velocity servo
    # set_position_servo(1, 100)
    # set_velocity_servo(2,10)
    # data.ctrl[1] = np.pi #position

    #torque control;
    # set_torque_servo(0, 1)
    # data.ctrl[0] = -10*(data.qpos[0]-np.pi)  #torque (spring)
    # data.ctrl[0] = -100*(data.qvel[0]-0.5) #speed control]
    # data.ctrl[0] = -100*(data.qpos[0]-np.pi) -10*data.qvel[0] #position control

def set_torque_servo(actuator_no, flag):
    if (flag==0):
        model.actuator_gainprm[actuator_no, 0] = 0
    else:
        model.actuator_gainprm[actuator_no, 0] = 1

def set_position_servo(actuator_no, kp):
    model.actuator_gainprm[actuator_no, 0] = kp
    model.actuator_biasprm[actuator_no, 1] = -kp

def set_velocity_servo(actuator_no, kv):
    model.actuator_gainprm[actuator_no, 0] = kv
    model.actuator_biasprm[actuator_no, 2] = -kv


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

#get the full path
xml_path = os.path.abspath(xml_path)

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration
# cam.azimuth = 90
# cam.elevation = -45
# cam.distance = 2
# cam.lookat = np.array([0.0, 0.0, 0])
cam.azimuth = -0.2568020402892691 ; cam.elevation = -1.2800579336767537 ; cam.distance =  0.936735440362224
cam.lookat =np.array([ 0.0 , 0.0 , 0.0 ])

data.qpos[0] = np.pi/2

actual_speeds = [3.56592958919683,	4.55877935154993, 5.54824913615609, 6.54213921878825, 7.51894904882895, 8.52646175725307, 9.51331553503774, 10.5092064733105, 11.5075797297270, 12.4956671148198]
target_speeds = [3, 4, 5, 6, 7, 8, 9, 10, 11, 12]
gains = [0.1, 0.3, 0.5, 0.7, 0.9]

init_controller(model, data)

#set the controller
mj.set_mjcb_control(controller)

mj.mj_resetDataKeyframe(model, data, 0)
print(model.body_mass)

time_steps = []
angular_velocity = []
torque = []

while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        mj.mj_step(model, data)

    if (data.time>=simend):
        break;
    # DATA READOUTS: 
    cur_torque = data.sensor('torque_sensor').data[0]
    cur_vel = data.actuator_velocity[2]
    time = data.time
    time_steps.append(data.time)
    angular_velocity.append(cur_vel)
    torque.append(cur_torque)
    row = [str(time), str(cur_vel), str(cur_torque)]
    writer.writerow(row)
    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

dpi = 120
width = 600
height = 800
figsize = (width / dpi, height / dpi)
_, ax = plt.subplots(2, 1, figsize=figsize, dpi=dpi, sharex=True)

ax[0].plot(time_steps, angular_velocity)
ax[0].set_title('angular velocity')
ax[0].set_ylabel('radians / second')

ax[1].plot(time_steps, torque)
ax[1].set_xlabel('time (seconds)')
ax[1].set_ylabel('Torque')
_ = ax[1].set_title('N*m')
plt.show()

f.close()
glfw.terminate()
