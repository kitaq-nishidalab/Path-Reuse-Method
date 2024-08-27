import numpy as np
import ast
import sys
import datetime
from functions import (normalize_vector, cross_three_dim, convert_data, colored_text)

np.set_printoptions(threshold=np.inf)
np.set_printoptions(linewidth=np.inf)
np.set_printoptions(suppress=True)

###### User Input Section Below ##################################################################################

# Input for generating a path with a new start and goal
s_new = [-0.9539568448800573, 0.14286462209588358, -2.4228928517236046, 8.308474924234588e-05, 2.280063497889632, -0.9539539344058481] # New start joint angles
g_new = [0.8440930198932115, -0.09426596864204129, -2.120484722079457, -5.4759826845440784e-05, 2.2146980161053804, 0.8440667593654201] # New goal joint angles

################################################################################################################


waypoint_file = sys.argv[1]
mode1 = input(colored_text("Select a mode \n 1: Generate a new path with different start and goal \n 2: Save pathseed to a file \n >>>>>>>>>>>>> " , "33"))
with open(waypoint_file, "r") as f:
    lines = f.readlines()
    s = np.array(ast.literal_eval(lines[0])).reshape(-1,1)
    g = np.array(ast.literal_eval(lines[-1])).reshape(-1,1)
s1,s2 = np.split(s, 2, axis=0)
g1,g2 = np.split(g, 2, axis=0)

# encode
w_num = len(lines) - 2
v_array = g - s
v1_array = g1 - s1
v2_array = g2 - s2
e_array = np.array(normalize_vector(v_array))
e1 = np.array(normalize_vector(v1_array))
e2 = np.array(normalize_vector(v2_array))

w_array = np.empty((6,0), float)
for i in range(1, len(lines) - 1):
        w = np.array(ast.literal_eval(lines[i])).reshape(-1,1)
        w_array = np.hstack([w_array, w])
w1,w2 = np.split(w_array, 2, axis=0)

l1_array = np.empty((3,0), float)
l2_array = np.empty((3,0), float)
for i in range(w_num):
    l1 = s1 + np.dot((w_array[:3,i].reshape(-1,1) - s1).T, e1) * e1
    l2 = s2 + np.dot((w_array[3:6,i].reshape(-1,1) - s2).T, e2) * e2
    l1_array = np.hstack([l1_array, l1])
    l2_array = np.hstack([l2_array, l2])

r1_array = np.empty((1,0), float)
r2_array = np.empty((1,0), float)
for i in range(w_num):
    r1 = ((np.linalg.norm(l1_array[:,i] - s1.T))/((np.linalg.norm(v1_array)))).reshape(-1,1)
    r1_array = np.hstack([r1_array, r1])
    r2 = ((np.linalg.norm(l2_array[:,i] - s2.T))/((np.linalg.norm(v2_array)))).reshape(-1,1)
    r2_array = np.hstack([r2_array, r2])

tau1_array = np.empty((1,0), float)
tau2_array = np.empty((1,0), float)
for i in range(w_num):
    tau1 = np.linalg.norm(w1[:,i] - l1_array[:,i]).reshape(-1,1)
    tau2 = np.linalg.norm(w2[:,i] - l2_array[:,i]).reshape(-1,1)
    tau1_array = np.hstack([tau1_array, tau1])
    tau2_array = np.hstack([tau2_array, tau2])

delta1_array = np.empty((0,3), float)
delta2_array = np.empty((0,3), float)
wl1 = w1 - l1_array
wl2 = w2 - l2_array
for i in range(w_num):
    delta1 = cross_three_dim(e1, wl1[:,i])
    delta2 = cross_three_dim(e2, wl2[:,i])
    delta1 = np.squeeze(delta1)
    delta2 = np.squeeze(delta2)
    normlaize_delta1 = np.array(normalize_vector(delta1))
    normlaize_delta2 = np.array(normalize_vector(delta2))
    delta1_array = np.vstack([delta1_array, normlaize_delta1])
    delta2_array = np.vstack([delta2_array, normlaize_delta2])
delta_array = np.concatenate((delta1_array, delta2_array), axis=1)

# decode
if mode1 == "1":
    print(colored_text("[NOTICE] If you want to change the new start and goal, please edit it in this file.", "34"))
    s = np.array(s_new).reshape(-1,1)
    g = np.array(g_new).reshape(-1,1)
    s1,s2 = np.split(s, 2, axis=0)
    g1,g2 = np.split(g, 2, axis=0)
    v_array = g - s
    v1_array = g1 - s1
    v2_array = g2 - s2
    e_array = np.array(normalize_vector(v_array))
    e1 = np.array(normalize_vector(v1_array))
    e2 = np.array(normalize_vector(v2_array))

    r1_array = np.squeeze(r1_array)
    r2_array = np.squeeze(r2_array)
    tau1_array = np.squeeze(tau1_array)
    tau2_array = np.squeeze(tau2_array)
    delta_array = np.squeeze(delta_array)

    l3_array = np.empty((3,0), float)
    l4_array = np.empty((3,0), float)
    for i in range(w_num):
        l3 = s1 + (r1_array[i] * v1_array)
        l4 = s2 + (r2_array[i] * v2_array)
        l3_array = np.hstack([l3_array, l3])
        l4_array = np.hstack([l4_array, l4])

    w1_array = np.empty((0,3), float)
    w2_array = np.empty((0,3), float)
    for i in range(w_num):
        cross1 = cross_three_dim(delta1_array.T[:,i], e1)
        cross2 = cross_three_dim(delta2_array.T[:,i], e2)
        cross1 = np.squeeze(cross1)
        cross2 = np.squeeze(cross2)
        w1 = l3_array[:,i] + tau1_array[i] * (cross1)
        w2 = l4_array[:,i] + tau2_array[i] * (cross2)
        w1_array = np.vstack([w1_array, w1])
        w2_array = np.vstack([w2_array, w2])
    w = np.concatenate((w1_array, w2_array), axis=1)

    mode2 = input(colored_text("Select a mode \n 1: Display generated waypoints \n 2: Save generated waypoints to a file \n >>>>>>>>>>>>> ","33"))
    if mode2 == "1":
      print("w:\n",convert_data(w))

    elif mode2 == "2":
      file_name2 = input("Enter the name of the file to save generated waypoints >> ")
      with open(file_name2, "w") as file:
        file.write(convert_data(w))
      print("Generated path saved to", file_name2)

    else:
      print(colored_text("Invalid mode selected", "31"))

elif mode1 == "2":
  file_name1 = "pathseed/pathseed_" + datetime.datetime.now().strftime("%Y%m%d%H%M%S") + ".seed"
  with open(file_name1, "w") as file:
    file.write(convert_data(r1_array) + "\n")
    file.write(convert_data(r2_array) + "\n")
    file.write(convert_data(tau1_array) + "\n")
    file.write(convert_data(tau2_array) + "\n")
    file.write(convert_data(delta_array) + "\n")
  print("Pathseed saved to", file_name1)

else:
    print(colored_text("Invalid mode selected", "31"))