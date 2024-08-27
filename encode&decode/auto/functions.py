import rospy
from moveit_commander import RobotCommander, MoveGroupCommander
from moveit_msgs.msg import ExecuteTrajectoryActionGoal
import datetime
import os
import re
import sys
import numpy as np
import ast

def normalize_vector(v): # ベクトルを正規化する関数
    norm = np.linalg.norm(v)
    if norm == 0: # ノルムがゼロのときは正規化できない
       return v
    return v / norm

def cross_three_dim(vector1, vector2): # 3次元の外積演算を行う関数
    result = np.array([
        [vector1[1]*vector2[2] - vector1[2]*vector2[1]],
        [vector1[2]*vector2[0] - vector1[0]*vector2[2]],
        [vector1[0]*vector2[1] - vector1[1]*vector2[0]]
    ])
    return result

def convert_data(matrix): # decordにおけるファイル読み込みを容易にするためのに変換する関数
    list = matrix.tolist()
    str_list = ['[' + ', '.join(map(str, sublist)) + ']' for sublist in list]
    output = '\n'.join(str_list)
    return output

def read_matrix_from_file(file_path): # ファイルからウェイポイントだけを読み取る関数（sとgを除く）
    matrix = []
    with open(file_path, "r") as file:
        for line in file:
            values = line.strip().strip("[]").split(", ")
            row = [float(value) for value in values]
            matrix.append(row)
    return np.array(matrix[1:-1])

def colored_text(text, color_code): # ターミナルでのテキストの色を変える関数
    return f"\033[{color_code}m{text}\033[0m"

def set_equal_axis(ax, xlims, ylims, zlims):
    max_range = np.array([xlims.max()-xlims.min(), ylims.max()-ylims.min(), zlims.max()-zlims.min()]).max() / 2.0
    mid_x = (xlims.max() + xlims.min()) * 0.5
    mid_y = (ylims.max() + ylims.min()) * 0.5
    mid_z = (zlims.max() + zlims.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)
    return

def extract_and_save_angles(directory):
    traj_path = os.path.join(directory, "traj.txt")
    angle_path = os.path.join(directory, "angle.txt")
    
    with open(traj_path, 'r') as file:
        data = file.read()
        pattern = r'positions:\s*(\[.*?\])'
        matches = re.findall(pattern, data)

        with open(angle_path, 'w') as angle_file:
            for match in matches:
                match_with_comma = match.replace(']', ']')
                angle_file.write(match_with_comma + '\n')

def process_data_and_continue(directory, mode):
    waypoint_file = os.path.join(directory, "angle.txt")

    try:
        with open(waypoint_file, "r") as f:
            lines = f.readlines()
            s = np.array(ast.literal_eval(lines[0])).reshape(-1,1) # sを格納（ファイルの最初の行）
            g = np.array(ast.literal_eval(lines[-1])).reshape(-1,1) # gを格納（ファイルの最後の行）

            s1,s2 = np.split(s, 2, axis=0)
            g1,g2 = np.split(g, 2, axis=0)

            # エンコード

            w_num = len(lines) - 2 # ウェイポイントの数（sとgを除くから「-2」）
            v_array = g - s
            v1_array = g1 - s1
            v2_array = g2 - s2
            e_array = np.array(normalize_vector(v_array))
            e1 = np.array(normalize_vector(v1_array))
            e2 = np.array(normalize_vector(v2_array))

            w_array = np.empty((6,0), float)
            for i in range(1, len(lines) - 1):
                    w = np.array(ast.literal_eval(lines[i])).reshape(-1,1) # wを格納
                    w_array = np.hstack([w_array, w])
            w1,w2 = np.split(w_array, 2, axis=0)

            l1_array = np.empty((3,0), float)
            l2_array = np.empty((3,0), float)
            for i in range(w_num):
                l1 = s1 + np.dot((w_array[:3,i].reshape(-1,1) - s1).T, e1) * e1
                l2 = s2 + np.dot((w_array[3:6,i].reshape(-1,1) - s2).T, e2) * e2
                l1_array = np.hstack([l1_array, l1])
                l2_array = np.hstack([l2_array, l2])
            l_array = np.concatenate((l1_array, l2_array), axis=0)

            r1_array = np.empty((1,0), float)
            r2_array = np.empty((1,0), float)
            for i in range(w_num):
                r1 = ((np.linalg.norm(l1_array[:,i] - s1.T))/((np.linalg.norm(v1_array)))).reshape(-1,1) # np.linalg.normでノルムを計算
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
            tau_array = np.concatenate((tau1_array, tau2_array), axis=0)

            delta1_array = np.empty((0,3), float)
            delta2_array = np.empty((0,3), float)
            wl1 = w1 - l1_array
            wl2 = w2 - l2_array
            for i in range(w_num):
                delta1 = cross_three_dim(e1, wl1[:,i]) # 外積を計算
                delta2 = cross_three_dim(e2, wl2[:,i])
                delta1 = np.squeeze(delta1)# 不要な次元を削除
                delta2 = np.squeeze(delta2)
                normlaize_delta1 = np.array(normalize_vector(delta1)) # 正規化
                normlaize_delta2 = np.array(normalize_vector(delta2))
                delta1_array = np.vstack([delta1_array, normlaize_delta1])
                delta2_array = np.vstack([delta2_array, normlaize_delta2])
            delta_array = np.concatenate((delta1_array, delta2_array), axis=1)
            delta_plot_array = delta_array.T

            if mode == "c":
                file_name = input("Enter the name of the pathseed file >> ")
                with open(file_name, "w") as file: # パスシードを生成
                    file.write(convert_data(r1_array) + "\n")
                    file.write(convert_data(r2_array) + "\n")
                    file.write(convert_data(tau1_array) + "\n")
                    file.write(convert_data(tau2_array) + "\n")
                    file.write(convert_data(delta_array) + "\n")

            # デコード

            if mode == "b":
                text4 = colored_text("[NOTICE] If you want to change the new start and goal, please edit it in this file.", "34")
                print(text4)
                s_new = [-0.9539568448800573, 0.14286462209588358, -2.4228928517236046, 8.308474924234588e-05, 2.280063497889632, -0.9539539344058481]# 新たなs,gを与える
                g_new = [0.8136175984563769, -0.09024680608716136, -2.1256102835988493, -5.242347809818976e-05, 2.215805905627148, 0.8135918323523483]
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

                r1_array = np.squeeze(r1_array) # 不要な次元を削除
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
                l_array_d = np.concatenate((l3_array, l4_array), axis=0)

                w1_array = np.empty((0,3), float)
                w2_array = np.empty((0,3), float)
                for i in range(w_num):
                    cross1 = cross_three_dim(delta1_array.T[:,i], e1) # 外積を計算
                    cross2 = cross_three_dim(delta2_array.T[:,i], e2)
                    cross1 = np.squeeze(cross1) # 不要な次元を削除
                    cross2 = np.squeeze(cross2)
                    w1 = l3_array[:,i] + tau1_array[i] * (cross1)
                    w2 = l4_array[:,i] + tau2_array[i] * (cross2)
                    w1_array = np.vstack([w1_array, w1])
                    w2_array = np.vstack([w2_array, w2])
                w = np.concatenate((w1_array, w2_array), axis=1)
                w_plot_array = w.T
                converted_data = convert_data(w)
                print("w:\n",converted_data) # 完成したウェイポイント

                waypoint = read_matrix_from_file(waypoint_file)
                error = waypoint - w # 誤差の計算
                print("Error for each element:")
                for row in error:
                    print(row)
                
                return converted_data

            if mode == "a":
                r1_array = np.squeeze(r1_array) # 不要な次元を削除
                r2_array = np.squeeze(r2_array)
                tau1_array = np.squeeze(tau1_array)
                tau2_array = np.squeeze(tau2_array)
                delta_array = np.squeeze(delta_array)

                l3_array = np.empty((3,0), float)
                l4_array = np.empty((3,0), float)

                for i in range(w_num):
                    l3 = s1 + (r1_array[i] * v1_array)
                    l4 = s2 + (r2_array[i] * v2_array)
                    l3_array = np.hstack([l1_array, l1])
                    l4_array = np.hstack([l2_array, l2])
                l_array_d = np.concatenate((l3_array, l4_array), axis=0)

                w1_array = np.empty((0,3), float)
                w2_array = np.empty((0,3), float)
                for i in range(w_num):
                    cross1 = cross_three_dim(delta1_array.T[:,i], e1) # 外積を計算
                    cross2 = cross_three_dim(delta2_array.T[:,i], e2)
                    cross1 = np.squeeze(cross1) # 不要な次元を削除
                    cross2 = np.squeeze(cross2)
                    w1 = l3_array[:,i] + tau1_array[i] * (cross1)
                    w2 = l4_array[:,i] + tau2_array[i] * (cross2)
                    w1_array = np.vstack([w1_array, w1])
                    w2_array = np.vstack([w2_array, w2])
                w = np.concatenate((w1_array, w2_array), axis=1)
                w_plot_array = w.T
                converted_data = convert_data(w)
                print("w:\n",convert_data(w)) # 完成したウェイポイント

                return converted_data
                
    except FileNotFoundError:
        rospy.logerr("File not found")

    return

def process_angle_file(directory, file_name):
    angle_path = os.path.join(directory, file_name)
    
    if not os.path.exists(angle_path):
        rospy.logwarn(f"{file_name} does not exist")
        return

    with open(angle_path, 'r') as file:
        data = file.read()
    
    # Remove '[' and replace ']' with ','
    formatted_data = data.replace('[', '').replace(']', ',')[:-1]

    formatted_data = formatted_data.rstrip(',') + ';'
    
    num_rows = data.count('\n')

    # Count the number of columns in the first row
    first_row = data.split('\n')[0].replace('[', '').replace(']', '').split(',')
    num_columns = len(first_row)

    # Prepare the structured data
    structured_data = {
        'rows': num_rows,
        'columns': num_columns,
        'data': formatted_data
    }
    
    # Save the formatted data to a new file
    formatted_angle_path = os.path.join(directory, f"formatted_{file_name}.txt")
    with open(formatted_angle_path, 'w') as file:
        file.write("rows:\n")
        file.write(f"    {num_rows}\n")
        file.write("columns:\n")
        file.write(f"    {num_columns}\n")
        file.write("data:\n")
        file.write(f"    {formatted_data}\n")

    rospy.loginfo("Saved formatted data")

def saved_path_to_pathseed(data):
    # 現在の日時を取得してフォーマット
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    # pathseedsフォルダ内にタイムスタンプでサブフォルダを作成
    directory = os.path.join("pathseeds", "pathseed_" + timestamp)
    if not os.path.exists(directory):
        os.makedirs(directory)
    # traj.txtファイルをサブフォルダ内に保存
    traj_filename = os.path.join(directory, "traj.txt")
    with open(traj_filename, "w") as file:
        file.write(str(data))

    # 2. trajectoryの中から関節角度を抽出して保存
    # traj.txtからデータを抽出してangle.txtに保存
    extract_and_save_angles(directory)
    rospy.loginfo("Extracted and saved angles")

    # if angle.txt is exist, then process the data
    if not os.path.exists(os.path.join(directory, "angle.txt")):
        rospy.logwarn("angle.txt does not exist")
        return

    # 3. angle.txtのデータを処理
    # process_angle_file(directory, "angle.txt")

    # 3. 新たなスタートとゴールを設定して新たなパスを生成（展開）
    # モードを選択
    text1 = colored_text("a: The new start and goal are the same", "31") # モードの選択
    text2 = colored_text("b: The new start and goal are different", "34")
    text3 = colored_text("c: Save only seed paths to a new file", "33")
    print(text1)
    print(text2)
    print(text3)
    mode = input("Select a mode >> ")

    # データ処理と続行の処理に移行
    pathseed = process_data_and_continue(directory, mode)
    rospy.loginfo("Processed data and continued")

    # 4. パスシードを保存
    if pathseed is not None:
        pathseed_filename = os.path.join(directory, "pathseed")
        with open(pathseed_filename, "w") as file:
            file.write(pathseed)
        
        # データを処理してprocess.txtに保存
        process_angle_file(directory, "pathseed.txt")

        rospy.loginfo("Saved pathseed")

def to_given_goal():
    rospy.init_node("xArm6")
    robot = RobotCommander()
    xarm = MoveGroupCommander("xarm6")

    # ゴールの設定(関節角度で指定)
    fixed_joint_values = [0.9704946662267524, -0.5775184827066822, -0.9503822306458227, 0.00010799484719203889, 1.5279174894136958, 0.9704959568637523]

    xarm.set_start_state_to_current_state()
    xarm.set_joint_value_target(fixed_joint_values)

    # プランニング
    success, plan, _, _ = xarm.plan()

    if not success:
        rospy.logerr("Planning failed")
        return
    else:
        rospy.loginfo("Received trajectory goal")
        saved_path_to_pathseed(plan)

def callback(data):
    rospy.loginfo("Received trajectory goal")
    saved_path_to_pathseed(data)

def listener():
    rospy.init_node('trajectory_listener', anonymous=True)
    print("Waiting for trajectory goal")
    rospy.Subscriber("/execute_trajectory/goal", ExecuteTrajectoryActionGoal, callback)
    rospy.spin()
