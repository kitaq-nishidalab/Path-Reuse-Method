#!/usr/bin/env python3

import os
import rospy
from prompt_toolkit import prompt
from prompt_toolkit.completion import WordCompleter
from moveit_commander import RobotCommander, MoveGroupCommander

# FIRST_POINT_JOINT_VALUES = [1.64814883, -0.1098108, -1.79263395, -0.00285593, 1.88177633, 0.01209036]
# SECOND_POINT_JOINT_VALUES = [1.95332498, 0.70807293, -1.8769777, -0.02519488, 1.14922491, 0.29193703]
# THIRD_POINT_JOINT_VALUES = [-0.18247784, 0.2274629, -2.31973316, 0.02837033, 2.07522441, -0.21744345]
# FORTH_POINT_JOINT_VALUES = [-0.8694099, 0.37453012, -1.38156122, 2.03669573, 1.0239705, -2.34464359]
DEFAULT_GOAL_JOINT_VALUES = [1.64814883, -0.1098108, -1.79263395, -0.00285593, 1.88177633, 0.01209036]
PATHSEED_DIR = "/home/nishidalab/nishidalab_ws/src/Path-Reuse-Method/use_pathseed/pathseeds/"

def main():
   rospy.init_node("xArm6")
   robot = RobotCommander()
   xarm = MoveGroupCommander("xarm6")

   while not rospy.is_shutdown():
      # ゴールの設定(関節角度で指定)
      fixed_joint_values_input = input("Enter the joint values(example: [1.64814883, -0.1098108, -1.79263395, -0.00285593, 1.88177633, 0.01209036]): ")
      if not fixed_joint_values_input:
         rospy.logerr("No joint values entered. using default values.")
         fixed_joint_values = DEFAULT_GOAL_JOINT_VALUES
      else:
         try:
            fixed_joint_values = eval(fixed_joint_values_input)
         except Exception as e:
            rospy.logerr(f"Invalid input for joint values: {e}")
            return

      # pathseedファイルの選択
      pathseed_files = os.listdir(PATHSEED_DIR)
      pathseed_completer = WordCompleter(pathseed_files)
      pathseed_file = prompt("Select pathseed file(push the tab key): ", completer=pathseed_completer)
      
      if not pathseed_file or pathseed_file not in pathseed_files:
         rospy.logerr("Invalid or no pathseed file selected. using default values.")
         pathseed_file = "pathseed1.txt"
      
      pathseed_file = os.path.join(PATHSEED_DIR, pathseed_file)
      rospy.set_param("/pathseed_file", pathseed_file)

      xarm.set_start_state_to_current_state()
      xarm.set_joint_value_target(fixed_joint_values)

      # プランニング
      success, plan, _, _ = xarm.plan()

      if success:
         xarm.execute(plan)
      else:
         print("Planning failed.")

if __name__ == '__main__':
   main()
