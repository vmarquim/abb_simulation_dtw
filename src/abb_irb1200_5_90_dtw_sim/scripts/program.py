#!/usr/bin/env python
import rospy
import sys

from gui import TrajectoryDTWApp
from PyQt5.QtWidgets import QApplication


def main():
    rospy.init_node("move_group_test", anonymous=False)

    app = QApplication(sys.argv)
    window = TrajectoryDTWApp()
    window.show()
    sys.exit(app.exec_())

    
if __name__ == "__main__":
    try: 
        main()
    except rospy.ROSInterruptException:
        pass