from PyQt5 import QtGui, QtWidgets
import sys

from dialog import Ui_Dialog

class mywindow(QtWidgets.QWidget, Ui_Dialog):
    def __init__(self):
        super(mywindow, self).__init__()
        self.setupUi(self)

    def accept(self):
        print("accept")
    def reject(self):
        print("reject")


app = QtWidgets.QApplication(sys.argv)
window = mywindow()
window.show()
sys.exit(app.exec_())