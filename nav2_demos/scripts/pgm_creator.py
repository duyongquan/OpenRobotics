import sys
from PyQt5.QtWidgets import QApplication, QGraphicsScene, QGraphicsView, QScrollArea, QFileDialog, QWidget, QFrame, QPushButton, QToolTip, QDesktopWidget, QGridLayout, QLabel, QSizePolicy, QGraphicsItem, QColorDialog, QComboBox, QHBoxLayout, QVBoxLayout, QSlider, QLineEdit, QCheckBox
from PyQt5.QtGui import QFont, QPainter, QBrush, QColor, QIntValidator
from PyQt5.QtCore import Qt, QRectF, QLineF


class Field(QGraphicsItem):
    """ Cell object which is drawn as a square, containing a color and ID for the pgm. """
    def __init__(self, x, y, w, idx=0):
        super(Field, self).__init__()
        self.setFlag(QGraphicsItem.ItemIsSelectable, True)
        self.x = x
        self.y = y
        self.width = w
        self.idx = idx
        self.rectF = QRectF(x, y, w, w)
        self._brush = QBrush(Qt.white)

    def setBrush(self, brush):
        self._brush = brush
        self.update()

    def boundingRect(self):
        return self.rectF

    def paint(self, painter=None, style=None, widget=None):
        painter.fillRect(self.rectF, self._brush)
        painter.drawRect(self.rectF)

    def setIdx(self, idx):
        self.idx = idx


class GeomScene(QGraphicsScene):
    """ GraphicsScene containing the grid. """
    def __init__(self, parent, xDim=20, yDim=10, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.parent = parent
        self.drawing = False
        self.brush_size = 1
        self.width = 20
        self.cols = xDim
        self.rows = yDim
        self.top = 0
        self.bot = self.rows * self.width
        self.left = 0
        self.right = self.cols * self.width

        positions = [(i, j) for i in range(self.cols) for j in range(self.rows)]
        for (i, j) in positions:
            rect = Field(i * self.width, j * self.width, self.width)
            rect.setBrush(QColor(self.parent.activeColor))
            rect.setIdx(self.parent.activeID)
            self.addItem(rect)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            for square in self.items():
                if (abs(event.scenePos().x() - square.sceneBoundingRect().center().x()) < (self.brush_size / 2) * self.width
                    and abs(event.scenePos().y() -  square.sceneBoundingRect().center().y()) < (self.brush_size / 2) * self.width):
                    square.setBrush(QColor(self.parent.activeColor))
                    square.setIdx(self.parent.activeID)
                    square.update()
            self.drawing = True

    def mouseMoveEvent(self, event):
        if self.drawing:
            for square in self.items():
                if (abs(event.scenePos().x() - square.sceneBoundingRect().center().x()) < (self.brush_size / 2) * self.width
                    and abs(event.scenePos().y() -  square.sceneBoundingRect().center().y()) < (self.brush_size / 2) * self.width):
                    square.setBrush(QColor(self.parent.activeColor))
                    square.setIdx(self.parent.activeID)
                    square.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drawing = False

    def addTop(self):
        for i in range(self.cols):
            rect = Field(i * self.width + self.left, self.top - self.width, self.width, self.views()[0].parent.activeID)
            rect.setBrush(QColor(self.views()[0].parent.activeColor))
            self.addItem(rect)
        self.top -= self.width

    def addBottom(self):
        for i in range(self.cols):
            rect = Field(i * self.width + self.left, self.bot, self.width, self.views()[0].parent.activeID)
            rect.setBrush(QColor(self.views()[0].parent.activeColor))
            self.addItem(rect)
        self.bot += self.width

    def addLeft(self):
        for i in range(self.rows):
            rect = Field(self.left - self.width, i * self.width + self.top, self.width, self.views()[0].parent.activeID)
            rect.setBrush(QColor(self.views()[0].parent.activeColor))
            self.addItem(rect)
        self.left -= self.width

    def addRight(self):
        for i in range(self.rows):
            rect = Field(self.right, i * self.width + self.top, self.width, self.views()[0].parent.activeID)
            rect.setBrush(QColor(self.views()[0].parent.activeColor))
            self.addItem(rect)
        self.right += self.width

    def removeTop(self):
        for item in self.items( self.left, self.top, self.right-self.left, self.width / 2, Qt.IntersectsItemBoundingRect, Qt.AscendingOrder):
            self.removeItem(item)
        self.top += self.width

    def removeBottom(self):
        for item in self.items( self.left, self.bot - self.width / 2, self.right - self.left, self.width / 2, Qt.IntersectsItemBoundingRect, Qt.AscendingOrder):
            self.removeItem(item)
        self.bot -= self.width

    def removeLeft(self):
        for item in self.items( self.left, self.top, self.width / 2, self.bot - self.top, Qt.IntersectsItemBoundingRect, Qt.AscendingOrder):
            self.removeItem(item)
        self.left += self.width

    def removeRight(self):
        for item in self.items( self.right - self.width / 2, self.top, self.width / 2, self.bot - self.top, Qt.IntersectsItemBoundingRect, Qt.AscendingOrder):
            self.removeItem(item)
        self.right -= self.width

    def setBrushSize(self, value):
        self.brush_size = value


class Geom(QGraphicsView):
    """ Graphics view with scrolling and dragging functionality. """
    def __init__(self, parent, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.parent = parent
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.setDragMode(QGraphicsView.NoDrag)
        self.startPos = None
        self.scene = None

    def wheelEvent(self, event):
        if event.angleDelta().y() > 0:
            self.scale(1.25, 1.25)
        else:
            self.scale(0.8, 0.8)

    def mousePressEvent(self, event):
        if event.button() == Qt.RightButton:
            self.startPos = event.pos()
        super(Geom, self).mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self.startPos is not None:
            delta = self.startPos - event.pos()
            transform = self.transform()
            # m11 refers to the horizontal scale, m22 to the vertical scale;
            deltaX = delta.x() / transform.m11()
            deltaY = delta.y() / transform.m22()
            self.setSceneRect(self.sceneRect().translated(deltaX, deltaY))
            self.startPos = event.pos()
        else:
            super(Geom, self).mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.RightButton:
            self.startPos = None
        super(Geom, self).mouseReleaseEvent(event)

    def addLine(self, side):
        if side == 'top':
            self.scene.addTop()
            self.scene.rows += 1
            self.parent.lneYDim.setText(str(self.scene.rows))
        elif side == 'bottom':
            self.scene.addBottom()
            self.scene.rows += 1
            self.parent.lneYDim.setText(str(self.scene.rows))
        elif side == 'right':
            self.scene.addRight()
            self.scene.cols += 1
            self.parent.lneXDim.setText(str(self.scene.cols))
        elif side == 'left':
            self.scene.addLeft()
            self.scene.cols += 1
            self.parent.lneXDim.setText(str(self.scene.cols))

    def removeLine(self, side):
        if side == 'top':
            self.scene.removeTop()
            self.scene.rows -= 1
            self.parent.lneYDim.setText(str(self.scene.rows))
        elif side == 'bottom':
            self.scene.removeBottom()
            self.scene.rows -= 1
            self.parent.lneYDim.setText(str(self.scene.rows))
        elif side == 'right':
            self.scene.removeRight()
            self.scene.cols -= 1
            self.parent.lneXDim.setText(str(self.scene.cols))
        elif side == 'left':
            self.scene.removeLeft()
            self.scene.cols -= 1
            self.parent.lneXDim.setText(str(self.scene.cols))

    def resetScene(self, scene):
        self.scene = scene
        self.setScene(scene)


class CellChoice(QHBoxLayout):
    """ 
    GUI element that contains name, color and ID for a cell category. 
    Can be added dynamically to the mainwindow.
    """
    def __init__(self, name, id, col='white', parent=None):
        super().__init__()
        self.name = name
        self.id = id
        self.color = col
        self.parent = parent
        self.initUI()

    def initUI(self):
        self.lblName = QLineEdit(self.name)
        self.lblName.setAlignment(Qt.AlignCenter)
        self.addWidget(self.lblName)
        self.btnColor = QPushButton('')
        self.btnColor.setStyleSheet("background-color: {}; min-width: 15px".format(self.color))
        self.addWidget(self.btnColor)
        self.btnColor.clicked.connect(self.changeColor)  
        self.cmbID = QComboBox()
        self.cmbID.addItems([str(num) for num in range(128)])
        self.cmbID.setCurrentIndex(self.id)
        self.cmbID.setStyleSheet("combobox-popup: 0")
        self.cmbID.currentIndexChanged.connect(self.changeID)
        self.addWidget(self.cmbID)
        self.selectBox = QCheckBox()
        self.selectBox.setStyleSheet("QCheckBox::indicator {width: 20px; height: 20px;} ") #QCheckBox::indicator:checked {background-color: green}")
        self.addWidget(self.selectBox)
        self.selectBox.released.connect(self.select)
        if self.id == 0:
            self.selectBox.setCheckState(Qt.Checked)
            self.parent.activeColor = self.color
            self.parent.activeID = self.id

    def changeColor(self):
        """ Select new color for the category. """
        col = QColorDialog.getColor()
        if col.isValid():
            self.color = col.name()
        self.btnColor.setStyleSheet("background-color: {}; min-width: 15px".format(self.color))
        if self.selectBox.checkState() == Qt.Checked:
            self.parent.activeColor = self.color
        for item in self.parent.scene.items():
            if item.idx == self.id:
                item.setBrush(QColor(self.color))

    def changeID(self, idx):
        for item in self.parent.scene.items():
            if item.idx == self.id:
                item.idx = idx
        self.id = idx
        if self.selectBox.checkState() == Qt.Checked:
            self.parent.activeID = self.id

    def select(self):
        """ Selects checkbox and deletes previous selection"""
        for cat in self.parent.categories:
            cat.selectBox.setCheckState(Qt.Unchecked)
        self.selectBox.setCheckState(Qt.Checked)
        self.parent.activeColor = self.color
        self.parent.activeID = self.id


class MainWidget(QWidget):
    """ Main GUI """
    def __init__(self):
        super().__init__()
        
        self.categories = []
        self.activeColor = 'lightblue'
        self.activeID = 0

        self.initUI()

    def initUI(self):
        """Set up GUI"""
        self.grid = QGridLayout()
        self.setLayout(self.grid)
        self.grid.setColumnStretch(9, 1)

        self.view = Geom(parent=self)
        self.scene = GeomScene(self, 20, 10)
        self.view.resetScene(self.scene)
        self.grid.addWidget(self.view, 1, 9, -1, 1)

        QToolTip.setFont(QFont('SansSerif', 10))

        btnTopPlus = QPushButton('+', self)
        btnTopPlus.setFixedSize(30, 25)
        btnTopPlus.clicked.connect(lambda: self.view.addLine('top'))
        btnTopMinus = QPushButton('-', self)
        btnTopMinus.setFixedSize(30, 25)
        btnTopMinus.clicked.connect(lambda: self.view.removeLine('top'))

        btnsTop = QHBoxLayout()
        btnsTop.setSpacing(0)
        btnsTop.addWidget(btnTopPlus)
        btnsTop.addWidget(btnTopMinus)
        self.grid.addLayout(btnsTop, 1, 9, -1, 1, alignment=(Qt.AlignTop | Qt.AlignHCenter))

        btnBotPlus = QPushButton('+', self)
        btnBotPlus.setFixedSize(30, 25)
        btnBotPlus.clicked.connect(lambda: self.view.addLine('bottom'))
        btnBotMinus = QPushButton('-', self)
        btnBotMinus.setFixedSize(30, 25)
        btnBotMinus.clicked.connect(lambda: self.view.removeLine('bottom'))

        btnsBot = QHBoxLayout()
        btnsBot.setSpacing(0)
        btnsBot.addWidget(btnBotPlus)
        btnsBot.addWidget(btnBotMinus)
        self.grid.addLayout(btnsBot, 1, 9, -1, 1, alignment=(Qt.AlignBottom | Qt.AlignHCenter))

        btnLeftPlus = QPushButton('+', self)
        btnLeftPlus.setFixedSize(25, 30)
        btnLeftPlus.clicked.connect(lambda: self.view.addLine('left'))
        btnLeftMinus = QPushButton('-', self)
        btnLeftMinus.setFixedSize(25, 30)
        btnLeftMinus.clicked.connect(lambda: self.view.removeLine('left'))

        btnsLeft = QVBoxLayout()
        btnsLeft.setSpacing(0)
        btnsLeft.addWidget(btnLeftPlus)
        btnsLeft.addWidget(btnLeftMinus)
        self.grid.addLayout(btnsLeft, 1, 9, -1, 1, alignment=Qt.AlignLeft)

        btnRightPlus = QPushButton('+', self)
        btnRightPlus.setFixedSize(25, 30)
        btnRightPlus.clicked.connect(lambda: self.view.addLine('right'))

        btnRightMinus = QPushButton('-', self)
        btnRightMinus.setFixedSize(25, 30)
        btnRightMinus.clicked.connect(lambda: self.view.removeLine('right'))

        btnsRight = QVBoxLayout()
        btnsRight.setSpacing(0)
        btnsRight.addWidget(btnRightPlus)
        btnsRight.addWidget(btnRightMinus)
        self.grid.addLayout(btnsRight, 1, 9, -1, 1, alignment=Qt.AlignRight)

        intValidator = QIntValidator(1, 10000)
        lblXDim = QLabel('X:')
        self.lneXDim = QLineEdit('20')
        self.lneXDim.setValidator(intValidator)
        self.lneXDim.setFixedWidth(48)
        self.lneXDim.setAlignment(Qt.AlignCenter)
        lblYDim = QLabel('Y:')
        self.lneYDim = QLineEdit('10')
        self.lneYDim.setValidator(intValidator)
        self.lneYDim.setFixedWidth(48)
        self.lneYDim.setAlignment(Qt.AlignCenter)
        btnResetGrid = QPushButton('Reset Grid')
        btnResetGrid.clicked.connect(self.resetGrid)
        gridLayout = QHBoxLayout()
        gridLayout.addWidget(lblXDim)
        gridLayout.addWidget(self.lneXDim)
        gridLayout.addWidget(lblYDim)
        gridLayout.addWidget(self.lneYDim)
        btnResetGrid.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        gridLayout.addWidget(btnResetGrid, 1)


        self.grid.addLayout(gridLayout, 0, 0)

        fluidChoice = CellChoice('Fluid', 0, 'lightblue', parent=self)
        self.grid.addLayout(fluidChoice, 1, 0)
        inletChoice = CellChoice('Inlet', 1, 'salmon', parent=self)
        self.grid.addLayout(inletChoice, 2, 0)
        outletChoice = CellChoice('Outlet', 2, 'darkred', parent=self)
        self.grid.addLayout(outletChoice, 3, 0)
        wallChoice = CellChoice('Wall', 3, 'dimgrey', parent=self)
        self.grid.addLayout(wallChoice, 4, 0)
        self.categories = [fluidChoice, inletChoice, outletChoice, wallChoice]

        brushCell = QHBoxLayout()
        lblBrushSizeName = QLabel('Brush Size: ')
        brushCell.addWidget(lblBrushSizeName)
        self.grid.addLayout(brushCell, 19, 0)
        sldBrushSize = QSlider(Qt.Horizontal)
        sldBrushSize.setRange(1, 20)
        sldBrushSize.setPageStep(2)
        sldBrushSize.valueChanged.connect(self.setBrushSize)
        brushCell.addWidget(sldBrushSize)
        self.lblBrushSize = QLabel('1')
        brushCell.addWidget(self.lblBrushSize)

        btnAdd = QPushButton('Add Color', self)
        btnAdd.setToolTip('This is a <b>QPushButton</b> widget')
        btnAdd.resize(btnAdd.sizeHint())
        btnAdd.clicked.connect(self.addCategory)
        self.grid.addWidget(btnAdd, 20, 0)

        btnSave = QPushButton('Save PGM', self)
        btnSave.resize(btnAdd.sizeHint())
        btnSave.clicked.connect(self.savePGM)
        self.grid.addWidget(btnSave, 21, 0)

        self.setGeometry(300, 300, 1000, 600)
        self.center()
        self.setWindowTitle('PGM Creator')

        self.show()

    def center(self):
        """ Open application in the center of the display. """
        qr = self.frameGeometry()
        cp = QDesktopWidget().availableGeometry(-1).center()
        qr.moveCenter(cp)
        self.move(qr.topLeft())

    def addCategory(self):
        """ Adds a new cell category to the GUI. """
        choice = CellChoice('name', len(self.categories), parent=self)
        self.grid.addLayout(choice, len(self.categories) + 1, 0)

    def setBrushSize(self, size):
        """ Set the brush size to a square of size cells. """
        self.scene.setBrushSize(size)
        self.lblBrushSize.setText(str(size))

    def resetGrid(self):
        """ Reset the grid to given dimensions and set cells to active category. """
        xDim = int(self.lneXDim.text())
        yDim = int(self.lneYDim.text())
        self.scene = GeomScene(self, xDim, yDim)
        self.view.resetScene(self.scene)

    def savePGM(self):
        """ Save the grid as a pgm file. """
        name, extension = QFileDialog.getSaveFileName(self, 'Save File', '', " PGM File(*.pgm);; All Files (*)")
        if name != '':
            with open(name, 'w') as f:
                f.write("P2\n")
                f.write("# {}\n".format(name))
                f.write("{} {}\n".format(self.scene.cols, self.scene.rows))
                items = [(item.y, item.x, item.idx) for item in self.scene.items()]
                uniqueIDs = set([item[2] for item in items])                
                f.write("{}\n".format(max(uniqueIDs)))

                # Write the cells in correct order
                items.sort()
                for j in range(self.scene.rows):
                    for i in range(self.scene.cols):
                        f.write(str(items[j * self.scene.cols + i][2]).ljust(4))
                    f.write("\n")
                f.write("\n")

                # write a small legend
                for i in sorted(uniqueIDs):
                    for cat in self.categories:
                        if cat.id == i:
                            f.write("# {}: {}\n".format(i, cat.name))
        


def main():
    app = QApplication(sys.argv)
    w = MainWidget()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()