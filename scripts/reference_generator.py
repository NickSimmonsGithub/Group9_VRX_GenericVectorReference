#!/usr/bin/env python
# license removed for brevity

import rospy
import numpy
import tkinter as tk
from generic_reference.msg import generic_reference_vector
from tkinter import messagebox
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import (FigureCanvasTkAgg, NavigationToolbar2Tk)

class ReferenceGeneratorApplication():

    # main tkinter window.
    root = tk.Tk()

    # x, y and z variables. These represent the x, y and z scalar components of the generated reference vector.
    x = 0
    y = 0
    z = 0

    # magnitude and direction properties for arrow-key control in 2D reference mode.
    magnitude_2D = 0
    direction_2D = 0

    # current value of the checkbox.
    checkboxValue = tk.IntVar()

    # value which the arrow keys increment the reference by.
    incrementVal = 0.5

    # figure which will hold the plot of the 2D reference vector, if the checkbox is enabled.
    fig = Figure(figsize = (4, 4), dpi=100)
    
    # the ROS publisher this node uses.
    pub = rospy.Publisher('/wamv/reference', generic_reference_vector, queue_size=10)

    # generic_reference_vector custom message type whose contents are published on the /wamv/reference topic.
    vector = generic_reference_vector()

    # Class constructor. Called when a new instance of the GUI object is created.
    def __init__(self):

        self.root.geometry('500x580')

        # Initialise the frames. (used to group widgets (UI elements) together.)
        title_Frame = tk.Frame(self.root, padx=2, pady=2)
        title_Frame.grid(row=0, columnspan=2)

        input_Frame = tk.Frame(self.root, padx=2, pady=2)
        input_Frame.grid(row=1, column=0)

        label_Frame = tk.Frame(self.root, padx=2, pady=2)
        label_Frame.grid(row=1, column=1)

        vector_Frame = tk.Frame(self.root, padx=2, pady=2)
        vector_Frame.grid(row=2, columnspan=2)

        # Initialise the title.
        Title = tk.Label(title_Frame, text="Generic Vector Reference Generator", font=('Arial', 16)).pack()

        # Initialise the x, y and z entries.
        self.x_Entry = tk.Entry(input_Frame)
        self.y_Entry = tk.Entry(input_Frame)
        self.z_Entry = tk.Entry(input_Frame)

        self.x_Entry.pack()
        self.y_Entry.pack()
        self.z_Entry.pack()

        # Initialise the 2D reference vector arrow key check button.
        self.checkbox = tk.Checkbutton(input_Frame, variable=self.checkboxValue)
        self.checkbox.pack()

        # Initialise the x, y, z and checkbox labels.
        x_Label = tk.Label(label_Frame, text="x component of reference").pack()
        y_Label = tk.Label(label_Frame, text="y component of reference").pack()
        z_Label = tk.Label(label_Frame, text="z component of reference").pack()
        checkbox_label = tk.Label(label_Frame, text="enable arrow key control").pack()

        # Define the canvas where the 2D vector reference will be drawn.
        self.canvas = FigureCanvasTkAgg(self.fig, master = vector_Frame)

        # Define the plot object where the reference vector will be plotted.
        self.vectorPlot = self.fig.add_subplot()
        self.vectorPlot.set_xlim(-10, 10)
        self.vectorPlot.set_ylim(-10, 10)

        # Create the matplotlib toolbar for the canvas.
        self.toolbar = NavigationToolbar2Tk(self.canvas, vector_Frame)

        # Place the canvas on the tkinter window
        self.canvas.get_tk_widget().pack()

        # bind the enter key to update_x, update_y and update_z events (when the cursor is in the respective entry box)
        self.x_Entry.bind('<Return>', self.update_x)
        self.y_Entry.bind('<Return>', self.update_y)
        self.z_Entry.bind('<Return>', self.update_z)

        # bind the 4 arrow keys to arrow_left, arrow_right, arrow_up, arrow_down events.
        self.root.bind('<Left>', self.arrow_left)
        self.root.bind('<Right>', self.arrow_right)
        self.root.bind('<Up>', self.arrow_up)
        self.root.bind('<Down>', self.arrow_down)

        # Initialise the node.
        rospy.init_node('reference_generator', anonymous=False)

        # Start the GUI.
        self.root.mainloop()

    def plot(self):
        self.vectorPlot.clear()
        self.vectorPlot.set_xlim(-10, 10)
        self.vectorPlot.set_ylim(-10, 10)
        self.vectorPlot.plot([0, self.x], [0, self.y])
        self.canvas.draw()


    # One of these 3 methods is called whenever the user presses enter while their cursor is inside an entry box.
    def update_x(self, event):
        try:
            # copy the contents of the entry box into the variable. If it fires an exception, show an error box to the user and delete the contents of the text box.
            self.x = float(self.x_Entry.get())
            self.vector.x = self.x
            print(self.x)
            self.plot()
            self.publishVector()
        except:
            messagebox.showerror(title=None, message="invalid input!")
            self.x_Entry.delete(0, tk.END)

    def update_y(self, event):
        try:
            # copy the contents of the entry box into the variable. If it fires an exception, show an error box to the user and delete the contents of the text box.
            self.y = float(self.y_Entry.get())
            self.vector.y = self.y
            print(self.y)
            self.plot()
            self.publishVector()
        except:
            messagebox.showerror(title=None, message="invalid input!")
            self.y_Entry.delete(0, tk.END)
  
    def update_z(self, event):
        try:
            # copy the contents of the entry box into the variable. If it fires an exception, show an error box to the user and delete the contents of the text box.
            self.z = float(self.z_Entry.get())
            self.vector.z = self.z
            print(self.z)
            self.plot()
            self.publishVector()
        except:
            messagebox.showerror(title=None, message="invalid input!")
            self.z_Entry.delete(0, tk.END)

    # One of these 4 methods is called whenever the corresponding arrow key is pressed. Changes the stored reference accordingly.
    def arrow_left(self, event):
        if(self.checkboxValue.get()):
            self.x = self.x - self.incrementVal
            self.x_Entry.delete(0, tk.END)
            self.x_Entry.insert(0, self.x)
            self.vector.x = self.x
            self.plot()
            self.publishVector()

    def arrow_right(self, event):
        if(self.checkboxValue.get()):
            self.x = self.x + self.incrementVal
            self.x_Entry.delete(0, tk.END)
            self.x_Entry.insert(0, self.x)
            self.vector.x = self.x
            self.plot()
            self.publishVector()

    def arrow_up(self, event):
        if(self.checkboxValue.get()):
            self.y = self.y + self.incrementVal
            self.y_Entry.delete(0, tk.END)
            self.y_Entry.insert(0, self.y)
            self.vector.y = self.y
            self.plot()
            self.publishVector()

    def arrow_down(self, event):
        if(self.checkboxValue.get()):
            self.y = self.y - self.incrementVal
            self.y_Entry.delete(0, tk.END)
            self.y_Entry.insert(0, self.y)
            self.vector.y = self.y
            self.plot()
            self.publishVector()

    # This method publishes the generic vector reference to the topic "/wamv/reference"
    def publishVector(self):
        rospy.loginfo(self.vector)
        self.pub.publish(self.vector)
    
    
if __name__ == '__main__':
    try:
        Application = ReferenceGeneratorApplication()
    except rospy.ROSInterruptException:
        pass
