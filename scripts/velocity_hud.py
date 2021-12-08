import tkinter as tk
from PIL import ImageTk
from PIL import Image

import nav_params

# from geometry_msgs.msg import Twist, Pose2D, PoseStamped
# import rospy

linear_speed = 1
angular_speed = 0

def callback(data):
	linear = data.linear
	angular = data.angular
	linear_speed = (linear.x ** 2 + linear.y ** 2 + linear.z ** 2) ** (1 / 2)
	angular_speed = angular.x

class HUDController(object):
    def __init__(self, master, filename, **kwargs):
        self.master = master
        self.filename = filename
        self.canvas = tk.Canvas(master, width=300, height=200)
        self.canvas.pack()

        self.update = self.draw().__next__
        master.after(100, self.update)

    def draw(self):
        image = Image.open(self.filename)
        angle = 0
        while True:
            tkimage = ImageTk.PhotoImage(image.rotate(angle))
            canvas_obj = self.canvas.create_image(
                150, 100, image=tkimage)
            self.master.after_idle(self.update)

            speed = self.canvas.create_rectangle(270, 200 - linear_speed * 100, 300, 200, fill='red')
            max_speed = self.canvas.create_rectangle(270, 200 - nav_params.v_max * 100, 300, 200, fill='black')

            yield
            self.canvas.delete(canvas_obj)
            self.canvas.delete(speed)
            self.canvas.delete(max_speed)
            angle += angular_speed * 50
            angle %= 360

root = tk.Tk()
app = HUDController(root, 'hud/wheel.png')
# rospy.Subscriber('/cmd_vel', Twist, callback)

root.mainloop()