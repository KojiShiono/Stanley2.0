'''Rough script to convert a list of .png to .mp4. More or less what we did in
the first project
https://github.com/alemelis/autonomous-car/blob/master/lane-lines/lane-lines.ipynb

It needs moviepy (pip) and ffmpeg (install through imageio).
'''

import matplotlib.image as mpimg
from moviepy.editor import ImageSequenceClip
from IPython.display import HTML
import os

imgs = os.listdir("loop_with_traffic_light/")
os.chdir("loop_with_traffic_light/")
clip = ImageSequenceClip(imgs, fps=25)
clip.write_videofile("../loop.mp4")

imgs = os.listdir("../just_traffic_light/")
os.chdir("../just_traffic_light/")
clip = ImageSequenceClip(imgs, fps=25)
clip.write_videofile("../just.mp4")
