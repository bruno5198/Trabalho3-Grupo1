#!/usr/bin/env python3

from tkinter import *
from colorama import *

root = Tk()
root.title('Driver Interface')
root.geometry('400x800')

# LABELS -------------------------------------------------
# Name label
name = 'blue1'
nameLabel = Label(root, text= 'Name: ' + name)

# State Label
state = 'Wondering'
stateLabel = Label(root, text='State: ' + state)

# GRIDS ---------------------------------------------------
# nameLabel.grid(row=0, column=0)
# stateLabel.grid(row=1, column=0)

nameLabel.pack(padx=0, pady=0)
stateLabel.pack(padx=1, pady=0)



# FRAMES --------------------------------------------------


# Detection
detectionFrame = LabelFrame(root, text='Detection', relief=GROOVE, height=200, width=400, padx=5, pady=5)
detectionFrame.pack(padx=10, pady=10)

    # Goal
goalFrame = LabelFrame(detectionFrame, text='Goal', relief=GROOVE, padx=5, pady=5)
goalFrame.pack(padx=10, pady=10)


goal_detection = True
goal_position = (0,0)

goal_detectionLabel = Label(goalFrame, text='Goal detection: ' + str(goal_detection))
goal_detectionLabel.pack()
goal_nearLabel = Label(goalFrame, text='Goal position: ' + str(goal_position))
goal_nearLabel.pack()

    # Target
targetFrame = LabelFrame(detectionFrame, text='Target', relief=GROOVE, padx=5, pady=5)
targetFrame.pack(padx=10, pady=10)

target_detection = True
target_near = False
target_position = (0,0)

target_detectionLabel = Label(targetFrame, text='Target detection: ' + str(target_detection))
target_detectionLabel.pack()
target_nearLabel = Label(targetFrame, text='Target near: ' + str(target_near))
target_nearLabel.pack()
target_positionLabel = Label(targetFrame, text='Target position: ' + str(target_position))
target_positionLabel.pack()

    # Threat
threatFrame = LabelFrame(detectionFrame, text='Threat', relief=GROOVE, padx=5, pady=5)
threatFrame.pack(padx=10, pady=10)

threat_detection = True
threat_near = False
threat_position = (0,0)

threat_detectionLabel = Label(threatFrame, text='Threat detection: ' + str(threat_detection))
threat_detectionLabel.pack()
threat_nearLabel = Label(threatFrame, text='Threat near: ' + str(threat_near))
threat_nearLabel.pack()
threat_positionLabel = Label(threatFrame, text='Threat position: ' + str(threat_position))
threat_positionLabel.pack()

    # Walls
wallsFrame = LabelFrame(detectionFrame, text='Walls', relief=GROOVE, padx=5, pady=5)
wallsFrame.pack(padx=10, pady=10)

front_wall = True
right_wall = True
left_wall = True
back_wall = True

frontwallLabel = Label(wallsFrame, text='Front Wall: ' + str(front_wall))
frontwallLabel.pack()
right_wallLabel = Label(wallsFrame, text='Right Wall: ' + str(right_wall))
right_wallLabel.pack()
left_wallLabel = Label(wallsFrame, text='Left Wall: ' + str(left_wall))
left_wallLabel.pack()
back_wallLabel = Label(wallsFrame, text='Back Wall: ' + str(back_wall))
back_wallLabel.pack()

# Detection
detectionFrame = LabelFrame(root, text='Detection', relief=GROOVE, height=200, width=400, padx=5, pady=5)
detectionFrame.pack(padx=10, pady=10)

root.mainloop()



