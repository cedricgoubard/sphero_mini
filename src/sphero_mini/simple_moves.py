"""
Simple moves
------------

This module contains functions to have the Sphero mini do some simple movements (e.g. circle).
"""
import sphero_mini
import sys


def forward(sphero):
    # Aiming:
    sphero.setLEDColor(red = 0, green = 0, blue = 0) # Turn main LED off
    sphero.stabilization(False) # Turn off stabilization
    sphero.setBackLEDIntensity(255) # turn back LED on
    sphero.wait(3) # Non-blocking pau`se
    sphero.resetHeading() # Reset heading
    sphero.stabilization(True) # Turn on stabilization
    sphero.setBackLEDIntensity(0) # Turn back LED off

    # Move around:
    sphero.setLEDColor(red = 0, green = 0, blue = 255) # Turn main LED blue
    sphero.roll(100, 0)      # roll forwards (heading = 0) at speed = 50

    sphero.wait(3)         # Keep rolling for three seconds

    sphero.roll(0, 0)       # stop
    sphero.wait(1)          # Allow time to stop

    sphero.setLEDColor(red = 0, green = 255, blue = 0) # Turn main LED green
    sphero.roll(-100, 0)     # Keep facing forwards but roll backwards at speed = 50
    sphero.wait(3)          # Keep rolling for three seconds

    sphero.roll(0, 0)       # stop
    sphero.wait(1)          # Allow time to stop
