#!/usr/bin/env python
""" Open3D BlueSky start script """
from __future__ import print_function
import bluesky as bs
import threading
import time


def main():
    """ Start the mainloop (and possible other threads) """
    bs.init(gui='open3d')

    bs.sim.op()

    threading.Thread(target=update).start()

    bs.scr.app.run()

    print('Open3d normal end.')


def update():
    # Main loop for Open3d
    while not bs.sim.state == bs.END:
        bs.sim.step()  # Update sim

        # ======= # GUI update ========
        # update and draw aircraft trajectories
        if bs.scr.is_done:
            print(f'Quit the process')
            bs.sim.quit()
            if bs.scr.sensor.subwindow_opened:
                bs.scr.sensor.subwindow_sensor_render.close()
            break
        else:
            bs.scr.app.post_to_main_thread(bs.scr.window, bs.scr.update)
            # update real-time sensor stream visualization
            if bs.scr.sensor.subwindow_opened:
                bs.scr.app.post_to_main_thread(bs.scr.sensor.subwindow_sensor_render,
                                               bs.scr.sensor.update_sensor_stream)


if __name__ == '__main__':
    print("   *****   BlueSky Open ATM simulator *****")
    print("Distributed under GNU General Public License v3")
    # Run mainloop if BlueSky_open3d is called directly
    main()
