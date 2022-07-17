#!/usr/bin/env python
""" Open3D BlueSky start script """
from __future__ import print_function
import bluesky as bs
import threading


def main():
    """ Start the mainloop (and possible other threads) """
    bs.init(gui='open3d')

    bs.sim.op()
    # bs.scr.init()

    threading.Thread(target=update).start()

    bs.scr.app.run()

    print('Open3d normal end.')


def update():
    # Main loop for Open3d
    while not bs.sim.state == bs.END:
        bs.sim.step()  # Update sim

        # ======= # GUI update ========
        # update and draw aircraft trajectories
        # self.app.post_to_main_thread(self.window, lambda: self._update_next_frame(frame_i))
        # bs.scr.update()   # GUI update

        if bs.scr.is_done:
            print(f'break the process')
            break
        else:
            bs.scr.app.post_to_main_thread(bs.scr.window, bs.scr.update)

    bs.sim.quit()


if __name__ == '__main__':
    print("   *****   BlueSky Open ATM simulator *****")
    print("Distributed under GNU General Public License v3")
    # Run mainloop if BlueSky_pygame is called directly
    main()
