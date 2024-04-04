""" BlueSky plugin template. The text you put here will be visible
    in BlueSky as the description of your plugin. """
# Import the global bluesky objects. Uncomment the ones you need
from bluesky import stack, traf, navdb  #, settings, navdb, traf, sim, scr, tools
from bluesky.tools.geo import qdrdist

### Initialization function of your plugin. Do not change the name of this
### function, as it is the way BlueSky recognises this file as a plugin.

def init_plugin():

    # Addtional initilisation code
    reset_environment()

    # Configuration parameters
    config = {
        # The name of your plugin
        'plugin_name':     'EXAMPLE1',

        # The type of this plugin. For now, only simulation plugins are possible.
        'plugin_type':     'sim',

        # Update interval in seconds. By default, your plugin's update function(s)
        # are called every timestep of the simulation. If your plugin needs less
        # frequent updates provide an update interval.
        'update_interval': 10,

        # The update function is called after traffic is updated. Use this if you
        # want to do things as a result of what happens in traffic. If you need to
        # something before traffic is updated please use preupdate.
        'update':          update,

        # The preupdate function is called before traffic is updated. Use this
        # function to provide settings that need to be used by traffic in the current
        # timestep. Examples are ASAS, which can give autopilot commands to resolve
        # a conflict.
        'preupdate':       preupdate,

        # If your plugin has a state, you will probably need a reset function to
        # clear the state in between simulations.
        'reset':         reset
        }

    stackfunctions = {}

    # init_plugin() should always return these two dicts.
    return config, stackfunctions

def preupdate():
    pass


### Periodic update functions that are called by the simulation. You can replace
### this by anything, so long as you communicate this in init_plugin

def update():
    bearing, dist = cal_dist()
    print(f'preupdate: bearing {bearing}, dist {dist}')
    if dist < 1:
        stack.stack('KL204 ALT FL250')  # command

def reset():
    pass

def reset_environment():
    stack.stack('CRE KL204 B747 IDRID 30 Fl190 450')
    stack.stack('addwpt KL204, VOLLA')
    stack.stack('addwpt KL204, SPL')
    stack.stack('addwpt KL204, ANDIK')


    stack.stack('CRE KL205 A320 VALKO 40 20000 300')
    stack.stack('addwpt KL205, SPL')
    stack.stack('addwpt KL205  PAM')
    stack.stack('addwpt KL205  IVLUT')
    stack.stack('addwpt KL205, LUNIX')
    stack.stack('addwpt KL205, RENDI')
    stack.stack('addwpt KL205, EDUPO')

def cal_dist():
    KL204 = traf.id2idx('KL204')
    KL204_lat, Kl204_lon = traf.lat[KL204], traf.lon[KL204]
    # print(f"KL204 latitude {KL204_lat}, KL204 longtitude {Kl204_lon}")

    SPL = navdb.getwpidx('SPL') # SPL waypoint
    SPL_lat, SPL_lon = navdb.wplat[SPL], navdb.wplon[SPL]
    print(f'SPL {navdb.wptype[SPL]} {navdb.wpelev[SPL]}')

    bearing, dist = qdrdist(KL204_lat, Kl204_lon, SPL_lat, SPL_lon) # distance between KL204 and SPL

    return bearing, dist