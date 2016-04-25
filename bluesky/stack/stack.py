from math import *
import numpy as np
from random import random, randint, seed
import os
import sys

from ..tools.aero import kts, ft, fpm, nm, lbs,\
                         qdrdist, cas2tas, mach2tas, tas2cas, tas2eas, tas2mach,\
                         eas2tas, cas2mach, density
from ..tools.misc import txt2alt, txt2spd, col2rgb, cmdsplit,  txt2lat, txt2lon
from .. import settings

# import pdb


class Commandstack:
    """
    Commandstack class definition : command stack & processing class

    Methods:
        Commandstack()          :  constructor
        stack(cmdline)          : add a command to the command stack
        openfile(scenname)      : start playing a scenario file scenname.SCN
                                  from scenario folder
        savefile(scenname,traf) : save current traffic situation as
                                  scenario file scenname.SCN
        checkfile(t)            : check whether commands need to be
                                  processed from scenario file

        process(sim, traf, scr) : central command processing method
                                  (with long elif tree with all commands)

    Created by  : Jacco M. Hoekstra (TU Delft)
    """
    def __init__(self, sim, traf, scr):

        #Command dictionary: command, helptext, arglist, function to call
        # Enclose optional arguments with []
        # Separate argument type variants with /
        #--------------------------------------------------------------------
        self.cmddict = {
            "ADDNODES": [
                "ADDNODES number",
                "int",
                sim.addNodes],
            "ADDWPT": [
                "ADDWPT acid, (wpname/lat,lon),[alt],[spd],[afterwp]",
                "acid,latlon/txt,[alt,spd,txt]",
                # lambda: short-hand for using function output as argument, equivalent with:
                #
                # def fun(idx, args):
                #     return traf.route[idx].addwptStack(traf, idx, *args)
                # fun(idx,*args)
                lambda idx, *args: traf.route[idx].addwptStack(traf, idx, *args)
            ],
            "ALT": [
                "ALT acid, alt, [vspd]",
                "acid,alt,[vspd]",
                traf.selalt
            ],
            "ASAS": [
                "ASAS ON/OFF",
                "[onoff]",
                traf.dbconf.toggle
            ],
            "BATCH": [
                "BATCH filename",
                "txt",
                sim.batch],
            "BOX": [
                "BOX name,lat,lon,lat,lon",
                "txt,latlon,latlon",
                lambda name, *coords: scr.objappend(2, name, coords)
            ],
            "CIRCLE": [
                "CIRCLE name,lat,lon,radius",
                "txt,latlon,float",
                lambda name, *coords: scr.objappend(3, name, coords)
            ],
            "CRE": [
                "CRE acid,type,lat,lon,hdg,alt,spd",
                "txt,txt,latlon,hdg,alt,spd",
                traf.create
            ],
            "DATAFEED":  [
                "DATAFEED [ON/OFF]",
                "[onoff]",
                sim.datafeed
            ],
            "DEL": [
                "DEL acid/shape",
                "txt",
                lambda a: traf.delete(a) if traf.id.count(a) > 0 \
                     else scr.objappend(0, a, None)
                ],
            "DELWPT": [
                "DELWPT acid,wpname",
                "acid,txt",
                lambda idx, wpname: traf.route[idx].delwpt(wpname)
            ],
            "DEST": [
                "DEST acid, latlon/airport",
                "acid,latlon/txt",
                lambda idx, *args: traf.setDestOrig("DEST", idx, *args)
            ],
            "DIRECT": [
                "DIRECT acid wpname",
                "acid,txt",
                lambda idx, wpname: traf.route[idx].direct(traf, idx, wpname)
            ],
            "DIST": [
                "DIST lat0, lon0, lat1, lon1",
                "latlon,latlon",
                lambda *args: scr.echo("Dist = %.3f nm, QDR = %.2f deg" % qdrdist(*args))
            ],
            "DT": [
                "DT dt",
                "float",
                sim.setDt
            ],
            "DTMULT": [
                "DTMULT multiplier",
                "float",
                sim.setDtMultiplier
            ],
            "DUMPRTE": [
                "DUMPRTE acid",
                "acid",
                lambda idx: traf.route[idx].dumpRoute(traf, idx)
            ],
            "ECHO": [
                "ECHO txt",
                "txt",
                scr.echo
            ],
            "ENG": [
                "ENG acid,[engine_id]",
                "acid,[txt]",
                traf.perf.engchange
            ],
            "FF":  [
                "FF [tend]",
                "[time]",
                sim.fastforward
            ],
            "FIXDT": [
                "FIXDT ON/OFF [tend]",
                "onoff,[time]",
                sim.setFixdt
            ],
            "HDG": [
                "HDG acid,hdg (deg,True)",
                "acid,float",
                traf.selhdg
            ],
            "HELP": [
                "HELP [command]",
                "[txt]",
                lambda *args: scr.echo(self.help(*args))
            ],
            "HOLD": [
                "HOLD",
                "",
                sim.pause
            ],
            "IC": [
                "IC [IC/filename]",
                "[txt]",
                lambda *args: self.ic(scr, *args)
            ],
            "INSEDIT": [
                "INSEDIT txt",
                "txt",
                scr.cmdline
            ],
            "LINE": [
                "LINE name,lat,lon,lat,lon",
                "txt,latlon,latlon",
                lambda name, *coords: scr.objappend(1, name, coords)
            ],
            "LISTRTE": [
                "LISTRTE acid, [pagenr]",
                "acid,[int]",
                lambda idx, *args: traf.route[idx].listrte(scr, *args)
            ],
            "LNAV": [
                "LNAV acid,[ON/OFF]",
                "acid,[onoff]",
                traf.setLNAV
            ],
            "LOG": [
                "LOG acid/area/*,dt",
                "txt,float",
                sim.datalog.start
            ],
            "MCRE": [
                "MCRE n, [type/*, alt/*, spd/*, dest/*]",
                "int,[txt,alt,spd,txt]",
                lambda *args: traf.mcreate(*args, area=scr.getviewlatlon())
            ],
            "MOVE": [
                "MOVE acid,lat,lon,[alt,hdg,spd,vspd]",
                "acid,latlon,[alt,hdg,spd,vspd]",
                traf.move
            ],
            "ND": [
                "ND acid",
                "txt",
                lambda acid: scr.feature("ND", acid)
            ],
            "NOISE": [
                "NOISE [ON/OFF]",
                "[onoff]",
                traf.setNoise
            ],
            "NOM": [
                "NOM acid",
                "acid",
                traf.nom],
            "OP": [
                "OP",
                "",
                sim.start
            ],
            "ORIG": [
                "ORIG acid, latlon/airport",
                "acid,latlon/txt",
                lambda *args: traf.setDestOrig("ORIG", *args)
            ],
            "PCALL": [
                "PCALL filename [REL/ABS]",
                "txt,[txt]",
                lambda *args: self.openfile(*args, mergeWithExisting=True)
            ],
            "POLY": [
                "POLY name,lat,lon,lat,lon, ...",
                "txt,latlon,...",
                lambda name, *coords: scr.objappend(4, name, coords)
            ],
            "RESET": [
                "RESET",
                "",
                sim.reset],
            "SAVEIC": [
                "SAVEIC filename",
                "txt",
                lambda fname: self.saveic(fname, sim, traf)
            ],
            "SCEN": [
                "SCEN scenname",
                "txt",
                sim.scenarioInit
            ],
            "SEED": [
                "SEED value",
                "int",
                self.setSeed],
            "SPD": [
                "SPD acid,spd (CAS-kts/Mach)",
                "acid,spd",
                traf.selspd
            ],
            "SSD": [
                "SSD acid/ALL/OFF",
                "txt",
                scr.showssd
            ],
            "STOP": [
                "STOP",
                "",
                sim.stop
            ],
            "SWRAD": [
                "SWRAD GEO/GRID/APT/VOR/WPT/LABEL/ADSBCOVERAGE/TRAIL [dt]/[value]",
                "txt,[float]",
                scr.feature
            ],
            "SYMBOL":  [
                "SYMBOL",
                "",
                scr.symbol
            ],
            "TAXI": [
                "TAXI ON/OFF : OFF auto deletes traffic below 1500 ft",
                "onoff",
                traf.setTaxi
            ],
            "VNAV": [
                "VNAV acid,[ON/OFF]",
                "acid,[onoff]",
                traf.setVNAV
            ],
            "VS": [
                "VS acid,vspd (ft/min)",
                "acid,vspd",
                traf.selvspd]
        }

        #--------------------------------------------------------------------
        # Command synonym dictionary
        self.cmdsynon = {
            "CONTINUE": "OP",
            "CREATE": "CRE",
            "DELETE":"DEL",
            "DIRECTTO": "DIRECT",
            "DIRTO": "DIRECT",
            "DISP": "SWRAD",
            "DTLOOK": "ASA_DTLOOK",
            "END": "STOP",
            "EXIT": "STOP",
            "FWD": "FF",
            "PAUSE": "HOLD",
            "Q": "STOP",
            "QUIT": "STOP",
            "RUN": "OP",
            "START": "OP",
            "TURN": "HDG",
            "?": "HELP"
        }
        #--------------------------------------------------------------------

        self.cmdstack  = []
        self.scentime  = []
        self.scenfile  = ""
        self.scentime = []
        self.scencmd = []

        # Display Help text on start of program
        self.stack("ECHO BlueSky Console Window: Enter HELP or ? for info.\n" +
            "Or select IC to Open a scenario file.")

        # Pan to initial location
        self.stack('PAN ' + settings.start_location)
        self.stack("ZOOM 0.4")

        # ------------------ [start] Deprecated -------------------
        # An alternative way to add your own commands: 
        # add your entry to the dictionary.
        # The dictionary should be formed as {"Key":module'}.

        # "Key" is a FOUR-symbol reference used at the start of command.
        # 'module' is the name of the .py-file in which the 
        # commands are located (without .py).

        # Make sure that the module has a function "process" with
        # arguments:
        #   command, number of args, array of args, sim, traf, scr, cmd

        self.extracmdmodules = {
            "SYN_": 'synthetic', 
            "ASA_":'asascmd', 
           # "LOG_":'log' # Old logging module
        }

        # Import modules from the list
        self.extracmdrefs={}
        sys.path.append('bluesky/stack/')
        for key in self.extracmdmodules:
            obj=__import__(self.extracmdmodules[key],globals(),locals(),[],0)
            self.extracmdrefs[key]=obj
        # ------------------ [end] Deprecated -------------------

        return

    def help(self, cmd=''):
        if len(cmd) == 0:
            text = "To get help on a command, enter it without arguments.\n" + \
                   "Some basic commands are given below:\n\n"
            text2 = ""
            for key in self.cmddict:
                text2 += (key + " ")
                if len(text2) >= 60:
                    text += (text2 + "\n")
                    text2 = ""
            text += (text2 + "\nSee Info subfolder for more info.")
            return text
        elif cmd in self.cmddict:
            return self.cmddict[cmd][0]
        else:
            return "HELP: Unknown command: " + cmd

    def setSeed(self, value):
        seed(value)
        np.random.seed(value)

    def reset(self):
        self.scentime = []
        self.scencmd  = []

    def stack(self, cmdline):
        # Stack one or more commands separated by ";"
        cmdline = cmdline.strip()
        if len(cmdline) > 0:
            for line in cmdline.split(';'):
                self.cmdstack.append(line)

    def openfile(self, scenname, absrel='ABS', mergeWithExisting=False):
        # If timestamps in file should be interpreted as relative we need to add
        # the current simtime to every timestamp
        t_offset = self.sim.simt if absrel == 'REL' else 0.0

        # Add .scn extension if necessary
        if scenname.lower().find(".scn") < 0:
            scenname = scenname + ".scn"

        # If it is with a path don't touch it, else add path
        if scenname.find("/") < 0 and scenname.find( "\\") < 0:
            scenfile = settings.scenario_path
            if scenfile[-1] is not '/':
                scenfile += '/'
            scenfile += scenname.lower()
        else:
            scenfile = scenname

        if not os.path.exists(scenfile):
            return False, "Error: cannot find file: " + scenfile

        # Split scenario file line in times and commands
        if not mergeWithExisting:
            # When a scenario file is read with PCALL the resulting commands
            # need to be merged with the existing commands. Otherwise the
            # old scenario commands are cleared.
            self.scentime = []
            self.scencmd  = []

        with open(scenfile, 'r') as fscen:
            for line in fscen:
                if len(line.strip()) > 12 and line[0] != "#":
                    # Try reading timestamp and command
                    try:
                        icmdline = line.index('>')
                        tstamp = line[:icmdline]
                        ttxt = tstamp.strip().split(':')
                        ihr = int(ttxt[0])
                        imin = int(ttxt[1])
                        xsec = float(ttxt[2])
                        self.scentime.append(ihr * 3600. + imin * 60. + xsec + t_offset)
                        self.scencmd.append(line[icmdline + 1:-1])
                    except:
                        print "except this:", line
                        pass  # nice try, we will just ignore this syntax error

        if mergeWithExisting:
            # If we are merging we need to sort the resulting command list
            self.scentime, self.scencmd = [list(x) for x in zip(*sorted(
                zip(self.scentime, self.scencmd), key=lambda pair: pair[0]))]

        return True

    def ic(self, scr, filename=''):
        if filename == '':
            filename = scr.show_file_dialog()
        elif filename == "IC":
            filename = self.scenfile

        if len(filename) > 0:
            result = self.openfile(filename)
            if type(result) is bool:
                self.scenfile = filename
                return True, "Opened " + filename
            else:
                return result

    def checkfile(self, simt):
        # Empty command buffer when it's time
        while len(self.scencmd) > 0 and simt >= self.scentime[0]:
            self.stack(self.scencmd[0])
            del self.scencmd[0]
            del self.scentime[0]

        return

    def saveic(self, fname, sim, traf):
        # Add extension .scn if not already present
        if fname.find(".scn") < 0 and fname.find(".SCN"):
            fname = fname + ".scn"

        # If it is with path don't touch it, else add path
        if fname.find("/") < 0:
            scenfile = "./scenario/" + fname.lower()

        try:
            f = open(scenfile, "w")
        except:
            return False, "Error writing to file"

        # Write files
        timtxt = "00:00:00.00>"

        for i in range(traf.ntraf):
            # CRE acid,type,lat,lon,hdg,alt,spd
            cmdline = "CRE " + traf.id[i] + "," + traf.type[i] + "," + \
                      repr(traf.lat[i]) + "," + repr(traf.lon[i]) + "," + \
                      repr(traf.trk[i]) + "," + repr(traf.alt[i] / ft) + "," + \
                      repr(tas2cas(traf.tas[i], traf.alt[i]) / kts)

            f.write(timtxt + cmdline + chr(13) + chr(10))

            # VS acid,vs
            if abs(traf.vs[i]) > 0.05:  # 10 fpm dead band
                if abs(traf.avs[i]) > 0.05:
                    vs_ = traf.avs[i] / fpm
                else:
                    vs_ = traf.vs[i] / fpm

                cmdline = "VS " + traf.id[i] + "," + repr(vs_)
                f.write(timtxt + cmdline + chr(13) + chr(10))

            # Autopilot commands
            # Altitude
            if abs(traf.alt[i] - traf.aalt[i]) > 10.:
                cmdline = "ALT " + traf.id[i] + "," + repr(traf.aalt[i] / ft)
                f.write(timtxt + cmdline + chr(13) + chr(10))

            # Heading as well when heading select
            delhdg = (traf.trk[i] - traf.ahdg[i] + 180.) % 360. - 180.
            if abs(delhdg) > 0.5:
                cmdline = "HDG " + traf.id[i] + "," + repr(traf.ahdg[i])
                f.write(timtxt + cmdline + chr(13) + chr(10))

            # Speed select? => Record
            rho = density(traf.alt[i])  # alt in m!
            aptas = sqrt(1.225 / rho) * traf.aspd[i]
            delspd = aptas - traf.tas[i]

            if abs(delspd) > 0.4:
                cmdline = "SPD " + traf.id[i] + "," + repr(traf.aspd[i] / kts)
                f.write(timtxt + cmdline + chr(13) + chr(10))

            # DEST acid,dest-apt
            if traf.dest[i] != "":
                cmdline = "DEST " + traf.id[i] + "," + traf.dest[i]
                f.write(timtxt + cmdline + chr(13) + chr(10))

            # ORIG acid,orig-apt
            if traf.orig[i] != "":
                cmdline = "ORIG " + traf.id[i] + "," + \
                          traf.orig[i]
                f.write(timtxt + cmdline + chr(13) + chr(10))

        # Saveic: should close
        f.close()
        return True

    def process(self, sim, traf, scr):
        """process and empty command stack"""
        # Process stack of commands
        for line in self.cmdstack:
            # Empty line: next command
            line = line.strip()
            if len(line) == 0:
                continue

            # Split command line into command and arguments, pass traf ids to check for
            # switched acid and command
            cmd, args = cmdsplit(line.upper(), traf.id)
            numargs   = len(args)

            # Check if this is a POS command with only an aircraft id
            if numargs == 0 and traf.id.count(cmd) > 0:
                args    = [cmd]
                cmd     = 'POS'
                numargs = 1

            # Assume syntax is ok (default)
            synerr = False

            # Catch general errors
#            try:
            if True:  # optional to switch error protection off

                #**********************************************************************
                #=====================  Start of command branches =====================
                #**********************************************************************

                #----------------------------------------------------------------------
                # First check command synonymes list, then in dictionary
                #----------------------------------------------------------------------
                if cmd in self.cmdsynon.keys():
                    cmd = self.cmdsynon[cmd]

                if cmd in self.cmddict.keys():
                    helptext, argtypelist, function = self.cmddict[cmd]
                    argvsopt = argtypelist.split('[')
                    argtypes = argvsopt[0].strip(',').split(",")
                    if argtypes == ['']:
                        argtypes = []

                    # Check if at least the number of mandatory arguments is given.
                    if numargs < len(argtypes):
                        print numargs, len(argtypes)
                        scr.echo("Syntax error: Too few arguments")
                        scr.echo(line)
                        scr.echo(helptext)
                        continue

                    # Add optional argument types if they are given
                    if len(argvsopt) == 2:
                        argtypes += argvsopt[1].strip(']').split(',')

                    # Process arg list
                    # Special case: single text argument: this can also be a string, so pass the original
                    if argtypes == ['txt']:
                        arglist = [line[len(cmd) + 1:]]
                    else:
                        arglist = []
                        curtype = curarg = 0
                        while curtype < len(argtypes) and curarg < len(args):
                            if argtypes[curtype] == '...':
                                curtype -= 1
                            argtype    = argtypes[curtype].strip().split('/')
                            for i in range(len(argtype)):
                                try:
                                    parsed_arg = self.argparse(argtype[i], curarg, args, traf, scr)
                                    arglist += parsed_arg
                                    curarg  += len(parsed_arg)
                                    break
                                except:
                                    # not yet last type possible here?
                                    if i < len(argtype) - 1:
                                        # We have alternative argument formats that we can try
                                        continue
                                    else:
                                        synerr = True
                                        scr.echo("Syntax error in processing arguments")
                                        scr.echo(line)
                                        scr.echo(helptext)
                            curtype += 1

                    # Call function return flag,text
                    # flag: indicates sucess
                    # text: optional error message
                    if not synerr:
                        results = function(*arglist)  # * = unpack list to call arguments

                        if type(results) == bool:  # Only flag is returned
                            synerr = not results
                            if synerr:
                                if numargs <= 0 or args[curarg] == "?":
                                    scr.echo(helptext)
                                else:
                                    scr.echo("Syntax error: " + helptext)
                                synerr =  False  # Prevent further nagging

                        elif type(results) == list or type(results) == tuple:
                            # Maybe there is also an error message returned?
                            if len(results) >= 1:
                                synerr = not results[0]

                            if len(results) >= 2:
                                scr.echo(cmd+":"+results[1])
                                synerr = False

                    else:  # synerr:
                        scr.echo("Syntax error: " + helptext)

                #----------------------------------------------------------------------
                # POS command: traffic info; ("KL204", "POS KL204" or "KL204 ?")
                #----------------------------------------------------------------------
                elif cmd == "POS" or cmd == "?":
                    if numargs >= 1:
                        acid = args[0]

                        # Does aircraft exist?
                        idx = traf.id2idx(acid)
                        if idx < 0:
                            scr.echo("POS: " + acid + " not found.")

                        # print info on aircraft if found
                        else:
                            scr.echo("Info on " + acid + " " + traf.type[idx]+\
                                                          "   index = " + str(idx))
                            taskts = int(round(traf.tas[idx]/kts))                              
                            caskts = int(round(tas2cas(traf.tas[idx],traf.alt[idx])/kts))                              
                            scr.echo("Pos = " + str(traf.lat[idx])+" , "+str(traf.lon[idx]))
                            scr.echo(str(caskts)+" kts (TAS: "+str(taskts)+" kts) at " \
                                     + str(int(traf.alt[idx] / ft)) + " ft")
                            scr.echo("Hdg = " + str(int(traf.trk[idx])))
                            if traf.swvnav[idx]:
                                vnavtxt = "VNAV "
                            else:
                                vnavtxt = ""
                            if traf.swlnav[idx] and traf.route[idx].nwp>0 and  \
                               traf.route[idx].iactwp>=0:
                                 scr.echo(vnavtxt + "LNAV to "+   \
                                  traf.route[idx].wpname[traf.route[idx].iactwp])
                            
                            txt = "Flying"
                            if traf.orig[idx]!="":
                                txt = txt + " from "+traf.orig[idx]
                            if traf.dest[idx]!="":
                                txt = txt + " to "+traf.dest[idx]
                            if len(txt)>0:
                                scr.echo(txt)

                        # Show route for this aircraft (toggle switch)
                        scr.showroute(acid)

                    else:
                         synerr = True

                #----------------------------------------------------------------------
                # ZOOM command (or use ++++  or --  to zoom in or out)
                #----------------------------------------------------------------------
                elif cmd[:4] == "ZOOM" or cmd[0] == "+" or cmd[0] == "=" or cmd[0] == "-":
                    if cmd[0] != "Z":
                        nplus = cmd.count("+") + cmd.count("=")  #= equals + (same key)
                        nmin = cmd.count("-")
                        zoomfac = sqrt(2) ** nplus / (sqrt(2) ** nmin)
                        scr.zoom(zoomfac)
                    else:
                        synerr = not(len(args) == 1)
                        if not synerr:
                            if args[0] == "IN":
                                scr.zoom(1.4142135623730951)  # sqrt(2.)

                            elif args[0] == "OUT":
                                scr.zoom(0.70710678118654746)  #1./sqrt(2.)
                            else:
                                try:
                                    zoomfac = float(args[0])
                                    scr.zoom(zoomfac, True)
                                except:
                                    synerr = True

                        if synerr:
                            print "Syntax error in command"
                            scr.echo("Syntax error in command")
                            scr.echo("ZOOM IN/OUT")
                            continue  # Skip default syntyax message

                #----------------------------------------------------------------------
                # PAN command
                #----------------------------------------------------------------------
                elif cmd[:4] == "PAN":

                    if not (numargs == 1 or numargs == 2):
                        if numargs>0:
                            scr.echo("Syntax error in command")
                        scr.echo("PAN LEFT/RIGHT/UP/DOWN/acid/airport/navid")
                        continue

                    # LEFT/RIGHT/UP/DOWN
                    elif numargs == 1:
                        if args[0] == "LEFT":
                            scr.pan((0.0, -0.5)) # move half screen left
                            continue
                        elif args[0] == "RIGHT":
                            scr.pan((0.0, 0.5)) # move half screen right
                            continue

                        elif args[0] == "UP":
                            scr.pan((0.5, 0.0))  # move half screen up
                            continue

                        elif args[0] == "DOWN":
                            scr.pan((-0.5, 0.0)) # move half screen down
                            continue
                        else:
                            # Try aicraft id, waypoint of airport
                            i = traf.id2idx(args[0])
                            if i >= 0:
                                lat = traf.lat[i]
                                lon = traf.lon[i]
                                if (np.isnan(lat) or np.isnan(lon)):
                                    continue
                            else:
                                i = traf.navdb.getwpidx(args[0], 0.0, 0.0)  # TODO: get current pan from display?
                                if i >= 0:
                                    lat = traf.navdb.wplat[i]
                                    lon = traf.navdb.wplon[i]
                                    if (np.isnan(lat) or np.isnan(lon)):
                                        continue
                                else:
                                    i = traf.navdb.getapidx(args[0])
                                    if i >= 0:
                                        lat = traf.navdb.aplat[i]
                                        lon = traf.navdb.aplon[i]
                                        if (np.isnan(lat) or np.isnan(lon)):
                                            continue
                                    else:
                                        synerr= True
                                        scr.echo(args[0] + " not found.")
                            if not synerr and (not (np.isnan(lat) or np.isnan(lon))):
                                scr.pan((lat, lon), absolute=True)

                    # PAN to lat,lon position
                    elif numargs == 2:
                        lat = float(args[0])
                        lon = float(args[1])

                        if not (np.isnan(lat) or np.isnan(lon)):
                            scr.pan((lat, lon), absolute=True)

                #----------------------------------------------------------------------
                # METRICS command: METRICS/METRICS OFF/0/1/2 [dt]  analyze traffic complexity metrics
                #----------------------------------------------------------------------
                elif cmd[:6] == "METRIC":
                    if sim.metric is None:
                        scr.echo("METRICS module disabled")

                    elif numargs < 1:
                        if sim.metric.metric_number < 0:
                            scr.echo("No metric active, to configure run:")
                            scr.echo("METRICS OFF/0/1/2 [dt]")
                        else:
                            scr.echo("")
                            scr.echo("Active: " + "(" + str(sim.metric.metric_number + 1) + ") " + sim.metric.name[
                                sim.metric.metric_number])
                            scr.echo("Current dt: " + str(sim.metric.dt) + " s")

                    elif args[0] == "OFF":  # arguments are strings
                        sim.metric.metric_number = -1
                        scr.echo("Metric is off")

                    else:
                        if not args[0][1:].isdigit():
                            # print args[0][1:].isdigit()
                            scr.echo("Command argument invalid")
                            return
                        sim.metric.metric_number = int(args[0]) - 1
                        if sim.metric.metric_number < 0:
                            scr.echo("Metric is off")
                        elif sim.metric.metric_number <= len(sim.metric.name):
                            if traf.area == "Circle":
                                scr.echo("(" + str(sim.metric.metric_number + 1) + ") " + sim.metric.name[
                                    sim.metric.metric_number] + " activated")
                                try:
                                    metric_dt = float(args[1])
                                    if metric_dt > 0:
                                        sim.metric.dt = metric_dt
                                        scr.echo("with dt = " + str(metric_dt))
                                    else:
                                        scr.echo("No valid dt")
                                except:
                                    scr.echo("with dt = " + str(sim.metric.dt))
                            else:
                                scr.echo("First define AREA FIR")
                        else:
                            scr.echo("No such metric")

                #----------------------------------------------------------------------
                # AREA command: AREA lat0,lon0,lat1,lon1[,lowalt]
                #               AREA FIR fir radius [lowalt]
                #----------------------------------------------------------------------
                elif cmd == "AREA":
                    
                    # debugger
#                    pdb.set_trace()                    
                    
                    if numargs == 0:
                        scr.echo("AREA lat0,lon0,lat1,lon1[,lowalt]")
                        scr.echo("or")
                        scr.echo("AREA fir,radius[,lowalt]")
                        scr.echo("or")
                        scr.echo("AREA circle,lat0,lon0,radius[,lowalt] ")
                    elif numargs == 1 and args[0] != "OFF" and args[0] != "FIR":
                        scr.echo("AREA lat0,lon0,lat1,lon1[,lowalt]")
                        scr.echo("or")
                        scr.echo("AREA fir,radius[,lowalt]")
                        scr.echo("or")
                        scr.echo("AREA circle,lat0,lon0,radius[,lowalt] ")
                        
                    elif numargs == 1:
                        if args[0] == "OFF":
                            if traf.swarea:
                                traf.swarea = False
                                scr.redrawradbg = True
                                traf.area = ""
                                scr.objappend(2, "AREA", None) # delete square areas
                                scr.objappend(3, "AREA", None) # delete circle areas
                        if args[0] == "FIR":
                            scr.echo("Specify FIR")

                    elif numargs > 1 and args[0][0].isdigit():

                        lat0 = float(args[0])  # [deg]
                        lon0 = float(args[1])  # [deg]
                        lat1 = float(args[2])  # [deg]
                        lon1 = float(args[3])  # [deg]

                        traf.arealat0 = min(lat0, lat1)
                        traf.arealat1 = max(lat0, lat1)
                        traf.arealon0 = min(lon0, lon1)
                        traf.arealon1 = max(lon0, lon1)

                        if numargs == 5:
                            traf.areafloor = float(args[4]) * ft
                        else:
                            traf.areafloor = -9999999.

                        traf.area = "Square"
                        traf.swarea = True
                        scr.redrawradbg = True
                        scr.objappend(2, "AREA", [lat0, lon0, lat1, lon1])

                        # Avoid mass delete due to redefinition of area
                        traf.inside = traf.ntraf * [False]

                    elif numargs > 2 and args[0] == "FIR":

                        for i in range(0, len(traf.navdb.fir)):
                            if args[1] == traf.navdb.fir[i][0]:
                                break
                        if args[1] != traf.navdb.fir[i][0]:
                            scr.echo("Unknown FIR, try again")
                        if sim.metric is not None:
                            sim.metric.fir_number = i
                            sim.metric.fir_circle_point = sim.metric.metric_Area.FIR_circle(traf.navdb, sim.metric.fir_number)
                            sim.metric.fir_circle_radius = float(args[2])
                        else:
                            scr.echo("warning: FIR not loaded into METRICS module because not active")

                        if numargs == 4:
                            traf.areafloor = float(args[3]) * ft
                        else:
                            traf.areafloor = -9999999.
                        if numargs > 4:
                            scr.echo("AREA command unknown")

                        traf.area = "Circle"
                        traf.swarea = True
                        scr.drawradbg()
                        traf.inside = traf.ntraf * [False]
                    
                    # circle code
                    elif (numargs > 2 and args[0] == "CIRCLE"):
                        
                        # draw circular experiment area
                        lat0 = np.float(args[1])   # Latitude of circle center [deg]
                        lon0 = np.float(args[2])   # Longitude of circle center [deg]
                        radius = np.float(args[3]) # Radius of circle Center [NM]                      
                                               
                        # Deleting traffic flying out of experiment area
                        traf.area = "Circle"
                        traf.swarea = True
                        traf.arearadius = radius
                        traf.arealat0 = lat0 # center of circle sent to traf
                        traf.arealon0 = lon0
                        
                        if numargs == 5:
                            traf.areafloor = float(args[4]) * ft # [m]
                        else:
                            traf.areafloor = -9999999. # [m]
                            
                        # draw the circular experiment area on the radar gui  
                        scr.redrawradbg = True                        
                        scr.objappend(3, "AREA", [lat0,lon0,radius])
                        
                        # Avoid mass delete due to redefinition of area
                        traf.inside = traf.ntraf * [False]
                        
                     
                    else:
                        scr.echo("AREA command unknown")
                        scr.echo("AREA lat0,lon0,lat1,lon1[,lowalt]")
                        scr.echo("or")
                        scr.echo("AREA fir,radius[,lowalt]")
                        scr.echo("or")
                        scr.echo("AREA circle,lat0,lon0,radius[,lowalt] ")

                #----------------------------------------------------------------------
                # TRAILS ON/OFF
                #----------------------------------------------------------------------
                elif cmd[:5] == "TRAIL":
                    if numargs == 0:
                        scr.echo("TRAIL ON/OFF [dt]/TRAIL acid color")
                        if traf.swtrails:
                            scr.echo("Trails are currently ON")
                            scr.echo("Trails dt=" + str(traf.trails.dt))
                        else:
                            scr.echo("Trails are currently OFF")
                    else:
                        if args[0] == "ON":
                            traf.swtrails = True
                            if numargs == 2:
                                try:
                                    trdt = float(args[1])
                                    traf.trails.dt = trdt
                                except:
                                    scr.echo("TRAIL ON dt")

                        elif args[0] == "OFF" or args[0] == "OF":
                            traf.swtrails = False
                            traf.trails.clear()

                        elif len(args[0]) != 0:
                            correctCommand = True
                            # Check if a color was selected
                            if len(args) == 1:
                                scr.echo('Syntax error')
                                scr.echo("TRAIL acid color")
                                correctCommand = False

                            if correctCommand:
                                acid = args[0]
                                color = args[1]

                                # Does aircraft exist?
                                idx = traf.id2idx(acid)
                                if idx < 0:
                                    scr.echo("TRAILS: " + acid + " not found.")
                                    idx = -1

                                # Does the color exist?
                                if color not in ("BLUE", "RED", "YELLOW"):
                                    scr.echo("Color not found, use BLUE, RED or YELLOW")
                                    idx = -1

                                # Change trail color of aircraft for which there are data
                                if idx >= 0:
                                    traf.changeTrailColor(color, idx)
                                    # scr.echo("TRAIL color of " + acid + " switched to: " + color)
                        else:
                            scr.echo('Syntax error')
                            scr.echo("TRAILS ON/OFF")

                #----------------------------------------------------------------------
                # CALC  expression
                #----------------------------------------------------------------------
                elif cmd[:4] == "CALC":
                    if numargs == 0:
                        scr.echo("CALC expression")
                    else:
                        try:
                            x = eval(line[5:])  # lower for units!
                            scr.echo("Ans = " + str(x))
                        except:
                            scr.echo("CALC: Syntax error")

                #------------------------------------------------------------------
                # !!! This is a template, please make a copy and keep it !!!
                # Insert new command here: first three chars should be unique
                #------------------------------------------------------------------
                elif cmd[:3] == "XXX":
                    if numargs == 0:
                        scr.echo("cmd arg1, arg2")
                    else:
                        arg1 = args[0]  # arguments are strings
                        arg2 = args[1]  # arguments are strings

                #-------------------------------------------------------------------
                # Reference to other command files
                # Check external references
                #-------------------------------------------------------------------
                elif cmd[:4] in self.extracmdrefs:
                    self.extracmdrefs[cmd[:4]].process(cmd[4:], numargs, [cmd] + args, sim, traf, scr, self)

                #-------------------------------------------------------------------
                # Command not found
                #-------------------------------------------------------------------
                else:
                    if numargs == 0:
                        scr.echo("Unknown command or aircraft: " + cmd)
                    else:
                        scr.echo("Unknown command: " + cmd)

                #**********************************************************************
                #======================  End of command branches ======================
                #**********************************************************************

                # Syntax not ok, => Default syntax error message:
                if synerr:
                    scr.echo("Syntax error in command:" + cmd)
                    scr.echo(line)

        # End of for-loop of cmdstack
        self.cmdstack = []
        return

    def argparse(self, argtype, argidx, args, traf, scr):
        parsed_args = []

        if args[argidx] == "" or args[argidx] == "*":  # Empty arg or wildcard => parse None
            parsed_args.append(None)

        elif argtype == "acid":  # aircraft id => parse index
            idx = traf.id2idx(args[argidx])
            if idx < 0:
                scr.echo(cmd + ":" + args[idx] + " not found")
                raise IndexError
            else:
                parsed_args.append(idx)

        elif argtype == "txt":  # simple text
            parsed_args.append(args[argidx])

        elif argtype == "float":  # float number
            parsed_args.append(float(args[argidx]))

        elif argtype == "int":   # integer
            parsed_args.append(int(args[argidx]))

        elif argtype == "onoff" or argtype == "bool":
            sw = (args[argidx] == "ON" or
                  args[argidx] == "1" or args[argidx] == "TRUE")
            parsed_args.append(sw)

        elif argtype == "latlon":
            parsed_args.append(txt2lat(args[argidx]))
            parsed_args.append(txt2lon(args[argidx + 1]))

        elif argtype == "spd":  # CAS[kts] Mach
            spd = float(args[argidx].upper().replace("M", ".").replace("..", "."))
            if not 0.1 < spd < 1.0:
                spd *= kts
            parsed_args.append(spd)  # speed CAS[m/s] or Mach (float)

        elif argtype == "vspd":
            parsed_args.append(fpm * float(args[argidx]))

        elif argtype == "alt":  # alt: FL250 or 25000 [ft]
            parsed_args.append(ft * txt2alt(args[argidx]))  # alt in m

        elif argtype == "hdg":
            # TODO: for now no difference between magnetic/true heading
            hdg = float(args[argidx].upper().replace('T', '').replace('M', ''))
            parsed_args.append(hdg)

        elif argtype == "time":
            ttxt = args[argidx].strip().split(':')
            if len(ttxt) >= 3:
                ihr  = int(ttxt[0])
                imin = int(ttxt[1])
                xsec = float(ttxt[2])
                parsed_args.append(ihr * 3600. + imin * 60. + xsec)
            else:
                parsed_args.append(float(args[argidx]))

        return parsed_args
