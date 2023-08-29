def importFile(slotid=0):
    import os, sys
 
    with open("/flash/program/" + "{0:0=2d}".format(slotid) + "/program.mpy", "rb") as f:
        program = f.read()
    
    try:
        os.remove("/flash/importFile.mpy")
    except:
        pass
    with open("/flash/importFile.mpy","w+") as f:
        f.write(program)
    
    if ("importFile" in sys.modules):
        del sys.modules["importFile"]
    exec("from importFile import *")
