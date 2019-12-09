import os
import sys

path= sys.argv[1]
path= os.path.abspath( path)
if not path[-1] == "/":
    path+= "/"
for fp in os.listdir( path):
    if ".xml" == fp[-4:]:
        with open( path + fp, "r") as myfile:
            f= myfile.read()
            f= f[:f.index( "<folder>") + 8] + path.split("/")[-2] + f[f.index("</folder>"):]
            f= f[:f.index( "<path>") + 6] + path + fp + f[f.index("</path>"):]
        with open( path + fp, "w") as myfile:
            myfile.write( f)
        
