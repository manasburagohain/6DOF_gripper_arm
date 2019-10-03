
import os
def main():
     #Open the file back and read the contents
     # mypath ="/home/student/armlab-f19/util"
     # if not os.path.exists(mypath):
     #     os.makedirs(mypath,0755)
     #     print"Path is created"
     # fname = mypath + "/" + "calibration.cfg"
     # with open(fname,"w") as x:
     #      content=x.read()
     #      print content

    f=open("calibration.cfg", "r")
    if f.mode == 'r': 
        contents =f.read()
        # print (contents)
     # #or, readlines reads the individual line into a list
        fl =f.readlines()
        for x in fl:
            print (x)
if __name__== "__main__":
     main()