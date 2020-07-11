import os
import re
import shutil
import time

downloads = 'C:\\Users\\easha\\Downloads\\'
destination_root = 'D:\\OpNav\\data\\CislunarFullTraj_60secs\\iterations\\'

while(True):
    print('Looking for images in Downloads')
    for filename in os.listdir(downloads):
        if filename.endswith(".png"): 
            print(os.path.join(downloads, filename))
            filenamereg = re.compile(r'(\d+)-(\d+)-(\d+)-(\d+)-(\d+)T(\d+) (\d+) (\d+)Z.png') 
            mo = filenamereg.search(filename) 
            if mo is not None:
                iteration = int(mo.groups()[0])
                view = int(mo.groups()[1])
                print(view, iteration)
                camera = 1
                if view <= 15 and view >= 8:
                    camera = 2
                
                elif view >= 16:
                    camera = 3

                dest = os.path.join(destination_root, str(iteration))
                if not os.path.exists(dest):
                    os.makedirs(dest)

                dest = os.path.join(destination_root, str(iteration), str(camera))
                if not os.path.exists(dest):
                    os.makedirs(dest)

                dest = os.path.join(destination_root, str(iteration), str(camera), filename)
                shutil.move(os.path.join(downloads, filename),dest)
    time.sleep(3)
            