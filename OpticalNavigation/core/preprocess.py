from __future__ import division
from math import *
import cv2
import numpy as np
from core.const import CisLunarCameraParameters
import time

def rolling_shutter(img):
    """
    Perform rolling shutter correction on frame
    [img]: frame
    @returns
    [new_img]: frame corrected for rolling shutter
    """
    time.sleep(3)
    raise NotImplementedError("implement rolling shutter transformation")

def extract_frames(vid_dir):
    """
    Extracts frames from video located at path [vid_dir]
    @returns
    [frames]: returns list of paths to extracted frames
    """
    # return np.zeros((10, int(CisLunarCameraParameters.hPix), int(CisLunarCameraParameters.vPix), 3), dtype=np.float)
    time.sleep(3)
    raise NotImplementedError("implement frame extraction")
 
def rect_to_stereo_proj(img, fov=62.2, fov2=48.8):
    """
    Source:     http://lexafrancis.com/rectilinear-to-stereographic-image-converter-python/

    Converts [img] from the camera rectilinear projection to stereographic projection.
    This is necessary for body detection.
    [img]: (hxwx3) numpy array representing the image in rectilinear projection
    [fov]: (float) horizontal field of view in degrees. For us, it is 62.2
    [fov2]: (float) vertical field of view. For us, it is 48.8 
    
    Returns new image in stereographic projection
    """
    imgh, imgw, bits = img.shape
    wh  = int( imgw / 2 )
    hh  = int( imgh / 2 )
    
    img2 = img.copy()
    img2[0:imgh] = (0,0,0)
    
    def getcentre( p ): return (p[0] - wh, p[1] - hh)
    def decentre ( p ): return (int( p[0] + wh ), int( p[1] + hh ))
    def polar    ( p ): return (sqrt( p[0]**2 + p[1]**2 ), atan2( p[1], p[0] ))
    def cartesian( p ): return (p[0] * cos( p[1] ), p[0] * sin( p[1] ))
    def normalize( p ):
        return (radians( fov2 ) * (p[0]/wh), radians( fov2 ) * (p[1]/wh))
    def spaceout ( p ):
        return ((p[0] * wh) / radians( fov2 ), (p[1] * wh) / radians( fov2 ))
    
    scale      = 1.0 - ((fov - 90) / 340) ** 1.5 if fov > 90 else 1.0
    
    for y in range( imgh ):
        for x in range( imgw ):
            p = polar( normalize( getcentre( (x, y) ) ) )
            p = (tan( 2*atan( p[0]/2 ) ) * scale, p[1])
            p = decentre( spaceout( cartesian( p ) ) )
            if p[1] >= 0 and p[1] < imgh and p[0] > 0 and p[0] < imgw:
                img2[y,x] = img[p[1],p[0]]
    
    return img2