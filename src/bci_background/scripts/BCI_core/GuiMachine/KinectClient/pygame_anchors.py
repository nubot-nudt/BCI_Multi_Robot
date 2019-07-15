#author:mrtang
#date:2017.7
#version:1.0
#email:mrtang@nudt.edu.cn

import numpy as np

anchors ={ 'lefttop':       np.array([0,0]),
           'left':          np.array([0,-0.5]),
           'leftbottom':    np.array((0,-1)),
           'righttop':      np.array((-1,0)),
           'right':         np.array((-1,-0.5)),
           'rightbottom':   np.array((-1,-1)),
           'top':           np.array((-0.5,0)),
           'bottom':        np.array((-0.5,-1)),
           'center':        np.array((-0.5,-0.5))}

def getcorner(siz,anchor):
    if anchors.has_key(anchor):
        return (-1*np.array(siz)*anchors[anchor]).astype(np.int32)
    else:
        return (0,0)

def blit_pos1(siz,position,anchor):
    if anchors.has_key(anchor):
        position = np.array(position)
        siz = np.array(siz)
        anch = anchors[anchor]
        return (position + anch*siz).astype(np.int32)
    else:
        return position

def blit_pos(sur,position,anchor):
    return blit_pos1(sur.get_size(),position,anchor)