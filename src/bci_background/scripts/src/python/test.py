#!/usr/bin/env python
# # Software License Agreement (BSD License)
# # Copyright (c) 2008, Willow Garage, Inc.
# # All rights reserved.
# # Revision $Id$

# import rospy
# from nubot_common.msg import StrategyInfo
# # import msgs.StrategyInfo
# from bci_background.msg import StrategyInfo
# def recommend_subscribe():
#     rospy.init_node('BCI_miao')
#     # rospy.init_node('BCI_miao', anonymous=False)
#     rospy.Subscriber('/nubot1/nubotcontrol/recommend_strategy', StrategyInfo, callback1)
#     rospy.spin()
#
#
# def callback1(data):
#
#     print('maio',data)
#
# if __name__ ==  '__main__':
#
#     recommend_subscribe()
import scipy.io as sio
import scipy.signal
import numpy as np

# a = np.array([[1,2,3],[2,3,4]])
# print(a)
# print(np.std(a))
# print(np.var(a))

# a = np.zeros((1,3))
# print(a)
# b = np.hstack((a,a))
# c = np.vstack((b,b))
# print(c[:-1,:6])


# print(a[0][0])
# b = [0]
# b[1:] = [0,0,0]
# if len(b)>1 and not b[1:] == [0,0,0]:
#     print('miao')
# print(b)
# c = [0]
# if c == 0:
#     print('maiomiao')
#

testdata = sio.loadmat('/home/mars/lyrworkspace/BCI_Multi_Robot/BCI_Multi_Robot/src/bci_background/scripts/src/python/nontarget_score_offline_1_lxb.mat')
offline = testdata['nontarget_score_offline_1']
print(offline[0].shape)

#
#
a = [1.2,2,4,5]
print(a)
print(a[:-1])
# print(a.index(2))
# print(len(a))
# result = a
# print(result)
# result = [3,4,5,6]
# print(result)
# # ##########test CCA
# # from sklearn.cross_decomposition import PLSCanonical, PLSRegression,CCA
# testdata = sio.loadmat('/home/mars/lyrworkspace/BCI_Multi_Robot/BCI_Multi_Robot/src/bci_background/scripts/src/python/SSVEP/offlineprocess/lxbdata.mat')
# signal = testdata['signal']
# signal = np.transpose(signal[:,0:-1])
# print(signal.shape)
# ss = signal[-1,:]
# n = 3
# MdB = 20
# bprange = np.array([6.0, 55.0])
# Ws = bprange / (200/2)
# b, a = scipy.signal.iirfilter(n, Ws, rs=MdB, ftype='cheby2')  # Hd_Bandpass
# signal_f = scipy.signal.lfilter(b, a, np.transpose(signal), axis=0)  # shuzhe
# print(np.std(signal_f)**2)
# print(np.var(signal_f))

# n = 3
# MdB = 20
# bprange = np.array([6.0, 35.0])
# Ws = bprange / (200/2)
# b, a = scipy.signal.iirfilter(n, Ws, rs=MdB, ftype='cheby2')  # Hd_Bandpass
# signal_f = scipy.signal.lfilter(b, a, np.transpose(signal), axis=0)  # shuzhe
# t = np.arange(0.005,0.55,0.005)
# Y = {}
# frequency = [9,11.7,14.5]
# for f in range(len(frequency)):
#     y = np.array([np.sin(2*np.pi*frequency[f]*t),np.cos(2*np.pi*frequency[f]*t),np.sin(4*np.pi*frequency[f]*t),np.cos(4*np.pi*frequency[f]*t),np.sin(6*np.pi*frequency[f]*t),np.cos(6*np.pi*frequency[f]*t)])
#     Y[str(f)] = y
# cca = CCA(n_components=2)
# xx = signal_f[0:100,0:2]
# yy = (Y[str(0)][0:3,0:100]).T
# x_scores, y_scores = cca.fit_transform(xx,yy)
# result = np.corrcoef(x_scores.T, y_scores.T)
# R = np.diag(result[-2:,:-2])
# print(R)
# ###########



#
# # # !/usr/bin/env python
# #
# # import warnings
# #
# # # import numpy as np
# # import scipy.stats
# #
# #
# # def pobs(x):
# #     if (type(x) != np.ndarray):
# #         raise Exception('Input x must be a numpy ndarray datatype!')
# #
# #     shape_x = x.shape
# #     if (len(shape_x) < 2):
# #         x = x.reshape((shape_x[0], 1))
# #     n = x.shape[0]
# #     d = x.shape[1]
# #
# #     u = np.zeros(shape=x.shape)
# #     for ii in range(d):
# #         u[:, ii] = scipy.stats.rankdata(x) / (n + 1.)
# #
# #     return u
# #
# #
# # def cca(X, Y):
# #     """
# #     Canonical Correlation Analysis
# #     Currently only returns the canonical correlations.
# #     """
# #     n, p1 = X.shape
# #     n, p2 = Y.shape
# #
# #     # center X and Y
# #     meanX = X.mean(axis=0)
# #     meanY = Y.mean(axis=0)
# #     X = X - meanX[np.newaxis, :]
# #     Y = Y - meanY[np.newaxis, :]
# #
# #     Qx, Rx = np.linalg.qr(X)
# #     Qy, Ry = np.linalg.qr(Y)
# #
# #     rankX = np.linalg.matrix_rank(Rx)
# #     if rankX == 0:
# #         raise Exception('Rank(X) = 0! Bad Data!')
# #     elif rankX < p1:
# #         # warnings.warn("X not full rank!")
# #         Qx = Qx[:, 0:rankX]
# #         Rx = Rx[0:rankX, 0:rankX]
# #
# #     rankY = np.linalg.matrix_rank(Ry)
# #     if rankY == 0:
# #         raise Exception('Rank(X) = 0! Bad Data!')
# #     elif rankY < p2:
# #         # warnings.warn("Y not full rank!")
# #         Qy = Qy[:, 0:rankY]
# #         Ry = Ry[0:rankY, 0:rankY]
# #
# #     d = min(rankX, rankY)
# #     svdInput = np.dot(Qx.T, Qy)
# #
# #     U, r, V = np.linalg.svd(svdInput)
# #     r = np.clip(r, 0, 1)
# #     # A = np.linalg.lstsq(Rx, U[:,0:d]) * np.sqrt(n-1)
# #     # B = np.linalg.lstsq(Ry, V[:,0:d]) * np.sqrt(n-1)
# #
# #     # TODO: resize A to match inputs
# #
# #     # return (A,B,r)
# #     return r
# #
# #
# # def rdc(x, y, k=20, s=1. / 6.):
# #     """
# #     Computes the RDC between two sets of (possibly multivariate)
# #     random variables.  x and y are matrices, both with n samples, and
# #     d1 and d2 (possibly w/ d1=d2) dimensions.  They must be numpy
# #     datatypes!
# #     See: https://papers.nips.cc/paper/5138-the-randomized-dependence-coefficient.pdf
# #     """
# #     if (type(x) != np.ndarray or type(y) != np.ndarray):
# #         raise Exception('Inputs x and y must be numpy ndarray datatypes!')
# #     shape_x = x.shape
# #     shape_y = y.shape
# #     if (len(shape_x) > 2 or len(shape_y) > 2):
# #         raise Exception('Tensor inputs not allowed!')
# #     if (len(shape_x) == 1):
# #         shape_x = (shape_x[0], 1)
# #         x = x.reshape((shape_x[0], 1))
# #     if (len(shape_y) == 1):
# #         shape_y = (shape_y[0], 1)
# #         y = y.reshape((shape_y[0], 1))
# #     # ensure we have the same # of observations for x and y
# #     if (shape_x[0] != shape_y[0]):
# #         raise Exception('The number of observations for RVs x and y must be the same!')
# #
# #     # compute pseudo-observations & insert into CCA computation vector
# #     xx = np.ones(shape=(shape_x[0], shape_x[1] + 1))
# #     yy = np.ones(shape=(shape_y[0], shape_y[1] + 1))
# #     u = pobs(x)
# #     v = pobs(y)
# #     # copy the data into xx & yy
# #     for ii in range(shape_x[1]):
# #         xx[:, ii] = u[:, ii]
# #     for ii in range(shape_y[1]):
# #         yy[:, ii] = v[:, ii]
# #
# #     xn = np.random.normal(size=(xx.shape[1], k))
# #     yn = np.random.normal(size=(yy.shape[1], k))
# #
# #     xs = np.sin(s / xx.shape[1] * np.dot(xx, xn))  # dot is matrix multiplication for ndarray datatypes
# #     ys = np.sin(s / yy.shape[1] * np.dot(yy, yn))  # dot is matrix multiplication for ndarray datatypes
# #
# #     xu = np.ones(shape=(xs.shape[0], xs.shape[1] + 1))
# #     yu = np.ones(shape=(ys.shape[0], ys.shape[1] + 1))
# #
# #     # copy the data into xu and yu
# #     for ii in range(xs.shape[1]):
# #         xu[:, ii] = xs[:, ii]
# #     for ii in range(ys.shape[1]):
# #         yu[:, ii] = ys[:, ii]
# #
# #     r = cca(xu, yu)
# #     # res = r[0]
# #     res = r
# #     return res
# #
# #
# # if __name__ == '__main__':
# #     import scipy.io as sio
# #
# #     np.set_printoptions(precision=4,
# #                         threshold=10000,
# #                         linewidth=150,
# #                         suppress=True)
# #
# #     # M = 500
# #     #
# #     # rho = 0.5
# #     # mu = [0, 0]
# #     # R = [[1, rho], [rho, 1]]
# #     # x = np.random.multivariate_normal(mu, R, M)
# #     # rdcVal = rdc(x[:, 0], x[:, 1])
# #     # print(rdcVal)
# #
# #     # test perfect non-monotonic functional dependence
# #
# #     testdata = sio.loadmat(
# #         '/home/mars/lyrworkspace/BCI_Multi_Robot/BCI_Multi_Robot/src/bci_background/scripts/src/python/SSVEP/offlineprocess/lxbdata.mat')
# #     state = testdata['state']
# #     signal = testdata['signal']
# #     signal = np.transpose(signal[:, 0:-1])
# #     n = 3
# #     MdB = 20
# #     bprange = np.array([6.0, 35.0])
# #     Ws = bprange / (200 / 2)
# #     b, a = scipy.signal.iirfilter(n, Ws, rs=MdB, ftype='cheby2')  # Hd_Bandpass
# #     signal_f = scipy.signal.lfilter(b, a, np.transpose(signal), axis=0)  # shuzhe
# #     # print(signal_f[10,:])
# #     # print(signal_f.shape)
# #     a = range(20)
# #
# #     t = np.arange(0.005, 0.55, 0.005)
# #     Y = {}
# #     frequency = [9, 11.7, 14.5]
# #     for f in range(len(frequency)):
# #         y = np.array([np.sin(2 * np.pi * frequency[f] * t), np.cos(2 * np.pi * frequency[f] * t),
# #                       np.sin(4 * np.pi * frequency[f] * t), np.cos(4 * np.pi * frequency[f] * t),
# #                       np.sin(6 * np.pi * frequency[f] * t), np.cos(6 * np.pi * frequency[f] * t)])
# #         # y = np.array([np.sin(2*np.pi*f*t),np.cos(2*np.pi*f*t),np.sin(4*np.pi*f*t),np.cos(4*np.pi*f*t),np.sin(6*np.pi*f*t),np.cos(6*np.pi*f*t)])
# #         Y[str(f)] = y
# #     # print(Y['1'][:,0:10])
# #     score = np.zeros((1, len(frequency)))
# #     # print(score)
# #     # print(y.shape)
# #     # print(y[:,0:10])
# #     # cca = CCA(n_components=1)
# #     # cca = PLSRegression(n_components=1)
# #     xx = signal_f[0:30, 0:1]
# #     print(xx)
# #     # yy = np.transpose(Y[str(0)][0:2,0:30])
# #     yy = (Y[str(0)][0:1, 0:30]).T
# #     print(yy)
# #     # x = np.random.uniform(low=-1.0, high=1.0, size=(M, 1))
# #     # y = np.power(x, 2)
# #     rdcVal = rdc(xx, yy)
# #     print(rdcVal)
