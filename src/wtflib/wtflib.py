
# TODO: pywt is not in rosdep rules ...
import pywt
import pandas as pd
from scipy import sparse
from sklearn.metrics import mean_squared_error
import datetime
import numpy as np

'''

Library for the WaveModels package:



'''


def chooseThreshold(X, T, resampling = '30S', mother_wavelet_type = 'rbio2.2', wave_mode = 'periodization', lenThVector=10):
    '''

    :param X: Input sensor data
    :param T: Unix timestamps of that data
    :param resampling: sampling period, as string. Useful when data is not at constant rate
    :param mother_wavelet_type: check families with pywt.families() and wavelets in each with pywt.wavelist(family_i, kind='discrete')
    :param wave_mode: periodization ....
    :param lenThVector:  how many swaps...
    :return: threshold_vector,mse_list
    '''
    # first create a full model
    (data, times, N) = conditionDWT(X, T, resampling)

    mother_wavelet = pywt.Wavelet(mother_wavelet_type)
    cA, cD = pywt.dwt(data, wavelet=mother_wavelet, mode=wave_mode)

    # build a list of threshold values based on the coefficient values
    tmp = np.abs(np.append(cA, cD))
    tmp = np.ceil(100 * tmp) / 100
    tmp = list(set(tmp))
    tmp.sort()
    stepTh = int(len(tmp) / lenThVector)
    if stepTh == 0:
        stepTh = 1

    # I remove the last one because makes no sense, it would be without data
    tmp = tmp[0:-1:stepTh]

    threshold_vector = tmp
    mse_list = []
    for coef_thre in threshold_vector:
        cAhat = cA.copy()
        cDhat = cD.copy()

        cAhat[np.abs(cA) < coef_thre] = 0.0
        cDhat[np.abs(cD) < coef_thre] = 0.0

        # Inverse transform   ....................................................
        xhat = pywt.idwt(cAhat, cDhat, wavelet=mother_wavelet, mode=wave_mode)
        xhat = np.array(xhat)

        # Binarize   ....................................................
        xrec = conditionIDWT(xhat,N)

        # calculate performance metric  ....................................................
        ri = mean_squared_error(data, xrec )
        mse_list.append(ri)

    return (threshold_vector,mse_list)

def conditionIDWT(xh,N):
    '''

    :param xh: output from idwt
    :param N:  max lenth the vector should have
    :return:  binarized xn
    '''
    xrec =xh.copy()

    if len(xrec) != N:
        xrec = xrec[0:-1]  # makes x and xhat same length... WHY is THIS NEEDED????

    xrec[xrec < 0.5] = 0.0
    xrec[xrec > 0.0] = 1.0
    return xrec

def conditionDWT(X,T,resampling = '30S'):
    '''

    :param X: input data
    :param T: timestamps (int) corresponding data
    :param resampling:  resampling time to continuous data
    :return:  X resampled to provided period, data timestamps and length
    '''

    timeSerie = pd.Series(X, T)
    timeSerie = timeSerie.resample(resampling).ffill()

    data = timeSerie.values
    times = timeSerie.index.values
    N = len(times)
    return (data,times,N)


def createModel(X, T, c_threshold, resampling = '30S', mother_wavelet_type = 'rbio2.2', wave_mode = 'periodization'):
    '''
    :param X: input data
    :param T: sampling times
    :param c_threshold: wavelet coefs threshold
    :param resampling: resampling time
    :param mother_wavelet_type: mother wavelet used
    :param wave_mode:  mode....
    :return: (cAhat, cDhat, N, times[0], resampling, wave_mode, mother_wavelet_type)
    '''

    # condition data:
    (data, times, N) = conditionDWT(X, T,resampling)
    mother_wavelet = pywt.Wavelet(mother_wavelet_type)
    cA, cD = pywt.dwt(data, wavelet=mother_wavelet, mode=wave_mode)
    cAhat = cA.copy()
    cDhat = cD.copy()
    cAhat[np.abs(cAhat) < c_threshold] = 0.0
    cDhat[np.abs(cDhat) < c_threshold] = 0.0
    cAhat = sparse.coo_matrix(cAhat)
    cDhat = sparse.coo_matrix(cDhat)

    model = (cAhat, cDhat, N, times[0], resampling, wave_mode, mother_wavelet_type)
    return model

# . . . . . . . . . . . . . . . . .. . . . . .
def predictWaveletValues(tArray, waveletModel):
    '''
    :param tArray: times for prediction
    :param waveletModel:  model used
    :return:  x_predicted
    '''
    (cAhat, cDhat, N, t0, resampling, wave_mode, mother_wavelet_type)  = waveletModel

    mother_wavelet = pywt.Wavelet(mother_wavelet_type)
    xhat = pywt.idwt(cAhat, cDhat, wavelet=mother_wavelet, mode=wave_mode)
    xhat = conditionIDWT(xhat, N)

    # t0 initial timestamp from model in seconds
    # fs sampling frequency in model in hertzs
    # N number of samples in model

    fs = 1.0/ resampling
    tM = t0 + np.arange(0, (N/fs), 1/fs)
    dM = map(datetime.datetime.fromtimestamp,tM)

    # get indexers from model
    weekM =np.array(map(lambda x: x.weekday(), dM))
    hourM =np.array(map(lambda x: x.hour, dM))
    minM =np.array(map(lambda x: x.minute, dM))
    secM =np.array(map(lambda x: x.second, dM))

    x_pred=[]
    debugC=0
    for tf in tArray:
        n_i =  getSampleReal(tf,weekM,hourM,minM,secM)
        x_i =  xhat[n_i]
        x_pred.append(x_i)

        # debug
        if debugC==0:
            fmt = "%d/%m/%y - %H:%M:%S"
            df = datetime.datetime.utcfromtimestamp(tf)
            dm = dM[n_i]
            print 'tf (' + df.strftime(fmt) + ' == tm(' + dm.strftime(fmt) + ')'
        debugC = (debugC+1)%2000

    return x_pred

def getSampleReal(tf, weekM, hourM, minM, secM):
    # tf future timestamp in seconds

    # tf = (df-datetime.datetime(1970,1,1,0,0,0)).total_seconds()
    df = datetime.datetime.fromtimestamp(tf)

    # candidates have same weekday hour, minute and second.

    indexs = (weekM == df.weekday()) & (hourM == df.hour) & (minM == df.minute) & (secM == df.second)
    if not indexs.any():
        indexs = (hourM == df.hour) & (minM == df.minute) & (secM == df.second)

    # todo affine this
    selectedIndex = np.nonzero(indexs)[0][0]

    ans = selectedIndex

    return ans