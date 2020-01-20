
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


def chooseThreshold(X, T, resampling = 30, mother_wavelet_type = 'rbio2.2', wave_mode = 'per', lenThVector=10):
    '''

    :param X: Input sensor data
    :param T: Unix timestamps of that data
    :param resampling: sampling period, as string. Useful when data is not at constant rate
    :param mother_wavelet_type: check families with pywt.families() and wavelets in each with pywt.wavelist(family_i, kind='discrete')
    :param wave_mode: check modes with print(pywt.MODES.modes) and see https://pywavelets.readthedocs.io/en/latest/ref/signal-extension-modes.html
    :param lenThVector:  how many swaps...
    :return: threshold_vector,mse_list
    '''
    # first create a full model
    (data, times, N) = conditionDWT(X, T, resampling)

    mother_wavelet = pywt.Wavelet(mother_wavelet_type)
    cA, cD = pywt.dwt(data, wavelet=mother_wavelet, mode=wave_mode)

    # build a list of threshold values based on the coefficient values
    tmp = np.abs(np.append(cA, cD))
    tmp = list(set(tmp))
    tmp = np.ceil(100 * tmp) / 100
    tmp.sort()

    stepTh = int(len(tmp) / lenThVector)
    if stepTh == 0:
        stepTh = 1

    # I remove the last one because makes no sense, it would be without data
    threshold_vector = tmp[0:-1:stepTh]

    # need one at least
    if (len(threshold_vector)==0):
        threshold_vector = [ tmp[-2] ]
    
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

def conditionDWT(X,T,resampling = 30):
    '''

    :param X: input data
    :param T: timestamps (int) corresponding data
    :param resampling:  resampling time to continuous data
    :return:  X resampled to provided period, data timestamps and length
    '''
    
    # from POSIX seconds to pandas datetime
    T1 = pd.to_datetime(T,unit='s')
    timeSerie = pd.Series(X, T1)
    
    tDelta = pd.Timedelta(seconds=resampling)
    timeSerie = timeSerie.resample(tDelta).ffill()

    data = timeSerie.values
    times = timeSerie.index.values.astype(np.int64) // 10**6
    N = len(times)
    return (data,times,N)


def createModel(X, T, c_threshold, resampling = 30, mother_wavelet_type = 'rbio2.2', wave_mode = 'per'):
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

    # coefficients are densely stored
    cAhat = cAhat.todense().A1
    cDhat = cDhat.todense().A1

    mother_wavelet = pywt.Wavelet(mother_wavelet_type)
    xhat = pywt.idwt(cAhat, cDhat, wavelet=mother_wavelet, mode=wave_mode)
    xhat = conditionIDWT(xhat, N)

    n = getSamplesReal(tArray, resampling, N, t0 )
    x_pred=[]
    for n_i in n:
        x_i =  xhat[n_i]
        x_pred.append(x_i)

    return x_pred


    # t0 initial timestamp from model in seconds
    # fs sampling frequency in model in hertzs
    # N number of samples in model

def getSamplesReal(tArray, resampling, N, t0 ):

    fs = 1.0/ resampling
    tM = t0 + np.arange(0, (N/fs), 1/fs)
    
    # print("N is " + str(N))
    # print("tM len is "+str(len(tM) ) )

    dM = map(datetime.datetime.fromtimestamp,tM)

    # get indexers from model
    weekM =np.array(map(lambda x: x.weekday(), dM))
    hourM =np.array(map(lambda x: x.hour, dM))
    minM =np.array(map(lambda x: x.minute, dM))
    secM =np.array(map(lambda x: x.second, dM))

    debugC=0
    n_=[ getSampleReal(tf,resampling,weekM,hourM,minM,secM) for tf in tArray ]

    return n_


def getSampleReal(tf0, ts, weekM, hourM, minM, secM):
    # tf0 is future timestamp in epoc seconds

    # we round it with sampling time ts
    tf = (tf0 // ts) * ts

    # tf = (df-datetime.datetime(1970,1,1,0,0,0)).total_seconds()
    df = datetime.datetime.fromtimestamp(tf)

    # candidates have same weekday hour, minute and second.
    sameWeek = (weekM == df.weekday()) 
    sameHour = (hourM == df.hour) 
    sameMin = (minM == df.minute) 
    sameSec = (secM == df.second)

    # print("future time is (" + str(tf0) + "): "+datetime2str(datetime.datetime.fromtimestamp(tf0))) 
    # print("Rounded to (" + str(tf) + "): "+datetime2str(df)) 

    indexs = sameSec & sameMin & sameHour & sameWeek
    if not indexs.any():
        indexs = sameSec & sameMin & sameHour
    if not indexs.any():
        indexs = sameSec & sameMin
    if not indexs.any():
        indexs = sameSec 

    # todo affine this
    selectedIndex = np.nonzero(indexs)[0][0]

    ans = selectedIndex

    return ans

def datetime2str(d):
    fmt = "%d/%m/%y - %H:%M:%S"
    #d = datetime.datetime.utcfromtimestamp(tf)
    return d.strftime(fmt)