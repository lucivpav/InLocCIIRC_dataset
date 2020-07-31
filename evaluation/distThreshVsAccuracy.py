import os
import pandas as pd

def getAccuracy(distanceThreshold, angularThreshold, errorsDf):
    nCorrect = 0
    nQueries = len(errorsDf)
    for i in range(nQueries):
        translationError = errorsDf['translation'][i]
        orientationError = errorsDf['orientation'][i]
        if translationError < distanceThreshold and orientationError < angularThreshold:
            nCorrect += 1
    accuracy = nCorrect / nQueries * 100
    return accuracy

experimentName = 's10e-v4.2' # TODO: adjust
angularThreshold = 10 # in degrees
datasetDir = '/Volumes/GoogleDrive/Můj disk/ARTwin/InLocCIIRC_dataset'
evaluationDir = os.path.join(datasetDir, 'evaluation-' + experimentName)
errorsPath = os.path.join(evaluationDir, 'errors.csv')
outputPath = os.path.join(evaluationDir, 'distThreshVsAccuracy.csv')
inLocDataPath = 'evaluation/InLocDistThreshVsAccuracy.csv'

errorsDf = pd.read_csv(errorsPath, sep=',')
inLocDf = pd.read_csv(inLocDataPath, sep=';', header=None)
outputFile = open(outputPath, 'w')

for i in range(len(inLocDf)):
    distanceThreshold = inLocDf[0][i]
    accuracy = getAccuracy(distanceThreshold, angularThreshold, errorsDf) # InLocCIIRC accuracy
    outputFile.write('%f; %f\n' % (distanceThreshold, accuracy))

outputFile.close()