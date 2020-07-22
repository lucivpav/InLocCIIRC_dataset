import numpy as np
import os
import sys
import matplotlib.pyplot as plt
import scipy.io as sio
from PIL import Image
sys.path.insert(1, os.path.join(sys.path[0], '../functions'))
from InLocCIIRC_utils.buildCutoutName.buildCutoutName import buildCutoutName
import matlab.engine
matlabEngine = matlab.engine.start_matlab()

def saveFigure(fig, path, width, height):
    plt.axis('off')
    fig.savefig(path, bbox_inches='tight', pad_inches=0)
    img = Image.open(path)
    img = img.resize((width, height), resample=Image.NEAREST)
    img.save(path)

def renderForQuery(queryId, mode):
    inlierColor = '#00ff00'
    inlierMarkerSize = 3
    targetWidth = 600
    cutoutSize = [1600, 1200] # width, height
    targetAspectRatio = cutoutSize[0] / cutoutSize[1]
    targetHeight = np.round(targetWidth / targetAspectRatio).astype(np.int64)
    extension = '.png'

    datasetDir = '/Volumes/GoogleDrive/Můj disk/ARTwin/InLocCIIRC_dataset'
    #queryDir = os.path.join(datasetDir, 'query-s10e')
    queryDir = os.path.join(datasetDir, 'query-HoloLens1')
    #outputDir = os.path.join(datasetDir, 'outputs-s10e')
    outputDir = os.path.join(datasetDir, 'outputs')
    #outputDir = os.path.join(datasetDir, 'outputs-HL1-v2')
    cutoutDir = os.path.join(datasetDir, 'cutouts')

    if mode == 'PV':
        shortlistPath = os.path.join(outputDir, 'densePV_top10_shortlist.mat')
    elif mode == 'PE':
        shortlistPath = os.path.join(outputDir, 'densePE_top100_shortlist.mat')
    else:
        raise 'Unsupported mode!'

    denseInlierDir = os.path.join(outputDir, 'PnP_dense_inlier')
    synthesizedDir = os.path.join(outputDir, 'synthesized')
    #evaluationDir = os.path.join(datasetDir, 'evaluation-s10e-v1')
    evaluationDir = os.path.join(datasetDir, 'evaluation')
    #evaluationDir = os.path.join(datasetDir, 'evaluation-HL1-v2')
    queryPipelineDir = os.path.join(evaluationDir, 'queryPipeline')

    queryName = str(queryId) + '.jpg'
    queryPath = os.path.join(queryDir, queryName)
    query = plt.imread(queryPath)
    queryWidth = query.shape[1]
    queryHeight = query.shape[0]
    query = matlabEngine.load_query_image_compatible_with_cutouts(queryPath, matlab.double(cutoutSize), nargout=1)
    query = np.asarray(query)

    ImgList = sio.loadmat(shortlistPath, squeeze_me=True)['ImgList']
    ImgListRecord = next((x for x in ImgList if x['queryname'] == queryName), None)
    cutoutPath = ImgListRecord['topNname'][0]

    synthPath = os.path.join(synthesizedDir, queryName, buildCutoutName(cutoutPath, '.synth.mat'))
    synthData = sio.loadmat(synthPath, squeeze_me=True)
    synth = synthData['RGBpersp']
    errmap = synthData['errmap']

    inlierPath = os.path.join(denseInlierDir, queryName, buildCutoutName(cutoutPath, '.pnp_dense_inlier.mat'))
    inlierData = sio.loadmat(inlierPath)
    inls = inlierData['inls']
    tentatives_2d = inlierData['tentatives_2d']
    inls = np.reshape(inls, (inls.shape[1],)).astype(np.bool)
    inls_2d = tentatives_2d[:,inls] - 1 # MATLAB is 1-based

    cutout = plt.imread(os.path.join(cutoutDir, cutoutPath))

    if not os.path.isdir(queryPipelineDir):
        os.mkdir(queryPipelineDir)
    
    modeDir = os.path.join(queryPipelineDir, mode)
    if not os.path.isdir(modeDir):
        os.mkdir(modeDir)

    thisQueryPipelineDir = os.path.join(modeDir, queryName)

    if not os.path.isdir(thisQueryPipelineDir):
        os.mkdir(thisQueryPipelineDir)

    fig = plt.figure()
    plt.imshow(query)
    plt.plot(inls_2d[0,:], inls_2d[1,:], '.', markersize=inlierMarkerSize, color=inlierColor)
    queryNameNoExt = queryName.split('.')[0]
    queryStepPath = os.path.join(thisQueryPipelineDir, 'query_' + queryNameNoExt + extension)
    saveFigure(fig, queryStepPath, targetWidth, targetHeight)

    fig = plt.figure()
    plt.imshow(cutout)
    plt.plot(inls_2d[2,:], inls_2d[3,:], '.', markersize=inlierMarkerSize, color=inlierColor)
    cutoutStepPath = os.path.join(thisQueryPipelineDir, 'chosen_' + buildCutoutName(cutoutPath, extension))
    saveFigure(fig, cutoutStepPath, targetWidth, targetHeight)

    synthStepPath = os.path.join(thisQueryPipelineDir, 'synthesized' + 'PV.' + extension)
    synth = Image.fromarray(synth)
    synth = synth.resize((queryWidth, queryHeight), resample=Image.NEAREST)
    synth = np.asarray(synth)
    plt.imsave(synthStepPath, synth)

    # NOTE: the errmap typically does not have the same aspect ratio, so it will be stretched
    errmapStepPath = os.path.join(thisQueryPipelineDir, 'errmap' + extension)
    errmap = Image.fromarray(errmap)
    errmap = errmap.resize((targetWidth, targetHeight), resample=Image.NEAREST)
    errmap = np.asarray(errmap)
    plt.imsave(errmapStepPath, errmap, cmap='jet')

matlabEngine.addpath(r'functions/InLocCIIRC_utils/at_netvlad_function',nargout=0)
modes = ['PV']
queryIds = [55]
for mode in modes:
    for queryId in queryIds:
        renderForQuery(queryId, mode)