
import numpy as np
import pylab
from skimage.measure import ransac

# as column vectors


def MahalanobisDist(x, y):
    covariance_xy = np.cov(x,y, rowvar=0)
    inv_covariance_xy = np.linalg.inv(covariance_xy)
    xy_mean = np.mean(x),np.mean(y)
    x_diff = np.array([x_i - xy_mean[0] for x_i in x])
    y_diff = np.array([y_i - xy_mean[1] for y_i in y])
    diff_xy = np.transpose([x_diff, y_diff])
    
    md = []
    for i in range(len(diff_xy)):
        md.append(np.sqrt(np.dot(np.dot(np.transpose(diff_xy[i]),inv_covariance_xy),diff_xy[i])))
    return md

def MD_removeOutliers(x, y):
    MD = MahalanobisDist(x, y)
    threshold = np.mean(MD) * 1.5 # adjust 1.5 accordingly 
    nx, ny, outliers_x,outliers_y = [], [], [] ,[]
    for i in range(len(MD)):
        if MD[i] <= threshold:
            nx.append(x[i])
            ny.append(y[i])
        else:
            outliers_x.append(x[i])
            outliers_y.append(y[i])
    return (np.array(nx), np.array(ny), np.array(outliers_x),np.array(outliers_y))

x = np.random.poisson(5,100)
y = np.random.poisson(5,100)

x,y,o_x,o_y=MD_removeOutliers(x,y)
print x,y,o_x,o_y

pylab.plot(x,y,'r+',o_x,o_y,'og')
pylab.show()
