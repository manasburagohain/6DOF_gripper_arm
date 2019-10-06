import cv2
from sklearn.cluster import KMeans
import freenect

class DominantColors:

    CLUSTERS = None
    IMAGE = None
    COLORS = None
    LABELS = None
    
    def __init__(self, image, clusters=3):
        self.CLUSTERS = clusters
        self.IMAGE = image
        
    def dominantColors(self):
    
        #read image
        img = freenect.sync_get_video()[0]
        h = 70
        w = 65
        img = img[241:241+h,397:397+w]
        #convert to rgb from bgr
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                
        #reshaping to a list of pixels
        img = img.reshape((img.shape[0] * img.shape[1], 3))
        
        #save image after operations
        self.IMAGE = img
        
        #using k-means to cluster pixels
        kmeans = KMeans(n_clusters = self.CLUSTERS)
        kmeans.fit(img)
        
        #the cluster centers are our dominant colors.
        self.COLORS = kmeans.cluster_centers_
        
        #save labels
        self.LABELS = kmeans.labels_
        
        #returning after converting to integer from float
        return self.COLORS.astype(int)

img = freenect.sync_get_video()[0]
clusters = 8
dc = DominantColors(img, clusters) 
colors = dc.dominantColors()
print(colors)