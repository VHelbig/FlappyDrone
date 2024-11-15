import pandas as pd
import numpy as np
from libs.utm import utmconv
from libs.exportkml import kmlclass
from scipy.ndimage import convolve
from enum import Enum

class PathStateEnum(Enum):
    NONE=0
    PANDAS_FRAME=1
    UTMPATH=2
    LAT_LONG_PATH=3

class PathManagement():
    def __init__(self) -> None:
        self.currentPath=None
        self.currentPathState=PathStateEnum.NONE
        self.hemisphere=None
        self.UTMZone=None
        self.UTMLetter=None
        self.SimplifyMap=None
    
    def LoadPath(self,filePath):
        self.currentPath=pd.read_csv(filePath)
        self.currentPathState=PathStateEnum.PANDAS_FRAME

    
    def ConvertToUTM(self):
        if self.currentPathState != PathStateEnum.PANDAS_FRAME:
            raise RuntimeError("Can only convert path, that has been freshly loaded")
        
        uv=utmconv()
        utmPath=list()
        (self.hemisphere, self.UTMZone, self.UTMLetter, e1, n1)=uv.geodetic_to_utm(self.currentPath["lattitude"][0],self.currentPath["longitude"][0])
        for i in range(len(self.currentPath)):
            (_,_,_,x,y)=uv.geodetic_to_utm(self.currentPath["lattitude"][i],self.currentPath["longitude"][i])
            utmPath.append((self.currentPath["time(s)"][i],x,y,self.currentPath["altitude"][i]))
        self.currentPath=np.array(utmPath)
        self.currentPathState=PathStateEnum.UTMPATH

    def RemoveOutliers(self,max_speed):
        if self.currentPathState != PathStateEnum.UTMPATH:
            raise RuntimeError("Can only process UTM paths, convert to utm first")
        
        #devide path into segments, in which the deviation is not bigger than the max plausible deviation
        sections=list()
        index=0
        for i in range(1,len(self.currentPath)):
            max_deviation,vec_mag=self.ExtractDeviationMaxDeviation(self.currentPath[i-1],self.currentPath[i],max_speed)
            if vec_mag> max_deviation:
                sections.append((index,i-1))
                index=i
        sections.append((index,len(self.currentPath)-1))

        #first pass: remove segments with only some points (extract plausible path segments)
        i=0
        while(i<len(sections)):
            if (sections[i][1]-sections[i][0]+1)<5:
                sections.pop(i)
            else:
                i+=1

        #second pass: remove segments, that dont make sense with starting segment
        i=1
        while(i<len(sections)):
            max_deviation,vec_mag=self.ExtractDeviationMaxDeviation(
                self.currentPath[sections[i-1][1]],
                self.currentPath[sections[i][0]],max_speed)
            
            if vec_mag> max_deviation:
                sections.pop(i)
            else:
                i+=1

        #Assemble good sections to path
        cleanData=list()
        for i in range(len(sections)):
            cleanData.extend(self.currentPath[sections[i][0]:(sections[i][1]+1)])
        self.currentPath=np.array(cleanData)

    def ExtractDeviationMaxDeviation(self,point1,point2,max_speed):
        diff=point2-point1
        if diff[0]==0: #delta time
            diff[0]=1
        max_deviation=max_speed*diff[0]
        vec_mag=np.sqrt(diff[1]**2+diff[2]**2)
        return max_deviation,vec_mag
    
    def ComputeSimplifyMap(self):
        if self.currentPathState != PathStateEnum.UTMPATH:
            raise RuntimeError("Can only process UTM paths, convert to utm first")
        
        NodeMap=list()
        bucketList=list()
        for i in range(len(self.currentPath)):
            bucketList.append(self.currentPath[i])

        while(len(bucketList)>2):
            AreaList=self.ComputeAreaList(bucketList)
            NodeMap.append(AreaList[0])
            bucketList.pop(AreaList[0][1])

        self.SimplifyMap=NodeMap

    def ComputeAreaList(self,path):
        NodeMap=list()

        for i in range(1,len(path)-1):
            area=self.ComputeArea(path[i-1],
                                  path[i],
                                  path[i+1])
            NodeMap.append((area,i,path[i]))

        NodeMap.sort(key=lambda x:(x[0]))
        return NodeMap

    def ComputeArea(self,point1,currentPoint,point2):
        point1=point1-currentPoint
        point2=point2-currentPoint
        return abs(np.cross(point1[1:3],point2[1:3]))

    def SimplifyPath(self,NodeNumber):
        if self.currentPathState != PathStateEnum.UTMPATH:
            raise RuntimeError("Can only process UTM paths, convert to utm first")
        
        selectedNodes=self.SimplifyMap[-NodeNumber:]
        selectedPath=list()
        for i in range(len(selectedNodes)):
            selectedPath.append(selectedNodes[i][2])
        selectedPath.sort(key=lambda x:(x[0]))
        self.currentPath=np.array(selectedPath)

    def ConvertToGeodedic(self):
        if self.currentPathState == PathStateEnum.UTMPATH:
        
            uv=utmconv()
            #convert back to lat long
            latLongPath=list()
            meanAlt=self.currentPath[:,3].mean()
            for i in range(len(self.currentPath)):
                (lat,long)=uv.utm_to_geodetic(self.hemisphere,self.UTMZone,self.currentPath[i,1],self.currentPath[i,2])
                latLongPath.append((self.currentPath[i,0],lat,long,meanAlt))#self.currentPath[i,3]))
            self.currentPath=np.array(latLongPath)
            self.currentPathState=PathStateEnum.LAT_LONG_PATH
        elif self.currentPathState == PathStateEnum.PANDAS_FRAME:
            self.currentPath=self.currentPath[["time(s)","lattitude","longitude","altitude"]].values
            self.currentPathState=PathStateEnum.LAT_LONG_PATH
        else:
            raise RuntimeError("Can only process UTM paths or freshly loaded paths, convert to utm first")

    def ExportKml(self,filePath):
        if self.currentPathState != PathStateEnum.LAT_LONG_PATH:
            raise Exception("Convert to lattitude longitude path first")

        kml=kmlclass()
        kml.begin(filePath, 'Example', 'Example on the use of kmlclass', 0.7)
        kml.trksegbegin ('', '', 'red', 'absolute') 
        for i in range(len(self.currentPath)):
            if not np.isnan(self.currentPath[i]).any():
                kml.pt(self.currentPath[i,1],self.currentPath[i,2],self.currentPath[i,3])
        kml.trksegend()
        kml.end()