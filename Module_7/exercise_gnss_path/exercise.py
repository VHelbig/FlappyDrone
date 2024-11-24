import pandas as pd
import numpy as np
from libs.utm import utmconv
from libs.exportkml import kmlclass
from PathManagement import PathManagement

folder="Module_7/exercise_gnss_path/"
filePath=folder+"gps_log-copy.csv"
PathManager=PathManagement()
PathManager.LoadPath(filePath)
PathManager.ConvertToUTM()
max_speed=10 #m/s
PathManager.RemoveOutliers(max_speed)
PathManager.ComputeSimplifyMap()
PathManager.SimplifyPath(10)
PathManager.ConvertToGeodedic()
PathManager.ExportKml("output.kml")
PathManager.ExportQGCPlan("mission_plan.json")
