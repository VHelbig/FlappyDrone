import pandas as pd
import numpy as np
import matplotlib.pyplot  as plt
from libs.utm import utmconv

uv=utmconv()

def LoadPath(filepath):
    data=pd.read_csv(filepath)
    path=list()
    (_,_,_, e1, n1)=uv.geodetic_to_utm(data["lattitude"][0],data["longitude"][0])
    for i in range(len(data)):
        (_,_,_,x,y)=uv.geodetic_to_utm(data["lattitude"][i],data["longitude"][i])
        distance=np.sqrt((x-e1)**2+(y-n1)**2)
        path.append((data["time(s)"][i],distance,data["rssi"][i],data["remote-rssi"][i]))
    path=np.array(path)
    sorted_index=np.argsort(path[:,1])
    return path[sorted_index]

normalAntPath=LoadPath("exercise/normal antenna.csv")
ourAntPath=LoadPath("exercise/our antenna.csv")
fig, axes=plt.subplots(2,1)

axes[0].set_title("RSSI")
axes[0].plot(ourAntPath[:,1],ourAntPath[:,2],label="Our Antenna")
axes[0].plot(normalAntPath[:,1],normalAntPath[:,2],label="normal Antenna")
axes[0].legend()
axes[0].set_xlabel("Distance[m]")

axes[1].set_title("Remote RSSI")
axes[1].plot(ourAntPath[:,1],ourAntPath[:,3],label="Our Antenna")
axes[1].plot(normalAntPath[:,1],normalAntPath[:,3],label="normal Antenna")
axes[1].set_xlabel("Distance[m]")
axes[1].legend()

fig.tight_layout()
plt.show()


