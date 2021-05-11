#parses UInt32 msgs
def uint32(byt):
    import numpy as np
    l = 4
    return int.from_bytes(byt[-l:],"little")

#Parses my float64 WQ data
def wq_float64(byt):
    import numpy as np
    return np.frombuffer(byt[4:],dtype = np.float64)[2:]
    
#Parses the Float32 motors
def float32_motors(byt):
    import numpy as np
    return np.frombuffer(byt, dtype = np.float32)[-2:]

# Parses generic float64 arrays
def float64(byt):
    import numpy as np
    return np.frombuffer(byt[4:], dtype = np.float64)[2:]

# Parses GPS data

def gps_float64(byt):
    import numpy as np
    d = np.frombuffer(byt[20:-4],dtype = np.float64)[:3]
    if d.size <3:
        d = [0,0,0] # to stop erroring for no fix gps 0.0.0
    return d
'''
The big problem is that all data is stored as bytes, and thus need to be transformed back to actual information.
The float64 multiarrays are decoded into lists with a handy funvtion from numpy. But returns a list. This list
is split into multiple section later in the script
'''
# topics = topics df
# msgs = msgs df
# topic = topic to be selected 
# parser = one of the above
# cols = columns to split the data into, basically the order in which ROS is pulbishing them 
def parse_df(topics, msgs, topic, parser, cols = ["volts", "amps"]):
    import sqlite3
    import pandas as pd
    import numpy as np
    import os
    #find the specific topic_id that the topic has gained in the bag
    t_id = topics.id[topics["name"].str.contains(topic)]
    #subselect this data out of all the msgs
    df1 = msgs.loc[msgs["topic_id"].isin([t_id])].reset_index()
    #apply the conversion from bytes to actual numbers
    df1[topic] = df1["data"].apply(parser)
    #Create different columns from the lists created by the parser
    df = pd.DataFrame(df1[topic].to_list(), columns = cols)
    #add timestamp again
    df["timestamp"] = df1["timestamp"]
    return df

def rasterbuilder(val, x,y, filename):

    '''
    val = np.ndarray of an interpolation output
    x = np.linspace of the raster
    y = np.linspace of the raster
    filename = string of the title, location and .tif get appended to it.
    

    returns a string of the file location, where it stored the result
    '''
    ## Transform it to rasterdata! via rasterio
    import rasterio
    import os
    import numpy as np
    #projstrings

    rdnew = "+proj=sterea +lat_0=52.15616055555555 +lon_0=5.38763888888889 +k=0.9999079 +x_0=155000 +y_0=463000 +ellps=bessel +towgs84=565.417,50.3319,465.552,-0.398957,0.343988,-1.8774,4.0725 +units=m +no_defs ",

    wgs84 = "+proj=longlat +datum=WGS84 +no_defs"

    transform = rasterio.transform.guard_transform(rasterio.transform.from_bounds(min(x), min(y), max(x), max(y), 100,100))
        
    new_dataset = rasterio.open(
        os.path.join(os.getcwd(),"data" , filename + ".tif"),
        "w",
        driver = "GTiff",
        height = 100,
        width = 100,
        count = 1,
        dtype =  val.dtype,
        crs= wgs84,
        transform = transform
    )

    new_dataset.write(val,1)
    new_dataset.close()
   
    return os.path.join(os.getcwd(),"data" , filename + ".tif")

def interkrige(lon,lat,var,res, c_type = "geographic", model = "linear"):
    '''
    Interpolates all the data into a grid using the Ordinary Kriging model    
    lon = list of longitudes in WGS
    lat = lits of latitudes in WGS
    var = list of variables to interpolate
    res = int number of pixels for x and y
    
    c_type = string "geographic" for circular lat -90 to 90 and lon  0 to 360 , "euclidean" for xy
    
    returns a the values, z;the SS;the x and y gridspacing
    '''
    
    from pykrige.ok import OrdinaryKriging
    import numpy as np
    minx = min(lon) 
    maxx= max(lon)
    miny = min(lat)
    maxy = max(lat)

    #GOtta do smth about these lines to create a bit larger of a grid
    x = np.linspace(start = maxx,stop = minx, num = res,retstep = True)
    y = np.linspace(start = miny, stop = maxy, num = res,retstep = True)
    
    gridx = np.linspace(start = minx, stop = maxx,num = res)
    gridy = np.linspace(start = maxy, stop = miny,num = res)

    try:
        z,SS = OrdinaryKriging(x = lon,
                               y = lat, 
                               z = var, 
                               variogram_model = model,
                               coordinates_type = "geographic").execute("grid",xpoints = gridx, ypoints = gridy)
    except ValueError:
        xx,yy = np.meshgrid(x,y)
        z = np.zeros_like(xx,dtype = np.uint8)
        SS=0
        
    return (z,SS,gridx,gridy)
