def load_db3(db_loc):
    import sqlite3
    import pandas as pd
    import numpy as np
    
    def uint32(byt):
        l = 4
        return int.from_bytes(byt[-l:],"little")

    def wq_float64(byt):
            return np.frombuffer(byt[4:],dtype = np.float64)[2:]

    def gps_float64(byt):
            d = np.frombuffer(byt[20:-4],dtype = np.float64)[:3]
            if d.size <3:
                d = [np.nan,np.nan,np.nan] # to stop erroring for no fix gps 0.0.0
            return d

    con = sqlite3.connect(db_loc)
    topics = pd.read_sql("SELECT * from topics", con)
    msgs = pd.read_sql("SELECT * from messages", con)

    dist_id = topics.id[topics["name"].str.contains("sonar_lora")]
    
    ### change for other GPS TOPIC ###
    gps_id = topics.id[topics["name"].str.contains("gps_lora")]
    
    
    sensor_id = topics.id[topics["name"].str.contains("wq_lora")]

    df= msgs.loc[msgs["topic_id"].isin([dist_id,gps_id,sensor_id])].reset_index()


    #Preprocess data from the sonar from bytearrat to individual column
    distdf = msgs.loc[msgs["topic_id"].isin(dist_id)].reset_index()
    distdf["distance"] = distdf["data"].apply(uint32)
    distdf = distdf.drop("data",1)


    #Preprocess data of the gps from bytearrays to individual columns gpdf
    gcols = ["lat", "lon", "alt"]
    gpsdf = msgs.loc[msgs["topic_id"].isin(gps_id)].reset_index()
    gpsdf["latlonalt"]= gpsdf["data"].apply(gps_float64)

    gpdf = pd.DataFrame(gpsdf["latlonalt"].to_list(), columns = gcols)
    gpdf["timestamp"]  = gpsdf["timestamp"]
    gpdf["topic_id"] = gpsdf["topic_id"]

    #Preprocess data of the sensors from bytearrays to individual columns to sdf
    cols = ["temp","tds","turb","t_v","ph","ph_v"]
    sensordf = msgs.loc[msgs["topic_id"].isin(sensor_id)].reset_index()
    sensordf["info"] = sensordf["data"].apply(wq_float64)

    sdf = pd.DataFrame(sensordf["info"].to_list(),columns = cols)
    sdf["timestamp"] = sensordf["timestamp"]
    sdf["topic_id"] = sensordf["topic_id"]
    
    # and remerge the dataset back to a nice, tidy DF    
    df = pd.merge_asof(distdf,gpdf, on = "timestamp")
    df = pd.merge_asof(df, sdf, on = "timestamp").dropna()
    con.close()
    return df


def gdf_builder(df):
    import geopandas as gpd
    '''
    Creates "test" with some random data
    df = Pandas Dataframe with sensor observations
    '''
    #create gdf to convert WGS84 to UTM
    gdf = gpd.GeoDataFrame(df, geometry=gpd.points_from_xy(df.lon, df.lat), crs = "EPSG:4326")
    return(gdf)

def reducer(df,names,w):
    '''
    reduces all data with a rolling mean filter every w steps
    df = input dataframe
    names = list of columns to be filtered
    w = window size
    
    returns a new dataframe
    '''


    import pandas as pd
    import geopandas as gpd
    
    gdf = gpd.GeoDataFrame()  
    
    for name in names:
        l = df[name].rolling(window =w).mean()
        gdf[name] = l.iloc[::w]
    gdf["lat"] = df["lat"].iloc[::w]
    gdf["lon"] = df["lon"].iloc[::w]
    
    return gdf.dropna()
    
def interkrige(lon,lat,var,res, c_type = "geographic"):
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

    #GOtta do smth about these lines to create a bit larger of a grid
    x = np.linspace(start = min(lon),stop = max(lon), num = res,retstep = True)
    y = np.linspace(start = min(lat), stop = max(lat), num = res,retstep = True)
    
    gridx = np.linspace(start = min(lon)-x[1], stop = max(lon)+x[1],num = res)
    gridy = np.linspace(start = min(lat)+y[1], stop = max(lat)-y[1],num = res)

    try:
        z,SS = OrdinaryKriging(x = lon,
                               y = lat, 
                               z = var, 
                               variogram_model = "linear",
                               coordinates_type = "geographic").execute("grid",xpoints = gridx, ypoints = gridy)
    except ValueError:
        xx,yy = np.meshgrid(x,y)
        z = np.zeros_like(xx,dtype = np.uint8)
        SS=0
        
    return (z,SS,gridx,gridy)

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

def html_points(gdf,zoomlvl,out):
    '''
    gdf = geodataframe with .lat and .lon
    zoomlvl = int somewhere between 16-20
    out = string of filename
    
    returns a string of the file location, where it stored the result
    '''
    import os
    import folium
    import numpy as np
    import rasterio 
    import geopandas
    import matplotlib

    #set up base map
    my_coords = (np.mean(gdf.lat),np.mean(gdf.lon))
    m = folium.Map(location = my_coords, zoom_start =zoomlvl)

    #create points from the GPS input data
    latitudes = gdf.lat
    longitudes = gdf.lon

    for lat, lng in zip(latitudes, longitudes):
        folium.Marker(
          location = [lat, lng], 
          icon = folium.Icon(color='red', icon='info-sign')
         ).add_to(m) 

    m.save(os.path.join(os.getcwd(),"html_folium" , out + ".html"))
       
    return os.path.join(os.getcwd(),"html_folium" , out + ".html")
    
def html_raster(gdf,zoomlvl, rasterloc,colormap,out):
    '''
    gdf = geodataframe with .lat and .lon
    zoomlvl = int somewhere between 16-20
    rasterloc = string of .geotiff file
    colormap = string of Green, Red, Blue or anyhting else (becomes grey)
    out = string of filename
    
    returns a string of the file location, where it stored the result
    '''
    import os
    import folium
    import numpy as np
    import rasterio 
    import geopandas
    from matplotlib.pyplot import cm
    
    if colormap == "Green":
        resultcm = cm.Greens
    elif colormap == "Red":
        resultcm = cm.Reds
    elif colormap == "Blue":
        resultcm = cm.Blues
    else:
        resultcm = cm.Greys

    r =  rasterio.open(rasterloc)
    data = r.read(1)
    bounds = r.bounds


    #Reshape data to form min = 0 and max =1 except when the data consists of zeros
    if np.all(data==0):
        image = data
    else:
        image = (data - np.min(data))/np.ptp(data)

    #set up base map
    my_coords = (np.mean(gdf.lat),np.mean(gdf.lon))
    m = folium.Map(location = my_coords, zoom_start =zoomlvl)

    #show the interpolated inputs
    folium.raster_layers.ImageOverlay(
    image=image,
    bounds=[[bounds.bottom, bounds.left], [bounds.top, bounds.right]],
    opacity=1,
    origin='lower',
    colormap = resultcm).add_to(m)

    m.save(os.path.join(os.getcwd(),"html_folium" , out + ".html"))

    return os.path.join(os.getcwd(),"html_folium" , out + ".html")
