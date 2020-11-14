def load_db3(db_loc,rosbagname):
    '''
    db_loc = string of b3 file locations
    rosbagname = name of the rosbag
    
    returns df
    '''
    #SCP DATA import from db3 file created w/ ROS2 bag
    import os
    import pandas as pd
    import sqlite3

    def load_data(path):
        con = sqlite3.connect(path)
        return pd.read_sql_query("SELECT * from messages", con)

    df = load_data(db_loc)

    #Regex method to convert all the byte strings to data
    df["decode"] = df["data"].str.decode("utf-8")
    #regex string to pick all the numbers after "Sensor: " creating a new dataframe
    df2 = df["decode"].str.extract("Temperature:\s([\-0-9\.]+).*TDS:\s([\-0-9\.]+).*Voltage:\s([\-0-9\.]+).*Acidity:\s([\-0-9\.]+)",expand = True)
    #rename the columns
    df2.columns = ["temp","tds","turb","ph"]
    #convert to floats
    df2 = df2[df2.columns].apply(pd.to_numeric)
    #add the timestamp list
    df2["timestamp"] = df["timestamp"]
    return df2


def gdf_builder(df):
    '''
    Creates "test" with some random data
    df = Pandas Dataframe with sensor observations
    returns gdf
    '''
    
    import numpy as np
    import geopandas as gpd

    #Create fake Lat and Long data
    df["lat"] = (np.random.standard_normal(len(df))/1000)+52
    df["lon"] = (np.random.standard_normal(len(df))/1000)+5

    #create some fake test data as well
    df["test"] = np.random.rand(len(df))

    #create gdf to convert WGS84 to UTM
    gdf = gpd.GeoDataFrame(df, geometry=gpd.points_from_xy(df.lon, df.lat), crs = "EPSG:4326")
    return(gdf)

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

    z,SS = OrdinaryKriging(x = lon,
                           y = lat, 
                           z = var, 
                           variogram_model = "linear",
                           coordinates_type = "geographic").execute("grid",xpoints = gridx, ypoints = gridy)
    
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

    #projstrings

    rdnew = "+proj=sterea +lat_0=52.15616055555555 +lon_0=5.38763888888889 +k=0.9999079 +x_0=155000 +y_0=463000 +ellps=bessel +towgs84=565.417,50.3319,465.552,-0.398957,0.343988,-1.8774,4.0725 +units=m +no_defs ",

    wgs84 = "+proj=longlat +datum=WGS84 +no_defs"

    transform = rasterio.transform.guard_transform(rasterio.transform.from_bounds(min(x), min(y), max(x), max(y), 100,100))

    new_dataset = rasterio.open(
        "./data/" + filename + ".tif",
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
   
    return "./data/" + filename + ".tif"

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

    m.save(os.path.join('html_folium', out+ '.html'))
       
    return os.path.join('html_folium', out+ '.html')
    
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


    #Reshape data to form min = 0 and max =1
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

    m.save(os.path.join('html_folium', out+ '.html'))

    return os.path.join('html_folium', out+ '.html')
