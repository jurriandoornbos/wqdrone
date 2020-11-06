def interkrige(lon,lat,var):
    from pykrige.uk import UniversalKriging
    import numpy as np

    x = np.linspace(min(lon),max(lon), len(lon))
    y = np.linspace(max(lat),min(lat), len(lat))

    z,SS = UniversalKriging(lat,
                          lon, 
                          var,
                          variogram_model = "linear",
                          drift_terms=["regional_linear"]).execute("grid",x,y)
    return (z,SS)
