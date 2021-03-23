# -*- coding: utf-8 -*-

# Run this app with `python app.py` and
# visit http://127.0.0.1:8050/ in your web browser.
from urllib.request import urlopen


def internet_on():
    try:
        urlopen('http://216.58.192.142', timeout=1)
        return True
    except:
        return False

if internet_on():
	bm = "stamen-terrain"
else:
	bm = "white-bg"
	


import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output

import plotly.express as px

import os

from scripts.helpers import load_db3, gdf_builder, interkrige, rasterbuilder, html_points, html_raster, reducer

#Set file location names
rosbagname = "lora_Mar26"
db_loc = os.path.join(os.path.expanduser('~'), rosbagname , rosbagname+"_0.db3")

gdf = gdf_builder(load_db3(db_loc))

#depth figure
fig = px.scatter(gdf, y="distance", x="timestamp")
#pH figure
fig2 = px.line(gdf,y = 'ph', x = "timestamp")
#maps thingymajigz

ph = px.scatter_mapbox(gdf, lat="lat", lon = "lon",color = "ph",mapbox_style = bm, zoom = 17)
tds = px.scatter_mapbox(gdf, lat="lat", lon = "lon",color = "tds",mapbox_style = bm, zoom = 17)
turb = px.scatter_mapbox(gdf, lat="lat", lon = "lon",color = "turb",mapbox_style = bm, zoom = 17)
temp =px.scatter_mapbox(gdf, lat="lat", lon = "lon",color = "temp",mapbox_style = bm, zoom = 17)
depth = px.scatter_mapbox(gdf, lat="lat", lon = "lon",color = "distance",mapbox_style = bm, zoom = 17)

app = dash.Dash(__name__)


app.layout = html.Div(children=[
                      html.Div(className='row', 
 # Define the row element
                               children=[
                                  html.Div(className='four columns div-user-controls',
                                           children = [
                                  html.H2('Water Utility Rover Monitor'),
                                                html.P('''Time Series measurements'''),
                                                dcc.Graph(id = "imu1", figure = fig),
                                                dcc.Graph(id = "imu2", figure = fig2),
                                                dcc.Interval(id='interval-plots', interval=1*2000, n_intervals=0)
                                           ]),# Every 2 sec, update plot
                                  html.Div(className='four columns div-for-charts bg-grey',
                                           children = [
                                               dcc.Graph(id = "temp", figure = temp),
                                               dcc.Graph(id = "depth",figure = depth),
                                               dcc.Interval(id='interval-maps1', interval=3*5000, n_intervals=0)#every 10s update the points
                                           ]),  # Define the middle elements
                                    html.Div(className='four columns div-for-charts bg-grey',
                                           children = [
                                                
                                                dcc.Graph(id = "tds", figure = tds),
                                                dcc.Graph(id = "turb",figure = turb),
                                                dcc.Graph(id = "ph",figure = ph),
                                               dcc.Interval(id='interval-maps2', interval=3*5000, n_intervals=0)#every 20s update the rasters
                                           ])  # Define the right element
                                  
                                        ])
                                ])


@app.callback(Output("imu1", 'figure'),
              Output("imu2", 'figure'),
             [Input('interval-plots', 'n_intervals')])

def update_plots(n):
    gdf = gdf_builder(load_db3(db_loc))
    fig = px.scatter(gdf, y="temp", x="timestamp", title = "Temp: %f" % (gdf["temp"].iloc[-1]))
    fig2 = px.line(gdf, y="ph", x= "timestamp",title = "pH: %f" % (gdf["ph"].iloc[-1]))
    return fig,fig2


@app.callback(Output('temp', 'figure'),
                Output('depth', 'figure'),

              [Input('interval-maps1', 'n_intervals')])


def update_maps(n):
    gdf = gdf_builder(load_db3(db_loc))
        
    temp =px.scatter_mapbox(gdf, lat="lat", lon = "lon",color = "temp",mapbox_style = bm, zoom = 17)
    depth = px.scatter_mapbox(gdf, lat="lat", lon = "lon",color = "distance",mapbox_style = bm, zoom = 17)
   
    return (temp,depth)


@app.callback(Output('tds', 'figure'),
              Output('turb', 'figure'), 
              Output('ph', 'figure'),          
              [Input('interval-maps2', 'n_intervals')])

def update_maps(n):
    gdf = gdf_builder(load_db3(db_loc))
    
    ph = px.scatter_mapbox(gdf, lat="lat", lon = "lon",color = "ph",mapbox_style = bm, zoom = 17)
    tds = px.scatter_mapbox(gdf, lat="lat", lon = "lon",color = "tds",mapbox_style = bm, zoom = 17)
    turb = px.scatter_mapbox(gdf, lat="lat", lon = "lon",color = "turb",mapbox_style = bm, zoom = 17)


    
    return (tds,turb,ph)



if __name__ == '__main__':
    app.run_server(debug=True)
