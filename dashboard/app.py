# -*- coding: utf-8 -*-

# Run this app with `python app.py` and
# visit http://127.0.0.1:8050/ in your web browser.

import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output

import plotly.express as px

import os

from scripts.helpers import load_db3, gdf_builder, interkrige, rasterbuilder, html_points, html_raster

#Set file location names
rosbagname = "laptoptest"
db_loc = os.path.join(os.path.expanduser('~'), rosbagname , rosbagname+"_0.db3")

gdf = gdf_builder(load_db3(db_loc))
fig = px.scatter(gdf, y="distance", x="timestamp", title = "Depth")
fig2 = px.line(gdf,y = 'ph', x = "timestamp", title = "pH")
#app thingymajigz
app = dash.Dash(__name__)


app.layout = html.Div(children=[
                      html.Div(className='row', 
 # Define the row element
                               children=[
                                  html.Div(className='four columns div-user-controls',
                                           children = [
                                                html.H2('Striker v2 Progress Monitor'),
                                                html.P('''IMU measurements'''),
                                                dcc.Graph(id = "imu1", figure = fig),
                                                dcc.Graph(id = "imu2", figure = fig2),
                                                dcc.Interval(id='interval-plots', interval=1*2000, n_intervals=0)
                                           ]),# Every 2 sec, update plot
                                  html.Div(className='four columns div-for-charts bg-grey',
                                           children = [
                                               html.H2('''GPS Points acquired'''),
                                               html.Iframe(id = "map_points", srcDoc = open("./html_folium/points.html","r").read(),width = "100%", height = "100%"),
                                               dcc.Interval(id='interval-maps1', interval=1*5000, n_intervals=0)#every 5s update the points
                                           ]),  # Define the middle elements
                                    html.Div(className='four columns div-for-charts bg-grey',
                                           children = [
                                                html.P('''Depth'''),
                                               html.Iframe(id = "map_raster1", srcDoc = open("./html_folium/points.html","r").read(),width = "100%", height = "25%"),
                                               html.P('''Turbidity'''),
                                               html.Iframe(id = "map_raster2", srcDoc = open("./html_folium/points.html","r").read(),width = "100%", height = "25%"),
                                               html.P('''Temperature'''),
                                               html.Iframe(id = "map_raster3", srcDoc = open("./html_folium/points.html","r").read(),width = "100%", height = "25%"),
                                               html.P('''pH'''),
                                               html.Iframe(id = "map_raster4", srcDoc = open("./html_folium/points.html","r").read(),width = "100%", height = "25%"),
                                               dcc.Interval(id='interval-maps2', interval=1*10000, n_intervals=0)#every 10s update the rasters
                                           ])  # Define the right element
                                  
                                        ])
                                ])


@app.callback(Output("imu1", 'figure'),
              Output("imu2", 'figure'),
             [Input('interval-plots', 'n_intervals')])

def update_plots(n):
    gdf = gdf_builder(load_db3(db_loc))
    fig = px.scatter(gdf, y="distance", x="timestamp", title = "Depth")
    fig2 = px.line(gdf, y="ph", x= "timestamp",title = "pH")
    return fig,fig2


@app.callback(Output('map_points', 'srcDoc'),

              [Input('interval-maps1', 'n_intervals')])


def update_maps(n):
    gdf = gdf_builder(load_db3(db_loc))
        
    p_html = html_points(gdf = gdf, zoomlvl = 18, out = "points")
   
    return open(p_html, 'r').read()


@app.callback(Output('map_raster1', 'srcDoc'),
              Output('map_raster2', 'srcDoc'),
              Output('map_raster3', 'srcDoc'),
              Output('map_raster4', 'srcDoc'),        
              [Input('interval-maps2', 'n_intervals')])

def update_maps(n):
    gdf = gdf_builder(load_db3(db_loc))
    
    dist_z, dist_SS,x,y = interkrige(gdf["lon"], gdf["lat"], gdf["distance"], res=int(round(len(gdf)/10,0)))
    temp_z, temp_SS,x,y = interkrige(gdf["lon"], gdf["lat"], gdf["temp"], res=int(round(len(gdf)/10,0)))
    ph_z, ph_SS,x,y = interkrige(gdf["lon"], gdf["lat"], gdf["ph"], res=int(round(len(gdf)/10,0)))
    turb_z, turb_SS,x,y = interkrige(gdf["lon"], gdf["lat"], gdf["turb"], res=int(round(len(gdf)/10,0)))
        
    r1_html = html_raster(gdf = gdf, zoomlvl = 18, rasterloc = rasterbuilder(dist_z, x,y,"depth"), colormap = "Green", out = "depth")
    
    r2_html = html_raster(gdf = gdf, zoomlvl = 18, rasterloc = rasterbuilder(turb_z, x,y,"turb"), colormap = "Blue", out = "turb")
    
    r3_html = html_raster(gdf = gdf, zoomlvl = 18, rasterloc = rasterbuilder(ph_z, x,y,"ph"), colormap = "Red", out = "ph")
    
    r4_html = html_raster(gdf = gdf, zoomlvl = 18, rasterloc = rasterbuilder(temp_z, x,y,"temp"), colormap = "Grey", out = "temp")
    
    return open( r1_html, 'r').read(), open( r2_html, 'r').read(), open( r3_html, 'r').read(), open(r4_html, 'r').read()



if __name__ == '__main__':
    app.run_server(debug=True)
