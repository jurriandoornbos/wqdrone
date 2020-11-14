# -*- coding: utf-8 -*-

# Run this app with `python app.py` and
# visit http://127.0.0.1:8050/ in your web browser.

import dash
import dash_core_components as dcc
import dash_html_components as html
from dash.dependencies import Input, Output

import os

from scripts.helpers import load_db3, gdf_builder, interkrige, rasterbuilder, html_points, html_raster

#set GDAL_DAAA environment; mioght be surplus at this point
os.environ["GDAL_DATA"] = '/home/pop/.conda/envs/interpol/lib/python3.8/site-packages/rasterio/gdal_data'

#Set file location names
rosbagname = "my_test_bag"
db_loc = os.path.join(os.path.expanduser('~'), rosbagname,rosbagname+"_0.db3")

#app thingymajigz
app = dash.Dash(__name__)


app.layout = html.Div(children=[
                      html.Div(className='row',  # Define the row element
                               children=[
                                  html.Div(className='four columns div-user-controls',
                                           children = [
                                                html.H2('Striker v2 Progress Monitor'),
                                                html.P('''Settings below'''),
                                                html.P('''''')]),  # Define the left element
                                  html.Div(className='eight columns div-for-charts bg-grey',
                                           children = [
                                               html.Iframe(id = "map_points", srcDoc = open("./html_folium/points.html","r").read(), width = "300", height = "300"),
                                               html.Iframe(id = "map_raster1", srcDoc = open("./html_folium/test_raster.html","r").read(), width = "300", height = "300"),
                                               dcc.Interval(id='interval-component', interval=1*10000, n_intervals=0)#every 10s update the maps
                                           ])  # Define the right element
                                  
                                        ])
                                ])

@app.callback(Output('map_points', 'srcDoc'),
              Output('map_raster1', 'srcDoc'),
              [Input('interval-component', 'n_intervals')])


def update_map1(n):
    gdf = gdf_builder(load_db3(db_loc, rosbagname))
    
    tds_z, tds_SS,x,y = interkrige(gdf["lon"], gdf["lat"], gdf["test"], res=100)
    
    p_html = html_points(gdf = gdf, zoomlvl = 16, out = "points")
    
    r1_html = html_raster(gdf = gdf, zoomlvl = 16, rasterloc = rasterbuilder(tds_z, x,y,"TDS"), colormap = "Green", out = "TDS")
    
    r2_html = "Turb"
    
    r3_html = "pH"
    
    r4_html = "Temp"
    
    return open('./' + p_html, 'r').read(), open('./' + r1_html, 'r').read()


if __name__ == '__main__':
    app.run_server(debug=True)
